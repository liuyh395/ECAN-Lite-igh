/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#define _DEFAULT_SOURCE

/*
 * Minimal 250 us / four-channel example.
 *
 * The pipeline owns EtherCAT, PDO packing and real-time scheduling.  User code
 * only consumes received CAN frames and produces frames for the active channel.
 */
#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "igh_250us_pipeline.h"

#define DEMO_CAN_ID       0x101U
#define DEMO_RX_CAPACITY  128U
#define DEMO_GAP_BUCKETS  16U
#define DEMO_MAX_FRAMES_PER_SLOT  8U

typedef struct {
    uint32_t next_sequence;
    uint32_t next_tx_sequence;
    uint32_t pending_sequence;
    uint64_t pending_sent_cycle;
    uint8_t initialized;
    uint8_t pending;
} demo_stream_state_t;

static volatile sig_atomic_t stop_requested;

static void stop_handler(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void build_command(uint8_t channel, uint32_t sequence, uint8_t data[8])
{
    data[0] = 0xECU;
    data[1] = 0x25U;
    data[2] = channel;
    data[3] = (uint8_t)sequence;
    data[4] = (uint8_t)(sequence >> 8);
    data[5] = (uint8_t)(sequence >> 16);
    data[6] = (uint8_t)(sequence >> 24);
    data[7] = (uint8_t)(data[0] ^ data[1] ^ data[2] ^ data[3] ^
                        data[4] ^ data[5] ^ data[6]);
}

static int command_is_valid(const igh_mcan_rx_frame_t *rx,
                            uint32_t frames_per_slot,
                            uint8_t rx_channel_xor)
{
    const uint8_t *data = rx->frame.data;
    uint8_t checksum;

    if (rx->frame.can_id < DEMO_CAN_ID ||
        rx->frame.can_id >= DEMO_CAN_ID + frames_per_slot ||
        rx->frame.is_canfd == 0U ||
        rx->frame.data_len != 8U || data[0] != 0xECU || data[1] != 0x25U ||
        data[2] >= IGH_250US_PIPELINE_PHASE_COUNT ||
        (uint8_t)(data[2] ^ rx_channel_xor) != rx->frame.channel) {
        return 0;
    }
    checksum = (uint8_t)(data[0] ^ data[1] ^ data[2] ^ data[3] ^
                         data[4] ^ data[5] ^ data[6]);
    return data[7] == checksum;
}

static uint32_t command_sequence(const igh_mcan_rx_frame_t *rx)
{
    const uint8_t *data = rx->frame.data;
    return (uint32_t)data[3] | ((uint32_t)data[4] << 8) |
           ((uint32_t)data[5] << 16) | ((uint32_t)data[6] << 24);
}

static void account_received_frame(const igh_mcan_rx_frame_t *rx,
                                   uint64_t cycle_index,
                                   uint32_t frames_per_slot,
                                   uint8_t rx_channel_xor,
                                   uint8_t closed_loop,
                                   const uint8_t phase_channels[IGH_250US_PIPELINE_PHASE_COUNT],
                                   demo_stream_state_t streams[ECAN_IGH_MAX_SLAVES]
                                                                      [IGH_250US_PIPELINE_PHASE_COUNT]
                                                                      [DEMO_MAX_FRAMES_PER_SLOT],
                                   uint64_t *valid_rx,
                                   uint64_t *invalid_rx,
                                   uint64_t *sequence_errors,
                                   uint64_t *late_frames,
                                   uint64_t histogram[DEMO_GAP_BUCKETS])
{
    demo_stream_state_t *stream;
    uint8_t tx_channel;
    uint32_t sequence;
    uint32_t frame_slot;
    uint64_t sent_cycle;
    uint64_t gap;
    size_t bucket;
    uint8_t phase = 0U;

    if (!command_is_valid(rx, frames_per_slot, rx_channel_xor)) {
        (*invalid_rx)++;
        return;
    }
    if (rx->slave_pos < 0 || rx->slave_pos >= (int)ECAN_IGH_MAX_SLAVES) {
        (*invalid_rx)++;
        return;
    }
    tx_channel = rx->frame.data[2];
    frame_slot = rx->frame.can_id - DEMO_CAN_ID;
    sequence = command_sequence(rx);
    stream = &streams[rx->slave_pos][tx_channel][frame_slot];
    if (stream->initialized == 0U) {
        stream->initialized = 1U;
        stream->next_sequence = 0U;
    }
    if (sequence != stream->next_sequence) {
        (*sequence_errors)++;
        stream->next_sequence = sequence + 1U;
    } else {
        stream->next_sequence++;
    }
    if (closed_loop != 0U) {
        if (stream->pending == 0U || sequence != stream->pending_sequence) {
            (*sequence_errors)++;
            (*invalid_rx)++;
            return;
        }
        sent_cycle = stream->pending_sent_cycle;
        stream->pending = 0U;
    } else {
        while (phase < IGH_250US_PIPELINE_PHASE_COUNT &&
               phase_channels[phase] != tx_channel) phase++;
        if (phase == IGH_250US_PIPELINE_PHASE_COUNT) {
            (*invalid_rx)++;
            return;
        }
        sent_cycle = (uint64_t)sequence * IGH_250US_PIPELINE_PHASE_COUNT + phase;
    }
    gap = cycle_index >= sent_cycle ? cycle_index - sent_cycle : UINT64_MAX;
    bucket = gap < DEMO_GAP_BUCKETS - 1U ?
             (size_t)gap : DEMO_GAP_BUCKETS - 1U;
    histogram[bucket]++;
    if (gap > IGH_250US_PIPELINE_PHASE_COUNT) (*late_frames)++;
    (*valid_rx)++;
}

static void usage(const char *program)
{
    printf("Usage: %s [--sm] [--slaves N]"
           " [--cycles N] [--tx-slave-mask MASK] [--drain-cycles N]"
           " [--rx-channel-xor N] [--warmup-cycles N]"
           " [--cpu N] [--rt-priority N]"
           " [--tx | --closed-loop] [--frames-per-slot N] [--stage-timing]"
           " [--interleave-pairs] [--control-div N]\n",
           program);
}

int main(int argc, char **argv)
{
    igh_250us_pipeline_config_t config;
    igh_250us_pipeline_t pipeline = IGH_250US_PIPELINE_INITIALIZER;
    igh_250us_pipeline_stats_t stats;
    uint32_t cycle_limit = 40000U;
    uint32_t drain_cycles = 8U;
    uint32_t frames_per_slot = 1U;
    uint32_t control_div = 1U;
    uint32_t tx_slave_mask = UINT32_MAX;
    uint64_t valid_rx = 0U;
    uint64_t invalid_rx = 0U;
    uint64_t sequence_errors = 0U;
    uint64_t late_frames = 0U;
    uint64_t command_cycles = 0U;
    uint64_t closed_loop_blocked_slots = 0U;
    uint64_t gap_histogram[DEMO_GAP_BUCKETS] = {0};
    demo_stream_state_t streams[ECAN_IGH_MAX_SLAVES]
                               [IGH_250US_PIPELINE_PHASE_COUNT]
                               [DEMO_MAX_FRAMES_PER_SLOT] = {{{{0}}}};
    uint8_t transmit = 0U;
    uint8_t closed_loop = 0U;
    uint8_t rx_channel_xor = 0U;
    int rc = IGH_ECAN_EXIT_FAIL;

    igh_250us_pipeline_default_config(&config);
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return IGH_ECAN_EXIT_OK;
        }
        else if (strcmp(argv[i], "--sm") == 0) config.mode = IGH_250US_PIPELINE_SM;
        else if (strcmp(argv[i], "--tx") == 0) transmit = 1U;
        else if (strcmp(argv[i], "--closed-loop") == 0) {
            transmit = 1U;
            closed_loop = 1U;
        }
        else if (strcmp(argv[i], "--stage-timing") == 0) config.stage_timing = 1U;
        else if (strcmp(argv[i], "--interleave-pairs") == 0) {
            if (igh_250us_pipeline_config_set_phase_order(
                    &config, 0U, 2U, 1U, 3U) != IGH_ECAN_OK) {
                return IGH_ECAN_EXIT_ARG_ERROR;
            }
        }
        else if (strcmp(argv[i], "--slaves") == 0 && i + 1 < argc)
            config.slave_count = igh_example_size(argv[++i]);
        else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc)
            cycle_limit = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--drain-cycles") == 0 && i + 1 < argc)
            drain_cycles = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--warmup-cycles") == 0 && i + 1 < argc)
            config.warmup_cycles = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--frames-per-slot") == 0 && i + 1 < argc)
            frames_per_slot = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--control-div") == 0 && i + 1 < argc)
            control_div = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--tx-slave-mask") == 0 && i + 1 < argc)
            tx_slave_mask = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--rx-channel-xor") == 0 && i + 1 < argc)
            rx_channel_xor = igh_example_u8(argv[++i]);
        else if (strcmp(argv[i], "--cpu") == 0 && i + 1 < argc)
            config.cpu = igh_example_int(argv[++i]);
        else if (strcmp(argv[i], "--rt-priority") == 0 && i + 1 < argc)
            config.rt_priority = igh_example_int(argv[++i]);
        else {
            usage(argv[0]);
            return IGH_ECAN_EXIT_ARG_ERROR;
        }
    }
    if (config.slave_count == 0U || config.slave_count > ECAN_IGH_MAX_SLAVES) {
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    {
        uint32_t valid_slave_mask = (1UL << config.slave_count) - 1UL;
        if (tx_slave_mask == UINT32_MAX) tx_slave_mask = valid_slave_mask;
        else if ((tx_slave_mask & ~valid_slave_mask) != 0U) {
            return IGH_ECAN_EXIT_ARG_ERROR;
        }
    }
    if (cycle_limit == 0U || frames_per_slot == 0U ||
        frames_per_slot > DEMO_MAX_FRAMES_PER_SLOT ||
        control_div == 0U || (control_div > 1U && closed_loop == 0U) ||
        rx_channel_xor >= IGH_250US_PIPELINE_PHASE_COUNT) {
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    signal(SIGINT, stop_handler);
    signal(SIGTERM, stop_handler);
    {
        int open_rc = igh_250us_pipeline_open(&pipeline, &config);
        if (open_rc != IGH_ECAN_OK) {
            igh_250us_pipeline_open_diag_t diag;
            igh_250us_pipeline_get_last_open_diag(&diag);
            fprintf(stderr, "failed to open 250 us pipeline: rc=%d stage=%s\n",
                    open_rc, igh_250us_pipeline_open_stage_name(
                        igh_250us_pipeline_last_open_stage()));
            fprintf(stderr, "open_diag domain=%u all_op=%u wkc=%u expected=%u"
                    " mismatch=%" PRIu64 " al=",
                    diag.domain_complete, diag.all_operational,
                    diag.working_counter,
                    diag.expected_working_counter,
                    diag.ethercat.wkc_mismatch_count);
            for (size_t slave = 0U; slave < diag.slave_count; slave++) {
                fprintf(stderr, "%s0x%02X", slave == 0U ? "" : ",",
                        diag.slave_al_state[slave]);
            }
            fprintf(stderr, "\n");
            if (diag.last_event_valid != 0U) {
                fprintf(stderr, "last_event cycle=%" PRIu64 " first_bad=%" PRId32,
                        diag.last_event.ethercat_cycle,
                        diag.last_event.first_bad_domain);
                for (uint32_t domain = 0U;
                     domain < diag.last_event.domain_count; domain++) {
                    const ecan_igh_diag_domain_sample_t *sample =
                        &diag.last_event.domains[domain];
                    fprintf(stderr, " d%u=s%u/k%u/w%u/st%u", domain,
                            sample->slave_index, sample->kind,
                            sample->working_counter, sample->wc_state);
                }
                fprintf(stderr, "\n");
            }
            return IGH_ECAN_EXIT_FAIL;
        }
    }

    printf("250us pipeline mode=SM subcycle=250us control=1ms phases=MCAN%u/%u/%u/%u"
           " channel_command_period=%" PRIu32 "ms slaves=%zu",
           config.phase_channels[0], config.phase_channels[1],
           config.phase_channels[2], config.phase_channels[3],
           control_div,
           config.slave_count);
    printf("\n");

    for (uint32_t i = 0U; i < cycle_limit && !stop_requested; i++) {
        igh_mcan_rx_frame_t rx[DEMO_RX_CAPACITY];
        igh_250us_pipeline_cycle_t cycle;
        uint8_t command[8];
        int cycle_rc;

        cycle_rc = igh_250us_pipeline_cycle_begin(&pipeline, rx,
                                                   DEMO_RX_CAPACITY, &cycle);
        if (cycle_rc != IGH_ECAN_OK && cycle_rc != IGH_ECAN_NO_SPACE) goto done;
        for (size_t n = 0U; n < cycle.rx_count; n++) {
            account_received_frame(&rx[n], cycle.index, frames_per_slot,
                                   rx_channel_xor, closed_loop,
                                   config.phase_channels, streams,
                                   &valid_rx, &invalid_rx, &sequence_errors,
                                   &late_frames, gap_histogram);
        }

        if (transmit != 0U &&
            ((cycle.index / IGH_250US_PIPELINE_PHASE_COUNT) % control_div) == 0U) {
            for (size_t slave = 0U; slave < config.slave_count; slave++) {
                if ((tx_slave_mask & (1UL << slave)) == 0U) continue;
                for (uint32_t frame = 0U; frame < frames_per_slot; frame++) {
                    demo_stream_state_t *stream =
                        &streams[slave][cycle.active_channel][frame];
                    uint32_t sequence;

                    if (closed_loop != 0U && stream->pending != 0U) {
                        closed_loop_blocked_slots++;
                        continue;
                    }
                    sequence = closed_loop != 0U ? stream->next_tx_sequence :
                               (uint32_t)(cycle.index /
                                          IGH_250US_PIPELINE_PHASE_COUNT);
                    build_command(cycle.active_channel,
                                  sequence, command);
                    if (igh_250us_pipeline_send_canfd(
                            &pipeline, (int)slave, DEMO_CAN_ID + frame, 0, 1,
                            command, sizeof(command)) != IGH_ECAN_OK) {
                        (void)igh_250us_pipeline_cycle_abort(&pipeline);
                        goto done;
                    }
                    if (closed_loop != 0U) {
                        stream->pending = 1U;
                        stream->pending_sequence = sequence;
                        stream->pending_sent_cycle = cycle.index;
                        stream->next_tx_sequence++;
                    }
                }
            }
        }
        if (igh_250us_pipeline_cycle_end(&pipeline) != IGH_ECAN_OK) goto done;
        command_cycles++;
    }

    for (uint32_t i = 0U; i < drain_cycles && !stop_requested; i++) {
        igh_mcan_rx_frame_t rx[DEMO_RX_CAPACITY];
        igh_250us_pipeline_cycle_t cycle;
        int cycle_rc = igh_250us_pipeline_cycle_begin(&pipeline, rx,
                                                       DEMO_RX_CAPACITY, &cycle);
        if (cycle_rc != IGH_ECAN_OK && cycle_rc != IGH_ECAN_NO_SPACE) goto done;
        for (size_t n = 0U; n < cycle.rx_count; n++) {
            account_received_frame(&rx[n], cycle.index, frames_per_slot,
                                   rx_channel_xor, closed_loop,
                                   config.phase_channels, streams,
                                   &valid_rx, &invalid_rx, &sequence_errors,
                                   &late_frames, gap_histogram);
        }
        if (igh_250us_pipeline_cycle_end(&pipeline) != IGH_ECAN_OK) goto done;
    }

    memset(&stats, 0, sizeof(stats));
    if (igh_250us_pipeline_get_stats(&pipeline, &stats) == IGH_ECAN_OK) {
        printf("result cycles=%" PRIu64 " rx_frames=%" PRIu64
               " tx_frames=%" PRIu64 " rx_errors=%" PRIu64
               " tx_errors=%" PRIu64 " no_space=%" PRIu64
               " resyncs=%" PRIu32 " wkc_mismatches=%" PRIu64 "\n",
               stats.cycles, stats.rx_frames, stats.tx_frames,
               stats.receive_errors, stats.send_errors,
               stats.no_space_events, stats.clock_resyncs,
               stats.ethercat.wkc_mismatch_count);
        printf("payload valid_rx=%" PRIu64 " invalid_rx=%" PRIu64
               " sequence_errors=%" PRIu64 " late_gt_n4=%" PRIu64
               " closed_loop_blocked_slots=%" PRIu64 "\n",
               valid_rx, invalid_rx, sequence_errors, late_frames,
               closed_loop_blocked_slots);
        printf("latency_subcycles");
        for (size_t i = 0U; i < DEMO_GAP_BUCKETS; i++) {
            if (gap_histogram[i] == 0U) continue;
            if (i == DEMO_GAP_BUCKETS - 1U) printf(" >=%zu:%" PRIu64, i, gap_histogram[i]);
            else printf(" %zu:%" PRIu64, i, gap_histogram[i]);
        }
        printf("\n");
        igh_rt_print_period_stats("period", &stats.period,
                                  IGH_250US_PIPELINE_CYCLE_US * 1000ULL);
        if (stats.stage_samples != 0U) {
            printf("stage_timing_ns samples=%" PRIu64
                   " transport_avg=%" PRIu64 " transport_max=%" PRIu64
                   " parse_avg=%" PRIu64 " parse_max=%" PRIu64
                   " application_avg=%" PRIu64 " application_max=%" PRIu64
                   " publish_avg=%" PRIu64 " publish_max=%" PRIu64 "\n",
                   stats.stage_samples,
                   stats.transport_begin_total_ns / stats.stage_samples,
                   stats.transport_begin_max_ns,
                   stats.rx_parse_total_ns / stats.stage_samples,
                   stats.rx_parse_max_ns,
                   stats.application_total_ns / stats.stage_samples,
                   stats.application_max_ns,
                   stats.publish_total_ns / stats.stage_samples,
                   stats.publish_max_ns);
        }
        {
            uint64_t expected_frames = stats.tx_frames;
            uint64_t pending_frames = 0U;
            if (closed_loop != 0U) {
                for (size_t slave = 0U; slave < config.slave_count; slave++) {
                    for (uint8_t channel = 0U;
                         channel < IGH_250US_PIPELINE_PHASE_COUNT; channel++) {
                        for (uint32_t frame = 0U; frame < frames_per_slot; frame++) {
                            pending_frames += streams[slave][channel][frame].pending;
                        }
                    }
                }
            } else if (transmit != 0U) {
                uint64_t selected_slaves = 0U;
                for (size_t slave = 0U; slave < config.slave_count; slave++) {
                    if ((tx_slave_mask & (1UL << slave)) != 0U) selected_slaves++;
                }
                expected_frames = command_cycles * selected_slaves * frames_per_slot;
            }
            printf("expected tx=%" PRIu64 " rx=%" PRIu64
                   " pending_streams=%" PRIu64
                   " blocked_slots=%" PRIu64 "\n",
                   expected_frames, expected_frames, pending_frames,
                   closed_loop_blocked_slots);
            if (stats.receive_errors == 0U && stats.send_errors == 0U &&
            stats.no_space_events == 0U && stats.clock_resyncs == 0U &&
                stats.ethercat.wkc_mismatch_count == 0U && invalid_rx == 0U &&
                sequence_errors == 0U && late_frames == 0U &&
                closed_loop_blocked_slots == 0U && pending_frames == 0U &&
                stats.tx_frames == expected_frames && valid_rx == expected_frames) {
                rc = IGH_ECAN_EXIT_OK;
            }
        }
    }

done:
    igh_250us_pipeline_close(&pipeline);
    return rc;
}
