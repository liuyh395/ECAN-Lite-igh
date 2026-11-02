/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "igh_mcan_port.h"
#include "igh_rt_support.h"
#include "mit_canfd_protocol.h"

#define MIT_DEMO_RX_CAPACITY 64U
#define MIT_DEMO_PI 3.14159265358979323846

typedef struct {
    int slave;
    uint8_t channel;
    uint8_t node_id;
    uint32_t command_id;
    uint32_t feedback_id;
    uint32_t cycles;
    uint32_t cycle_us;
    uint32_t wait_op_ms;
    uint32_t warmup_cycles;
    uint32_t drain_cycles;
    uint32_t max_missing;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t nominal_sp_x10;
    uint16_t data_sp_x10;
    uint8_t pdo_multiplier;
    uint8_t brs;
    uint8_t extended_id;
    uint8_t enable_motor;
    uint8_t zero_position;
    uint8_t disable_on_exit;
    uint8_t lock_memory;
    int rt_priority;
    int cpu;
    uint32_t spin_us;
    mit_canfd_limits_t limits;
    mit_canfd_command_t command;
    float sine_position_amplitude;
    uint32_t sine_period_cycles;
} demo_config_t;

typedef struct {
    uint64_t commands_sent;
    uint64_t feedback_received;
    uint64_t unrelated_frames;
    uint64_t malformed_feedback;
    uint64_t node_mismatch;
    uint64_t extra_feedback;
    uint64_t no_space;
    uint64_t send_errors;
    uint64_t latency_samples;
    uint64_t latency_sum_cycles;
    uint64_t latency_hist[10];
    uint32_t latency_min_cycles;
    uint32_t latency_max_cycles;
    mit_canfd_feedback_t last_feedback;
} demo_stats_t;

static volatile sig_atomic_t stop_requested;

static void on_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void default_config(demo_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->slave = 0;
    cfg->channel = 0U;
    cfg->node_id = 1U;
    cfg->command_id = 0x001U;
    cfg->feedback_id = 0x101U;
    cfg->cycles = 5000U;
    cfg->cycle_us = 1000U;
    cfg->wait_op_ms = 10000U;
    cfg->warmup_cycles = 32U;
    cfg->drain_cycles = 8U;
    cfg->max_missing = 16U;
    cfg->nominal_bitrate = 1000000U;
    cfg->data_bitrate = 5000000U;
    cfg->nominal_sp_x10 = 875U;
    cfg->data_sp_x10 = 875U;
    cfg->pdo_multiplier = 3U;
    cfg->brs = 1U;
    cfg->disable_on_exit = 1U;
    cfg->lock_memory = 1U;
    cfg->rt_priority = 80;
    cfg->cpu = -1;
    cfg->spin_us = 100U;
    cfg->sine_period_cycles = 1000U;
    mit_canfd_default_limits(&cfg->limits);
}

static void print_usage(const char *program)
{
    printf("Usage: %s [options]\n", program);
    printf("  --slave N --channel N --node N\n");
    printf("  --command-id ID --feedback-id ID [--extended-id]\n");
    printf("  --cycles N --cycle-us N --pdo-n N --max-missing N\n");
    printf("  default profile: DC 1ms, Sync0 shift 900us, N->N+1 closed loop\n");
    printf("  --bitrate N --data-bitrate N [--no-brs]\n");
    printf("  --nominal-sp-x10 N --data-sp-x10 N\n");
    printf("  --position F --velocity F --kp F --kd F --torque F\n");
    printf("  --sine-position-amplitude F --sine-period-cycles N\n");
    printf("  --p-min F --p-max F --v-min F --v-max F\n");
    printf("  --kp-min F --kp-max F --kd-min F --kd-max F\n");
    printf("  --torque-min F --torque-max F\n");
    printf("  --enable-motor [--zero-position] [--keep-enabled]\n");
    printf("  --wait-op-ms N --warmup-cycles N --drain-cycles N\n");
    printf("  --rt-priority N --cpu N --spin-us N [--no-mlock]\n");
    printf("\nSafety: --enable-motor and --zero-position are never implied. Verify motor\n");
    printf("ranges, mechanics, CAN IDs and emergency stop before using real hardware.\n");
}

static int need_value(int index, int argc, const char *option)
{
    if (index + 1 < argc) {
        return 1;
    }
    fprintf(stderr, "%s requires a value\n", option);
    return 0;
}

static int parse_args(int argc, char **argv, demo_config_t *cfg)
{
    for (int i = 1; i < argc; i++) {
#define PARSE_U32(name, field) \
        if (strcmp(argv[i], name) == 0) { \
            if (!need_value(i, argc, name)) return -1; \
            field = igh_example_u32(argv[++i]); \
        }
#define PARSE_FLOAT(name, field) \
        if (strcmp(argv[i], name) == 0) { \
            if (!need_value(i, argc, name)) return -1; \
            field = igh_example_float(argv[++i]); \
        }
        PARSE_U32("--command-id", cfg->command_id)
        else PARSE_U32("--feedback-id", cfg->feedback_id)
        else PARSE_U32("--cycles", cfg->cycles)
        else PARSE_U32("--cycle-us", cfg->cycle_us)
        else PARSE_U32("--wait-op-ms", cfg->wait_op_ms)
        else PARSE_U32("--warmup-cycles", cfg->warmup_cycles)
        else PARSE_U32("--drain-cycles", cfg->drain_cycles)
        else PARSE_U32("--max-missing", cfg->max_missing)
        else PARSE_U32("--bitrate", cfg->nominal_bitrate)
        else PARSE_U32("--data-bitrate", cfg->data_bitrate)
        else PARSE_U32("--spin-us", cfg->spin_us)
        else PARSE_U32("--sine-period-cycles", cfg->sine_period_cycles)
        else PARSE_FLOAT("--position", cfg->command.position)
        else PARSE_FLOAT("--velocity", cfg->command.velocity)
        else PARSE_FLOAT("--kp", cfg->command.kp)
        else PARSE_FLOAT("--kd", cfg->command.kd)
        else PARSE_FLOAT("--torque", cfg->command.torque)
        else PARSE_FLOAT("--sine-position-amplitude", cfg->sine_position_amplitude)
        else PARSE_FLOAT("--p-min", cfg->limits.position_min)
        else PARSE_FLOAT("--p-max", cfg->limits.position_max)
        else PARSE_FLOAT("--v-min", cfg->limits.velocity_min)
        else PARSE_FLOAT("--v-max", cfg->limits.velocity_max)
        else PARSE_FLOAT("--kp-min", cfg->limits.kp_min)
        else PARSE_FLOAT("--kp-max", cfg->limits.kp_max)
        else PARSE_FLOAT("--kd-min", cfg->limits.kd_min)
        else PARSE_FLOAT("--kd-max", cfg->limits.kd_max)
        else PARSE_FLOAT("--torque-min", cfg->limits.torque_min)
        else PARSE_FLOAT("--torque-max", cfg->limits.torque_max)
        else if (strcmp(argv[i], "--slave") == 0 && need_value(i, argc, argv[i])) {
            cfg->slave = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--channel") == 0 && need_value(i, argc, argv[i])) {
            cfg->channel = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--node") == 0 && need_value(i, argc, argv[i])) {
            cfg->node_id = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--pdo-n") == 0 && need_value(i, argc, argv[i])) {
            cfg->pdo_multiplier = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--nominal-sp-x10") == 0 && need_value(i, argc, argv[i])) {
            cfg->nominal_sp_x10 = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--data-sp-x10") == 0 && need_value(i, argc, argv[i])) {
            cfg->data_sp_x10 = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--rt-priority") == 0 && need_value(i, argc, argv[i])) {
            cfg->rt_priority = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--cpu") == 0 && need_value(i, argc, argv[i])) {
            cfg->cpu = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--extended-id") == 0) {
            cfg->extended_id = 1U;
        } else if (strcmp(argv[i], "--no-brs") == 0) {
            cfg->brs = 0U;
        } else if (strcmp(argv[i], "--enable-motor") == 0) {
            cfg->enable_motor = 1U;
        } else if (strcmp(argv[i], "--zero-position") == 0) {
            cfg->zero_position = 1U;
        } else if (strcmp(argv[i], "--keep-enabled") == 0) {
            cfg->disable_on_exit = 0U;
        } else if (strcmp(argv[i], "--no-mlock") == 0) {
            cfg->lock_memory = 0U;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 1;
        } else {
            fprintf(stderr, "unknown option: %s\n", argv[i]);
            return -1;
        }
#undef PARSE_U32
#undef PARSE_FLOAT
    }

    if (cfg->slave < 0 || cfg->slave >= (int)ECAN_IGH_MAX_SLAVES ||
        cfg->channel >= IGH_MCAN_CHANNEL_COUNT || cfg->node_id == 0U ||
        cfg->cycles == 0U || cfg->cycle_us == 0U ||
        cfg->nominal_bitrate == 0U || cfg->data_bitrate == 0U ||
        !ecan_igh_pdo_size_encoding_is_valid(cfg->pdo_multiplier) ||
        cfg->nominal_sp_x10 < 500U || cfg->nominal_sp_x10 > 950U ||
        cfg->data_sp_x10 < 500U || cfg->data_sp_x10 > 950U ||
        cfg->rt_priority < 0 || cfg->rt_priority > 99 ||
        cfg->sine_period_cycles == 0U || !mit_canfd_limits_valid(&cfg->limits)) {
        fprintf(stderr, "invalid MIT CAN FD demo configuration\n");
        return -1;
    }
    if ((!cfg->extended_id && (cfg->command_id > 0x7FFU || cfg->feedback_id > 0x7FFU)) ||
        (cfg->extended_id && (cfg->command_id > 0x1FFFFFFFU ||
                             cfg->feedback_id > 0x1FFFFFFFU))) {
        fprintf(stderr, "CAN ID is outside the selected standard/extended range\n");
        return -1;
    }
    return 0;
}

static int send_payload(igh_mcan_port_t *mcan, const demo_config_t *cfg,
                        const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE])
{
    return igh_mcan_port_send_canfd(mcan, cfg->slave, cfg->channel,
                                    cfg->command_id, cfg->extended_id, cfg->brs,
                                    payload, MIT_CANFD_PAYLOAD_SIZE);
}

static int send_control_cycle(igh_mcan_port_t *mcan, const demo_config_t *cfg,
                              mit_canfd_control_t control)
{
    uint8_t payload[MIT_CANFD_PAYLOAD_SIZE];
    int rc;
    int end_rc;

    mit_canfd_pack_control(control, payload);
    rc = igh_mcan_port_cycle_begin_checked(mcan);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = send_payload(mcan, cfg, payload);
    end_rc = igh_mcan_port_cycle_end_checked(mcan);
    return rc != IGH_ECAN_OK ? rc : end_rc;
}

static int collect_feedback(const demo_config_t *cfg, demo_stats_t *stats,
                            const igh_mcan_rx_frame_t *frames, size_t count)
{
    for (size_t i = 0U; i < count; i++) {
        const igh_mcan_frame_t *frame = &frames[i].frame;
        mit_canfd_feedback_t feedback;
        uint64_t outstanding;
        uint32_t latency;

        if (frames[i].slave_pos != cfg->slave || frame->channel != cfg->channel ||
            frame->can_id != cfg->feedback_id || frame->is_extended != cfg->extended_id) {
            stats->unrelated_frames++;
            continue;
        }
        if (!frame->is_canfd ||
            mit_canfd_unpack_feedback(&cfg->limits, frame->data, frame->data_len,
                                      &feedback) != 0) {
            stats->malformed_feedback++;
            continue;
        }
        if (feedback.node_id != cfg->node_id) {
            stats->node_mismatch++;
            continue;
        }
        stats->last_feedback = feedback;
        if (stats->feedback_received >= stats->commands_sent) {
            stats->extra_feedback++;
            continue;
        }
        outstanding = stats->commands_sent > stats->feedback_received ?
                      stats->commands_sent - stats->feedback_received : 0U;
        latency = outstanding > UINT32_MAX ? UINT32_MAX : (uint32_t)outstanding;
        stats->latency_hist[latency <= 8U ? latency : 9U]++;
        if (stats->latency_samples == 0U || latency < stats->latency_min_cycles) {
            stats->latency_min_cycles = latency;
        }
        if (latency > stats->latency_max_cycles) {
            stats->latency_max_cycles = latency;
        }
        stats->latency_sum_cycles += latency;
        stats->latency_samples++;
        stats->feedback_received++;
    }
    return 0;
}

int main(int argc, char **argv)
{
    demo_config_t cfg;
    demo_stats_t stats;
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    igh_rt_periodic_clock_t clock;
    igh_rt_period_stats_t period_stats;
    uint8_t channel_mask;
    int rc;
    int final_rc = IGH_ECAN_EXIT_FAIL;

    default_config(&cfg);
    memset(&stats, 0, sizeof(stats));
    rc = parse_args(argc, argv, &cfg);
    if (rc != 0) {
        return rc > 0 ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_ARG_ERROR;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    if (igh_rt_apply_runtime_tuning_checked(cfg.lock_memory,
                                            cfg.rt_priority,
                                            cfg.cpu) != 0) {
        fprintf(stderr, "requested real-time tuning could not be applied\n");
        return IGH_ECAN_EXIT_FAIL;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    channel_mask = IGH_ECAN_BIT_U8(cfg.channel);
    if (ecan_igh_config_apply_realtime_profile(
            &igh_config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK ||
        ecan_igh_config_set_slave_count(&igh_config, (size_t)cfg.slave + 1U) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&igh_config, cfg.cycle_us) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(&igh_config, cfg.pdo_multiplier) != IGH_ECAN_OK ||
        igh_mcan_port_set_channel_mask(&mcan_config, (size_t)cfg.slave,
                                       channel_mask, channel_mask) != IGH_ECAN_OK ||
        igh_mcan_port_set_bitrate(&mcan_config, (size_t)cfg.slave,
                                  cfg.nominal_bitrate, cfg.data_bitrate) != IGH_ECAN_OK ||
        igh_mcan_port_set_sample_point_range(&mcan_config, (size_t)cfg.slave,
                                             cfg.nominal_sp_x10,
                                             cfg.nominal_sp_x10,
                                             cfg.data_sp_x10,
                                             cfg.data_sp_x10) != IGH_ECAN_OK ||
        igh_mcan_port_apply_config(&igh_config, &mcan_config) != IGH_ECAN_OK) {
        fprintf(stderr, "failed to compose IgH/MCAN configuration\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    printf("MIT CAN FD: slave=%d channel=%u node=%u command=0x%08" PRIX32
           " feedback=0x%08" PRIX32 " cycle=%" PRIu32 "us cycles=%" PRIu32
           " CAN=%" PRIu32 "/%" PRIu32 " SP=%u/%u BRS=%u PDO=%uB mode=%s\n",
           cfg.slave, cfg.channel, cfg.node_id, cfg.command_id, cfg.feedback_id,
           cfg.cycle_us, cfg.cycles, cfg.nominal_bitrate, cfg.data_bitrate,
           cfg.nominal_sp_x10, cfg.data_sp_x10, cfg.brs,
           (unsigned)ecan_igh_pdo_decode_bytes(cfg.pdo_multiplier),
           ecan_igh_realtime_profile_name(ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE));
    printf("MIT limits: p=[%.3f,%.3f] v=[%.3f,%.3f] kp=[%.3f,%.3f]"
           " kd=[%.3f,%.3f] torque=[%.3f,%.3f]\n",
           cfg.limits.position_min, cfg.limits.position_max,
           cfg.limits.velocity_min, cfg.limits.velocity_max,
           cfg.limits.kp_min, cfg.limits.kp_max, cfg.limits.kd_min,
           cfg.limits.kd_max, cfg.limits.torque_min, cfg.limits.torque_max);

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "ecan_igh_init failed: %d\n", rc);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = igh_mcan_port_init(&mcan, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "igh_mcan_port_init failed: %d\n", rc);
        goto close_igh;
    }
    rc = ecan_igh_wait_op_timeout(&igh, cfg.wait_op_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        goto close_igh;
    }
    for (uint32_t i = 0U; i < cfg.warmup_cycles; i++) {
        igh_mcan_rx_frame_t rx[MIT_DEMO_RX_CAPACITY];
        igh_mcan_port_warmup_result_t warmup;

        memset(&warmup, 0, sizeof(warmup));
        rc = igh_mcan_port_warmup_cycle_checked(&mcan, rx,
                                                MIT_DEMO_RX_CAPACITY, &warmup);
        if (rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) {
            fprintf(stderr, "warmup failed at cycle %" PRIu32 ": %d\n", i, rc);
            goto close_igh;
        }
    }
    if (cfg.zero_position) {
        rc = send_control_cycle(&mcan, &cfg, MIT_CANFD_CONTROL_ZERO_POSITION);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "zero-position control failed: %d\n", rc);
            goto close_igh;
        }
    }
    if (cfg.enable_motor) {
        rc = send_control_cycle(&mcan, &cfg, MIT_CANFD_CONTROL_ENABLE);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "enable control failed: %d\n", rc);
            goto close_igh;
        }
    }

    igh_rt_periodic_clock_start(&clock, cfg.cycle_us, cfg.spin_us, 0U);
    igh_rt_period_stats_reset(&period_stats);
    for (uint32_t cycle = 0U; cycle < cfg.cycles && !stop_requested; cycle++) {
        igh_mcan_rx_frame_t rx[MIT_DEMO_RX_CAPACITY];
        size_t rx_count = 0U;
        mit_canfd_command_t command = cfg.command;
        uint8_t payload[MIT_CANFD_PAYLOAD_SIZE];
        uint64_t now_ns;
        int end_rc;

        now_ns = igh_rt_periodic_clock_wait(&clock);
        igh_rt_period_stats_sample(&period_stats, now_ns, clock.cycle_ns);
        rc = igh_mcan_port_cycle_begin_checked(&mcan);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle begin failed at %" PRIu32 ": %d\n", cycle, rc);
            goto disable_motor;
        }
        rc = igh_mcan_port_receive(&mcan, rx, MIT_DEMO_RX_CAPACITY, &rx_count);
        if (rc == IGH_ECAN_NO_SPACE) {
            stats.no_space++;
        } else if (rc != IGH_ECAN_OK) {
            (void)igh_mcan_port_cycle_abort_checked(&mcan);
            fprintf(stderr, "receive failed at %" PRIu32 ": %d\n", cycle, rc);
            goto disable_motor;
        }
        collect_feedback(&cfg, &stats, rx, rx_count);
        if (cfg.sine_position_amplitude != 0.0F) {
            command.position += cfg.sine_position_amplitude *
                (float)sin(2.0 * MIT_DEMO_PI * (double)cycle /
                           (double)cfg.sine_period_cycles);
        }
        if (mit_canfd_pack_command(&cfg.limits, &command, payload) != 0) {
            (void)igh_mcan_port_cycle_abort_checked(&mcan);
            fprintf(stderr, "failed to encode command\n");
            goto disable_motor;
        }
        rc = send_payload(&mcan, &cfg, payload);
        if (rc == IGH_ECAN_OK) {
            stats.commands_sent++;
        } else {
            stats.send_errors++;
        }
        end_rc = igh_mcan_port_cycle_end_checked(&mcan);
        if ((rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) || end_rc != IGH_ECAN_OK) {
            fprintf(stderr, "send/end failed at %" PRIu32 ": send=%d end=%d\n",
                    cycle, rc, end_rc);
            goto disable_motor;
        }
    }

    for (uint32_t i = 0U; i < cfg.drain_cycles; i++) {
        igh_mcan_rx_frame_t rx[MIT_DEMO_RX_CAPACITY];
        size_t rx_count = 0U;
        int end_rc;

        (void)igh_rt_periodic_clock_wait(&clock);
        rc = igh_mcan_port_cycle_begin_checked(&mcan);
        if (rc != IGH_ECAN_OK) {
            goto disable_motor;
        }
        rc = igh_mcan_port_receive(&mcan, rx, MIT_DEMO_RX_CAPACITY, &rx_count);
        if (rc == IGH_ECAN_OK || rc == IGH_ECAN_NO_SPACE) {
            collect_feedback(&cfg, &stats, rx, rx_count);
        }
        end_rc = igh_mcan_port_cycle_end_checked(&mcan);
        if ((rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) || end_rc != IGH_ECAN_OK) {
            goto disable_motor;
        }
    }
    final_rc = IGH_ECAN_EXIT_OK;

disable_motor:
    if (cfg.enable_motor && cfg.disable_on_exit) {
        rc = send_control_cycle(&mcan, &cfg, MIT_CANFD_CONTROL_DISABLE);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "[warn] disable control failed: %d\n", rc);
            final_rc = IGH_ECAN_EXIT_FAIL;
        }
    }
    igh_rt_print_period_stats("mit_master_period", &period_stats,
                              (uint64_t)cfg.cycle_us * 1000ULL);
    printf("mit_result commands=%" PRIu64 " feedback=%" PRIu64
           " missing=%" PRIu64 " unrelated=%" PRIu64
           " malformed=%" PRIu64 " node_mismatch=%" PRIu64
           " extra_feedback=%" PRIu64
           " no_space=%" PRIu64 " send_errors=%" PRIu64
           " latency_cycles_min=%" PRIu32 " latency_cycles_max=%" PRIu32
           " latency_cycles_avg=%.3f"
           " latency_hist=0:%" PRIu64 ",1:%" PRIu64 ",2:%" PRIu64
           ",3:%" PRIu64 ",4:%" PRIu64 ",5:%" PRIu64
           ",6:%" PRIu64 ",7:%" PRIu64 ",8:%" PRIu64
           ",>8:%" PRIu64 " resyncs=%" PRIu32 "\n",
           stats.commands_sent, stats.feedback_received,
           stats.commands_sent > stats.feedback_received ?
               stats.commands_sent - stats.feedback_received : 0U,
           stats.unrelated_frames, stats.malformed_feedback, stats.node_mismatch,
           stats.extra_feedback,
           stats.no_space, stats.send_errors, stats.latency_min_cycles,
           stats.latency_max_cycles,
           stats.latency_samples == 0U ? 0.0 :
               (double)stats.latency_sum_cycles / (double)stats.latency_samples,
           stats.latency_hist[0], stats.latency_hist[1],
           stats.latency_hist[2], stats.latency_hist[3],
           stats.latency_hist[4], stats.latency_hist[5],
           stats.latency_hist[6], stats.latency_hist[7],
           stats.latency_hist[8], stats.latency_hist[9],
           clock.resyncs);
    if (stats.feedback_received != 0U) {
        printf("mit_last_feedback node=%u position=%.5f velocity=%.5f"
               " torque=%.5f status=0x%02X temperature=%uC\n",
               stats.last_feedback.node_id, stats.last_feedback.position,
               stats.last_feedback.velocity, stats.last_feedback.torque,
               stats.last_feedback.status, stats.last_feedback.temperature_c);
    }
    if (stats.commands_sent < stats.feedback_received ||
        stats.commands_sent - stats.feedback_received > cfg.max_missing ||
        stats.malformed_feedback != 0U || stats.node_mismatch != 0U ||
        stats.extra_feedback > (uint64_t)(cfg.enable_motor + cfg.zero_position) ||
        stats.send_errors != 0U ||
        stats.no_space != 0U) {
        final_rc = IGH_ECAN_EXIT_FAIL;
    }

close_igh:
    if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
        ecan_igh_print_runtime_diagnostics(&igh, 16U);
    }
    ecan_igh_close(&igh);
    return final_rc;
}
