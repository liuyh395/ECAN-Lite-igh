/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dm_robot.h"
#include "igh_rt_support.h"
#include "igh_example_args.h"

#define DM_TEST_CYCLE_US 1000U
#define DM_TEST_DEFAULT_LOG_EVERY 1000U
#define DM_TEST_WARMUP_CYCLES 32U

static volatile sig_atomic_t stop_requested;

typedef struct {
    uint64_t tx_attempted;
    uint64_t tx_sent;
    uint64_t tx_errors;
    uint64_t raw_rx_frames;
    uint64_t raw_rx_bytes;
    uint64_t format_rejected;
    uint64_t can_id_zero;
    uint64_t can_id_unmatched;
    uint64_t motor_id_mismatch;
    uint64_t decode_errors;
    uint64_t accepted_feedback;
    uint64_t receive_no_space;
    uint64_t receive_errors;
    uint64_t cycle_begin_errors;
    uint64_t cycle_end_errors;
    uint8_t can_id_zero_warning_printed;
} dm_test_stats_t;

static void on_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void print_frame(const char *direction, const igh_mcan_frame_t *frame)
{
    printf("%s CAN%u ID=0x%0*" PRIX32 " %s DLC=%u DATA=",
           direction, frame->channel,
           frame->is_extended ? 8 : 3, frame->can_id,
           frame->is_extended ? "EXT" : "STD", frame->data_len);
    for (size_t i = 0; i < frame->data_len; ++i)
        printf("%02X%s", frame->data[i], i + 1U == frame->data_len ? "" : " ");
    putchar('\n');
}

static int find_motor(const dm_robot_t *robot, const igh_mcan_frame_t *frame)
{
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        const dm_robot_motor_config_t *c = &robot->motors[i].config;
        if (frame->channel == c->channel && frame->can_id == c->feedback_id &&
            frame->is_extended == c->extended_id) return (int)i;
    }
    return -1;
}

static int find_motor_by_payload_id(const dm_robot_t *robot,
                                    const igh_mcan_rx_frame_t *rx)
{
    uint8_t motor_id;
    if (rx->frame.can_id != 0U || rx->frame.data_len != DM_MOTOR_PAYLOAD_SIZE)
        return -1;
    motor_id = (uint8_t)(rx->frame.data[0] & 0x0FU);
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        const dm_robot_motor_config_t *c = &robot->motors[i].config;
        if (rx->frame.channel == c->channel && c->motor_id == motor_id)
            return (int)i;
    }
    return -1;
}

static void print_feedback(dm_robot_t *robot, const igh_mcan_rx_frame_t *rx,
                           dm_test_stats_t *stats)
{
    dm_motor_feedback_t feedback;
    int index;
    if (rx->slave_pos != (int)robot->slave_pos || rx->frame.is_canfd ||
        rx->frame.data_len != DM_MOTOR_PAYLOAD_SIZE) {
        stats->format_rejected++;
        return;
    }
    if (rx->frame.can_id == 0U) stats->can_id_zero++;
    index = find_motor(robot, &rx->frame);
    if (index < 0 && rx->frame.can_id == 0U) {
        index = find_motor_by_payload_id(robot, rx);
        if (index >= 0 && !stats->can_id_zero_warning_printed) {
            printf("  warning: RX CAN ID is 0x000; matching by feedback payload ID\n");
            stats->can_id_zero_warning_printed = 1U;
        }
    }
    if (index < 0) {
        stats->can_id_unmatched++;
        return;
    }
    if (dm_motor_unpack_feedback(&robot->motors[index].config.limits,
                                 rx->frame.data, &feedback) != 0) {
        stats->decode_errors++;
        printf("  feedback decode failed for motor[%d]\n", index);
        return;
    }
    if (feedback.motor_id != robot->motors[index].config.motor_id) {
        stats->motor_id_mismatch++;
        printf("  feedback motor ID mismatch: expected=%u got=%u\n",
               robot->motors[index].config.motor_id, feedback.motor_id);
        return;
    }
    robot->motors[index].feedback = feedback;
    robot->motors[index].feedback_received++;
    robot->motors[index].has_feedback = 1U;
    // printf("  parsed motor[%d] CAN%u motor_id=%u error=0x%X "
    //        "pos=%.5f rad vel=%.5f rad/s torque=%.5f Nm MOS=%u C rotor=%u C\n",
    //        index, rx->frame.channel, feedback.motor_id, feedback.error_state,
    //        feedback.position, feedback.velocity, feedback.torque,
    //        feedback.mos_temperature, feedback.rotor_temperature);
}

static void usage(const char *program)
{
    printf("usage: %s [--cycles N] [--slave N] [--bitrate N] [--log-every N]\n", program);
    printf("Runs a 1ms four-channel Classic CAN test; default bitrate 1Mbps.\n");
    printf("Log-every controls how often all 12 transmitted frames are printed; "
           "received feedback is printed immediately.\n");
}

static int parse_args(int argc, char **argv, uint32_t *cycles, size_t *slave,
                      uint32_t *bitrate, uint32_t *log_every)
{
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc)
            *cycles = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc)
            *slave = igh_example_size(argv[++i]);
        else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc)
            *bitrate = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--log-every") == 0 && i + 1 < argc)
            *log_every = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 1;
        } else {
            usage(argv[0]);
            return -1;
        }
    }
    if (*cycles == 0U || *bitrate == 0U || *log_every == 0U ||
        *slave >= ECAN_IGH_MAX_SLAVES) return -1;
    return 0;
}

int main(int argc, char **argv)
{
    ecan_igh_config_t igh_config;
    igh_mcan_port_config_t mcan_config;
    ecan_igh_context_t igh;
    igh_mcan_port_t port;
    dm_robot_motor_config_t configs[DM_ROBOT_MOTOR_COUNT];
    dm_robot_t robot;
    dm_motor_command_t commands[DM_ROBOT_MOTOR_COUNT];
    igh_rt_periodic_clock_t clock;
    uint32_t cycles = UINT32_MAX;
    uint32_t bitrate = 1000000U;
    uint32_t log_every = DM_TEST_DEFAULT_LOG_EVERY;
    dm_test_stats_t stats;
    size_t slave = 0U;
    int rc;
    int final_rc = IGH_ECAN_EXIT_OK;

    memset(&stats, 0, sizeof(stats));

    rc = parse_args(argc, argv, &cycles, &slave, &bitrate, &log_every);
    if (rc != 0) return rc > 0 ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_ARG_ERROR;
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    dm_robot_default_config(configs);
    if (dm_robot_init(&robot, slave, configs) != 0) {
        fprintf(stderr, "invalid DM robot configuration\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    /* Fixed, deliberately non-zero test command. This is not a safe-to-run-on-motor profile. */
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        commands[i].position = 1.0F;
        commands[i].velocity = 2.0F;
        commands[i].kp = 10.0F;
        commands[i].kd = 0.5F;
        commands[i].torque = 1.0F;
    }
    if (dm_robot_submit_all(&robot, commands) != 0) {
        fprintf(stderr, "failed to initialize test commands\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (ecan_igh_config_apply_realtime_profile(
            &igh_config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK ||
        ecan_igh_config_set_slave_count(&igh_config, slave + 1U) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&igh_config, DM_TEST_CYCLE_US) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(&igh_config, 3U) != IGH_ECAN_OK ||
        dm_robot_apply_classic_can_config(&igh_config, &mcan_config,
                                          slave, bitrate) != 0) {
        fprintf(stderr, "failed to configure four Classic CAN channels\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) return IGH_ECAN_EXIT_FAIL;
    rc = igh_mcan_port_init(&port, &igh, &mcan_config);

    if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }
    rc = ecan_igh_wait_op_timeout(&igh, 10000U);
    if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }

    for (uint32_t i = 0U; i < DM_TEST_WARMUP_CYCLES; ++i) {
        igh_mcan_rx_frame_t warmup_frames[DM_ROBOT_RX_CAPACITY];
        igh_mcan_port_warmup_result_t warmup;
        memset(&warmup, 0, sizeof(warmup));
        rc = igh_mcan_port_warmup_cycle_checked(&port, warmup_frames,
                                                DM_ROBOT_RX_CAPACITY, &warmup);
        if (rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) {
            fprintf(stderr, "warmup failed at cycle %" PRIu32 ": %d\n", i, rc);
            final_rc = IGH_ECAN_EXIT_FAIL;
            goto close_igh;
        }
    }

    printf("DM USB2CAN test: slave=%zu channels=CAN0..CAN3 bitrate=%" PRIu32
           " cycle=%u us profile=%s warmup=%u; TX log every %" PRIu32 " cycles\n",
           slave, bitrate, DM_TEST_CYCLE_US,
           ecan_igh_realtime_profile_name(ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE),
           DM_TEST_WARMUP_CYCLES, log_every);
    printf("fixed MIT command: position=1.0 velocity=2.0 kp=10.0 kd=0.5 torque=1.0\n");
    printf("default control IDs: 0x001..0x003 per channel; feedback IDs: 0x101..0x10C\n");
    printf("Connect USB2CAN to one selected CAN port; Ctrl-C stops the test.\n");

    if (igh_rt_periodic_clock_start_checked(&clock, DM_TEST_CYCLE_US, 0U, 0U) != 0) {
        fprintf(stderr, "failed to start 1ms periodic clock\n");
        final_rc = IGH_ECAN_EXIT_FAIL;
        goto close_igh;
    }
    for (uint32_t cycle = 0U; cycle < cycles && !stop_requested; ++cycle) {
        igh_mcan_rx_frame_t rx_frames[DM_ROBOT_RX_CAPACITY];
        size_t rx_count = 0U;
        (void) igh_rt_periodic_clock_wait(&clock);
        rc = igh_mcan_port_cycle_begin_checked(&port);
        if (rc != IGH_ECAN_OK) {
            stats.cycle_begin_errors++;
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }

        rc = igh_mcan_port_receive(&port, rx_frames, DM_ROBOT_RX_CAPACITY, &rx_count);
        if (rc == IGH_ECAN_NO_SPACE) {
            stats.receive_no_space++;
        } else if (rc != IGH_ECAN_OK) {
            stats.receive_errors++;
            (void)igh_mcan_port_cycle_abort_checked(&port);
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }
        for (size_t r = 0; r < rx_count; ++r) {
            // print_frame("RX", &rx_frames[r].frame);
            print_feedback(&robot, &rx_frames[r], &stats);
        }

        for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
            const dm_robot_motor_config_t *c = &robot.motors[i].config;
            uint8_t payload[DM_MOTOR_PAYLOAD_SIZE];
            rc = dm_motor_pack_command(&c->limits, &robot.motors[i].command, payload);
            if (rc != 0) {
                (void)igh_mcan_port_cycle_abort_checked(&port);
                final_rc = IGH_ECAN_EXIT_FAIL;
                goto finish_test;
            }
            if (cycle % log_every == 0U) {
                igh_mcan_frame_t tx_frame;
                memset(&tx_frame, 0, sizeof(tx_frame));
                tx_frame.channel = c->channel;
                tx_frame.can_id = c->command_id;
                tx_frame.is_extended = c->extended_id;
                tx_frame.data_len = DM_MOTOR_PAYLOAD_SIZE;
                memcpy(tx_frame.data, payload, sizeof(payload));
                // print_frame("TX", &tx_frame);
            }
            rc = igh_mcan_port_send_can(&port, (int)slave, c->channel,
                                        c->command_id, c->extended_id,
                                        payload, sizeof(payload));
            stats.tx_attempted++;
            if (rc != IGH_ECAN_OK) {
                stats.tx_errors++;
                (void)igh_mcan_port_cycle_abort_checked(&port);
                final_rc = IGH_ECAN_EXIT_FAIL;
                goto finish_test;
            }
            stats.tx_sent++;
            robot.motors[i].commands_sent++;
        }
        rc = igh_mcan_port_cycle_end_checked(&port);
        if (rc != IGH_ECAN_OK) {
            stats.cycle_end_errors++;
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }
        robot.cycles_completed++;
        if (cycle % log_every == 0U) fflush(stdout);
    }

finish_test:
    printf("stopped: cycles=%" PRIu64 "\n", robot.cycles_completed);
    printf("tx_stats attempted=%" PRIu64 " sent=%" PRIu64
           " errors=%" PRIu64 "\n",
           stats.tx_attempted, stats.tx_sent, stats.tx_errors);
    printf("rx_stats raw_frames=%" PRIu64 " raw_bytes=%" PRIu64
           " accepted=%" PRIu64 " format_rejected=%" PRIu64
           " can_id_zero=%" PRIu64 " can_id_unmatched=%" PRIu64
           " motor_id_mismatch=%" PRIu64 " decode_errors=%" PRIu64
           " no_space=%" PRIu64 " receive_errors=%" PRIu64
           " cycle_begin_errors=%" PRIu64 " cycle_end_errors=%" PRIu64 "\n",
           stats.raw_rx_frames, stats.raw_rx_bytes, stats.accepted_feedback,
           stats.format_rejected, stats.can_id_zero, stats.can_id_unmatched,
           stats.motor_id_mismatch, stats.decode_errors,
           stats.receive_no_space, stats.receive_errors,
           stats.cycle_begin_errors, stats.cycle_end_errors);
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i)
        printf("motor[%zu] CAN%u feedback_frames=%" PRIu64 "\n", i,
               robot.motors[i].config.channel, robot.motors[i].feedback_received);
close_igh:
    ecan_igh_close(&igh);
    return final_rc;
}
