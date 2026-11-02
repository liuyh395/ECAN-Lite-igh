/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dm_motor.h"
#include "igh_rt_support.h"
#include "igh_example_args.h"
#include "igh_example_stop.h"

#define DM_DEMO_MOTOR_COUNT 3U
#define DM_DEMO_CYCLE_US    1000U
#define DM_DEMO_DEFAULT_CYCLES 2000U
#define DM_DEMO_RX_CAPACITY 64U

static volatile sig_atomic_t stop_requested;

static void on_signal(int signal_number)
{
    (void)signal_number;
    stop_requested = 1;
}

static void usage(const char *program)
{
    printf("usage: %s [--cycles N] [--slave N] [--bitrate N] [--enable]\n", program);
    printf("Classic CAN MIT demo: one EtherCAT slave, CAN0, three motors.\n");
    printf("Default command IDs: 1,2,3; feedback IDs: 0x101,0x102,0x103.\n");
}

static int parse_args(int argc, char **argv, uint32_t *cycles,
                      size_t *slave, uint32_t *bitrate, int *enable)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            *cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc) {
            *slave = igh_example_size(argv[++i]);
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            *bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--enable") == 0) {
            *enable = 1;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 1;
        } else {
            usage(argv[0]);
            return -1;
        }
    }
    if (*cycles == 0U || *bitrate == 0U || *slave >= ECAN_IGH_MAX_SLAVES) return -1;
    return 0;
}

static int send_control_all(dm_motor_manager_t *manager, igh_mcan_port_t *port,
                            dm_motor_control_t control)
{
    int rc;
    rc = igh_mcan_port_cycle_begin_checked(port);
    if (rc != IGH_ECAN_OK) return rc;
    for (size_t i = 0; i < manager->count; i++) {
        rc = dm_motor_send_control(&manager->motors[i], port,
                                   manager->slave_pos, control);
        if (rc != IGH_ECAN_OK) {
            (void)igh_mcan_port_cycle_abort_checked(port);
            return rc;
        }
    }
    return igh_mcan_port_cycle_end_checked(port);
}

int main(int argc, char **argv)
{
    ecan_igh_config_t igh_config;
    igh_mcan_port_config_t mcan_config;
    ecan_igh_context_t igh;
    igh_mcan_port_t port;
    dm_motor_config_t configs[DM_DEMO_MOTOR_COUNT];
    dm_motor_manager_t manager;
    igh_rt_periodic_clock_t clock;
    igh_rt_period_stats_t period_stats;
    uint32_t cycles = DM_DEMO_DEFAULT_CYCLES;
    uint32_t bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    size_t slave = 0U;
    int enable = 0;
    int rc;
    int final_rc = IGH_ECAN_EXIT_OK;

    rc = parse_args(argc, argv, &cycles, &slave, &bitrate, &enable);
    if (rc != 0) return rc > 0 ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_ARG_ERROR;
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    for (size_t i = 0; i < DM_DEMO_MOTOR_COUNT; i++) {
        dm_motor_default_config(&configs[i], 0U, (uint8_t)(i + 1U));
        /* Feedback IDs are deliberately separate and easy to replace. */
        configs[i].feedback_id = 0x101U + (uint32_t)i;
    }
    if (dm_motor_manager_init(&manager, slave, configs, DM_DEMO_MOTOR_COUNT) != 0) {
        fprintf(stderr, "invalid DM motor configuration\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (ecan_igh_config_set_slave_count(&igh_config, slave + 1U) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&igh_config, DM_DEMO_CYCLE_US) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(&igh_config, 3U) != IGH_ECAN_OK ||
        igh_mcan_port_set_channel_mask(&mcan_config, slave, 0x01U, 0x00U) != IGH_ECAN_OK ||
        igh_mcan_port_set_bitrate(&mcan_config, slave, bitrate, bitrate) != IGH_ECAN_OK ||
        igh_mcan_port_apply_config(&igh_config, &mcan_config) != IGH_ECAN_OK) {
        fprintf(stderr, "failed to compose Classic CAN configuration\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) return IGH_ECAN_EXIT_FAIL;
    rc = igh_mcan_port_init(&port, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }
    rc = ecan_igh_wait_op_timeout(&igh, 10000U);
    if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }

    printf("DM Classic CAN MIT: slave=%zu channel=0 motors=3 bitrate=%" PRIu32
           " cycle=%u us feedback_ids=0x101,0x102,0x103\n",
           slave, bitrate, DM_DEMO_CYCLE_US);

    if (enable) {
        rc = send_control_all(&manager, &port, DM_MOTOR_CONTROL_ENABLE);
        if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }
    }

    igh_rt_periodic_clock_start(&clock, DM_DEMO_CYCLE_US, 0U, 0U);
    igh_rt_period_stats_reset(&period_stats);
    for (uint32_t cycle = 0; cycle < cycles && !stop_requested; cycle++) {
        igh_mcan_rx_frame_t frames[DM_DEMO_RX_CAPACITY];
        size_t frame_count = 0U;
        uint64_t now_ns = igh_rt_periodic_clock_wait(&clock);
        igh_rt_period_stats_sample(&period_stats, now_ns, clock.cycle_ns);
        rc = igh_mcan_port_cycle_begin_checked(&port);
        if (rc != IGH_ECAN_OK) {
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }
        rc = igh_mcan_port_receive(&port, frames, DM_DEMO_RX_CAPACITY, &frame_count);
        if (rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) {
            (void)igh_mcan_port_cycle_abort_checked(&port);
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }
        (void)dm_motor_manager_receive(&manager, frames, frame_count);
        rc = dm_motor_manager_send(&manager, &port);
        if (rc == IGH_ECAN_OK) rc = igh_mcan_port_cycle_end_checked(&port);
        else (void)igh_mcan_port_cycle_abort_checked(&port);
        if (rc != IGH_ECAN_OK) {
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }
        if ((cycle % 500U) == 0U)
            printf("cycle=%" PRIu32 " rx=%zu m1_feedback=%" PRIu64 "\n",
                   cycle, frame_count, manager.motors[0].feedback_received);
    }

    if (enable) (void)send_control_all(&manager, &port, DM_MOTOR_CONTROL_DISABLE);
    igh_rt_print_period_stats("dm_motor_period", &period_stats,
                              (uint64_t)DM_DEMO_CYCLE_US * 1000ULL);
close_igh:
    ecan_igh_close(&igh);
    return final_rc;
}
