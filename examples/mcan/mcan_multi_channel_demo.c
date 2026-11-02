#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "igh_mcan_port.h"
#include "igh_rt_support.h"

#define MCAN_MULTI_RX_CAPACITY 64U
#define MCAN_MULTI_CYCLES      1000U
#define MCAN_MULTI_CYCLE_US    5000U

static void fill_payload(uint8_t *data, size_t size, unsigned counter)
{
    for (size_t i = 0U; i < size; i++) {
        data[i] = (uint8_t)(counter + i);
    }
}

static void print_rx(const igh_mcan_rx_frame_t *frames, size_t count)
{
    for (size_t i = 0U; i < count; i++) {
        const igh_mcan_frame_t *frame = &frames[i].frame;
        printf("rx slave=%d ch=%u id=0x%08" PRIX32 " canfd=%u len=%u\n",
               frames[i].slave_pos,
               frame->channel,
               frame->can_id,
               frame->is_canfd,
               frame->data_len);
    }
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        printf("usage: %s [--help]\nRuns a mixed CAN/CAN FD demo; sends CAN requests.\n", argv[0]);
        return argc == 2 && (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")) ? 0 : 2;
    }
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    igh_rt_periodic_clock_t clock;
    igh_rt_period_stats_t period_stats;
    int rc;

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (ecan_igh_config_set_slave_count(&igh_config, 1U) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&igh_config, MCAN_MULTI_CYCLE_US) != IGH_ECAN_OK ||
        igh_mcan_port_set_channel_mask(&mcan_config,
                                       0U,
                                       IGH_ECAN_BIT_U8(IGH_ECAN_MCAN_CH0) |
                                           IGH_ECAN_BIT_U8(IGH_ECAN_MCAN_CH1),
                                       IGH_ECAN_BIT_U8(IGH_ECAN_MCAN_CH1)) != IGH_ECAN_OK ||
        igh_mcan_port_set_bitrate(&mcan_config,
                                  0U,
                                  IGH_ECAN_DEFAULT_CAN_BITRATE,
                                  IGH_ECAN_DEFAULT_CANFD_BITRATE) != IGH_ECAN_OK ||
        igh_mcan_port_apply_config(&igh_config, &mcan_config) != IGH_ECAN_OK) {
        fprintf(stderr, "invalid multi-channel MCAN demo config\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "ecan_igh_init failed: %d\n", rc);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = igh_mcan_port_init(&mcan, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "igh_mcan_port_init failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = ecan_igh_wait_op_timeout(&igh, 10000U);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
            ecan_igh_print_runtime_diagnostics(&igh, 16U);
        }
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    igh_rt_periodic_clock_start(&clock, igh_config.cycle_us, 0U, 0U);
    igh_rt_period_stats_reset(&period_stats);
    for (unsigned cycle = 0U; cycle < MCAN_MULTI_CYCLES; cycle++) {
        uint8_t can_data[8];
        uint8_t canfd_data[24];
        igh_mcan_rx_frame_t rx[MCAN_MULTI_RX_CAPACITY];
        size_t rx_count = 0U;
        int end_rc;
        uint64_t now_ns;

        fill_payload(can_data, sizeof(can_data), cycle);
        fill_payload(canfd_data, sizeof(canfd_data), cycle + 0x40U);

        now_ns = igh_rt_periodic_clock_wait(&clock);
        igh_rt_period_stats_sample(&period_stats, now_ns, clock.cycle_ns);
        rc = igh_mcan_port_cycle_begin_checked(&mcan);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle %u begin failed: %d\n", cycle, rc);
            break;
        }
        rc = igh_mcan_port_receive(&mcan, rx, MCAN_MULTI_RX_CAPACITY, &rx_count);
        if (rc == IGH_ECAN_OK) {
            int send_rc = igh_mcan_port_send_can(&mcan,
                                                 0,
                                                 IGH_ECAN_MCAN_CH0,
                                                 0x123U,
                                                 0,
                                                 can_data,
                                                 sizeof(can_data));
            if (rc == IGH_ECAN_OK && send_rc != IGH_ECAN_OK) {
                rc = send_rc;
            }
            if (rc == IGH_ECAN_OK && (cycle % 10U) == 0U) {
                send_rc = igh_mcan_port_send_canfd(&mcan,
                                                   0,
                                                   IGH_ECAN_MCAN_CH1,
                                                   0x456U,
                                                   0,
                                                   1,
                                                   canfd_data,
                                                   sizeof(canfd_data));
                if (rc == IGH_ECAN_OK && send_rc != IGH_ECAN_OK) {
                    rc = send_rc;
                }
            }
        }
        if (rc == IGH_ECAN_OK) {
            end_rc = igh_mcan_port_cycle_end_checked(&mcan);
        } else {
            end_rc = igh_mcan_port_cycle_abort_checked(&mcan);
        }
        if (rc == IGH_ECAN_OK && end_rc != IGH_ECAN_OK) {
            rc = end_rc;
        }

        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle %u failed: %d\n", cycle, rc);
            break;
        }
        if ((cycle % 100U) == 0U) {
            printf("cycle=%u rx_count=%zu\n", cycle, rx_count);
            print_rx(rx, rx_count);
        }
    }

    igh_rt_print_period_stats("multi_channel_period", &period_stats, clock.cycle_ns);
    printf("multi_channel_period_resyncs=%" PRIu32 "\n", clock.resyncs);
    if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
        ecan_igh_print_runtime_diagnostics(&igh, 16U);
    }
    ecan_igh_close(&igh);
    return rc == IGH_ECAN_OK ?
           IGH_ECAN_EXIT_OK :
           IGH_ECAN_EXIT_FAIL;
}
