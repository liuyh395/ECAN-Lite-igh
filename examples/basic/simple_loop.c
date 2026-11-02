#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "igh_mcan_port.h"
#include "igh_rt_support.h"

#define SIMPLE_LOOP_CYCLES      1000U
#define SIMPLE_LOOP_RX_CAPACITY 32U

static void fill_counter_payload(uint8_t *data, unsigned counter)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        data[i] = (uint8_t)(counter + i);
    }
}

static void print_rx_frames(const igh_mcan_rx_frame_t *frames, size_t count)
{
    for (size_t i = 0U; i < count; i++) {
        const igh_mcan_frame_t *frame = &frames[i].frame;
        printf("rx slave=%d ch=%u id=0x%08" PRIX32 " dlc=%u len=%u\n",
               frames[i].slave_pos,
               frame->channel,
               frame->can_id,
               frame->dlc,
               frame->data_len);
    }
}

int main(int argc, char **argv)
{
    if (argc > 1) {
        printf("usage: %s [--help]\nRuns a one-channel Classic CAN counter demo; sends CAN requests.\n", argv[0]);
        return argc == 2 && (!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")) ? 0 : 2;
    }
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    igh_rt_periodic_clock_t clock;
    int rc;

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);

    rc = ecan_igh_config_apply_realtime_profile(
        &igh_config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE);
    if (rc == IGH_ECAN_OK)
        rc = ecan_igh_config_set_slave_count(&igh_config, 1U);
    if (rc == IGH_ECAN_OK) {
        rc = igh_mcan_port_set_channel_mask(&mcan_config, 0U, IGH_ECAN_BIT_U8(0U), 0U);
    }
    if (rc == IGH_ECAN_OK) {
        rc = igh_mcan_port_set_classic_channel(&mcan_config, 0U, 0U,
                                               IGH_ECAN_DEFAULT_CAN_BITRATE);
    }
    if (rc == IGH_ECAN_OK) {
        rc = igh_mcan_port_apply_config(&igh_config, &mcan_config);
    }
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "invalid MCAN demo config\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "init IgH failed: %d\n", rc);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = igh_mcan_port_init(&mcan, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "init MCAN port failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = ecan_igh_wait_op_timeout(&igh, 10000U);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    igh_rt_periodic_clock_start(&clock, igh_config.cycle_us, 30U, 0U);
    for (unsigned loop = 0U; loop < SIMPLE_LOOP_CYCLES; loop++) {
        uint8_t payload[8];
        igh_mcan_rx_frame_t rx[SIMPLE_LOOP_RX_CAPACITY];
        size_t rx_count = 0U;

        (void)igh_rt_periodic_clock_wait(&clock);
        fill_counter_payload(payload, loop);
        rc = igh_mcan_port_cycle_begin_checked(&mcan);
        if (rc == IGH_ECAN_OK) {
            rc = igh_mcan_port_send_can(&mcan, 0, 0U, 0x123U, 0,
                                        payload, sizeof(payload));
        }
        if (rc == IGH_ECAN_OK) {
            rc = igh_mcan_port_cycle_end_checked(&mcan);
        } else if (mcan.cycle_active != 0U) {
            (void)igh_mcan_port_cycle_abort_checked(&mcan);
        }
        if (rc == IGH_ECAN_OK) {
            rc = igh_mcan_port_receive(&mcan, rx, SIMPLE_LOOP_RX_CAPACITY, &rx_count);
        }
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle %u failed: %d\n", loop, rc);
            break;
        }
        if ((loop % 100U) == 0U) {
            printf("cycle=%u rx_count=%zu\n", loop, rx_count);
            print_rx_frames(rx, rx_count);
        }
    }

    ecan_igh_close(&igh);
    return rc == IGH_ECAN_OK ?
           IGH_ECAN_EXIT_OK :
           IGH_ECAN_EXIT_FAIL;
}
