#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cia402_demo_common.h"
#include "cia402_mcan_transport.h"
#include "igh_cia402_protocol.h"
#include "igh_adapter.h"
#include "igh_mcan_port.h"

#define POSITION_WATCH_CYCLE_US      10000U
#define POSITION_WATCH_DEFAULT_NODE  1U
#define POSITION_WATCH_DEFAULT_READS 20U
#define POSITION_WATCH_RX_CAPACITY   32U

typedef struct {
    int slave_pos;
    uint8_t channel;
    uint8_t node_id;
    igh_ecan_canfd_mode_t canfd;
    uint32_t bitrate;
    uint32_t data_bitrate;
    unsigned reads;
    uint32_t cycle_us;
    uint32_t wait_op_ms;
    uint32_t request_timeout_ms;
    int send_nmt_start;
} position_watch_config_t;

static const igh_cia402_demo_motor_slot_t k_motor_slots[] = {
    {IGH_ECAN_ENABLE, 0U, IGH_ECAN_MCAN_CH0, POSITION_WATCH_DEFAULT_NODE,
     IGH_CIA402_DEMO_DEFAULT_CANFD, IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
};

static void default_config(position_watch_config_t *config)
{
    const igh_cia402_demo_motor_slot_t *slot = &k_motor_slots[0];

    memset(config, 0, sizeof(*config));
    config->slave_pos = (int)slot->slave_pos;
    config->channel = (uint8_t)slot->mcan_channel;
    config->node_id = slot->node_id;
    config->canfd = slot->canfd;
    config->bitrate = slot->nominal_bitrate;
    config->data_bitrate = slot->data_bitrate;
    config->reads = POSITION_WATCH_DEFAULT_READS;
    config->cycle_us = POSITION_WATCH_CYCLE_US;
    config->wait_op_ms = 10000U;
    config->request_timeout_ms = 500U;
    config->send_nmt_start = 1;
}

static void print_usage(const char *app)
{
    printf("usage: %s [--slave N] [--channel 0..3] [--node N] [--bitrate N]\n", app);
    printf("          [--canfd] [--classic-can] [--data-bitrate N]\n");
    printf("          [--cycle-us N] [--reads N] [--timeout-ms N] [--wait-op-ms N]\n");
    printf("          [--no-nmt-start]\n");
}

static int parse_args(int argc, char **argv, position_watch_config_t *config)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc) {
            config->slave_pos = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            config->channel = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--node") == 0 && i + 1 < argc) {
            config->node_id = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            config->bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            config->data_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--canfd") == 0) {
            config->canfd = IGH_ECAN_CANFD_ENABLE;
        } else if (strcmp(argv[i], "--classic-can") == 0) {
            config->canfd = IGH_ECAN_CANFD_DISABLE;
        } else if (strcmp(argv[i], "--cycle-us") == 0 && i + 1 < argc) {
            config->cycle_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--reads") == 0 && i + 1 < argc) {
            config->reads = igh_example_uint(argv[++i]);
        } else if (strcmp(argv[i], "--timeout-ms") == 0 && i + 1 < argc) {
            config->request_timeout_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--wait-op-ms") == 0 && i + 1 < argc) {
            config->wait_op_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--no-nmt-start") == 0) {
            config->send_nmt_start = 0;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            exit(0);
        } else {
            return IGH_ECAN_INVALID_ARG;
        }
    }

    if (config->slave_pos < 0 ||
        config->channel >= IGH_MCAN_CHANNEL_COUNT ||
        config->node_id == 0U ||
        config->node_id > 0x7FU ||
        config->bitrate == 0U ||
        config->data_bitrate == 0U ||
        config->cycle_us == 0U ||
        config->reads == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    return IGH_ECAN_OK;
}

static int apply_config(ecan_igh_config_t *igh_config,
                        igh_mcan_port_config_t *mcan_config,
                        const position_watch_config_t *config)
{
    size_t slave_count;

    if ((igh_config == NULL) || (mcan_config == NULL) || (config == NULL) ||
        config->slave_pos < 0) {
        return IGH_ECAN_INVALID_ARG;
    }

    slave_count = (size_t)config->slave_pos + 1U;
    if (slave_count > ECAN_IGH_MAX_SLAVES) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ecan_igh_config_set_slave_count(igh_config, slave_count) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(igh_config, config->cycle_us) != IGH_ECAN_OK ||
        igh_mcan_port_set_channel_mask(mcan_config,
                                       (size_t)config->slave_pos,
                                       IGH_ECAN_BIT_U8(config->channel),
                                       (config->canfd == IGH_ECAN_CANFD_ENABLE) ?
                                           IGH_ECAN_BIT_U8(config->channel) :
                                           0U) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (config->canfd == IGH_ECAN_CANFD_ENABLE) {
        if (igh_mcan_port_set_canfd_channel(mcan_config,
                                            (size_t)config->slave_pos,
                                            config->channel,
                                            config->bitrate,
                                            config->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    } else if (igh_mcan_port_set_classic_channel(mcan_config,
                                                 (size_t)config->slave_pos,
                                                 config->channel,
                                                 config->bitrate) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    return igh_mcan_port_apply_config(igh_config, mcan_config);
}

static int send_nmt_start(igh_mcan_port_t *mcan, igh_cia402_context_t *cia)
{
    igh_mcan_rx_frame_t frames[POSITION_WATCH_RX_CAPACITY];
    size_t rx_count = 0U;
    int rc = IGH_ECAN_OK;

    igh_mcan_port_cycle_begin(mcan);
    rc = igh_mcan_port_receive(mcan, frames, POSITION_WATCH_RX_CAPACITY, &rx_count);
    if (rc == IGH_ECAN_OK || rc == IGH_ECAN_NO_SPACE) {
        for (size_t i = 0U; i < rx_count; i++) {
            igh_cia402_rx_frame_t rx;

            igh_cia402_mcan_rx_to_can(&frames[i], &rx);
            (void)igh_cia402_on_can_frame(cia, rx.slave_pos, &rx.frame);
        }
    }
    if (rc == IGH_ECAN_OK || rc == IGH_ECAN_NO_SPACE) {
        for (size_t i = 0U; i < cia->axis_count; i++) {
            igh_cia402_tx_frame_t tx;
            int send_rc;

            if (!cia->axes[i].enabled) {
                continue;
            }
            send_rc = igh_cia402_build_nmt_frame(&cia->axes[i],
                                                 IGH_CIA402_NMT_START_REMOTE_NODE,
                                                 &tx);
            if (send_rc == IGH_ECAN_OK) {
                send_rc = igh_cia402_mcan_send_frame(mcan, &tx);
            }
            if (rc == IGH_ECAN_OK && send_rc != IGH_ECAN_OK) {
                rc = send_rc;
            }
        }
    }
    igh_mcan_port_cycle_end(mcan);
    usleep(mcan->igh->cycle_us);
    return rc;
}

int main(int argc, char **argv)
{
    position_watch_config_t config;
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    igh_cia402_axis_t axis;
    igh_cia402_context_t cia;
    igh_cia402_mcan_transport_t transport;
    int rc;

    default_config(&config);
    if (parse_args(argc, argv, &config) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (apply_config(&igh_config, &mcan_config, &config) != IGH_ECAN_OK) {
        fprintf(stderr, "invalid position watch config\n");
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
    rc = ecan_igh_wait_op_timeout(&igh, config.wait_op_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    igh_cia402_axis_init(&axis,
                         config.slave_pos,
                         config.channel,
                         config.node_id,
                         0);
    igh_cia402_mcan_transport_init(&transport, &mcan);
    igh_cia402_context_init(&cia, &transport.transport, &axis, 1U);
    printf("[diag] position-watch slave=%d ch=%u node=0x%02X canfd=%d"
           " canopen_frame=classic-can nominal=%" PRIu32 " data=%" PRIu32 "\n",
           config.slave_pos,
           config.channel,
           config.node_id,
           config.canfd,
           config.bitrate,
           config.data_bitrate);

    if (config.send_nmt_start != 0) {
        rc = send_nmt_start(&mcan, &cia);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "NMT start failed: %d\n", rc);
            ecan_igh_close(&igh);
            return IGH_ECAN_EXIT_FAIL;
        }
        rc = igh_cia402_wait_heartbeat_timeout(&cia,
                                               &axis,
                                               IGH_CIA402_HEARTBEAT_OPERATIONAL,
                                               config.request_timeout_ms);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "wait heartbeat failed: %d\n", rc);
            ecan_igh_close(&igh);
            return IGH_ECAN_EXIT_FAIL;
        }
    }

    for (unsigned i = 0U; i < config.reads; i++) {
        rc = igh_cia402_request_actual_position_timeout(&cia,
                                                        &axis,
                                                        config.request_timeout_ms);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "position request %u failed: %d\n", i, rc);
            break;
        }
        printf("read=%u heartbeat=0x%02X actual=%" PRIi32 "\n",
               i,
               axis.heartbeat,
               axis.actual_position);
        usleep(config.cycle_us);
    }

    ecan_igh_close(&igh);
    return (rc == IGH_ECAN_OK) ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_FAIL;
}
