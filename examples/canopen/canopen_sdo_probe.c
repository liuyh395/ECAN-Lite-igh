#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "cia402_demo_common.h"
#include "igh_ecan_common.h"
#include "igh_cia402_protocol.h"
#include "igh_mcan_port.h"

#define SDO_PROBE_RX_CAPACITY        64U
#define SDO_PROBE_DEFAULT_NODE_ID    1U
#define SDO_PROBE_DEFAULT_CHANNEL    0U
#define SDO_PROBE_DEFAULT_SLAVE_POS  0
#define SDO_PROBE_DEFAULT_INDEX      0x1000U
#define SDO_PROBE_DEFAULT_SUBINDEX   0x00U
#define SDO_PROBE_DEFAULT_CYCLE_US   10000U
#define SDO_PROBE_DEFAULT_WAIT_MS    10000U
#define SDO_PROBE_DEFAULT_SETTLE     20U
#define SDO_PROBE_DEFAULT_REBOOT_WAIT_MS 12000U
#define SDO_PROBE_DEFAULT_PDO_MULTIPLIER 3U

typedef struct {
    int slave_pos;
    uint8_t channel;
    uint8_t node_id;
    igh_ecan_canfd_mode_t canfd;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t index;
    uint8_t subindex;
    uint32_t cycle_us;
    uint32_t wait_op_timeout_ms;
    uint8_t pdo_multiplier;
    unsigned settle_cycles;
    int scan;
    int send_nmt;
    uint8_t nmt_command;
    uint8_t write_command;
    uint32_t write_value;
    int dump_headers;
    int reboot_slaves;
    uint32_t reboot_wait_ms;
} sdo_probe_config_t;

static void default_config(sdo_probe_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->slave_pos = SDO_PROBE_DEFAULT_SLAVE_POS;
    config->channel = SDO_PROBE_DEFAULT_CHANNEL;
    config->node_id = SDO_PROBE_DEFAULT_NODE_ID;
    config->canfd = IGH_CIA402_DEMO_DEFAULT_CANFD;
    config->nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    config->data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    config->index = SDO_PROBE_DEFAULT_INDEX;
    config->subindex = SDO_PROBE_DEFAULT_SUBINDEX;
    config->cycle_us = SDO_PROBE_DEFAULT_CYCLE_US;
    config->wait_op_timeout_ms = SDO_PROBE_DEFAULT_WAIT_MS;
    config->pdo_multiplier = SDO_PROBE_DEFAULT_PDO_MULTIPLIER;
    config->settle_cycles = SDO_PROBE_DEFAULT_SETTLE;
    config->send_nmt = 1;
    config->nmt_command = IGH_CIA402_NMT_START_REMOTE_NODE;
    config->reboot_wait_ms = SDO_PROBE_DEFAULT_REBOOT_WAIT_MS;
}

static void print_usage(const char *app)
{
    printf("usage: %s [--node N] [--scan] [--channel 0..3] [--slave N]\n", app);
    printf("          [--bitrate N] [--canfd] [--classic-can] [--data-bitrate N]\n");
    printf("          [--index 0x1000] [--sub 0] [--cycle-us N] [--pdo-n 5+3-encoding]\n");
    printf("          [--write-u8 V] [--write-u16 V] [--write-u32 V]\n");
    printf("          [--settle N] [--nmt-start] [--nmt-preop] [--nmt-reset] [--no-nmt]\n");
    printf("          [--dump-headers] [--reboot-slaves] [--reboot-wait-ms N]\n");
    printf("default: node=1 channel=0 bitrate=1000000 index=0x1000 sub=0\n");
}

static int parse_args(int argc, char **argv, sdo_probe_config_t *config)
{
    if (config == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--node") == 0 && i + 1 < argc) {
            config->node_id = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--scan") == 0) {
            config->scan = 1;
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            config->channel = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc) {
            config->slave_pos = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            config->nominal_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            config->data_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--canfd") == 0) {
            config->canfd = IGH_ECAN_CANFD_ENABLE;
        } else if (strcmp(argv[i], "--classic-can") == 0) {
            config->canfd = IGH_ECAN_CANFD_DISABLE;
        } else if (strcmp(argv[i], "--index") == 0 && i + 1 < argc) {
            config->index = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--sub") == 0 && i + 1 < argc) {
            config->subindex = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--cycle-us") == 0 && i + 1 < argc) {
            config->cycle_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--pdo-n") == 0 && i + 1 < argc) {
            config->pdo_multiplier = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--settle") == 0 && i + 1 < argc) {
            config->settle_cycles = igh_example_uint(argv[++i]);
        } else if (strcmp(argv[i], "--write-u8") == 0 && i + 1 < argc) {
            config->write_command = IGH_CIA402_SDO_WRITE_U8;
            config->write_value = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--write-u16") == 0 && i + 1 < argc) {
            config->write_command = IGH_CIA402_SDO_WRITE_U16;
            config->write_value = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--write-u32") == 0 && i + 1 < argc) {
            config->write_command = IGH_CIA402_SDO_WRITE_U32;
            config->write_value = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--wait-op-ms") == 0 && i + 1 < argc) {
            config->wait_op_timeout_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--nmt-start") == 0) {
            config->send_nmt = 1;
            config->nmt_command = IGH_CIA402_NMT_START_REMOTE_NODE;
        } else if (strcmp(argv[i], "--nmt-preop") == 0) {
            config->send_nmt = 1;
            config->nmt_command = IGH_CIA402_NMT_ENTER_PREOPERATIONAL;
        } else if (strcmp(argv[i], "--nmt-reset") == 0) {
            config->send_nmt = 1;
            config->nmt_command = IGH_CIA402_NMT_RESET_NODE;
        } else if (strcmp(argv[i], "--no-nmt") == 0 ||
                   strcmp(argv[i], "--no-nmt-start") == 0) {
            config->send_nmt = 0;
        } else if (strcmp(argv[i], "--dump-headers") == 0) {
            config->dump_headers = 1;
        } else if (strcmp(argv[i], "--reboot-slaves") == 0) {
            config->reboot_slaves = 1;
        } else if (strcmp(argv[i], "--reboot-wait-ms") == 0 && i + 1 < argc) {
            config->reboot_wait_ms = igh_example_u32(argv[++i]);
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
        config->node_id > IGH_ECAN_CANOPEN_NODE_ID_MAX ||
        config->nominal_bitrate == 0U ||
        config->data_bitrate == 0U ||
        config->cycle_us == 0U ||
        !ecan_igh_pdo_size_encoding_is_valid(config->pdo_multiplier)) {
        return IGH_ECAN_INVALID_ARG;
    }
    return IGH_ECAN_OK;
}

static int apply_mcan_port_config(ecan_igh_config_t *igh_config,
                                  igh_mcan_port_config_t *mcan_config,
                                  const sdo_probe_config_t *probe)
{
    size_t slave_count;

    if ((igh_config == NULL) || (mcan_config == NULL) || (probe == NULL) ||
        probe->slave_pos < 0) {
        return IGH_ECAN_INVALID_ARG;
    }

    slave_count = (size_t)probe->slave_pos + 1U;
    if (slave_count > ECAN_IGH_MAX_SLAVES) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (ecan_igh_config_set_slave_count(igh_config, slave_count) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(igh_config, probe->cycle_us) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(igh_config, probe->pdo_multiplier) !=
            IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < slave_count; i++) {
        if (igh_mcan_port_set_channel_mask(mcan_config,
                                           i,
                                           IGH_ECAN_BIT_U8(0U),
                                           (probe->canfd == IGH_ECAN_CANFD_ENABLE) ?
                                               IGH_ECAN_BIT_U8(0U) :
                                               0U) !=
                IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(mcan_config, i, probe->nominal_bitrate,
                                        probe->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    if (igh_mcan_port_set_channel_mask(mcan_config,
                                         (size_t)probe->slave_pos,
                                         IGH_ECAN_BIT_U8(probe->channel),
                                         (probe->canfd == IGH_ECAN_CANFD_ENABLE) ?
                                             IGH_ECAN_BIT_U8(probe->channel) :
                                             0U) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (probe->canfd == IGH_ECAN_CANFD_ENABLE) {
        if (igh_mcan_port_set_canfd_channel(mcan_config,
                                            (size_t)probe->slave_pos,
                                            probe->channel,
                                            probe->nominal_bitrate,
                                            probe->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    } else if (igh_mcan_port_set_classic_channel(mcan_config,
                                                 (size_t)probe->slave_pos,
                                                 probe->channel,
                                                 probe->nominal_bitrate) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    return igh_mcan_port_apply_config(igh_config, mcan_config);
}

static int send_sdo_upload(igh_mcan_port_t *mcan,
                           const sdo_probe_config_t *probe,
                           uint8_t node_id)
{
    uint8_t data[8] = {0};
    int rc;

    data[0] = IGH_CIA402_SDO_UPLOAD;
    data[1] = (uint8_t)(probe->index & 0xFFU);
    data[2] = (uint8_t)(probe->index >> 8);
    data[3] = probe->subindex;
    igh_mcan_port_cycle_begin(mcan);
    rc = igh_mcan_port_send_can(mcan,
                                probe->slave_pos,
                                probe->channel,
                                IGH_CIA402_COB_SDO_REQUEST_BASE + node_id,
                                0,
                                data,
                                sizeof(data));
    igh_mcan_port_cycle_end(mcan);
    return rc;
}

static int send_sdo_download(igh_mcan_port_t *mcan,
                             const sdo_probe_config_t *probe,
                             uint8_t node_id)
{
    uint8_t data[8] = {0};
    int rc;

    data[0] = probe->write_command;
    data[1] = (uint8_t)(probe->index & 0xFFU);
    data[2] = (uint8_t)(probe->index >> 8);
    data[3] = probe->subindex;
    data[4] = (uint8_t)(probe->write_value & 0xFFU);
    data[5] = (uint8_t)((probe->write_value >> 8) & 0xFFU);
    data[6] = (uint8_t)((probe->write_value >> 16) & 0xFFU);
    data[7] = (uint8_t)((probe->write_value >> 24) & 0xFFU);
    igh_mcan_port_cycle_begin(mcan);
    rc = igh_mcan_port_send_can(mcan,
                                probe->slave_pos,
                                probe->channel,
                                IGH_CIA402_COB_SDO_REQUEST_BASE + node_id,
                                0,
                                data,
                                sizeof(data));
    igh_mcan_port_cycle_end(mcan);
    return rc;
}

static int send_nmt(igh_mcan_port_t *mcan,
                    const sdo_probe_config_t *probe,
                    uint8_t node_id)
{
    uint8_t data[2];
    int rc;

    data[0] = probe->nmt_command;
    data[1] = node_id;
    igh_mcan_port_cycle_begin(mcan);
    rc = igh_mcan_port_send_can(mcan,
                                probe->slave_pos,
                                probe->channel,
                                IGH_CIA402_COB_NMT,
                                0,
                                data,
                                sizeof(data));
    igh_mcan_port_cycle_end(mcan);
    return rc;
}

static void print_frame(const igh_mcan_rx_frame_t *rx)
{
    const igh_mcan_frame_t *frame = &rx->frame;

    printf("rx slave=%d ch=%u id=0x%03" PRIX32 " dlc=%u data=",
           rx->slave_pos,
           frame->channel,
           frame->can_id,
           frame->dlc);
    for (uint8_t i = 0U; i < frame->data_len; i++) {
        printf("%02X", frame->data[i]);
        if (i + 1U < frame->data_len) {
            printf(" ");
        }
    }
    printf("\n");
}

static void print_tpdo_headers(igh_mcan_port_t *mcan,
                               const char *tag,
                               unsigned cycle)
{
    if ((mcan == NULL) || (mcan->igh == NULL)) {
        return;
    }

    for (size_t slave = 0U; slave < mcan->igh->slave_count; slave++) {
        igh_mcan_pdo_header_t header;

        if (igh_mcan_port_get_tpdo_header(mcan, slave, &header) != IGH_ECAN_OK) {
            continue;
        }
        if ((header.payload_len == 0U) &&
            (header.flags == 0U) &&
            (header.overflow_sat == 0U) &&
            (header.channel_status[0] == 0U) &&
            (header.channel_status[1] == 0U) &&
            (header.channel_status[2] == 0U) &&
            (header.channel_status[3] == 0U)) {
            continue;
        }
        printf("hdr %s cycle=%u slave=%zu ver=%u ihl=%u seq=%u flags=0x%02X"
               " payload_len=%u mask=0x%02X overflow=%u"
               " chstat=%04X,%04X,%04X,%04X\n",
               tag,
               cycle,
               slave,
               header.version,
               header.ihl_words,
               header.seq_begin,
               header.flags,
               header.payload_len,
               header.channel_mask,
               header.overflow_sat,
               header.channel_status[0],
               header.channel_status[1],
               header.channel_status[2],
               header.channel_status[3]);
    }
}

static int receive_cycles(igh_mcan_port_t *mcan,
                          const sdo_probe_config_t *probe,
                          uint8_t node_id,
                          int *got_response)
{
    igh_mcan_rx_frame_t rx[SDO_PROBE_RX_CAPACITY];
    size_t rx_count = 0U;
    int rc = IGH_ECAN_OK;

    for (unsigned cycle = 0U; cycle < probe->settle_cycles; cycle++) {
        int cycle_rc;

        igh_mcan_port_cycle_begin(mcan);
        cycle_rc = igh_mcan_port_receive(mcan, rx, SDO_PROBE_RX_CAPACITY, &rx_count);
        igh_mcan_port_cycle_end(mcan);
        if ((rc == IGH_ECAN_OK) && (cycle_rc != IGH_ECAN_OK)) {
            rc = cycle_rc;
        }
        for (size_t i = 0U; i < rx_count; i++) {
            print_frame(&rx[i]);
            if (rx[i].frame.can_id == IGH_CIA402_COB_SDO_RESPONSE_BASE + node_id) {
                *got_response = 1;
            }
        }
        if (probe->dump_headers) {
            print_tpdo_headers(mcan, "rx", cycle);
        }
        usleep(probe->cycle_us);
    }
    return rc;
}

static int probe_node(igh_mcan_port_t *mcan,
                      const sdo_probe_config_t *probe,
                      uint8_t node_id)
{
    int got_response = 0;
    int rc;

    if (probe->send_nmt) {
        rc = send_nmt(mcan, probe, node_id);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        usleep(probe->cycle_us);
    }

    rc = (probe->write_command != 0U) ?
         send_sdo_download(mcan, probe, node_id) :
         send_sdo_upload(mcan, probe, node_id);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    if (probe->write_command != 0U) {
        printf("tx node=%u id=0x%03X download=0x%04X:%02X cmd=0x%02X value=0x%08" PRIX32 "\n",
               node_id,
               IGH_CIA402_COB_SDO_REQUEST_BASE + node_id,
               probe->index,
               probe->subindex,
               probe->write_command,
               probe->write_value);
    } else {
        printf("tx node=%u id=0x%03X upload=0x%04X:%02X\n",
               node_id,
               IGH_CIA402_COB_SDO_REQUEST_BASE + node_id,
               probe->index,
               probe->subindex);
    }
    rc = receive_cycles(mcan, probe, node_id, &got_response);
    printf("node=%u response=%s\n", node_id, got_response ? "yes" : "no");
    return (rc == IGH_ECAN_OK && got_response) ? IGH_ECAN_OK : IGH_ECAN_TIMEOUT;
}

int main(int argc, char **argv)
{
    sdo_probe_config_t probe;
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    int rc;
    int any_response = 0;

    default_config(&probe);
    if (parse_args(argc, argv, &probe) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (apply_mcan_port_config(&igh_config, &mcan_config, &probe) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    if (probe.reboot_slaves) {
        printf("rebooting %zu ECAN slaves by SDO, wait=%" PRIu32 " ms\n",
               igh_config.slave_count,
               probe.reboot_wait_ms);
        rc = ecan_igh_reboot_slaves((uint32_t)igh_config.master_index,
                                    igh_config.slave_count,
                                    probe.reboot_wait_ms);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "slave reboot request failed: %d\n", rc);
            return IGH_ECAN_EXIT_FAIL;
        }
    }

    printf("ECAN CANopen SDO probe: slave=%d ch=%u canfd=%d"
           " canopen_frame=classic-can bitrate=%" PRIu32 " data=%" PRIu32
           " pdo=%uB index=0x%04X:%02X scan=%d write_cmd=0x%02X\n",
           probe.slave_pos,
           probe.channel,
           probe.canfd,
           probe.nominal_bitrate,
           probe.data_bitrate,
           (unsigned)ecan_igh_pdo_decode_bytes(probe.pdo_multiplier),
           probe.index,
           probe.subindex,
           probe.scan,
           probe.write_command);

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "IgH init failed: %d\n", rc);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = igh_mcan_port_init(&mcan, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "MCAN port init failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = ecan_igh_wait_op_timeout(&igh, probe.wait_op_timeout_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    if (probe.scan) {
        for (uint8_t node = 1U; node <= IGH_ECAN_CANOPEN_NODE_ID_MAX; node++) {
            rc = probe_node(&mcan, &probe, node);
            if (rc == IGH_ECAN_OK) {
                any_response = 1;
            }
        }
    } else {
        rc = probe_node(&mcan, &probe, probe.node_id);
        any_response = (rc == IGH_ECAN_OK) ? 1 : 0;
    }

    ecan_igh_close(&igh);
    return any_response ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_FAIL;
}
