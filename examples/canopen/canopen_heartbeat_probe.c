#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "igh_cia402_protocol.h"
#include "igh_ecan_common.h"
#include "igh_mcan_port.h"

#define HEARTBEAT_PROBE_MAX_MOTORS      128U
#define HEARTBEAT_PROBE_RX_CAPACITY     256U
#define HEARTBEAT_PROBE_DEFAULT_SLAVES  3U
#define HEARTBEAT_PROBE_DEFAULT_MASK    0x0FU
#define HEARTBEAT_PROBE_DEFAULT_LISTEN_MS 3000U
#define HEARTBEAT_PROBE_DEFAULT_QUERY_MS  400U
#define HEARTBEAT_PROBE_DEFAULT_WAIT_MS   10000U

typedef struct {
    size_t slaves;
    uint8_t channel_mask;
    uint32_t cycle_us;
    uint32_t listen_ms;
    uint32_t query_ms;
    uint32_t wait_op_ms;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t query_index;
    uint8_t query_subindex;
} heartbeat_probe_config_t;

typedef struct {
    int slave_pos;
    uint8_t channel;
    uint8_t node_id;
    uint8_t heartbeat;
    uint8_t heartbeat_count;
    uint8_t sdo_response;
    uint8_t sdo_abort;
    uint8_t sdo_command;
    uint8_t sdo_data[4];
    uint32_t sdo_abort_code;
} heartbeat_motor_t;

typedef struct {
    heartbeat_motor_t motors[HEARTBEAT_PROBE_MAX_MOTORS];
    size_t count;
} heartbeat_motor_table_t;

static void default_config(heartbeat_probe_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->slaves = HEARTBEAT_PROBE_DEFAULT_SLAVES;
    cfg->channel_mask = HEARTBEAT_PROBE_DEFAULT_MASK;
    cfg->cycle_us = IGH_ECAN_DEFAULT_CYCLE_US;
    cfg->listen_ms = HEARTBEAT_PROBE_DEFAULT_LISTEN_MS;
    cfg->query_ms = HEARTBEAT_PROBE_DEFAULT_QUERY_MS;
    cfg->wait_op_ms = HEARTBEAT_PROBE_DEFAULT_WAIT_MS;
    cfg->nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    cfg->data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    cfg->query_index = 0x1000U;
    cfg->query_subindex = 0U;
}

static void print_usage(const char *app)
{
    printf("usage: %s [--slaves N] [--channels MASK] [--cycle-us N]\n", app);
    printf("          [--listen-ms N] [--query-ms N] [--wait-op-ms N]\n");
    printf("          [--bitrate N] [--data-bitrate N] [--index 0x1000] [--sub 0]\n");
    printf("default: slaves=3 channels=0x0f gateway=CANFD-capable CANopen-frame=classic-CAN\n");
}

static int parse_args(int argc, char **argv, heartbeat_probe_config_t *cfg)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--slaves") == 0 && i + 1 < argc) {
            cfg->slaves = igh_example_size(argv[++i]);
        } else if (strcmp(argv[i], "--channels") == 0 && i + 1 < argc) {
            cfg->channel_mask = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--cycle-us") == 0 && i + 1 < argc) {
            cfg->cycle_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--listen-ms") == 0 && i + 1 < argc) {
            cfg->listen_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--query-ms") == 0 && i + 1 < argc) {
            cfg->query_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--wait-op-ms") == 0 && i + 1 < argc) {
            cfg->wait_op_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            cfg->nominal_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            cfg->data_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--index") == 0 && i + 1 < argc) {
            cfg->query_index = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--sub") == 0 && i + 1 < argc) {
            cfg->query_subindex = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            exit(0);
        } else {
            return IGH_ECAN_INVALID_ARG;
        }
    }

    cfg->channel_mask &= 0x0FU;
    if (cfg->slaves == 0U || cfg->slaves > ECAN_IGH_MAX_SLAVES ||
        cfg->channel_mask == 0U || cfg->cycle_us == 0U ||
        cfg->nominal_bitrate == 0U || cfg->data_bitrate == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    return IGH_ECAN_OK;
}

static heartbeat_motor_t *find_motor(heartbeat_motor_table_t *table,
                                     int slave_pos,
                                     uint8_t channel,
                                     uint8_t node_id)
{
    for (size_t i = 0U; i < table->count; i++) {
        heartbeat_motor_t *motor = &table->motors[i];
        if (motor->slave_pos == slave_pos &&
            motor->channel == channel &&
            motor->node_id == node_id) {
            return motor;
        }
    }
    return NULL;
}

static heartbeat_motor_t *upsert_motor(heartbeat_motor_table_t *table,
                                       int slave_pos,
                                       uint8_t channel,
                                       uint8_t node_id)
{
    heartbeat_motor_t *motor = find_motor(table, slave_pos, channel, node_id);

    if (motor != NULL) {
        return motor;
    }
    if (table->count >= HEARTBEAT_PROBE_MAX_MOTORS) {
        return NULL;
    }
    motor = &table->motors[table->count++];
    memset(motor, 0, sizeof(*motor));
    motor->slave_pos = slave_pos;
    motor->channel = channel;
    motor->node_id = node_id;
    return motor;
}

static uint32_t get_u32_le(const uint8_t *data)
{
    return (uint32_t)data[0] |
           ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) |
           ((uint32_t)data[3] << 24);
}

static void on_frame(heartbeat_motor_table_t *table,
                     const igh_mcan_rx_frame_t *rx,
                     const heartbeat_probe_config_t *cfg)
{
    const igh_mcan_frame_t *frame = &rx->frame;

    if (frame->is_canfd != 0U) {
        return;
    }
    if (frame->can_id >= IGH_CIA402_COB_HEARTBEAT_BASE &&
        frame->can_id <= (IGH_CIA402_COB_HEARTBEAT_BASE + 0x7FU) &&
        frame->data_len >= 1U) {
        uint8_t node_id = (uint8_t)(frame->can_id - IGH_CIA402_COB_HEARTBEAT_BASE);
        heartbeat_motor_t *motor = upsert_motor(table, rx->slave_pos, frame->channel, node_id);
        if (motor != NULL) {
            motor->heartbeat = frame->data[0];
            if (motor->heartbeat_count < 0xFFU) {
                motor->heartbeat_count++;
            }
        }
        return;
    }
    if (frame->can_id >= IGH_CIA402_COB_SDO_RESPONSE_BASE &&
        frame->can_id <= (IGH_CIA402_COB_SDO_RESPONSE_BASE + 0x7FU) &&
        frame->data_len >= 8U) {
        uint8_t node_id = (uint8_t)(frame->can_id - IGH_CIA402_COB_SDO_RESPONSE_BASE);
        heartbeat_motor_t *motor = find_motor(table, rx->slave_pos, frame->channel, node_id);
        if (motor == NULL) {
            return;
        }
        if (((uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8)) != cfg->query_index ||
            frame->data[3] != cfg->query_subindex) {
            return;
        }
        motor->sdo_command = frame->data[0];
        if (frame->data[0] == IGH_CIA402_SDO_ABORT) {
            motor->sdo_abort = 1U;
            motor->sdo_abort_code = get_u32_le(&frame->data[4]);
        } else {
            motor->sdo_response = 1U;
            memcpy(motor->sdo_data, &frame->data[4], sizeof(motor->sdo_data));
        }
    }
}

static uint32_t cycles_from_ms(uint32_t ms, uint32_t cycle_us)
{
    uint64_t total_us = (uint64_t)ms * 1000ULL;
    uint32_t cycles = (uint32_t)((total_us + cycle_us - 1U) / cycle_us);

    return (cycles == 0U) ? 1U : cycles;
}

static int receive_for_cycles(igh_mcan_port_t *mcan,
                              heartbeat_motor_table_t *table,
                              const heartbeat_probe_config_t *cfg,
                              uint32_t cycles)
{
    igh_mcan_rx_frame_t frames[HEARTBEAT_PROBE_RX_CAPACITY];
    int rc = IGH_ECAN_OK;

    for (uint32_t cycle = 0U; cycle < cycles; cycle++) {
        size_t count = 0U;
        int cycle_rc;

        igh_mcan_port_cycle_begin(mcan);
        cycle_rc = igh_mcan_port_receive(mcan, frames, HEARTBEAT_PROBE_RX_CAPACITY, &count);
        igh_mcan_port_cycle_end(mcan);
        if ((rc == IGH_ECAN_OK) && cycle_rc != IGH_ECAN_OK) {
            rc = cycle_rc;
        }
        for (size_t i = 0U; i < count; i++) {
            on_frame(table, &frames[i], cfg);
        }
        usleep(cfg->cycle_us);
    }
    return rc;
}

static int send_sdo_upload(igh_mcan_port_t *mcan,
                           const heartbeat_motor_t *motor,
                           const heartbeat_probe_config_t *cfg)
{
    uint8_t data[8] = {0};
    int rc;

    data[0] = IGH_CIA402_SDO_UPLOAD;
    data[1] = (uint8_t)(cfg->query_index & 0xFFU);
    data[2] = (uint8_t)(cfg->query_index >> 8);
    data[3] = cfg->query_subindex;
    igh_mcan_port_cycle_begin(mcan);
    rc = igh_mcan_port_send_can(mcan,
                                motor->slave_pos,
                                motor->channel,
                                IGH_CIA402_COB_SDO_REQUEST_BASE + motor->node_id,
                                0,
                                data,
                                sizeof(data));
    igh_mcan_port_cycle_end(mcan);
    return rc;
}

static int configure_canfd_gateway(ecan_igh_config_t *igh_config,
                                   igh_mcan_port_config_t *mcan_config,
                                   const heartbeat_probe_config_t *cfg)
{
    if (ecan_igh_config_set_slave_count(igh_config, cfg->slaves) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(igh_config, cfg->cycle_us) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t slave = 0U; slave < cfg->slaves; slave++) {
        if (igh_mcan_port_set_channel_mask(mcan_config,
                                           slave,
                                           cfg->channel_mask,
                                           cfg->channel_mask) != IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(mcan_config,
                                      slave,
                                      cfg->nominal_bitrate,
                                      cfg->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    return igh_mcan_port_apply_config(igh_config, mcan_config);
}

static void print_summary(const heartbeat_motor_table_t *table,
                          const heartbeat_probe_config_t *cfg)
{
    printf("heartbeat_nodes=%zu query=0x%04X:%02X\n",
           table->count,
           cfg->query_index,
           cfg->query_subindex);
    for (size_t i = 0U; i < table->count; i++) {
        const heartbeat_motor_t *motor = &table->motors[i];
        int heartbeat_ok = (motor->heartbeat == IGH_CIA402_HEARTBEAT_OPERATIONAL);
        int query_ok = (motor->sdo_response != 0U);
        int normal = heartbeat_ok && query_ok;

        printf("motor slave=%d ch=%u node=%u heartbeat=0x%02X(%s) hb_count=%u"
               " query=%s normal=%s",
               motor->slave_pos,
               motor->channel,
               motor->node_id,
               motor->heartbeat,
               igh_cia402_heartbeat_state_name(motor->heartbeat),
               motor->heartbeat_count,
               query_ok ? "yes" : (motor->sdo_abort ? "abort" : "no"),
               normal ? "yes" : "no");
        if (query_ok) {
            printf(" sdo_cmd=0x%02X data=%02X %02X %02X %02X",
                   motor->sdo_command,
                   motor->sdo_data[0],
                   motor->sdo_data[1],
                   motor->sdo_data[2],
                   motor->sdo_data[3]);
        } else if (motor->sdo_abort) {
            printf(" abort=0x%08" PRIX32, motor->sdo_abort_code);
        }
        printf("\n");
    }
}

int main(int argc, char **argv)
{
    heartbeat_probe_config_t cfg;
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    heartbeat_motor_table_t table;
    int rc;

    default_config(&cfg);
    if (parse_args(argc, argv, &cfg) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (configure_canfd_gateway(&igh_config, &mcan_config, &cfg) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    printf("CANopen heartbeat probe: slaves=%zu channels=0x%02X gateway_canfd=1"
           " canopen_frame=classic-can bitrate=%" PRIu32 " data=%" PRIu32 "\n",
           cfg.slaves,
           cfg.channel_mask,
           cfg.nominal_bitrate,
           cfg.data_bitrate);

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
    rc = ecan_igh_wait_op_timeout(&igh, cfg.wait_op_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    memset(&table, 0, sizeof(table));
    rc = receive_for_cycles(&mcan, &table, &cfg,
                            cycles_from_ms(cfg.listen_ms, cfg.cycle_us));
    for (size_t i = 0U; i < table.count; i++) {
        int query_rc = send_sdo_upload(&mcan, &table.motors[i], &cfg);
        if (rc == IGH_ECAN_OK && query_rc != IGH_ECAN_OK) {
            rc = query_rc;
        }
        (void)receive_for_cycles(&mcan, &table, &cfg,
                                 cycles_from_ms(cfg.query_ms, cfg.cycle_us));
    }
    print_summary(&table, &cfg);

    ecan_igh_close(&igh);
    return (rc == IGH_ECAN_OK && table.count > 0U) ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_FAIL;
}
