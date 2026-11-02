#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "igh_mcan_port.h"
#include "igh_rt_support.h"
#include "../common/igh_example_stop.h"

#define MCAN_RX_DUMP_CAPACITY       128U
#define MCAN_RX_DUMP_DEFAULT_SEC    30U
#define MCAN_RX_DUMP_DEFAULT_DIAG   1000U
#define MCAN_RX_DUMP_WAIT_OP_MS     10000U

typedef struct {
    size_t slave_count;
    int filter_slave;
    int filter_channel;
    uint8_t channel_mask;
    uint8_t canfd_mask;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint32_t cycle_us;
    uint32_t wait_op_ms;
    uint32_t seconds;
    uint32_t cycles;
    uint32_t diag_every;
    uint32_t std_filter_id;
    uint32_t std_filter_mask;
    uint8_t pdo_multiplier;
    uint8_t align4;
    uint8_t std_filter_enable;
    uint8_t reject_nonmatch;
    uint8_t lock_memory;
    uint8_t catch_up;
    uint32_t spin_us;
    int rt_priority;
    int cpu;
} rx_dump_config_t;

static void print_usage(const char *app)
{
    printf("usage: %s [--slaves N] [--slave N] [--channel 0..3]\n", app);
    printf("          [--channel-mask M] [--canfd] [--classic-can] [--canfd-mask M]\n");
    printf("          [--bitrate N] [--data-bitrate N] [--pdo-n 5+3-encoding]\n");
    printf("          [--std-filter-id N] [--std-filter-mask N] [--reject-nonmatch]\n");
    printf("          [--cycle-us N] [--seconds N] [--cycles N] [--diag-every N]\n");
    printf("          [--rt-priority N] [--cpu N] [--spin-us N] [--catch-up] [--no-mlock]\n");
    printf("default: slaves=1 channels=0x0F canfd-mask=0x0F bitrate=1M data=5M\n");
}

static void default_config(rx_dump_config_t *config)
{
    memset(config, 0, sizeof(*config));
    config->slave_count = 1U;
    config->filter_slave = -1;
    config->filter_channel = -1;
    config->channel_mask = IGH_ECAN_MCAN_CHANNEL_MASK_ALL;
    config->canfd_mask = IGH_ECAN_MCAN_CHANNEL_MASK_ALL;
    config->nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    config->data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    config->cycle_us = 5000U;
    config->wait_op_ms = MCAN_RX_DUMP_WAIT_OP_MS;
    config->seconds = MCAN_RX_DUMP_DEFAULT_SEC;
    config->diag_every = MCAN_RX_DUMP_DEFAULT_DIAG;
    config->pdo_multiplier = IGH_ECAN_DEFAULT_PDO_MULTIPLIER;
    config->std_filter_mask = IGH_ECAN_CAN_STD_ID_MAX;
    config->lock_memory = 1U;
    config->rt_priority = 0;
    config->cpu = -1;
}

static int parse_args(int argc, char **argv, rx_dump_config_t *config)
{
    int canfd_mode = 1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--slaves") == 0 && i + 1 < argc) {
            config->slave_count = igh_example_size(argv[++i]);
        } else if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc) {
            config->filter_slave = igh_example_int(argv[++i]);
            if (config->filter_slave >= 0 &&
                config->slave_count <= (size_t)config->filter_slave) {
                config->slave_count = (size_t)config->filter_slave + 1U;
            }
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            config->filter_channel = igh_example_int(argv[++i]);
            if (config->filter_channel < 0 ||
                config->filter_channel >= (int)IGH_MCAN_CHANNEL_COUNT) {
                return IGH_ECAN_INVALID_ARG;
            }
            config->channel_mask = IGH_ECAN_BIT_U8((uint8_t)config->filter_channel);
        } else if (strcmp(argv[i], "--channel-mask") == 0 && i + 1 < argc) {
            config->channel_mask = igh_example_u8(argv[++i]);
            config->filter_channel = -1;
        } else if (strcmp(argv[i], "--canfd") == 0) {
            canfd_mode = 1;
        } else if (strcmp(argv[i], "--classic-can") == 0) {
            canfd_mode = 0;
        } else if (strcmp(argv[i], "--canfd-mask") == 0 && i + 1 < argc) {
            config->canfd_mask = igh_example_u8(argv[++i]);
            canfd_mode = 2;
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            config->nominal_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            config->data_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--cycle-us") == 0 && i + 1 < argc) {
            config->cycle_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--wait-op-ms") == 0 && i + 1 < argc) {
            config->wait_op_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
            config->seconds = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            config->cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--diag-every") == 0 && i + 1 < argc) {
            config->diag_every = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--std-filter-id") == 0 && i + 1 < argc) {
            config->std_filter_id = igh_example_u32(argv[++i]);
            config->std_filter_enable = 1U;
        } else if (strcmp(argv[i], "--std-filter-mask") == 0 && i + 1 < argc) {
            config->std_filter_mask = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--reject-nonmatch") == 0) {
            config->reject_nonmatch = 1U;
        } else if (strcmp(argv[i], "--pdo-n") == 0 && i + 1 < argc) {
            config->pdo_multiplier = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--align4") == 0) {
            config->align4 = 1U;
        } else if (strcmp(argv[i], "--rt-priority") == 0 && i + 1 < argc) {
            config->rt_priority = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--cpu") == 0 && i + 1 < argc) {
            config->cpu = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--spin-us") == 0 && i + 1 < argc) {
            config->spin_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--catch-up") == 0) {
            config->catch_up = 1U;
        } else if (strcmp(argv[i], "--no-mlock") == 0) {
            config->lock_memory = 0U;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            exit(0);
        } else {
            return IGH_ECAN_INVALID_ARG;
        }
    }

    if (canfd_mode == 1) {
        config->canfd_mask = config->channel_mask;
    } else if (canfd_mode == 0) {
        config->canfd_mask = 0U;
    } else {
        config->canfd_mask &= config->channel_mask;
    }

    if (config->slave_count == 0U ||
        config->slave_count > ECAN_IGH_MAX_SLAVES ||
        config->channel_mask == 0U ||
        (config->channel_mask & (uint8_t)~IGH_ECAN_MCAN_CHANNEL_MASK_ALL) != 0U ||
        config->nominal_bitrate == 0U ||
        config->data_bitrate == 0U ||
        config->cycle_us == 0U ||
        !ecan_igh_pdo_size_encoding_is_valid(config->pdo_multiplier) ||
        config->std_filter_id > IGH_ECAN_CAN_STD_ID_MAX ||
        config->std_filter_mask > IGH_ECAN_CAN_STD_ID_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }
    return IGH_ECAN_OK;
}

static uint32_t configured_cycles(const rx_dump_config_t *config)
{
    uint64_t cycles;

    if (config->cycles != 0U) {
        return config->cycles;
    }
    cycles = ((uint64_t)config->seconds * 1000000ULL) / config->cycle_us;
    if (cycles == 0U) {
        cycles = 1U;
    }
    if (cycles > UINT32_MAX) {
        cycles = UINT32_MAX;
    }
    return (uint32_t)cycles;
}

static int frame_matches(const rx_dump_config_t *config, const igh_mcan_rx_frame_t *rx)
{
    if (config->filter_slave >= 0 && rx->slave_pos != config->filter_slave) {
        return 0;
    }
    if (config->filter_channel >= 0 &&
        rx->frame.channel != (uint8_t)config->filter_channel) {
        return 0;
    }
    return 1;
}

static void print_data(const uint8_t *data, uint8_t len)
{
    for (uint8_t i = 0U; i < len; i++) {
        printf("%02X", data[i]);
        if (i + 1U < len) {
            printf(" ");
        }
    }
}

static void print_frame(uint32_t cycle, const igh_mcan_rx_frame_t *rx)
{
    const igh_mcan_frame_t *frame = &rx->frame;

    printf("rx cycle=%" PRIu32 " slave=%d ch=%u id=0x%08" PRIX32
           " ext=%u canfd=%u brs=%u rtr=%u dlc=%u len=%u seq=%u"
           " flags=0x%02X ch_status=0x%04X data=",
           cycle,
           rx->slave_pos,
           frame->channel,
           frame->can_id,
           frame->is_extended,
           frame->is_canfd,
           frame->brs,
           frame->rtr,
           frame->dlc,
           frame->data_len,
           rx->header.seq_begin,
           rx->header.flags,
           rx->header.channel_status[frame->channel]);
    print_data(frame->data, frame->data_len);
    printf("\n");
}

static void print_tpdo_headers(igh_mcan_port_t *mcan,
                               const rx_dump_config_t *config,
                               uint32_t cycle,
                               uint64_t printed_frames)
{
    for (size_t slave = 0U; slave < config->slave_count; slave++) {
        igh_mcan_pdo_header_t header;
        int rc = igh_mcan_port_get_tpdo_header(mcan, slave, &header);
        if (rc != IGH_ECAN_OK) {
            printf("[diag] cycle=%" PRIu32 " slave=%zu tpdo_header=unavailable rc=%d\n",
                   cycle, slave, rc);
            continue;
        }
        printf("[diag] cycle=%" PRIu32 " slave=%zu seq=%u flags=0x%02X"
               " mask=0x%02X payload_len=%u overflow=%u"
               " ch0=0x%04X ch1=0x%04X ch2=0x%04X ch3=0x%04X printed=%" PRIu64 "\n",
               cycle,
               slave,
               header.seq_begin,
               header.flags,
               header.channel_mask,
               header.payload_len,
               header.overflow_sat,
               header.channel_status[0],
               header.channel_status[1],
               header.channel_status[2],
               header.channel_status[3],
               printed_frames);
    }
}

static void print_parse_error(const igh_mcan_port_t *mcan)
{
    size_t slave = 0U;
    const igh_mcan_pdo_parse_diag_t *diag =
        igh_mcan_port_get_last_parse_error(mcan, &slave);

    if (diag == NULL) {
        return;
    }
    printf("[parse] slave=%zu code=%s offset=%zu end=%zu ctrl=0x%04X"
           " seq=%u frame_bytes=%u hdr_flags=0x%02X hdr_mask=0x%02X"
           " ch=%u dlc=%u tsf=%u kind=%u\n",
           slave,
           igh_mcan_pdo_parse_error_name(diag->code),
           diag->offset,
           diag->packet_end,
           diag->ctrl,
           diag->packet_sequence,
           diag->frame_bytes,
           diag->header_flags,
           diag->header_channel_mask,
           diag->channel,
           diag->dlc,
           diag->tsf,
           diag->kind);
}

static int apply_config(ecan_igh_config_t *igh_config,
                        igh_mcan_port_config_t *mcan_config,
                        const rx_dump_config_t *config)
{
    if (ecan_igh_config_set_slave_count(igh_config, config->slave_count) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(igh_config, config->cycle_us) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(igh_config, config->pdo_multiplier) !=
            IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (config->align4) {
        mcan_config->option_flags |= IGH_MCAN_OPT_ALIGN4;
    }
    for (size_t slave = 0U; slave < config->slave_count; slave++) {
        if (igh_mcan_port_set_channel_mask(mcan_config,
                                           slave,
                                           config->channel_mask,
                                           config->canfd_mask) != IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(mcan_config,
                                      slave,
                                      config->nominal_bitrate,
                                      config->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    if (igh_mcan_port_apply_config(igh_config, mcan_config) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (config->std_filter_enable != 0U) {
        size_t slave_begin = (config->filter_slave >= 0) ? (size_t)config->filter_slave : 0U;
        size_t slave_end = (config->filter_slave >= 0) ? (slave_begin + 1U) : config->slave_count;
        for (size_t slave = slave_begin; slave < slave_end; slave++) {
            for (uint8_t channel = 0U; channel < IGH_MCAN_CHANNEL_COUNT; channel++) {
                uint16_t index = (uint16_t)(IGH_MCAN_OD_CHANNEL_BASE + channel);
                if ((config->channel_mask & IGH_ECAN_BIT_U8(channel)) == 0U) {
                    continue;
                }
                if (config->filter_channel >= 0 && channel != (uint8_t)config->filter_channel) {
                    continue;
                }
                if (config->reject_nonmatch != 0U) {
                    (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_CHANNEL_STD_NONMATCH,
                                                   IGH_MCAN_FILTER_NONMATCH_REJECT);
                    (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                                   IGH_MCAN_OD_CHANNEL_EXT_NONMATCH,
                                                   IGH_MCAN_FILTER_NONMATCH_REJECT);
                }
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_STD_FILTER_COUNT, 1U);
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_STD_FILTER_INDEX, 0U);
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_STD_FILTER_TYPE,
                                               IGH_MCAN_FILTER_TYPE_CLASSIC);
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_STD_FILTER_ACTION,
                                               IGH_MCAN_FILTER_ACTION_RXFIFO0);
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_STD_FILTER_SYNC, 0U);
                (void)ecan_igh_config_add_sdo32(igh_config, slave, index,
                                                IGH_MCAN_OD_CHANNEL_STD_FILTER_ID1,
                                                config->std_filter_id);
                (void)ecan_igh_config_add_sdo32(igh_config, slave, index,
                                                IGH_MCAN_OD_CHANNEL_STD_FILTER_ID2,
                                                config->std_filter_mask);
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_STD_FILTER_CMD,
                                               IGH_MCAN_FILTER_CMD_SAVE_CURRENT);
                (void)ecan_igh_config_add_sdo8(igh_config, slave, index,
                                               IGH_MCAN_OD_CHANNEL_APPLY,
                                               IGH_ECAN_OD_APPLY_COMMAND);
            }
        }
    }

    return IGH_ECAN_OK;
}

int main(int argc, char **argv)
{
    rx_dump_config_t config;
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    uint64_t printed_frames = 0U;
    uint64_t truncated_batches = 0U;
    uint32_t cycles;
    igh_rt_periodic_clock_t clock;
    igh_rt_period_stats_t period_stats;
    int rc;

    default_config(&config);
    if (parse_args(argc, argv, &config) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (apply_config(&igh_config, &mcan_config, &config) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    cycles = configured_cycles(&config);
    printf("ECAN IgH MCAN RX dump: slaves=%zu filter_slave=%d filter_ch=%d"
           " channel_mask=0x%02X canfd_mask=0x%02X pdo=%uB cycle=%" PRIu32
           "us cycles=%" PRIu32 " rt_priority=%d cpu=%d spin_us=%" PRIu32
           " std_filter=%u std_id=0x%03" PRIX32
           " std_mask=0x%03" PRIX32 " reject_nonmatch=%u\n",
           config.slave_count,
           config.filter_slave,
           config.filter_channel,
           config.channel_mask,
           config.canfd_mask,
           (unsigned)ecan_igh_pdo_decode_bytes(config.pdo_multiplier),
           config.cycle_us,
           cycles,
           config.rt_priority,
           config.cpu,
           config.spin_us,
           config.std_filter_enable,
           config.std_filter_id,
           config.std_filter_mask,
           config.reject_nonmatch);

    if (igh_rt_apply_runtime_tuning_checked(config.lock_memory,
                                            config.rt_priority,
                                            config.cpu) != 0) {
        fprintf(stderr, "requested real-time tuning could not be applied\n");
        return IGH_ECAN_EXIT_FAIL;
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
        /* 部分设备可能已OP；先撤销未开始的业务会话，不能直接停止DC服务。
         * 原始rc保持失败，撤销成功不改写为正常启动或正常业务结束。 */
        igh_example_stop_result_t abort_result;
        int abort_rc = igh_example_startup_abort_preop(&mcan, igh_config.master_index,
                                                      "ethercat", 2000U, &abort_result);
        fprintf(stderr, "startup abort rc=%d elapsed_us=%" PRIu64
                " cli_status=%d cli_error=%d unreaped_child=%ld\n", abort_rc,
                abort_result.elapsed_us, abort_result.cli_status, abort_result.cli_error,
                (long)abort_result.unreaped_child);
        if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
            ecan_igh_print_runtime_diagnostics(&igh, 16U);
        }
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    igh_rt_periodic_clock_start(&clock, config.cycle_us, config.spin_us, config.catch_up);
    igh_rt_period_stats_reset(&period_stats);
    for (uint32_t cycle = 1U; cycle <= cycles; cycle++) {
        igh_mcan_rx_frame_t rx[MCAN_RX_DUMP_CAPACITY];
        size_t rx_count = 0U;
        int cycle_rc;
        int end_rc;
        uint64_t now_ns;

        now_ns = igh_rt_periodic_clock_wait(&clock);
        igh_rt_period_stats_sample(&period_stats, now_ns, clock.cycle_ns);

        cycle_rc = igh_mcan_port_cycle_begin_checked(&mcan);
        if (cycle_rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle begin failed cycle=%" PRIu32 " rc=%d\n", cycle, cycle_rc);
            rc = cycle_rc;
            break;
        }
        cycle_rc = igh_mcan_port_receive(&mcan, rx, MCAN_RX_DUMP_CAPACITY, &rx_count);
        if (cycle_rc == IGH_ECAN_NO_SPACE) {
            /* 容量错误独立锁存：后续周期成功或正常停机不能覆盖数据不完整。 */
            if (truncated_batches++ == 0U) {
                fprintf(stderr, "receive array capacity exceeded; capture is incomplete\n");
            }
        }
        end_rc = igh_mcan_port_cycle_end_checked(&mcan);
        if (end_rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle end failed cycle=%" PRIu32 " rc=%d\n", cycle, end_rc);
            rc = end_rc;
            break;
        }
        if (cycle_rc != IGH_ECAN_OK && cycle_rc != IGH_ECAN_NO_SPACE) {
            fprintf(stderr, "receive failed cycle=%" PRIu32 " rc=%d\n", cycle, cycle_rc);
            print_parse_error(&mcan);
            rc = cycle_rc;
            break;
        }

        for (size_t i = 0U; i < rx_count; i++) {
            if (frame_matches(&config, &rx[i])) {
                print_frame(cycle, &rx[i]);
                printed_frames++;
            }
        }

        if (config.diag_every != 0U && (cycle % config.diag_every) == 0U) {
            print_tpdo_headers(&mcan, &config, cycle, printed_frames);
        }
    }

    int stop_attempted = 0;
    /* 循环体读取的是begin阶段的映像；结束前消费最后一次end回收的映像。
     * 本示例不发送CAN命令，停止接收的截止点就是这次消费；后续外部帧不保证录入。 */
    if (rc == IGH_ECAN_OK || rc == IGH_ECAN_NO_SPACE) {
        igh_mcan_rx_frame_t tail[MCAN_RX_DUMP_CAPACITY];
        size_t tail_count = 0;
        int tail_rc = igh_mcan_port_receive(&mcan, tail, MCAN_RX_DUMP_CAPACITY, &tail_count);
        if (tail_rc == IGH_ECAN_NO_SPACE) {
            if (truncated_batches++ == 0U) {
                fprintf(stderr, "receive array capacity exceeded at final input; capture is incomplete\n");
            }
        }
        for (size_t i = 0; i < tail_count; ++i) {
            if (frame_matches(&config, &tail[i])) {
                print_frame(cycles, &tail[i]);
                printed_frames++;
            }
        }
        if (tail_rc != IGH_ECAN_OK) rc = tail_rc;
        if (rc == IGH_ECAN_OK || rc == IGH_ECAN_NO_SPACE) {
            igh_example_stop_result_t stop;
            stop_attempted = 1;
            int stop_rc = igh_example_stop_preop(&mcan, igh_config.master_index,
                                                "ethercat", 2000U, &stop);
            printf("stop rc=%d elapsed_us=%" PRIu64 " cli_status=%d cli_error=%d unreaped_child=%ld\n",
                   stop_rc, stop.elapsed_us, stop.cli_status, stop.cli_error, (long)stop.unreaped_child);
            if (stop_rc != IGH_ECAN_OK) rc = stop_rc;
        }
    }
    if (!stop_attempted && rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) {
        igh_example_stop_result_t stop;
        int stop_rc = igh_example_fault_stop_preop(&mcan, igh_config.master_index,
                                                   "ethercat", 2000U, &stop);
        fprintf(stderr, "fault stop rc=%d elapsed_us=%" PRIu64
                " cli_status=%d cli_error=%d unreaped_child=%ld\n", stop_rc,
                stop.elapsed_us, stop.cli_status, stop.cli_error, (long)stop.unreaped_child);
        /* PREOP退出成功不抵扣原周期错误；禁止把此次捕获报告为complete。 */
    }
    printf("ECAN IgH MCAN RX dump done: printed=%" PRIu64 "\n", printed_frames);
    printf("capture truncated_batches=%" PRIu64 " complete=%u\n",
           truncated_batches, (unsigned)(truncated_batches == 0U && rc == IGH_ECAN_OK));
    igh_rt_print_period_stats("rx_dump_period", &period_stats, clock.cycle_ns);
    printf("rx_dump_period_resyncs=%" PRIu32 "\n", clock.resyncs);
    if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
        ecan_igh_print_runtime_diagnostics(&igh, 16U);
    }
    ecan_igh_close(&igh);
    return (rc == IGH_ECAN_OK && truncated_batches == 0U) ?
           IGH_ECAN_EXIT_OK :
           IGH_ECAN_EXIT_FAIL;
}
