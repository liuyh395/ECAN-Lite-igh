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
#include "igh_ecan_common.h"
#include "igh_adapter.h"
#include "igh_mcan_port.h"

#define SINGLE_AXIS_RX_CAPACITY          32U
#define SINGLE_AXIS_DEFAULT_CYCLES       200U
#define SINGLE_AXIS_DEFAULT_STEP         500
#define SINGLE_AXIS_DEFAULT_CYCLE_US     1000U
#define SINGLE_AXIS_DEFAULT_WAIT_OP_MS   10000U
#define SINGLE_AXIS_DEFAULT_SDO_TIMEOUT_MS 3000U
#define SINGLE_AXIS_DEFAULT_DIAG_EVERY   100U
#define SINGLE_AXIS_DEFAULT_PDO_MULTIPLIER 3U
#define SINGLE_AXIS_TARGET_SDO_RETRIES   3U
#define SINGLE_AXIS_CONFIGURE_RETRIES    3U

/* 单轴 demo 是 10 轴 demo 的缩减版本：只保留一条电机 slot。 */
static const igh_cia402_demo_motor_slot_t k_motor_slots[] = {
    {IGH_ECAN_ENABLE, 0U, IGH_ECAN_MCAN_CH0, 1U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
};

static const igh_cia402_demo_motor_slot_t *find_default_motor(void)
{
    for (size_t i = 0U; i < IGH_ECAN_ARRAY_SIZE(k_motor_slots); i++) {
        if (k_motor_slots[i].enable == IGH_ECAN_ENABLE) {
            return &k_motor_slots[i];
        }
    }
    return NULL;
}

typedef struct {
    int slave_pos;
    uint8_t channel;
    uint8_t node_id;
    unsigned motion_cycles;
    unsigned nmt_reset_settle_cycles;
    unsigned nmt_start_settle_cycles;
    unsigned configure_settle_cycles;
    unsigned read_position_settle_cycles;
    unsigned enable_settle_cycles;
    unsigned stop_settle_cycles;
    int32_t target_step;
    igh_ecan_canfd_mode_t canfd;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint32_t wait_op_timeout_ms;
    uint32_t warmup_cycles;
    uint32_t warmup_stable_cycles;
    uint32_t sdo_timeout_ms;
    uint32_t diag_every;
    int verbose;
    int enable_motor;
    int skip_nmt_reset;
    int skip_csp_config;
    int csv_sdo_mode;
    int csv_pdo_mode;
    int allow_missing_position;
    int target_via_sdo;
    int scheduled_target;
    int scheduled_sync;
    int32_t csv_target_velocity;
    uint32_t scheduled_target_offset_ns;
    uint32_t scheduled_sync_offset_ns;
} single_axis_demo_config_t;

static void single_axis_default_config(single_axis_demo_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    const igh_cia402_demo_motor_slot_t *slot = find_default_motor();
    config->canfd = IGH_CIA402_DEMO_DEFAULT_CANFD;
    config->nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    config->data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    if (slot != NULL) {
        config->slave_pos = (int)slot->slave_pos;
        config->channel = (uint8_t)slot->mcan_channel;
        config->node_id = slot->node_id;
        config->canfd = slot->canfd;
        config->nominal_bitrate = slot->nominal_bitrate;
        config->data_bitrate = slot->data_bitrate;
    }
    config->motion_cycles = SINGLE_AXIS_DEFAULT_CYCLES;
    config->nmt_reset_settle_cycles = 1000U;
    config->nmt_start_settle_cycles = 50U;
    config->configure_settle_cycles = 5U;
    config->read_position_settle_cycles = 50U;
    config->enable_settle_cycles = 50U;
    config->stop_settle_cycles = 20U;
    config->target_step = SINGLE_AXIS_DEFAULT_STEP;
    config->csv_target_velocity = 50;
    config->wait_op_timeout_ms = SINGLE_AXIS_DEFAULT_WAIT_OP_MS;
    config->warmup_cycles = 32U;
    config->warmup_stable_cycles = 4U;
    config->sdo_timeout_ms = SINGLE_AXIS_DEFAULT_SDO_TIMEOUT_MS;
    config->diag_every = SINGLE_AXIS_DEFAULT_DIAG_EVERY;
    config->scheduled_target_offset_ns = 100000U;
    config->scheduled_sync_offset_ns = 200000U;
    config->skip_nmt_reset = 1;
#ifdef CIA402_SINGLE_AXIS_SDO_DEMO
    config->skip_csp_config = 1;
    config->target_via_sdo = 1;
#else
    config->skip_csp_config = 0;
    config->target_via_sdo = 0;
#endif
    config->verbose = 0;
}

static void print_usage(const char *app)
{
    printf("Motion requires --enable-motor after checking topology and mechanical safety.\n");
    printf("usage: %s [--slave N] [--channel 0..3] [--node N] [--bitrate N] [--pdo-n 5+3-encoding]\n", app);
    printf("          [--canfd] [--classic-can] [--data-bitrate N]\n");
    printf("          [--cycle-us N] [--cycles N] [--step N]\n");
    printf("          [--wait-op-ms N] [--warmup-cycles N] [--warmup-stable-cycles N]\n");
    printf("          [--sdo-timeout-ms N]\n");
    printf("          [--diag-every N] [--verbose]\n");
    printf("          [--nmt-reset] [--skip-nmt-reset] [--csp-config] [--skip-csp-config]\n");
    printf("          [--require-position] [--allow-missing-position]\n");
    printf("          [--target-pdo] [--target-sdo]\n");
    printf("          [--csp-sdo] [--csp-rpdo] [--csv-sdo] [--csv-pdo] [--csv-rpdo] [--velocity N]\n");
    printf("          [--scheduled-tx] [--scheduled-target] [--scheduled-sync] [--direct-tx]\n");
    printf("          [--schedule-target-offset-ns N] [--schedule-sync-offset-ns N]\n");
    printf("          [--align4] [--profile N] [--skip-sdo-config]\n");
    printf("          [--dc-sync0] [--dc-cycle-us N] [--dc-shift-us N]\n");
#ifdef CIA402_SINGLE_AXIS_SDO_DEMO
    printf("default: slave=0 channel=0 node=1 bitrate=1000000 pdo-n=3 cycle=1000us cycles=200 step=500 canfd=1 pdo_remap=stored target=sdo\n");
#else
    printf("default: slave=0 channel=0 node=1 bitrate=1000000 pdo-n=3 cycle=1000us cycles=200 step=500 canfd=1 pdo_remap=rpdo-verify target=pdo\n");
#endif
}

static int warmup_mcan_port_after_op(igh_mcan_port_t *mcan,
                                     uint32_t cycles,
                                     uint32_t stable_cycles,
                                     const char *label)
{
    igh_mcan_port_warmup_result_t result;

    if (cycles == 0U) {
        return IGH_ECAN_OK;
    }

    memset(&result, 0, sizeof(result));
    for (uint32_t i = 0U; i < cycles; i++) {
        igh_mcan_rx_frame_t rx[SINGLE_AXIS_RX_CAPACITY];
        int rc = igh_mcan_port_warmup_cycle_checked(mcan,
                                                    rx,
                                                    SINGLE_AXIS_RX_CAPACITY,
                                                    &result);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "[warmup] %s failed cycle=%" PRIu32
                    " rc=%d completed=%" PRIu32 " stable=%" PRIu32
                    " last_rx=%zu max_rx=%zu begin_err=%" PRIu32
                    " rx_err=%" PRIu32 " no_space=%" PRIu32
                    " end_err=%" PRIu32 "\n",
                    label,
                    i,
                    rc,
                    result.cycles_completed,
                    result.consecutive_stable_cycles,
                    result.last_rx_count,
                    result.max_rx_count,
                    result.begin_errors,
                    result.receive_errors,
                    result.no_space_events,
                    result.end_errors);
            return rc;
        }
        usleep(mcan->igh->cycle_us);
    }

    printf("[warmup] %s cycles=%" PRIu32 " stable=%" PRIu32
           " last_rx=%zu max_rx=%zu begin_err=%" PRIu32
           " rx_err=%" PRIu32 " no_space=%" PRIu32
           " end_err=%" PRIu32 "\n",
           label,
           result.cycles_completed,
           result.consecutive_stable_cycles,
           result.last_rx_count,
           result.max_rx_count,
           result.begin_errors,
           result.receive_errors,
           result.no_space_events,
           result.end_errors);
    if (stable_cycles != 0U &&
        result.consecutive_stable_cycles < stable_cycles) {
        fprintf(stderr, "[warmup] %s unstable: stable=%" PRIu32
                " required=%" PRIu32 " last_rx=%zu max_rx=%zu\n",
                label,
                result.consecutive_stable_cycles,
                stable_cycles,
                result.last_rx_count,
                result.max_rx_count);
        return IGH_ECAN_NOT_READY;
    }
    return IGH_ECAN_OK;
}

static int parse_args(int argc,
                      char **argv,
                      ecan_igh_config_t *igh_config,
                      igh_mcan_port_config_t *mcan_config,
                      single_axis_demo_config_t *demo_config)
{
    if ((igh_config == NULL) || (mcan_config == NULL) || (demo_config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--enable-motor") == 0) {
            demo_config->enable_motor = 1;
        } else if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc) {
            demo_config->slave_pos = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            demo_config->channel = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--node") == 0 && i + 1 < argc) {
            demo_config->node_id = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            demo_config->nominal_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            demo_config->data_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--canfd") == 0) {
            demo_config->canfd = IGH_ECAN_CANFD_ENABLE;
        } else if (strcmp(argv[i], "--classic-can") == 0) {
            demo_config->canfd = IGH_ECAN_CANFD_DISABLE;
        } else if (strcmp(argv[i], "--pdo-n") == 0 && i + 1 < argc) {
            if (ecan_igh_config_set_pdo_multiplier(igh_config,
                                                   igh_example_u8(argv[++i])) !=
                IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--cycle-us") == 0 && i + 1 < argc) {
            if (ecan_igh_config_set_cycle_us(igh_config, igh_example_u32(argv[++i])) !=
                IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc) {
            demo_config->motion_cycles = igh_example_uint(argv[++i]);
        } else if (strcmp(argv[i], "--step") == 0 && i + 1 < argc) {
            demo_config->target_step = igh_example_i32(argv[++i]);
        } else if (strcmp(argv[i], "--wait-op-ms") == 0 && i + 1 < argc) {
            demo_config->wait_op_timeout_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--warmup-cycles") == 0 && i + 1 < argc) {
            demo_config->warmup_cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--warmup-stable-cycles") == 0 && i + 1 < argc) {
            demo_config->warmup_stable_cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--sdo-timeout-ms") == 0 && i + 1 < argc) {
            demo_config->sdo_timeout_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--diag-every") == 0 && i + 1 < argc) {
            demo_config->diag_every = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--verbose") == 0) {
            demo_config->verbose = 1;
        } else if (strcmp(argv[i], "--nmt-reset") == 0) {
            demo_config->skip_nmt_reset = 0;
        } else if (strcmp(argv[i], "--skip-nmt-reset") == 0) {
            demo_config->skip_nmt_reset = 1;
        } else if (strcmp(argv[i], "--csp-config") == 0) {
            demo_config->skip_csp_config = 0;
        } else if (strcmp(argv[i], "--skip-csp-config") == 0) {
            demo_config->skip_csp_config = 1;
        } else if (strcmp(argv[i], "--csp-rpdo") == 0) {
            demo_config->csv_sdo_mode = 0;
            demo_config->csv_pdo_mode = 0;
            demo_config->skip_csp_config = 0;
            demo_config->target_via_sdo = 0;
        } else if (strcmp(argv[i], "--csv-pdo") == 0 ||
                   strcmp(argv[i], "--csv-rpdo") == 0) {
            demo_config->csv_sdo_mode = 0;
            demo_config->csv_pdo_mode = 1;
            demo_config->skip_csp_config = 0;
            demo_config->target_via_sdo = 0;
        } else if (strcmp(argv[i], "--csv-sdo") == 0) {
            demo_config->csv_sdo_mode = 1;
            demo_config->csv_pdo_mode = 0;
            demo_config->skip_csp_config = 1;
            demo_config->target_via_sdo = 0;
        } else if (strcmp(argv[i], "--velocity") == 0 && i + 1 < argc) {
            demo_config->csv_target_velocity = igh_example_i32(argv[++i]);
        } else if (strcmp(argv[i], "--require-position") == 0) {
            demo_config->allow_missing_position = 0;
        } else if (strcmp(argv[i], "--allow-missing-position") == 0) {
            demo_config->allow_missing_position = 1;
        } else if (strcmp(argv[i], "--target-sdo") == 0 ||
                   strcmp(argv[i], "--csp-sdo") == 0) {
            demo_config->csv_sdo_mode = 0;
            demo_config->csv_pdo_mode = 0;
            demo_config->target_via_sdo = 1;
            if (strcmp(argv[i], "--csp-sdo") == 0) {
                demo_config->skip_csp_config = 1;
            }
        } else if (strcmp(argv[i], "--target-pdo") == 0) {
            demo_config->csv_sdo_mode = 0;
            demo_config->csv_pdo_mode = 0;
            demo_config->target_via_sdo = 0;
        } else if (strcmp(argv[i], "--scheduled-tx") == 0) {
            demo_config->scheduled_target = 1;
            demo_config->scheduled_sync = 1;
        } else if (strcmp(argv[i], "--scheduled-target") == 0) {
            demo_config->scheduled_target = 1;
        } else if (strcmp(argv[i], "--scheduled-sync") == 0) {
            demo_config->scheduled_sync = 1;
        } else if (strcmp(argv[i], "--direct-tx") == 0) {
            demo_config->scheduled_target = 0;
            demo_config->scheduled_sync = 0;
        } else if (strcmp(argv[i], "--schedule-target-offset-ns") == 0 && i + 1 < argc) {
            demo_config->scheduled_target_offset_ns = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--schedule-sync-offset-ns") == 0 && i + 1 < argc) {
            demo_config->scheduled_sync_offset_ns = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--align4") == 0) {
            mcan_config->option_flags |= IGH_MCAN_OPT_ALIGN4;
        } else if (strcmp(argv[i], "--profile") == 0 && i + 1 < argc) {
            mcan_config->encapsulation_profile = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--skip-sdo-config") == 0) {
            igh_config->init_flags |= ECAN_IGH_INIT_SKIP_STARTUP_SDO;
        } else if (strcmp(argv[i], "--dc-sync0") == 0) {
            if (ecan_igh_config_enable_dc_sync0(igh_config, 0U, 0) != IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--dc-cycle-us") == 0 && i + 1 < argc) {
            uint32_t cycle_us = igh_example_u32(argv[++i]);
            int32_t shift_us = igh_config->dc_sync0_shift_ns / 1000;
            if (ecan_igh_config_enable_dc_sync0(igh_config, cycle_us, shift_us) != IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--dc-shift-us") == 0 && i + 1 < argc) {
            long shift_us = igh_example_long(argv[++i]);
            uint32_t cycle_us = (igh_config->dc_sync0_cycle_ns != 0U) ?
                                (igh_config->dc_sync0_cycle_ns / 1000U) :
                                0U;
            if (ecan_igh_config_enable_dc_sync0(igh_config, cycle_us, (int32_t)shift_us) !=
                IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            exit(0);
        } else {
            print_usage(argv[0]);
            return IGH_ECAN_INVALID_ARG;
        }
    }

    if (demo_config->slave_pos < 0 ||
        demo_config->channel >= IGH_MCAN_CHANNEL_COUNT ||
        demo_config->node_id == 0U ||
        demo_config->node_id > 0x7FU ||
        demo_config->nominal_bitrate == 0U ||
        demo_config->data_bitrate == 0U ||
        (demo_config->warmup_cycles != 0U &&
         demo_config->warmup_stable_cycles > demo_config->warmup_cycles) ||
        (demo_config->target_via_sdo != 0 && demo_config->scheduled_target != 0) ||
        (demo_config->scheduled_target != 0 &&
         demo_config->scheduled_sync != 0 &&
         demo_config->scheduled_sync_offset_ns <= demo_config->scheduled_target_offset_ns)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if ((demo_config->scheduled_target != 0 || demo_config->scheduled_sync != 0) &&
        igh_config->dc_assign_activate == 0U &&
        ecan_igh_config_enable_dc_sync0(igh_config, 0U, 0) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    return IGH_ECAN_OK;
}

static int apply_mcan_port_config(ecan_igh_config_t *igh_config,
                                  igh_mcan_port_config_t *mcan_config,
                                  const single_axis_demo_config_t *demo_config)
{
    size_t slave_count;

    if ((igh_config == NULL) || (mcan_config == NULL) || (demo_config == NULL) ||
        demo_config->slave_pos < 0) {
        return IGH_ECAN_INVALID_ARG;
    }

    slave_count = (size_t)demo_config->slave_pos + 1U;
    if (slave_count > ECAN_IGH_MAX_SLAVES) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (ecan_igh_config_set_slave_count(igh_config, slave_count) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < igh_config->slave_count; i++) {
        if (igh_mcan_port_set_channel_mask(mcan_config, i, IGH_ECAN_BIT_U8(0U), 0U) !=
                IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(mcan_config, i, demo_config->nominal_bitrate,
                                        demo_config->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    if (igh_mcan_port_set_channel_mask(mcan_config,
                                         (size_t)demo_config->slave_pos,
                                         IGH_ECAN_BIT_U8(demo_config->channel),
                                         (demo_config->canfd == IGH_ECAN_CANFD_ENABLE) ?
                                             IGH_ECAN_BIT_U8(demo_config->channel) :
                                             0U) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (demo_config->canfd == IGH_ECAN_CANFD_ENABLE) {
        if (igh_mcan_port_set_canfd_channel(mcan_config,
                                            (size_t)demo_config->slave_pos,
                                            demo_config->channel,
                                            demo_config->nominal_bitrate,
                                            demo_config->data_bitrate) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    } else if (igh_mcan_port_set_classic_channel(mcan_config,
                                                 (size_t)demo_config->slave_pos,
                                                 demo_config->channel,
                                                 demo_config->nominal_bitrate) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    return igh_mcan_port_apply_config(igh_config, mcan_config);
}

static int process_mcan_frames(igh_cia402_context_t *cia,
                               const igh_mcan_rx_frame_t *mcan_frames,
                               size_t count)
{
    int rc = IGH_ECAN_OK;

    if ((cia == NULL) || ((mcan_frames == NULL) && (count > 0U))) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < count; i++) {
        igh_cia402_rx_frame_t rx;
        int frame_rc;

        igh_cia402_mcan_rx_to_can(&mcan_frames[i], &rx);
        frame_rc = igh_cia402_on_can_frame(cia, rx.slave_pos, &rx.frame);
        if ((rc == IGH_ECAN_OK) && (frame_rc != IGH_ECAN_OK)) {
            rc = frame_rc;
        }
    }
    return rc;
}

static void run_one_cycle(igh_mcan_port_t *mcan, igh_cia402_context_t *cia)
{
    igh_mcan_rx_frame_t frames[SINGLE_AXIS_RX_CAPACITY];
    size_t frame_count = 0U;

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, SINGLE_AXIS_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    igh_mcan_port_cycle_end(mcan);
    usleep(mcan->igh->cycle_us);
}

static void flush_cycles(igh_mcan_port_t *mcan,
                         igh_cia402_context_t *cia,
                         unsigned settle_cycles)
{
    for (unsigned i = 0U; i < settle_cycles; i++) {
        run_one_cycle(mcan, cia);
    }
}

static int axis_ready_for_motion(const igh_cia402_axis_t *axis)
{
    if (axis == NULL) {
        return 0;
    }
    if (axis->emcy_valid != 0U) {
        return 0;
    }
    if (axis->statusword_valid != 0U) {
        return igh_cia402_statusword_is_operation_enabled(axis->statusword);
    }
    return (axis->heartbeat_valid != 0U &&
            axis->heartbeat == IGH_CIA402_HEARTBEAT_OPERATIONAL) ? 1 : 0;
}

static void print_stage(const char *stage)
{
    printf("[stage] %s\n", stage);
    fflush(stdout);
}

static int send_nmt_and_flush(igh_mcan_port_t *mcan,
                              igh_cia402_context_t *cia,
                              uint8_t command,
                              unsigned settle_cycles)
{
    igh_cia402_axis_t *axis;
    int rc;

    if ((mcan == NULL) || (cia == NULL) || cia->axis_count == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    axis = &cia->axes[0];
    igh_mcan_port_cycle_begin(mcan);
    rc = igh_cia402_send_nmt(cia, axis, command);
    igh_mcan_port_cycle_end(mcan);
    usleep(mcan->igh->cycle_us);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int send_nmt_broadcast_and_flush(igh_mcan_port_t *mcan,
                                        igh_cia402_context_t *cia,
                                        const igh_cia402_axis_t *axis,
                                        uint8_t command,
                                        unsigned settle_cycles)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    igh_mcan_port_cycle_begin(mcan);
    rc = igh_cia402_send_nmt_broadcast(cia, axis->slave_pos, axis->channel, command);
    igh_mcan_port_cycle_end(mcan);
    usleep(mcan->igh->cycle_us);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static uint32_t cycles_to_timeout_ms(const igh_mcan_port_t *mcan, unsigned cycles)
{
    uint64_t timeout_ms;

    if ((mcan == NULL) || (mcan->igh == NULL) || cycles == 0U) {
        return 0U;
    }
    timeout_ms = ((uint64_t)mcan->igh->cycle_us * cycles + 999U) / 1000U;
    if (timeout_ms == 0U) {
        return 1U;
    }
    return (timeout_ms > UINT32_MAX) ? UINT32_MAX : (uint32_t)timeout_ms;
}

static int write_target_position_sdo_retry(igh_mcan_port_t *mcan,
                                           igh_cia402_context_t *cia,
                                           igh_cia402_axis_t *axis,
                                           int32_t target_position,
                                           uint32_t timeout_ms)
{
    int rc = IGH_ECAN_FAIL;

    for (unsigned attempt = 0U; attempt < SINGLE_AXIS_TARGET_SDO_RETRIES; attempt++) {
        rc = igh_cia402_sdo_write_u32_timeout(cia,
                                              axis,
                                              IGH_CIA402_IDX_TARGET_POSITION,
                                              0x00U,
                                              (uint32_t)target_position,
                                              timeout_ms);
        if (rc == IGH_ECAN_OK) {
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_TIMEOUT) {
            return rc;
        }
        flush_cycles(mcan, cia, 2U);
    }
    return rc;
}

static int write_target_velocity_sdo_retry(igh_mcan_port_t *mcan,
                                           igh_cia402_context_t *cia,
                                           igh_cia402_axis_t *axis,
                                           int32_t target_velocity,
                                           uint32_t timeout_ms)
{
    int rc = IGH_ECAN_FAIL;

    for (unsigned attempt = 0U; attempt < SINGLE_AXIS_TARGET_SDO_RETRIES; attempt++) {
        rc = igh_cia402_sdo_write_u32_timeout(cia,
                                              axis,
                                              IGH_CIA402_IDX_TARGET_VELOCITY,
                                              IGH_CIA402_SUBIDX_VALUE,
                                              (uint32_t)target_velocity,
                                              timeout_ms);
        if (rc == IGH_ECAN_OK) {
            axis->target_velocity = target_velocity;
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_TIMEOUT) {
            return rc;
        }
        flush_cycles(mcan, cia, 2U);
    }
    return rc;
}

static const char *comm_mode_name(uint32_t mode)
{
    switch (mode) {
    case IGH_CIA402_PH_COMM_MODE_ETHERCAT:
        return "ethercat";
    case IGH_CIA402_PH_COMM_MODE_CANOPEN:
        return "canopen";
    default:
        return "unknown";
    }
}

static int read_comm_mode(igh_cia402_context_t *cia,
                          igh_cia402_axis_t *axis,
                          uint32_t timeout_ms,
                          uint32_t *mode)
{
    uint8_t size = 0U;
    int rc;

    rc = igh_cia402_sdo_upload_u32_timeout(cia,
                                           axis,
                                           IGH_CIA402_IDX_PH_COMM_MODE,
                                           0x00U,
                                           mode,
                                           &size,
                                           timeout_ms);
    if (rc == IGH_ECAN_OK && size < 1U) {
        return IGH_ECAN_FAIL;
    }
    return rc;
}

static int comm_mode_read_needs_nmt_retry(int rc, const igh_cia402_axis_t *axis)
{
    if (rc == IGH_ECAN_TIMEOUT) {
        return 1;
    }
    return (rc == IGH_ECAN_FAIL &&
            axis != NULL &&
            axis->sdo_response_command == 0U &&
            axis->sdo_abort_code == 0U) ? 1 : 0;
}

static int axis_sdo_needs_retry(int rc, const igh_cia402_axis_t *axis)
{
    return comm_mode_read_needs_nmt_retry(rc, axis);
}

static int ensure_canopen_mode(igh_mcan_port_t *mcan,
                               igh_cia402_context_t *cia,
                               igh_cia402_axis_t *axis,
                               uint32_t timeout_ms)
{
    uint32_t mode = 0U;
    int rc;

    print_stage("read-communication-mode");
    rc = read_comm_mode(cia, axis, timeout_ms, &mode);
    if (comm_mode_read_needs_nmt_retry(rc, axis)) {
        printf("[diag] 0x2004:00 no response before NMT start; send NMT start and retry.\n");
        if (send_nmt_broadcast_and_flush(mcan, cia, axis,
                                         IGH_CIA402_NMT_START_REMOTE_NODE, 20U) !=
            IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
        rc = read_comm_mode(cia, axis, timeout_ms, &mode);
    }
    if (rc != IGH_ECAN_OK) {
        if (comm_mode_read_needs_nmt_retry(rc, axis)) {
            printf("[diag] 0x2004:00 unsupported/no-response; assume CANopen mode for this axis.\n");
            return IGH_ECAN_OK;
        }
        fprintf(stderr,
                "read communication mode failed: rc=%d index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " %s\n",
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_abort_code,
                igh_cia402_sdo_abort_name(axis->sdo_abort_code));
        return rc;
    }
    printf("[diag] communication-mode 0x2004:00=%" PRIu32 " (%s)\n",
           mode & 0xFFU,
           comm_mode_name(mode & 0xFFU));

    if ((mode & 0xFFU) == IGH_CIA402_PH_COMM_MODE_CANOPEN) {
        return IGH_ECAN_OK;
    }
    if ((mode & 0xFFU) != IGH_CIA402_PH_COMM_MODE_ETHERCAT) {
        fprintf(stderr, "unsupported communication mode: 0x%02" PRIX32 "\n", mode & 0xFFU);
        return IGH_ECAN_UNSUPPORTED;
    }

    print_stage("switch-canopen-mode");
    rc = igh_cia402_sdo_write_verify_u8_timeout(cia,
                                                axis,
                                                IGH_CIA402_IDX_PH_COMM_MODE,
                                                0x00U,
                                                IGH_CIA402_PH_COMM_MODE_CANOPEN,
                                                timeout_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "switch communication mode to CANopen failed: rc=%d index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " %s\n",
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_abort_code,
                igh_cia402_sdo_abort_name(axis->sdo_abort_code));
        return rc;
    }
    rc = read_comm_mode(cia, axis, timeout_ms, &mode);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    if ((mode & 0xFFU) != IGH_CIA402_PH_COMM_MODE_CANOPEN) {
        fprintf(stderr, "communication mode verify failed: 0x%02" PRIX32 "\n", mode & 0xFFU);
        return IGH_ECAN_FAIL;
    }
    return IGH_ECAN_OK;
}

static int set_operation_mode_with_readback(igh_mcan_port_t *mcan,
                                            igh_cia402_context_t *cia,
                                            igh_cia402_axis_t *axis,
                                            uint8_t operation_mode,
                                            const char *mode_name,
                                            uint32_t timeout_ms,
                                            unsigned settle_cycles)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (unsigned attempt = 0U; attempt < 3U; attempt++) {
        uint32_t mode = 0U;
        uint8_t size = 0U;
        int read_rc;

        rc = igh_cia402_sdo_write_u8_timeout(cia,
                                             axis,
                                             IGH_CIA402_IDX_MODES_OF_OPERATION,
                                             IGH_CIA402_SUBIDX_VALUE,
                                             operation_mode,
                                             timeout_ms);
        if (rc == IGH_ECAN_OK) {
            flush_cycles(mcan, cia, settle_cycles);
            read_rc = igh_cia402_sdo_upload_u32_timeout(cia,
                                                        axis,
                                                        IGH_CIA402_IDX_MODES_OF_OPERATION,
                                                        IGH_CIA402_SUBIDX_VALUE,
                                                        &mode,
                                                        &size,
                                                        timeout_ms);
            if (read_rc == IGH_ECAN_OK) {
                if (size >= 1U &&
                    ((mode & IGH_CIA402_SDO_VALUE_MASK_U8) == operation_mode)) {
                    return IGH_ECAN_OK;
                }
                fprintf(stderr,
                        "%s mode readback mismatch: slave=%d ch=%u node=0x%02X"
                        " value=0x%02" PRIX32 " size=%u\n",
                        mode_name,
                        axis->slave_pos,
                        axis->channel,
                        axis->node_id,
                        (uint32_t)(mode & IGH_CIA402_SDO_VALUE_MASK_U8),
                        size);
                return IGH_ECAN_FAIL;
            }
            fprintf(stderr,
                    "[warn] %s mode write acknowledged but readback failed:"
                    " slave=%d ch=%u node=0x%02X read_rc=%d"
                    " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " %s\n",
                    mode_name,
                    axis->slave_pos,
                    axis->channel,
                    axis->node_id,
                    read_rc,
                    axis->sdo_response_index,
                    axis->sdo_response_subindex,
                    axis->sdo_response_command,
                    axis->sdo_abort_code,
                    igh_cia402_sdo_abort_name(axis->sdo_abort_code));
            return IGH_ECAN_OK;
        }

        if (!comm_mode_read_needs_nmt_retry(rc, axis)) {
            return rc;
        }
        if (attempt + 1U >= 3U) {
            break;
        }
        printf("[diag] %s mode write retry slave=%d ch=%u node=0x%02X"
               " attempt=%u/3 rc=%d index=0x%04X:%02X cmd=0x%02X;"
               " send NMT start.\n",
               mode_name,
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               rc,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command);
        if (send_nmt_broadcast_and_flush(mcan,
                                         cia,
                                         axis,
                                         IGH_CIA402_NMT_START_REMOTE_NODE,
                                         settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (comm_mode_read_needs_nmt_retry(rc, axis)) {
        printf("[diag] %s mode write has no final SDO ack; continue with stored/assumed mode.\n",
               mode_name);
        return IGH_ECAN_OK;
    }
    return IGH_ECAN_FAIL;
}

static int set_csp_mode_with_readback(igh_mcan_port_t *mcan,
                                      igh_cia402_context_t *cia,
                                      igh_cia402_axis_t *axis,
                                      uint32_t timeout_ms,
                                      unsigned settle_cycles)
{
    axis->rpdo1_payload_len = IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
    return set_operation_mode_with_readback(mcan,
                                            cia,
                                            axis,
                                            IGH_CIA402_MODE_CSP,
                                            "CSP",
                                            timeout_ms,
                                            settle_cycles);
}

static int set_csv_mode_with_readback(igh_mcan_port_t *mcan,
                                      igh_cia402_context_t *cia,
                                      igh_cia402_axis_t *axis,
                                      uint32_t timeout_ms,
                                      unsigned settle_cycles)
{
    return set_operation_mode_with_readback(mcan,
                                            cia,
                                            axis,
                                            IGH_CIA402_MODE_CSV,
                                            "CSV",
                                            timeout_ms,
                                            settle_cycles);
}

static int stop_and_flush(igh_mcan_port_t *mcan,
                          igh_cia402_context_t *cia,
                          unsigned settle_cycles)
{
    int rc;
    igh_mcan_rx_frame_t frames[SINGLE_AXIS_RX_CAPACITY];
    size_t frame_count = 0U;

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, SINGLE_AXIS_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    rc = igh_cia402_stop(cia, &cia->axes[0]);
    igh_mcan_port_cycle_end(mcan);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int send_csv_velocity_pdo_cycle(igh_mcan_port_t *mcan,
                                       igh_cia402_context_t *cia,
                                       igh_cia402_axis_t *axis,
                                       int32_t target_velocity)
{
    igh_mcan_rx_frame_t frames[SINGLE_AXIS_RX_CAPACITY];
    size_t frame_count = 0U;
    igh_cia402_tx_frame_t velocity_frame;
    igh_cia402_tx_frame_t sync_frame;
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, SINGLE_AXIS_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    rc = igh_cia402_build_velocity_frame(cia, axis, target_velocity, &velocity_frame);
    if (rc == IGH_ECAN_OK) {
        rc = igh_cia402_mcan_send_frame(mcan, &velocity_frame);
    }
    if (rc == IGH_ECAN_OK) {
        axis->target_velocity = target_velocity;
        rc = igh_cia402_build_sync_frame(cia, axis->slave_pos, axis->channel, &sync_frame);
    }
    if (rc == IGH_ECAN_OK) {
        rc = igh_cia402_mcan_send_frame(mcan, &sync_frame);
    }
    if (rc == IGH_ECAN_OK) {
        igh_mcan_port_cycle_end(mcan);
    } else {
        igh_mcan_port_cycle_abort(mcan);
    }
    return rc;
}

static void print_axis_snapshot(const igh_cia402_axis_t *axis)
{
    printf("axis slave=%d ch=%u node=0x%02X heartbeat_valid=%u heartbeat=0x%02X"
           " heartbeat_state=%s ready=%d"
           " status_valid=%u status=0x%04X state=%s actual_valid=%u"
           " actual=%" PRIi32 " target=%" PRIi32 " rpdo_len=%u mode_valid=%u mode=%d"
           " velocity_valid=%u actual_velocity=%" PRIi32 " target_velocity=%" PRIi32
           " last_sdo=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " abort_text=%s"
           " emcy_count=%" PRIu32 " emcy=0x%04X err_reg=0x%02X\n",
           axis->slave_pos,
           axis->channel,
           axis->node_id,
           axis->heartbeat_valid,
           axis->heartbeat,
           igh_cia402_heartbeat_state_name(axis->heartbeat),
           axis_ready_for_motion(axis),
           axis->statusword_valid,
           axis->statusword,
           igh_cia402_statusword_state_name(axis->statusword),
           axis->actual_position_valid,
           axis->actual_position,
           axis->target_position,
           axis->rpdo1_payload_len,
           axis->mode_display_valid,
           axis->mode_display,
           axis->actual_velocity_valid,
           axis->actual_velocity,
           axis->target_velocity,
           axis->sdo_response_index,
           axis->sdo_response_subindex,
           axis->sdo_response_command,
           axis->sdo_abort_code,
           igh_cia402_sdo_abort_name(axis->sdo_abort_code),
           axis->emcy_count,
           axis->emcy_code,
           axis->emcy_error_register);
}

static void print_mcan_port_status(igh_mcan_port_t *mcan, const igh_cia402_axis_t *axis)
{
    igh_mcan_pdo_header_t header;

    if ((mcan == NULL) || (axis == NULL) || axis->slave_pos < 0) {
        return;
    }
    if (igh_mcan_port_get_tpdo_header(mcan, (size_t)axis->slave_pos, &header) != IGH_ECAN_OK) {
        printf("mcan_port slave=%d tpdo_header=unavailable\n", axis->slave_pos);
        return;
    }

    printf("mcan_port slave=%d seq=%u flags=0x%02X channel_mask=0x%02X ch%u_status=0x%04X\n",
           axis->slave_pos,
           header.seq_begin,
           header.flags,
           header.channel_mask,
           axis->channel,
           header.channel_status[axis->channel]);
}

static int run_single_axis_demo(igh_mcan_port_t *mcan,
                                const single_axis_demo_config_t *config)
{
    igh_cia402_axis_t axis;
    igh_cia402_context_t cia;
    igh_cia402_mcan_transport_t transport;
    int result;
    int use_target_sdo;
    int use_csv_sdo;

    if ((mcan == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_cia402_axis_init(&axis,
                     config->slave_pos,
                     config->channel,
                     config->node_id,
                     config->target_step);
    igh_cia402_mcan_transport_init(&transport, mcan);
    igh_cia402_context_init(&cia, &transport.transport, &axis, 1U);
    result = igh_cia402_set_scheduled_tx(&cia,
                                         config->scheduled_target,
                                         config->scheduled_target_offset_ns,
                                         config->scheduled_sync,
                                         config->scheduled_sync_offset_ns);
    if (result != IGH_ECAN_OK) {
        fprintf(stderr, "invalid scheduled CiA402 configuration\n");
        return result;
    }
    use_csv_sdo = config->csv_sdo_mode;
    use_target_sdo = (config->csv_pdo_mode != 0 || use_csv_sdo != 0) ?
                     0 :
                     config->target_via_sdo;

    printf("[diag] single-axis setup slave=%d ch=%u node=0x%02X canfd=%d"
           " canopen_frame=classic-can nominal=%" PRIu32 " data=%" PRIu32 " sdo_timeout=%" PRIu32
           "ms diag_every=%" PRIu32 " pdo_remap=%s target_mode=%s csv_velocity=%" PRId32
           " scheduled_target=%d target_offset_ns=%" PRIu32
           " scheduled_sync=%d sync_offset_ns=%" PRIu32
           " position_start=%s\n",
           config->slave_pos,
           config->channel,
           config->node_id,
           config->canfd,
           config->nominal_bitrate,
           config->data_bitrate,
           config->sdo_timeout_ms,
           config->diag_every,
           use_csv_sdo ? "stored" :
                         (config->csv_pdo_mode ? "csv-rpdo-verify" :
                                  (config->skip_csp_config ? "stored" : "csp-rpdo-verify")),
           use_csv_sdo ? "csv-sdo" :
                         (config->csv_pdo_mode ? "csv-pdo" :
                                                  (use_target_sdo ? "csp-sdo" : "csp-pdo")),
           config->csv_target_velocity,
           config->scheduled_target,
           config->scheduled_target_offset_ns,
           config->scheduled_sync,
           config->scheduled_sync_offset_ns,
           config->allow_missing_position ? "allow-missing" : "require-actual");
    print_axis_snapshot(&axis);

    if (!config->skip_nmt_reset) {
        print_stage("nmt-reset");
        if (send_nmt_broadcast_and_flush(mcan, &cia, &axis, IGH_CIA402_NMT_RESET_NODE,
                                         config->nmt_reset_settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    result = ensure_canopen_mode(mcan, &cia, &axis, config->sdo_timeout_ms);
    if (result != IGH_ECAN_OK) {
        return result;
    }
    print_stage("nmt-start-config");
    if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_START_REMOTE_NODE,
                           config->nmt_start_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    print_stage("observe-node-state");
    if (use_csv_sdo) {
        printf("[diag] CSV-SDO startup sets 6060=9 and drives 60FF target velocity by SDO.\n");
    } else if (config->csv_pdo_mode) {
        printf("[diag] CSV-PDO startup follows field config flow: set 6060=9,"
               " map TPDO1=606C/6041/6061 and RPDO1=6040/6060/60FF,"
               " then send target velocity by RPDO and latch it by SYNC.\n");
    } else if (config->skip_csp_config) {
        printf("[diag] CSP startup uses stored drive PDO mapping; target path=%s.\n",
               use_target_sdo ? "sdo" : "pdo");
    } else {
        printf("[diag] CSP startup follows field config flow: set 6060/60C2,"
               " map TPDO1=6064/6041/6061 and RPDO1=6040/6060/607A,"
               " then require 6064 before motion.\n");
    }
    if (config->verbose) {
        print_axis_snapshot(&axis);
        print_mcan_port_status(mcan, &axis);
    }

    if (!config->skip_csp_config) {
        print_stage(config->csv_pdo_mode ? "configure-csv" : "configure-csp");
        for (unsigned attempt = 0U; attempt <= SINGLE_AXIS_CONFIGURE_RETRIES; attempt++) {
            result = config->csv_pdo_mode ?
                     igh_cia402_configure_csv_rpdo_timeout(&cia,
                                                           &axis,
                                                           config->sdo_timeout_ms) :
                     igh_cia402_configure_csp_rpdo_timeout(&cia,
                                                           &axis,
                                                           config->sdo_timeout_ms);
            if (result == IGH_ECAN_OK) {
                break;
            }
            if (!axis_sdo_needs_retry(result, &axis) ||
                attempt >= SINGLE_AXIS_CONFIGURE_RETRIES) {
                break;
            }
            printf("[diag] %s RPDO configure retry slave=%d ch=%u node=0x%02X"
                   " attempt=%u/%u rc=%d index=0x%04X:%02X cmd=0x%02X"
                   " write_rc=%d read_rc=%d; send NMT start.\n",
                   config->csv_pdo_mode ? "CSV" : "CSP",
                   axis.slave_pos,
                   axis.channel,
                   axis.node_id,
                   attempt + 1U,
                   SINGLE_AXIS_CONFIGURE_RETRIES,
                   result,
                   axis.sdo_response_index,
                   axis.sdo_response_subindex,
                   axis.sdo_response_command,
                   axis.sdo_last_write_rc,
                   axis.sdo_last_read_rc);
            if (send_nmt_and_flush(mcan,
                                   &cia,
                                   IGH_CIA402_NMT_START_REMOTE_NODE,
                                   config->configure_settle_cycles) != IGH_ECAN_OK) {
                return IGH_ECAN_FAIL;
            }
        }
        if (result != IGH_ECAN_OK) {
            fprintf(stderr,
                    "%s RPDO configure failed: slave=%d ch=%u node=0x%02X rc=%d"
                    " index=0x%04X:%02X cmd=0x%02X value=0x%08" PRIX32
                    " write_rc=%d read_rc=%d"
                    " abort=0x%08" PRIX32 " abort_text=%s\n",
                    config->csv_pdo_mode ? "CSV" : "CSP",
                    axis.slave_pos,
                    axis.channel,
                    axis.node_id,
                    result,
                    axis.sdo_response_index,
                    axis.sdo_response_subindex,
                    axis.sdo_response_command,
                    axis.sdo_response_value,
                    axis.sdo_last_write_rc,
                    axis.sdo_last_read_rc,
                    axis.sdo_abort_code,
                    igh_cia402_sdo_abort_name(axis.sdo_abort_code));
            return result;
        }
    } else {
        if (use_target_sdo == 0 && use_csv_sdo == 0) {
            axis.rpdo1_payload_len = IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
        }
        print_stage(use_csv_sdo ? "configure-csv-mode" : "configure-csp-mode");
        result = use_csv_sdo ?
                 set_csv_mode_with_readback(mcan,
                                            &cia,
                                            &axis,
                                            config->sdo_timeout_ms,
                                            config->configure_settle_cycles) :
                 set_csp_mode_with_readback(mcan,
                                            &cia,
                                            &axis,
                                            config->sdo_timeout_ms,
                                            config->configure_settle_cycles);
        if (result != IGH_ECAN_OK) {
            fprintf(stderr,
                    "%s mode set failed: slave=%d ch=%u node=0x%02X rc=%d"
                    " write_rc=%d read_rc=%d abort=0x%08" PRIX32
                    " abort_text=%s index=0x%04X:%02X cmd=0x%02X value=0x%08" PRIX32 "\n",
                    use_csv_sdo ? "CSV" : "CSP",
                    axis.slave_pos,
                    axis.channel,
                    axis.node_id,
                    result,
                    axis.sdo_last_write_rc,
                    axis.sdo_last_read_rc,
                    axis.sdo_abort_code,
                    igh_cia402_sdo_abort_name(axis.sdo_abort_code),
                    axis.sdo_response_index,
                    axis.sdo_response_subindex,
                    axis.sdo_response_command,
                    axis.sdo_response_value);
            return result;
        }
    }
    flush_cycles(mcan, &cia, config->configure_settle_cycles);

    print_stage("nmt-start");
    if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_START_REMOTE_NODE,
                           config->nmt_start_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    if (config->verbose) {
        print_axis_snapshot(&axis);
        print_mcan_port_status(mcan, &axis);
    }

    print_stage("nmt-start-enable");
    if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_START_REMOTE_NODE, 5U) !=
        IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    if (igh_cia402_wait_heartbeat_timeout(&cia,
                                           &axis,
                                           IGH_CIA402_HEARTBEAT_OPERATIONAL,
                                           config->sdo_timeout_ms) != IGH_ECAN_OK) {
        printf("[diag] heartbeat operational not observed before enable; continue with SDO status checks.\n");
    }

    print_stage("enable");
    result = igh_cia402_enable_timeout(&cia, &axis, config->sdo_timeout_ms);
    if (result != IGH_ECAN_OK) {
        if (!config->skip_csp_config && result == IGH_ECAN_TIMEOUT) {
            printf("[diag] enable status/ack not observed after field config; continue to 0x6064 gate before motion.\n");
        } else {
            fprintf(stderr,
                    "enable failed: slave=%d ch=%u node=0x%02X rc=%d status_valid=%u"
                    " status=0x%04X state=%s abort=0x%08" PRIX32 " abort_text=%s\n",
                    axis.slave_pos,
                    axis.channel,
                    axis.node_id,
                    result,
                    axis.statusword_valid,
                    axis.statusword,
                    igh_cia402_statusword_state_name(axis.statusword),
                    axis.sdo_abort_code,
                    igh_cia402_sdo_abort_name(axis.sdo_abort_code));
            print_axis_snapshot(&axis);
            return result;
        }
    }
    flush_cycles(mcan, &cia, config->enable_settle_cycles);

    if (config->csv_pdo_mode || use_csv_sdo) {
        print_stage(use_csv_sdo ? "csv-sdo-ready" : "csv-ready");
        flush_cycles(mcan, &cia, config->read_position_settle_cycles);
        print_stage("read-position-start");
        (void)igh_cia402_request_actual_position_timeout(
            &cia,
            &axis,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
    } else {
        print_stage("read-position");
        result = igh_cia402_request_actual_position_timeout(
            &cia,
            &axis,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
        if (result != IGH_ECAN_OK) {
            fprintf(stderr,
                    config->allow_missing_position ?
                    "[warn] node 0x%02X did not answer 0x6064 position request on slave=%d ch=%u; continue with target=0 because --allow-missing-position was set.\n" :
                    "[error] node 0x%02X did not answer 0x6064 position request on slave=%d ch=%u after enable; abort motion because target must start from actual position.\n",
                    axis.node_id,
                    axis.slave_pos,
                    axis.channel);
            print_axis_snapshot(&axis);
            print_mcan_port_status(mcan, &axis);
            if (!config->allow_missing_position) {
                (void)igh_cia402_stop(&cia, &axis);
                return result;
            }
            axis.actual_position = 0;
            axis.actual_position_valid = 1U;
            axis.target_position = 0;
        }
        flush_cycles(mcan, &cia, config->read_position_settle_cycles);
    }

    print_axis_snapshot(&axis);
    print_stage("motion");
    if (use_csv_sdo) {
        result = write_target_velocity_sdo_retry(mcan,
                                                 &cia,
                                                 &axis,
                                                 config->csv_target_velocity,
                                                 config->sdo_timeout_ms);
        if (result != IGH_ECAN_OK) {
            fprintf(stderr,
                    "CSV target velocity SDO write failed rc=%d velocity=%" PRId32 "\n",
                    result,
                    config->csv_target_velocity);
            return result;
        }
        printf("[diag] CSV-SDO velocity active velocity=%" PRId32 " cycles=%u\n",
               config->csv_target_velocity,
               config->motion_cycles);
    }
    for (unsigned loop = 0U; loop < config->motion_cycles; loop++) {
        igh_mcan_rx_frame_t frames[SINGLE_AXIS_RX_CAPACITY];
        size_t frame_count = 0U;
        int motion_rc;

        if (use_csv_sdo != 0) {
            igh_mcan_port_cycle_begin(mcan);
            (void)igh_mcan_port_receive(mcan, frames, SINGLE_AXIS_RX_CAPACITY, &frame_count);
            (void)process_mcan_frames(&cia, frames, frame_count);
            igh_mcan_port_cycle_end(mcan);
            motion_rc = IGH_ECAN_OK;
        } else if (config->csv_pdo_mode != 0) {
            motion_rc = send_csv_velocity_pdo_cycle(mcan,
                                                    &cia,
                                                    &axis,
                                                    config->csv_target_velocity);
        } else if (use_target_sdo != 0) {
            igh_cia402_target_step_t target_step;
            igh_cia402_tx_frame_t sync_frame;

            motion_rc = igh_cia402_prepare_target_step(&axis, &target_step);
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = write_target_position_sdo_retry(mcan,
                                                            &cia,
                                                            &axis,
                                                            target_step.next_position,
                                                            config->sdo_timeout_ms);
            }
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = igh_cia402_set_target_position(&axis, target_step.next_position);
            }
            if (motion_rc == IGH_ECAN_OK) {
                igh_mcan_port_cycle_begin(mcan);
                (void)igh_mcan_port_receive(mcan, frames,
                                            SINGLE_AXIS_RX_CAPACITY, &frame_count);
                (void)process_mcan_frames(&cia, frames, frame_count);
                motion_rc = igh_cia402_build_sync_frame(&cia,
                                                        axis.slave_pos,
                                                        axis.channel,
                                                        &sync_frame);
                if (motion_rc == IGH_ECAN_OK) {
                    motion_rc = igh_cia402_mcan_send_frame(mcan, &sync_frame);
                }
                igh_mcan_port_cycle_end(mcan);
            }
        } else {
            igh_cia402_target_step_t target_step;
            igh_cia402_tx_frame_t target_frame;
            igh_cia402_tx_frame_t sync_frame;

            igh_mcan_port_cycle_begin(mcan);
            (void)igh_mcan_port_receive(mcan, frames, SINGLE_AXIS_RX_CAPACITY, &frame_count);
            (void)process_mcan_frames(&cia, frames, frame_count);
            motion_rc = igh_cia402_prepare_target_step(&axis, &target_step);
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = igh_cia402_build_prepared_target_frame(&cia,
                                                                   &axis,
                                                                   &target_step,
                                                                   &target_frame);
            }
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = igh_cia402_mcan_send_frame(mcan, &target_frame);
            }
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = igh_cia402_set_target_position(&axis,
                                                           target_step.next_position);
            }
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = igh_cia402_build_sync_frame(&cia,
                                                        axis.slave_pos,
                                                        axis.channel,
                                                        &sync_frame);
            }
            if (motion_rc == IGH_ECAN_OK) {
                motion_rc = igh_cia402_mcan_send_frame(mcan, &sync_frame);
            }
            igh_mcan_port_cycle_end(mcan);
        }
        if (motion_rc != IGH_ECAN_OK) {
            fprintf(stderr, "motion send failed cycle=%u rc=%d ready=%d\n",
                    loop + 1U,
                    motion_rc,
                    axis_ready_for_motion(&axis));
            print_axis_snapshot(&axis);
            print_mcan_port_status(mcan, &axis);
            return motion_rc;
        }
        if ((config->diag_every != 0U) && (((loop + 1U) % config->diag_every) == 0U)) {
            if (use_csv_sdo != 0 || config->csv_pdo_mode != 0) {
                printf("[diag] %s cycle=%u/%u ready=%d velocity=%" PRId32
                       " actual_velocity_valid=%u actual_velocity=%" PRId32 "\n",
                       use_csv_sdo ? "csv-sdo-motion" : "csv-pdo-motion",
                       loop + 1U,
                       config->motion_cycles,
                       axis_ready_for_motion(&axis),
                       axis.target_velocity,
                       axis.actual_velocity_valid,
                       axis.actual_velocity);
            } else {
                printf("[diag] motion cycle=%u/%u ready=%d target=%" PRIi32 "\n",
                       loop + 1U,
                       config->motion_cycles,
                       axis_ready_for_motion(&axis),
                       axis.target_position);
            }
        }
        usleep(mcan->igh->cycle_us);
    }

    if (config->csv_pdo_mode != 0 || use_csv_sdo != 0) {
        uint32_t velocity = 0U;
        uint8_t size = 0U;
        print_stage("read-velocity-final");
        result = igh_cia402_sdo_upload_u32_timeout(&cia,
                                                   &axis,
                                                   IGH_CIA402_IDX_VELOCITY_ACTUAL_VALUE,
                                                   IGH_CIA402_SUBIDX_VALUE,
                                                   &velocity,
                                                   &size,
                                                   cycles_to_timeout_ms(
                                                       mcan,
                                                       config->read_position_settle_cycles));
        if (result == IGH_ECAN_OK && size >= 4U) {
            axis.actual_velocity = (int32_t)velocity;
            axis.actual_velocity_valid = 1U;
        }
        print_stage("zero-velocity");
        if (use_csv_sdo != 0) {
            (void)write_target_velocity_sdo_retry(mcan,
                                                  &cia,
                                                  &axis,
                                                  0,
                                                  config->sdo_timeout_ms);
            flush_cycles(mcan, &cia, 3U);
        } else {
            for (unsigned i = 0U; i < 3U; i++) {
                (void)send_csv_velocity_pdo_cycle(mcan, &cia, &axis, 0);
                usleep(mcan->igh->cycle_us);
            }
            flush_cycles(mcan, &cia, 3U);
        }
        print_stage("read-position-final");
        (void)igh_cia402_request_actual_position_timeout(
            &cia,
            &axis,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
    } else {
        (void)igh_cia402_request_actual_position_timeout(
            &cia,
            &axis,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
    }
    print_axis_snapshot(&axis);

    print_stage("stop");
    if (stop_and_flush(mcan, &cia, config->stop_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    print_axis_snapshot(&axis);
    return IGH_ECAN_OK;
}

int main(int argc, char **argv)
{
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    single_axis_demo_config_t demo_config;
    int result;

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    single_axis_default_config(&demo_config);
    if (ecan_igh_config_apply_realtime_profile(
            &igh_config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&igh_config, SINGLE_AXIS_DEFAULT_CYCLE_US) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(&igh_config, SINGLE_AXIS_DEFAULT_PDO_MULTIPLIER) !=
            IGH_ECAN_OK ||
        parse_args(argc, argv, &igh_config, &mcan_config, &demo_config) != IGH_ECAN_OK ||
        apply_mcan_port_config(&igh_config, &mcan_config, &demo_config) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    if (!demo_config.enable_motor) {
        fprintf(stderr, "Motion disabled: explicit --enable-motor is required; no master requested.\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    printf("ECAN IgH single-axis demo: slave=%d ch=%u node=0x%02X bitrate=%" PRIu32
           " pdo=%u bytes cycle=%" PRIu32 " us cycles=%u step=%" PRIi32
           " warmup=%" PRIu32 "/%" PRIu32 "\n",
           demo_config.slave_pos,
           demo_config.channel,
           demo_config.node_id,
           demo_config.nominal_bitrate,
           (unsigned)ecan_igh_config_get_pdo_bytes(&igh_config),
           igh_config.cycle_us,
           demo_config.motion_cycles,
           demo_config.target_step,
           demo_config.warmup_cycles,
           demo_config.warmup_stable_cycles);

    if (ecan_igh_init(&igh, &igh_config) != IGH_ECAN_OK) {
        fprintf(stderr, "IgH init failed\n");
        return IGH_ECAN_EXIT_FAIL;
    }
    if (igh_mcan_port_init(&mcan, &igh, &mcan_config) != IGH_ECAN_OK) {
        fprintf(stderr, "MCAN port init failed\n");
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    if (ecan_igh_wait_op_timeout(&igh, demo_config.wait_op_timeout_ms) != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed\n");
        if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
            ecan_igh_print_runtime_diagnostics(&igh, 16U);
        }
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    if (warmup_mcan_port_after_op(&mcan,
                                  demo_config.warmup_cycles,
                                  demo_config.warmup_stable_cycles,
                                  "single-axis") != IGH_ECAN_OK) {
        if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
            ecan_igh_print_runtime_diagnostics(&igh, 16U);
        }
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    result = run_single_axis_demo(&mcan, &demo_config);
    if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
        ecan_igh_print_runtime_diagnostics(&igh, 16U);
    }
    ecan_igh_close(&igh);
    return (result == IGH_ECAN_OK) ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_FAIL;
}
