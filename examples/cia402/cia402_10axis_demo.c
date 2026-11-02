#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <inttypes.h>
#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "cia402_demo_common.h"
#include "cia402_mcan_transport.h"
#include "igh_cia402_protocol.h"
#include "igh_ecan_common.h"
#include "igh_adapter.h"
#include "igh_mcan_port.h"

#define CIA402_10AXIS_DEMO_AXIS_MAX         12U
#define CIA402_10AXIS_DEMO_DEFAULT_STEP     1000
#define CIA402_10AXIS_DEMO_RX_CAPACITY      128U
#define CIA402_10AXIS_DEMO_DEFAULT_WAIT_OP_MS 10000U
#define CIA402_10AXIS_DEMO_DEFAULT_SDO_TIMEOUT_MS 3000U
#define CIA402_10AXIS_DEMO_DEFAULT_DIAG_EVERY 1000U
#define CIA402_10AXIS_DEMO_DEFAULT_GUARD_MAX_MISSES 100U
#define CIA402_10AXIS_DEMO_DEFAULT_MOTION_DIVISOR 1U
#define CIA402_10AXIS_DEMO_DEFAULT_PDO_MULTIPLIER 4U
#define CIA402_10AXIS_DEMO_THIRD_SLAVE_PDO_MULTIPLIER 2U
#define CIA402_10AXIS_DEMO_STARTUP_TRANSACTION_RETRIES 3U
#define CIA402_10AXIS_DEMO_COMM_MODE_RETRIES 10U
#define CIA402_10AXIS_DEMO_CONFIGURE_RETRIES 8U

typedef struct {
    unsigned motion_cycles;
    unsigned nmt_reset_settle_cycles;
    unsigned nmt_start_settle_cycles;
    unsigned configure_settle_cycles;
    unsigned read_position_settle_cycles;
    unsigned enable_settle_cycles;
    unsigned stop_settle_cycles;
    int32_t target_step;
    uint32_t wait_op_timeout_ms;
    uint32_t warmup_cycles;
    uint32_t warmup_stable_cycles;
    uint32_t sdo_timeout_ms;
    uint32_t diag_every;
    uint32_t ethercat_guard_max_misses;
    uint32_t motion_divisor;
    int ethercat_guard_fatal;
    int verbose;
    int enable_motor;
    int skip_nmt_reset;
    int skip_csp_config;
    int stop_on_emcy;
    int lock_memory;
    int traffic_fill_mode;
    int csv_sdo_mode;
    int csv_pdo_mode;
    uint8_t csv_operation_mode;
    int32_t csv_target_velocity;
    uint32_t csv_refresh_cycles;
    int scheduled_target;
    int scheduled_sync;
    uint32_t scheduled_target_offset_ns;
    uint32_t scheduled_sync_offset_ns;
    int only_axis;
    uint32_t axis_mask;
    size_t axis_slave_pos;
    igh_ecan_mcan_channel_t axis_channel;
    uint8_t axis_node_id;
    igh_ecan_canfd_mode_t axis_canfd;
    uint32_t axis_nominal_bitrate;
    uint32_t axis_data_bitrate;
    int slot_canfd_override_valid;
    igh_ecan_canfd_mode_t slot_canfd_override;
    uint32_t slot_nominal_bitrate;
    uint32_t slot_data_bitrate;
} cia402_10axis_demo_config_t;

typedef struct {
    uint8_t sent;
    uint8_t data_len;
    int32_t target_position;
    uint8_t data[IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN];
} cia402_target_trace_t;

typedef struct {
    struct timespec next;
    uint32_t cycle_us;
} cia402_cycle_pacer_t;

typedef enum {
    CIA402_SDO_WAIT_UPLOAD,
    CIA402_SDO_WAIT_DOWNLOAD_ACK
} cia402_sdo_wait_kind_t;

/* 修改这里即可调整 10 轴拓扑：单轴 demo 使用同一结构，只保留 1 条 slot。 */
static const igh_cia402_demo_motor_slot_t k_motor_slots[] = {
    {IGH_ECAN_ENABLE, 0U, IGH_ECAN_MCAN_CH0, 0x09U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 0U, IGH_ECAN_MCAN_CH1, 0x0AU, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 0U, IGH_ECAN_MCAN_CH2, 0x08U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 0U, IGH_ECAN_MCAN_CH3, 0x02U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 1U, IGH_ECAN_MCAN_CH0, 0x06U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 1U, IGH_ECAN_MCAN_CH1, 0x07U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 1U, IGH_ECAN_MCAN_CH2, 0x04U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 1U, IGH_ECAN_MCAN_CH3, 0x03U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_ENABLE, 2U, IGH_ECAN_MCAN_CH0, 0x05U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
    {IGH_ECAN_DISABLE, 2U, IGH_ECAN_MCAN_CH1, 0x01U, IGH_CIA402_DEMO_DEFAULT_CANFD,
     IGH_ECAN_DEFAULT_CAN_BITRATE, IGH_ECAN_DEFAULT_CANFD_BITRATE},
};

static void demo_default_config(cia402_10axis_demo_config_t *config)
{
    if (config == NULL) {
        return;
    }
    memset(config, 0, sizeof(*config));
    config->motion_cycles = IGH_ECAN_DEFAULT_PROFILE_CYCLES;
    config->nmt_reset_settle_cycles = 500U;
    config->nmt_start_settle_cycles = 150U;
    config->configure_settle_cycles = 50U;
    config->read_position_settle_cycles = 150U;
    config->enable_settle_cycles = 150U;
    config->stop_settle_cycles = 20U;
    config->target_step = CIA402_10AXIS_DEMO_DEFAULT_STEP;
    config->wait_op_timeout_ms = CIA402_10AXIS_DEMO_DEFAULT_WAIT_OP_MS;
    config->warmup_cycles = 32U;
    config->warmup_stable_cycles = 4U;
    config->sdo_timeout_ms = 5000U;
    config->diag_every = CIA402_10AXIS_DEMO_DEFAULT_DIAG_EVERY;
    config->ethercat_guard_max_misses = CIA402_10AXIS_DEMO_DEFAULT_GUARD_MAX_MISSES;
    config->motion_divisor = CIA402_10AXIS_DEMO_DEFAULT_MOTION_DIVISOR;
    config->ethercat_guard_fatal = 0;
    config->skip_nmt_reset = 1;
    config->skip_csp_config = 1;
    config->verbose = 0;
    config->lock_memory = 1;
    config->traffic_fill_mode = 0;
    config->csv_sdo_mode = 0;
    config->csv_pdo_mode = 0;
    config->csv_operation_mode = IGH_CIA402_MODE_CSV;
    config->csv_target_velocity = 50;
    config->csv_refresh_cycles = 100U;
    config->scheduled_target_offset_ns = 100000U;
    config->scheduled_sync_offset_ns = 200000U;
    config->only_axis = 0;
    config->axis_mask = 0U;
    config->axis_slave_pos = 0U;
    config->axis_channel = IGH_ECAN_MCAN_CH0;
    config->axis_node_id = 1U;
    config->axis_canfd = IGH_ECAN_CANFD_DISABLE;
    config->axis_nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    config->axis_data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    config->slot_canfd_override_valid = 0;
    config->slot_canfd_override = IGH_CIA402_DEMO_DEFAULT_CANFD;
    config->slot_nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    config->slot_data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
}

static int lock_memory_for_realtime(void)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "mlockall failed: errno=%d (%s)\n", errno, strerror(errno));
        return IGH_ECAN_FAIL;
    }
    printf("[diag] memory locked with mlockall(MCL_CURRENT|MCL_FUTURE)\n");
    return IGH_ECAN_OK;
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
        igh_mcan_rx_frame_t rx[CIA402_10AXIS_DEMO_RX_CAPACITY];
        int rc = igh_mcan_port_warmup_cycle_checked(mcan,
                                                    rx,
                                                    CIA402_10AXIS_DEMO_RX_CAPACITY,
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

static void build_axes(igh_cia402_axis_t *axes,
                       size_t *axis_count,
                       int32_t step,
                       const cia402_10axis_demo_config_t *config)
{
    if ((axes == NULL) || (axis_count == NULL)) {
        return;
    }

    *axis_count = 0U;
    if ((config != NULL) && (config->only_axis != 0)) {
        igh_cia402_axis_init(&axes[0],
                             (int)config->axis_slave_pos,
                             (uint8_t)config->axis_channel,
                             config->axis_node_id,
                             step);
        *axis_count = 1U;
        if (config->traffic_fill_mode == 0) {
            return;
        }
    }

    for (size_t i = 0U; i < IGH_ECAN_ARRAY_SIZE(k_motor_slots); i++) {
        const igh_cia402_demo_motor_slot_t *slot = &k_motor_slots[i];
        int duplicate_runtime_axis = ((config != NULL) &&
                                      (config->only_axis != 0) &&
                                      slot->slave_pos == config->axis_slave_pos &&
                                      slot->mcan_channel == config->axis_channel &&
                                      slot->node_id == config->axis_node_id);
        if (slot->enable != IGH_ECAN_ENABLE ||
            duplicate_runtime_axis ||
            ((config != NULL) && config->axis_mask != 0U &&
             (i >= 32U || (config->axis_mask & (1UL << i)) == 0U)) ||
            *axis_count >= CIA402_10AXIS_DEMO_AXIS_MAX) {
            continue;
        }
        igh_cia402_axis_init(&axes[*axis_count],
                             (int)slot->slave_pos,
                             (uint8_t)slot->mcan_channel,
                             slot->node_id,
                             step);
        (*axis_count)++;
    }
}

static size_t count_selected_motor_slots(const cia402_10axis_demo_config_t *config)
{
    size_t count = 0U;

    for (size_t i = 0U; i < IGH_ECAN_ARRAY_SIZE(k_motor_slots); i++) {
        const igh_cia402_demo_motor_slot_t *slot = &k_motor_slots[i];
        if (slot->enable != IGH_ECAN_ENABLE ||
            ((config != NULL) && config->axis_mask != 0U &&
             (i >= 32U || (config->axis_mask & (1UL << i)) == 0U))) {
            continue;
        }
        count++;
    }
    return count;
}

static igh_ecan_canfd_mode_t get_slot_canfd(const cia402_10axis_demo_config_t *config,
                                            const igh_cia402_demo_motor_slot_t *slot)
{
    if ((config != NULL) && config->slot_canfd_override_valid != 0) {
        return config->slot_canfd_override;
    }
    return slot->canfd;
}

static uint32_t get_slot_nominal_bitrate(const cia402_10axis_demo_config_t *config,
                                         const igh_cia402_demo_motor_slot_t *slot)
{
    if ((config != NULL) && config->slot_canfd_override_valid != 0) {
        return config->slot_nominal_bitrate;
    }
    return slot->nominal_bitrate;
}

static uint32_t get_slot_data_bitrate(const cia402_10axis_demo_config_t *config,
                                      const igh_cia402_demo_motor_slot_t *slot)
{
    if ((config != NULL) && config->slot_canfd_override_valid != 0) {
        return config->slot_data_bitrate;
    }
    return slot->data_bitrate;
}

static int apply_10axis_mcan_port_config(ecan_igh_config_t *igh_config,
                                         igh_mcan_port_config_t *mcan_config,
                                         const cia402_10axis_demo_config_t *demo_config)
{
    uint8_t channel_masks[ECAN_IGH_MAX_SLAVES] = {0};
    uint8_t canfd_masks[ECAN_IGH_MAX_SLAVES] = {0};
    uint32_t nominal_bitrates[ECAN_IGH_MAX_SLAVES];
    uint32_t data_bitrates[ECAN_IGH_MAX_SLAVES];

    if (igh_config == NULL || mcan_config == NULL ||
        igh_config->slave_count == 0U || igh_config->slave_count > ECAN_IGH_MAX_SLAVES) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t slave = 0U; slave < ECAN_IGH_MAX_SLAVES; slave++) {
        nominal_bitrates[slave] = IGH_ECAN_DEFAULT_CAN_BITRATE;
        data_bitrates[slave] = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    }

    if ((demo_config != NULL) && (demo_config->only_axis != 0) &&
        demo_config->traffic_fill_mode == 0) {
        uint8_t channel = (uint8_t)demo_config->axis_channel;
        uint8_t channel_bit = IGH_ECAN_BIT_U8(channel);

        if (demo_config->axis_slave_pos >= igh_config->slave_count ||
            channel >= IGH_MCAN_CHANNEL_COUNT ||
            demo_config->axis_node_id == 0U ||
            demo_config->axis_node_id > 0x7FU) {
            return IGH_ECAN_INVALID_ARG;
        }
        channel_masks[demo_config->axis_slave_pos] = channel_bit;
        canfd_masks[demo_config->axis_slave_pos] =
            (demo_config->axis_canfd == IGH_ECAN_CANFD_ENABLE) ? channel_bit : 0U;
        nominal_bitrates[demo_config->axis_slave_pos] = demo_config->axis_nominal_bitrate;
        data_bitrates[demo_config->axis_slave_pos] = demo_config->axis_data_bitrate;
    } else {
        if ((demo_config != NULL) && (demo_config->only_axis != 0) &&
            demo_config->traffic_fill_mode != 0) {
            uint8_t channel = (uint8_t)demo_config->axis_channel;
            uint8_t channel_bit = IGH_ECAN_BIT_U8(channel);

            if (demo_config->axis_slave_pos >= igh_config->slave_count ||
                channel >= IGH_MCAN_CHANNEL_COUNT ||
                demo_config->axis_node_id == 0U ||
                demo_config->axis_node_id > 0x7FU) {
                return IGH_ECAN_INVALID_ARG;
            }
            channel_masks[demo_config->axis_slave_pos] |= channel_bit;
            if (demo_config->axis_canfd == IGH_ECAN_CANFD_ENABLE) {
                canfd_masks[demo_config->axis_slave_pos] |= channel_bit;
            }
            nominal_bitrates[demo_config->axis_slave_pos] = demo_config->axis_nominal_bitrate;
            data_bitrates[demo_config->axis_slave_pos] = demo_config->axis_data_bitrate;
        }
        for (size_t i = 0U; i < IGH_ECAN_ARRAY_SIZE(k_motor_slots); i++) {
            const igh_cia402_demo_motor_slot_t *slot = &k_motor_slots[i];
            uint8_t channel = (uint8_t)slot->mcan_channel;
            uint8_t channel_bit = IGH_ECAN_BIT_U8(channel);

            if (slot->enable != IGH_ECAN_ENABLE ||
                (demo_config != NULL && demo_config->axis_mask != 0U &&
                 (i >= 32U || (demo_config->axis_mask & (1UL << i)) == 0U))) {
                continue;
            }
            if (slot->slave_pos >= igh_config->slave_count ||
                channel >= IGH_MCAN_CHANNEL_COUNT ||
                slot->node_id == 0U ||
                slot->node_id > 0x7FU) {
                return IGH_ECAN_INVALID_ARG;
            }

            channel_masks[slot->slave_pos] |= channel_bit;
            if (get_slot_canfd(demo_config, slot) == IGH_ECAN_CANFD_ENABLE) {
                canfd_masks[slot->slave_pos] |= channel_bit;
            }
            if (channel_masks[slot->slave_pos] == channel_bit) {
                nominal_bitrates[slot->slave_pos] = get_slot_nominal_bitrate(demo_config, slot);
                data_bitrates[slot->slave_pos] = get_slot_data_bitrate(demo_config, slot);
            }
        }
    }

    for (size_t slave = 0U; slave < igh_config->slave_count; slave++) {
        if (channel_masks[slave] == 0U) {
            channel_masks[slave] = IGH_ECAN_BIT_U8(0U);
        }
        if (igh_mcan_port_set_channel_mask(mcan_config, slave, channel_masks[slave],
                                             canfd_masks[slave]) != IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(mcan_config, slave, nominal_bitrates[slave],
                                        data_bitrates[slave]) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    return igh_mcan_port_apply_config(igh_config, mcan_config);
}

static void disable_axes_without_slave(igh_cia402_axis_t *axes, size_t axis_count, size_t slave_count)
{
    if (axes == NULL) {
        return;
    }

    for (size_t i = 0U; i < axis_count; i++) {
        if (axes[i].slave_pos < 0 || (size_t)axes[i].slave_pos >= slave_count) {
            axes[i].enabled = 0;
        }
    }
}

static size_t count_enabled_axes(const igh_cia402_axis_t *axes, size_t axis_count)
{
    size_t count = 0U;

    for (size_t i = 0U; i < axis_count; i++) {
        if (axes[i].enabled) {
            count++;
        }
    }
    return count;
}

static void prime_traffic_fill_axes_mode(igh_cia402_axis_t *axes,
                                         size_t axis_count,
                                         uint8_t mode,
                                         int32_t target_velocity)
{
    if (axes == NULL) {
        return;
    }
    for (size_t i = 0U; i < axis_count; i++) {
        igh_cia402_axis_t *axis = &axes[i];

        if (!axis->enabled) {
            continue;
        }
        axis->heartbeat = IGH_CIA402_HEARTBEAT_OPERATIONAL;
        axis->heartbeat_valid = 1U;
        axis->statusword = IGH_CIA402_STATUSWORD_OPERATION_ENABLED;
        axis->statusword_valid = 1U;
        axis->actual_position = (int32_t)(i * 1000);
        axis->target_position = axis->actual_position;
        axis->actual_position_valid = 1U;
        axis->target_velocity = target_velocity;
        axis->actual_velocity = 0;
        axis->actual_velocity_valid = 1U;
        axis->rpdo1_payload_len = (mode == IGH_CIA402_MODE_CSV) ?
                                  IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN :
                                  IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
        axis->mode_display = (int8_t)mode;
        axis->mode_display_valid = 1U;
    }
}

static void prime_traffic_fill_axes(igh_cia402_axis_t *axes, size_t axis_count)
{
    prime_traffic_fill_axes_mode(axes, axis_count, IGH_CIA402_MODE_CSP, 0);
}

static uint16_t demo_get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static void demo_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8);
}

static void demo_put_i32_le(uint8_t *dst, int32_t value)
{
    uint32_t raw = (uint32_t)value;

    dst[0] = (uint8_t)(raw & 0xFFU);
    dst[1] = (uint8_t)((raw >> 8) & 0xFFU);
    dst[2] = (uint8_t)((raw >> 16) & 0xFFU);
    dst[3] = (uint8_t)((raw >> 24) & 0xFFU);
}

static int32_t demo_get_i32_le(const uint8_t *src)
{
    uint32_t value = (uint32_t)src[0] |
                     ((uint32_t)src[1] << 8) |
                     ((uint32_t)src[2] << 16) |
                     ((uint32_t)src[3] << 24);

    return (int32_t)value;
}

static size_t demo_tsf_len(uint8_t tsf)
{
    switch (tsf) {
    case IGH_MCAN_TSF_DELTA16:
        return 2U;
    case IGH_MCAN_TSF_DELTA32:
        return 4U;
    case IGH_MCAN_TSF_FULL64:
        return 8U;
    case IGH_MCAN_TSF_NONE:
    default:
        return 0U;
    }
}

static size_t demo_align_packet_len(const igh_mcan_port_t *mcan, size_t raw_len)
{
    if ((mcan != NULL) && ((mcan->config.option_flags & IGH_MCAN_OPT_ALIGN4) != 0U)) {
        return (raw_len + 3U) & ~(size_t)3U;
    }
    return raw_len;
}

static void print_target_trace_error(const char *reason,
                                     unsigned cycle,
                                     const igh_cia402_axis_t *axis,
                                     const cia402_target_trace_t *trace,
                                     int slave_pos,
                                     size_t packet_offset,
                                     uint16_t ctrl,
                                     const uint8_t *data,
                                     size_t data_len)
{
    fprintf(stderr,
            "[fatal] target-pdo-check cycle=%u reason=%s axis_slave=%d axis_ch=%u node=0x%02X"
            " packet_slave=%d packet_offset=%zu ctrl=0x%04X expected=%" PRId32
            " expected_raw=%02X %02X %02X %02X %02X %02X %02X",
            cycle,
            reason,
            axis != NULL ? axis->slave_pos : -1,
            axis != NULL ? axis->channel : 0U,
            axis != NULL ? axis->node_id : 0U,
            slave_pos,
            packet_offset,
            ctrl,
            trace != NULL ? trace->target_position : (int32_t)0,
            trace != NULL ? trace->data[0] : 0U,
            trace != NULL ? trace->data[1] : 0U,
            trace != NULL ? trace->data[2] : 0U,
            trace != NULL ? trace->data[3] : 0U,
            trace != NULL ? trace->data[4] : 0U,
            trace != NULL ? trace->data[5] : 0U,
            trace != NULL ? trace->data[6] : 0U);
    if ((data != NULL) && (trace != NULL) && (data_len >= trace->data_len)) {
        const uint8_t pos_offset =
            (trace->data_len == IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN) ? 3U : 0U;

        fprintf(stderr,
                " got=%" PRId32 " got_raw=%02X %02X %02X %02X %02X %02X %02X\n",
                demo_get_i32_le(&data[pos_offset]),
                data[0],
                data[1],
                data[2],
                data[3],
                data[4],
                data[5],
                data[6]);
    } else {
        fprintf(stderr, " got_len=%zu\n", data_len);
    }
}

static int verify_target_pdo_trace(const igh_mcan_port_t *mcan,
                                   const igh_cia402_axis_t *axes,
                                   size_t axis_count,
                                   const cia402_target_trace_t *trace,
                                   size_t trace_count,
                                   unsigned cycle)
{
    uint8_t found[CIA402_10AXIS_DEMO_AXIS_MAX] = {0};

    if ((mcan == NULL) || (mcan->igh == NULL) || (axes == NULL) || (trace == NULL) ||
        axis_count > CIA402_10AXIS_DEMO_AXIS_MAX || trace_count < axis_count) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t slave = 0U; slave < mcan->igh->slave_count; slave++) {
        const igh_mcan_pdo_writer_t *writer = &mcan->writers[slave];
        size_t offset = writer->header_bytes;
        size_t end;

        if (writer->data == NULL) {
            continue;
        }
        end = offset + writer->frame_bytes;
        if (end > writer->size) {
            fprintf(stderr,
                    "[fatal] target-pdo-check cycle=%u reason=writer-overrun slave=%zu"
                    " frame_bytes=%u pdo_size=%zu\n",
                    cycle,
                    slave,
                    writer->frame_bytes,
                    writer->size);
            return IGH_ECAN_FAIL;
        }

        while (offset < end) {
            size_t packet_offset = offset;
            size_t id_len;
            size_t data_len;
            size_t data_window_len;
            size_t raw_len;
            size_t packet_len;
            size_t packet_end;
            size_t data_offset;
            uint16_t ctrl;
            uint8_t ch;
            uint8_t dlc;
            uint8_t tsf;
            uint8_t kind;
            int ide;
            int fdf;
            int rtr;
            uint32_t can_id;

            if (offset + 2U > end) {
                fprintf(stderr,
                        "[fatal] target-pdo-check cycle=%u reason=short-ctrl slave=%zu"
                        " offset=%zu end=%zu\n",
                        cycle,
                        slave,
                        offset,
                        end);
                return IGH_ECAN_FAIL;
            }

            ctrl = demo_get_u16_le(&writer->data[offset]);
            ch = (uint8_t)(ctrl & IGH_MCAN_CTRL_CH_MASK);
            dlc = (uint8_t)((ctrl & IGH_MCAN_CTRL_DLC_MASK) >> IGH_MCAN_CTRL_DLC_SHIFT);
            tsf = (uint8_t)((ctrl & IGH_MCAN_CTRL_TSF_MASK) >> IGH_MCAN_CTRL_TSF_SHIFT);
            kind = (uint8_t)((ctrl & IGH_MCAN_CTRL_KIND_MASK) >> IGH_MCAN_CTRL_KIND_SHIFT);
            ide = ((ctrl & IGH_MCAN_CTRL_IDE_MASK) != 0U);
            fdf = ((ctrl & IGH_MCAN_CTRL_FDF_MASK) != 0U);
            rtr = (!fdf && ((ctrl & IGH_MCAN_CTRL_X_MASK) != 0U));

            if (kind != IGH_MCAN_KIND_CAN_FRAME) {
                fprintf(stderr,
                        "[fatal] target-pdo-check cycle=%u reason=bad-kind slave=%zu"
                        " offset=%zu ctrl=0x%04X\n",
                        cycle,
                        slave,
                        packet_offset,
                        ctrl);
                return IGH_ECAN_FAIL;
            }

            id_len = ide ? 4U : 2U;
            data_len = rtr ? 0U : igh_mcan_dlc_to_len(dlc);
            data_window_len = ((ctrl & IGH_MCAN_CTRL_D64_MASK) != 0U) ? 64U : data_len;
            raw_len = 2U + id_len + demo_tsf_len(tsf) + data_window_len;
            packet_len = demo_align_packet_len(mcan, raw_len);
            packet_end = packet_offset + packet_len;
            if (packet_end > end) {
                fprintf(stderr,
                        "[fatal] target-pdo-check cycle=%u reason=truncated slave=%zu"
                        " offset=%zu packet_end=%zu end=%zu ctrl=0x%04X\n",
                        cycle,
                        slave,
                        packet_offset,
                        packet_end,
                        end,
                        ctrl);
                return IGH_ECAN_FAIL;
            }

            offset += 2U;
            if (ide) {
                can_id = (uint32_t)writer->data[offset] |
                         ((uint32_t)writer->data[offset + 1U] << 8) |
                         ((uint32_t)writer->data[offset + 2U] << 16) |
                         ((uint32_t)writer->data[offset + 3U] << 24);
                can_id &= IGH_ECAN_CAN_EXT_ID_MAX;
                offset += 4U;
            } else {
                can_id = demo_get_u16_le(&writer->data[offset]) & IGH_ECAN_CAN_STD_ID_MAX;
                offset += 2U;
            }
            offset += demo_tsf_len(tsf);
            data_offset = offset;

            for (size_t i = 0U; i < axis_count; i++) {
                const igh_cia402_axis_t *axis = &axes[i];
                uint32_t expected_id;

                if (trace[i].sent == 0U || axis->slave_pos != (int)slave ||
                    axis->channel != ch) {
                    continue;
                }

                expected_id = IGH_CIA402_COB_RPDO1_BASE + axis->node_id;
                if (!ide && !fdf && !rtr && can_id == expected_id) {
                    found[i]++;
                    if (dlc != trace[i].data_len ||
                        data_len < trace[i].data_len) {
                        print_target_trace_error("bad-dlc",
                                                 cycle,
                                                 axis,
                                                 &trace[i],
                                                 (int)slave,
                                                 packet_offset,
                                                 ctrl,
                                                 &writer->data[data_offset],
                                                 data_len);
                        return IGH_ECAN_FAIL;
                    }
                    if (memcmp(&writer->data[data_offset],
                               trace[i].data,
                               trace[i].data_len) != 0) {
                        print_target_trace_error("payload-mismatch",
                                                 cycle,
                                                 axis,
                                                 &trace[i],
                                                 (int)slave,
                                                 packet_offset,
                                                 ctrl,
                                                 &writer->data[data_offset],
                                                 data_len);
                        return IGH_ECAN_FAIL;
                    }
                }
            }

            offset = packet_end;
        }
    }

    for (size_t i = 0U; i < axis_count; i++) {
        if (trace[i].sent != 0U && found[i] != 1U) {
            for (size_t slave = 0U; slave < mcan->igh->slave_count; slave++) {
                const igh_mcan_pdo_writer_t *writer = &mcan->writers[slave];

                fprintf(stderr,
                        "[diag] target-pdo-writer cycle=%u slave=%zu"
                        " size=%zu channel_mask=0x%02X frame_bytes=%u frame_count=%u"
                        " overflow=%u\n",
                        cycle,
                        slave,
                        writer->size,
                        writer->channel_mask,
                        writer->frame_bytes,
                        writer->frame_count,
                        writer->overflow_sat);
            }
            print_target_trace_error(found[i] == 0U ? "missing-frame" : "duplicate-frame",
                                     cycle,
                                     &axes[i],
                                     &trace[i],
                                     axes[i].slave_pos,
                                     0U,
                                     0U,
                                     NULL,
                                     0U);
            return IGH_ECAN_FAIL;
        }
    }

    return IGH_ECAN_OK;
}

static int axis_ready_for_motion(const igh_cia402_axis_t *axis)
{
    if (axis == NULL) {
        return 0;
    }
    if (axis->emcy_valid != 0U &&
        (axis->emcy_code != 0U || axis->emcy_error_register != 0U)) {
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

static void print_topology(const ecan_igh_config_t *igh_config,
                           const igh_mcan_port_config_t *mcan_config,
                           size_t axis_count,
                           const cia402_10axis_demo_config_t *demo_config)
{
    if ((igh_config == NULL) || (mcan_config == NULL)) {
        return;
    }

    printf("[diag] topology slaves=%zu axes=%zu pdo-default=%uB cycle=%" PRIu32 "us\n",
           igh_config->slave_count,
           axis_count,
           (unsigned)ecan_igh_config_get_pdo_bytes(igh_config),
           igh_config->cycle_us);
    if ((demo_config != NULL) && demo_config->axis_mask != 0U) {
        printf("[diag] axis_mask=0x%08" PRIX32 " selects default topology slots\n",
               demo_config->axis_mask);
    }
    if ((demo_config != NULL) && demo_config->slot_canfd_override_valid != 0) {
        printf("[diag] slot_can_mode_override=%s nominal=%" PRIu32 " data=%" PRIu32 "\n",
               demo_config->slot_canfd_override == IGH_ECAN_CANFD_ENABLE ? "canfd" : "classic-can",
               demo_config->slot_nominal_bitrate,
               demo_config->slot_data_bitrate);
    }
    printf("[diag] cia402_canopen_frame=classic-can mcan_canfd_mask=configures gateway channel mode\n");
    for (size_t slave = 0U; slave < igh_config->slave_count; slave++) {
        const igh_mcan_slave_config_t *cfg = &mcan_config->slaves[slave];
        printf("[diag] slave=%zu pdo=%uB channel_mask=0x%02X canfd_mask=0x%02X nominal=%" PRIu32
               " data=%" PRIu32 "\n",
               slave,
               (unsigned)ecan_igh_config_get_slave_pdo_bytes(igh_config, slave),
               cfg->channel_mask,
               cfg->canfd_mask,
               cfg->nominal_bitrate,
               cfg->data_bitrate);
    }
    for (size_t i = 0U; i < IGH_ECAN_ARRAY_SIZE(k_motor_slots); i++) {
        const igh_cia402_demo_motor_slot_t *slot = &k_motor_slots[i];
        int selected = (slot->enable == IGH_ECAN_ENABLE &&
                        ((demo_config == NULL) || demo_config->axis_mask == 0U ||
                         (i < 32U && (demo_config->axis_mask & (1UL << i)) != 0U)));
        printf("[diag] slot=%zu enable=%d selected=%d slave=%zu ch=%u node=0x%02X canfd=%d bitrate=%" PRIu32 "\n",
               i,
               slot->enable,
               selected,
               slot->slave_pos,
               (unsigned)slot->mcan_channel,
               slot->node_id,
               get_slot_canfd(demo_config, slot),
               get_slot_nominal_bitrate(demo_config, slot));
    }
    if ((demo_config != NULL) && (demo_config->only_axis != 0)) {
        printf("[diag] runtime-only-axis slave=%zu ch=%u node=0x%02X canfd=%d"
               " nominal=%" PRIu32 " data=%" PRIu32 "\n",
               demo_config->axis_slave_pos,
               (unsigned)demo_config->axis_channel,
               demo_config->axis_node_id,
               demo_config->axis_canfd,
               demo_config->axis_nominal_bitrate,
               demo_config->axis_data_bitrate);
    }
}

static int apply_default_pdo_layout(ecan_igh_config_t *igh_config)
{
    if (igh_config == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ecan_igh_config_set_pdo_multiplier(
            igh_config,
            CIA402_10AXIS_DEMO_DEFAULT_PDO_MULTIPLIER) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (igh_config->slave_count > 2U) {
        return ecan_igh_config_set_slave_pdo_multiplier(
            igh_config,
            2U,
            CIA402_10AXIS_DEMO_THIRD_SLAVE_PDO_MULTIPLIER);
    }
    return IGH_ECAN_OK;
}

static void print_mcan_tpdo_diag(igh_mcan_port_t *mcan, const char *stage)
{
    if ((mcan == NULL) || (mcan->igh == NULL)) {
        return;
    }

    for (size_t slave = 0U; slave < mcan->igh->slave_count; slave++) {
        igh_mcan_pdo_header_t header;
        int rc = igh_mcan_port_get_tpdo_header(mcan, slave, &header);

        if (rc != IGH_ECAN_OK) {
            printf("[diag] %s slave=%zu tpdo_header=unavailable rc=%d\n", stage, slave, rc);
            continue;
        }
        printf("[diag] %s slave=%zu tpdo_seq=%u flags=0x%02X mask=0x%02X payload_len=%u"
               " ch_status=%04X,%04X,%04X,%04X\n",
               stage,
               slave,
               header.seq_begin,
               header.flags,
               header.channel_mask,
               header.payload_len,
               header.channel_status[0],
               header.channel_status[1],
               header.channel_status[2],
               header.channel_status[3]);
    }
}

static int ethercat_ready_for_motion(const igh_mcan_port_t *mcan)
{
    if ((mcan == NULL) || (mcan->igh == NULL)) {
        return 0;
    }
    return (ecan_igh_slaves_are_operational(mcan->igh) != 0) ? 1 : 0;
}

static void print_ethercat_motion_guard(const igh_mcan_port_t *mcan,
                                        unsigned cycle,
                                        const char *level)
{
    const ecan_igh_context_t *igh;

    if ((mcan == NULL) || (mcan->igh == NULL)) {
        return;
    }

    igh = mcan->igh;
    fprintf(stderr,
            "[%s] ethercat-motion-guard cycle=%u domain_complete=%d wc=%u"
            " wc_state=%d slaves_operational=%d",
            level != NULL ? level : "warn",
            cycle,
            ecan_igh_domain_is_complete(igh),
            ecan_igh_domain_working_counter(igh),
            (int)ecan_igh_domain_wc_state(igh),
            ecan_igh_slaves_are_operational(igh));
    for (size_t slave = 0U; slave < igh->slave_count; slave++) {
        fprintf(stderr,
                " slave%zu_op=%d slave%zu_al=0x%02X",
                slave,
                ecan_igh_slave_is_operational(igh, slave),
                slave,
                ecan_igh_slave_al_state(igh, slave));
    }
    fprintf(stderr, "\n");
}

static void pacer_add_us(struct timespec *ts, uint32_t us)
{
    if (ts == NULL) {
        return;
    }
    ts->tv_nsec += (long)(us % 1000000U) * 1000L;
    ts->tv_sec += (time_t)(us / 1000000U);
    while (ts->tv_nsec >= 1000000000L) {
        ts->tv_nsec -= 1000000000L;
        ts->tv_sec++;
    }
}

static int pacer_is_late(const struct timespec *now,
                         const struct timespec *next,
                         uint32_t cycle_us)
{
    int64_t now_ns;
    int64_t next_ns;

    if ((now == NULL) || (next == NULL)) {
        return 0;
    }
    now_ns = ((int64_t)now->tv_sec * 1000000000LL) + now->tv_nsec;
    next_ns = ((int64_t)next->tv_sec * 1000000000LL) + next->tv_nsec;
    return (now_ns - next_ns) > ((int64_t)cycle_us * 2000LL);
}

static void pacer_init(cia402_cycle_pacer_t *pacer, uint32_t cycle_us)
{
    if (pacer == NULL) {
        return;
    }
    (void)clock_gettime(CLOCK_MONOTONIC, &pacer->next);
    pacer->cycle_us = cycle_us;
}

static void pacer_wait_next(cia402_cycle_pacer_t *pacer)
{
    struct timespec now;
    int rc;

    if (pacer == NULL || pacer->cycle_us == 0U) {
        return;
    }

    pacer_add_us(&pacer->next, pacer->cycle_us);
    do {
        rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pacer->next, NULL);
    } while (rc == EINTR);

    if (clock_gettime(CLOCK_MONOTONIC, &now) == 0 &&
        pacer_is_late(&now, &pacer->next, pacer->cycle_us)) {
        pacer->next = now;
    }
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

static void run_one_cycle_io(igh_mcan_port_t *mcan, igh_cia402_context_t *cia)
{
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    igh_mcan_port_cycle_end(mcan);
}

static void flush_cycles(igh_mcan_port_t *mcan,
                         igh_cia402_context_t *cia,
                         unsigned settle_cycles)
{
    cia402_cycle_pacer_t pacer;

    pacer_init(&pacer, mcan->igh->cycle_us);
    for (unsigned i = 0U; i < settle_cycles; i++) {
        run_one_cycle_io(mcan, cia);
        pacer_wait_next(&pacer);
    }
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

static int send_axis_nmt_broadcast_and_flush(igh_mcan_port_t *mcan,
                                             igh_cia402_context_t *cia,
                                             const igh_cia402_axis_t *axis,
                                             uint8_t command,
                                             unsigned settle_cycles);

static uint64_t demo_monotonic_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * 1000U) + ((uint64_t)ts.tv_nsec / 1000000U);
}

static int demo_timeout_expired(uint64_t start_ms, uint32_t timeout_ms)
{
    return (timeout_ms != 0U && demo_monotonic_ms() - start_ms >= timeout_ms) ? 1 : 0;
}

static uint8_t demo_sdo_upload_size_from_command(uint8_t command)
{
    switch (command) {
    case IGH_CIA402_SDO_UPLOAD_RESP_U8:
        return 1U;
    case IGH_CIA402_SDO_UPLOAD_RESP_U16:
        return 2U;
    case IGH_CIA402_SDO_UPLOAD_RESP_U24:
        return 3U;
    case IGH_CIA402_SDO_UPLOAD_RESP_U32:
        return 4U;
    default:
        return 0U;
    }
}

static void prepare_axis_sdo_wait(igh_cia402_axis_t *axis,
                                  uint16_t index,
                                  uint8_t subindex)
{
    if (axis == NULL) {
        return;
    }

    axis->sdo_response_valid = 0U;
    axis->sdo_response_command = 0U;
    axis->sdo_response_index = index;
    axis->sdo_response_subindex = subindex;
    axis->sdo_response_size = 0U;
    axis->sdo_response_value = 0U;
    axis->sdo_abort_code = 0U;
}

static int axis_sdo_wait_done(const igh_cia402_axis_t *axis,
                              uint16_t index,
                              uint8_t subindex,
                              cia402_sdo_wait_kind_t kind)
{
    if (axis == NULL ||
        axis->sdo_response_valid == 0U ||
        axis->sdo_response_index != index ||
        axis->sdo_response_subindex != subindex) {
        return 0;
    }
    if (axis->sdo_response_command == IGH_CIA402_SDO_ABORT) {
        return -1;
    }
    if (kind == CIA402_SDO_WAIT_UPLOAD) {
        return (demo_sdo_upload_size_from_command(axis->sdo_response_command) != 0U) ? 1 : -1;
    }
    return (axis->sdo_response_command == IGH_CIA402_SDO_DOWNLOAD_ACK) ? 1 : -1;
}

static void print_sdo_wait_pending(const igh_cia402_context_t *cia,
                                   uint16_t index,
                                   uint8_t subindex,
                                   cia402_sdo_wait_kind_t kind,
                                   const char *stage)
{
    if (cia == NULL) {
        return;
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];
        int done;

        if (!axis->enabled) {
            continue;
        }
        done = axis_sdo_wait_done(axis, index, subindex, kind);
        if (done > 0) {
            continue;
        }
        fprintf(stderr,
                "[diag] %s pending slave=%d ch=%u node=0x%02X"
                " expect=0x%04X:%02X cmd=0x%02X last=0x%04X:%02X"
                " abort=0x%08" PRIX32 " %s\n",
                stage != NULL ? stage : "sdo-wait",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                index,
                subindex,
                axis->sdo_response_command,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_abort_code,
                igh_cia402_sdo_abort_name(axis->sdo_abort_code));
    }
}

static int wait_all_axis_sdo(igh_mcan_port_t *mcan,
                             igh_cia402_context_t *cia,
                             uint16_t index,
                             uint8_t subindex,
                             cia402_sdo_wait_kind_t kind,
                             uint32_t timeout_ms,
                             const char *stage)
{
    uint64_t start_ms;
    cia402_cycle_pacer_t pacer;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    start_ms = demo_monotonic_ms();
    pacer_init(&pacer, mcan->igh->cycle_us);
    for (;;) {
        size_t done_count = 0U;
        size_t enabled_count = 0U;

        for (size_t i = 0U; i < cia->axis_count; i++) {
            igh_cia402_axis_t *axis = &cia->axes[i];
            int done;

            if (!axis->enabled) {
                continue;
            }
            enabled_count++;
            done = axis_sdo_wait_done(axis, index, subindex, kind);
            if (done < 0) {
                print_sdo_wait_pending(cia, index, subindex, kind, stage);
                return IGH_ECAN_FAIL;
            }
            if (done > 0) {
                done_count++;
            }
        }
        if (done_count == enabled_count) {
            return IGH_ECAN_OK;
        }
        if (demo_timeout_expired(start_ms, timeout_ms)) {
            print_sdo_wait_pending(cia, index, subindex, kind, stage);
            return IGH_ECAN_TIMEOUT;
        }

        run_one_cycle_io(mcan, cia);
        pacer_wait_next(&pacer);
    }
}

static int send_sdo_upload_all_axes(igh_mcan_port_t *mcan,
                                    igh_cia402_context_t *cia,
                                    uint16_t index,
                                    uint8_t subindex)
{
    int rc = IGH_ECAN_OK;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        int axis_rc;

        if (!axis->enabled) {
            continue;
        }
        prepare_axis_sdo_wait(axis, index, subindex);
        axis_rc = igh_cia402_sdo_upload(cia, axis, index, subindex);
        if (rc == IGH_ECAN_OK && axis_rc != IGH_ECAN_OK) {
            rc = axis_rc;
        }
    }
    igh_mcan_port_cycle_end(mcan);
    return rc;
}

static int send_sdo_write_all_axes(igh_mcan_port_t *mcan,
                                   igh_cia402_context_t *cia,
                                   uint8_t command,
                                   uint16_t index,
                                   uint8_t subindex,
                                   uint32_t value)
{
    int rc = IGH_ECAN_OK;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        int axis_rc = IGH_ECAN_INVALID_ARG;

        if (!axis->enabled) {
            continue;
        }
        prepare_axis_sdo_wait(axis, index, subindex);
        switch (command) {
        case IGH_CIA402_SDO_WRITE_U8:
            axis_rc = igh_cia402_sdo_write_u8(cia, axis, index, subindex, (uint8_t)value);
            break;
        case IGH_CIA402_SDO_WRITE_U16:
            axis_rc = igh_cia402_sdo_write_u16(cia, axis, index, subindex, (uint16_t)value);
            break;
        case IGH_CIA402_SDO_WRITE_U32:
            axis_rc = igh_cia402_sdo_write_u32(cia, axis, index, subindex, value);
            break;
        default:
            axis_rc = IGH_ECAN_INVALID_ARG;
            break;
        }
        if (rc == IGH_ECAN_OK && axis_rc != IGH_ECAN_OK) {
            rc = axis_rc;
        }
    }
    igh_mcan_port_cycle_end(mcan);
    return rc;
}

static int sdo_write_all_axes_timeout(igh_mcan_port_t *mcan,
                                      igh_cia402_context_t *cia,
                                      uint8_t command,
                                      uint16_t index,
                                      uint8_t subindex,
                                      uint32_t value,
                                      uint32_t timeout_ms,
                                      const char *stage)
{
    int rc = send_sdo_write_all_axes(mcan, cia, command, index, subindex, value);

    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return wait_all_axis_sdo(mcan,
                             cia,
                             index,
                             subindex,
                             CIA402_SDO_WAIT_DOWNLOAD_ACK,
                             timeout_ms,
                             stage);
}

static int sdo_upload_all_axes_timeout(igh_mcan_port_t *mcan,
                                       igh_cia402_context_t *cia,
                                       uint16_t index,
                                       uint8_t subindex,
                                       uint32_t timeout_ms,
                                       const char *stage)
{
    int rc = send_sdo_upload_all_axes(mcan, cia, index, subindex);

    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return wait_all_axis_sdo(mcan,
                             cia,
                             index,
                             subindex,
                             CIA402_SDO_WAIT_UPLOAD,
                             timeout_ms,
                             stage);
}

static int send_enabled_channel_nmt_broadcasts_and_flush(igh_mcan_port_t *mcan,
                                                         igh_cia402_context_t *cia,
                                                         uint8_t command,
                                                         unsigned settle_cycles)
{
    uint8_t sent[ECAN_IGH_MAX_SLAVES][4] = {{0}};
    int rc = IGH_ECAN_OK;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        int axis_rc;

        if (!axis->enabled ||
            axis->slave_pos < 0 ||
            (size_t)axis->slave_pos >= ECAN_IGH_MAX_SLAVES ||
            axis->channel > IGH_ECAN_MCAN_CH3) {
            continue;
        }
        if (sent[axis->slave_pos][axis->channel] != 0U) {
            continue;
        }
        sent[axis->slave_pos][axis->channel] = 1U;
        axis_rc = igh_cia402_send_nmt_broadcast(cia,
                                                axis->slave_pos,
                                                axis->channel,
                                                command);
        if (rc == IGH_ECAN_OK && axis_rc != IGH_ECAN_OK) {
            rc = axis_rc;
        }
    }
    igh_mcan_port_cycle_end(mcan);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int enable_axis_with_retry(igh_mcan_port_t *mcan,
                                  igh_cia402_context_t *cia,
                                  igh_cia402_axis_t *axis,
                                  uint32_t sdo_timeout_ms,
                                  unsigned settle_cycles,
                                  int verbose)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (verbose) {
        printf("[diag] enable begin slave=%d ch=%u node=0x%02X\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id);
    }
    rc = igh_cia402_enable_timeout(cia, axis, sdo_timeout_ms);
    if (rc != IGH_ECAN_OK) {
        flush_cycles(mcan, cia, settle_cycles);
        (void)send_axis_nmt_broadcast_and_flush(mcan,
                                                cia,
                                                axis,
                                                IGH_CIA402_NMT_START_REMOTE_NODE,
                                                settle_cycles);
        rc = igh_cia402_enable_timeout(cia, axis, sdo_timeout_ms);
    }
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "enable failed: slave=%d ch=%u node=0x%02X rc=%d status_valid=%u"
                " status=0x%04X state=%s abort=0x%08" PRIX32 " abort_text=%s\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->statusword_valid,
                axis->statusword,
                igh_cia402_statusword_state_name(axis->statusword),
                axis->sdo_abort_code,
                igh_cia402_sdo_abort_name(axis->sdo_abort_code));
        return rc;
    }
    if (verbose) {
        printf("[diag] enable done slave=%d ch=%u node=0x%02X status=0x%04X state=%s\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               axis->statusword,
               igh_cia402_statusword_state_name(axis->statusword));
    }
    flush_cycles(mcan, cia, settle_cycles);
    rc = igh_cia402_request_statusword_timeout(cia, axis, sdo_timeout_ms);
    if (rc == IGH_ECAN_OK && axis_ready_for_motion(axis)) {
        return IGH_ECAN_OK;
    }
    if (rc == IGH_ECAN_TIMEOUT && axis_ready_for_motion(axis)) {
        return IGH_ECAN_OK;
    }
    fprintf(stderr,
            "enable post-check failed: slave=%d ch=%u node=0x%02X rc=%d"
            " status_valid=%u status=0x%04X state=%s emcy=0x%04X err=0x%02X\n",
            axis->slave_pos,
            axis->channel,
            axis->node_id,
            rc,
            axis->statusword_valid,
            axis->statusword,
            igh_cia402_statusword_state_name(axis->statusword),
            axis->emcy_code,
            axis->emcy_error_register);
    return IGH_ECAN_NOT_READY;
}

static int stop_and_flush(igh_mcan_port_t *mcan,
                          igh_cia402_context_t *cia,
                          unsigned settle_cycles)
{
    int rc;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    rc = igh_cia402_stop_all(cia);
    igh_mcan_port_cycle_end(mcan);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int send_nmt_and_flush(igh_mcan_port_t *mcan,
                              igh_cia402_context_t *cia,
                              uint8_t command,
                              unsigned settle_cycles)
{
    int rc;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    rc = igh_cia402_send_nmt_all(cia, command);
    igh_mcan_port_cycle_end(mcan);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int send_axis_nmt_broadcast_and_flush(igh_mcan_port_t *mcan,
                                             igh_cia402_context_t *cia,
                                             const igh_cia402_axis_t *axis,
                                             uint8_t command,
                                             unsigned settle_cycles)
{
    int rc;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    rc = igh_cia402_send_nmt_broadcast(cia, axis->slave_pos, axis->channel, command);
    igh_mcan_port_cycle_end(mcan);
    flush_cycles(mcan, cia, settle_cycles);
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
    if (rc == IGH_ECAN_TIMEOUT) {
        return 1;
    }
    return (rc == IGH_ECAN_FAIL &&
            axis != NULL &&
            axis->sdo_abort_code == 0U) ? 1 : 0;
}

static int ensure_axis_canopen_mode(igh_mcan_port_t *mcan,
                                    igh_cia402_context_t *cia,
                                    igh_cia402_axis_t *axis,
                                    uint32_t timeout_ms,
                                    unsigned settle_cycles)
{
    uint32_t mode = 0U;
    int rc = IGH_ECAN_TIMEOUT;

    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_COMM_MODE_RETRIES;
         attempt++) {
        rc = read_comm_mode(cia, axis, timeout_ms, &mode);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if (!comm_mode_read_needs_nmt_retry(rc, axis) ||
            attempt >= CIA402_10AXIS_DEMO_COMM_MODE_RETRIES) {
            break;
        }

        printf("[diag] 0x2004:00 no response slave=%d ch=%u node=0x%02X"
               " attempt=%u/%u; send channel NMT start and retry.\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               CIA402_10AXIS_DEMO_COMM_MODE_RETRIES);
        if (send_axis_nmt_broadcast_and_flush(mcan,
                                              cia,
                                              axis,
                                              IGH_CIA402_NMT_START_REMOTE_NODE,
                                              settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (rc != IGH_ECAN_OK) {
        if (comm_mode_read_needs_nmt_retry(rc, axis)) {
            printf("[diag] 0x2004:00 unsupported/no-response slave=%d ch=%u node=0x%02X; assume CANopen mode.\n",
                   axis->slave_pos,
                   axis->channel,
                   axis->node_id);
            return IGH_ECAN_OK;
        }
        fprintf(stderr,
                "read communication mode failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " %s\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_abort_code,
                igh_cia402_sdo_abort_name(axis->sdo_abort_code));
        return rc;
    }

    printf("[diag] communication-mode slave=%d ch=%u node=0x%02X 0x2004:00=%" PRIu32 " (%s)\n",
           axis->slave_pos,
           axis->channel,
           axis->node_id,
           mode & 0xFFU,
           comm_mode_name(mode & 0xFFU));
    if ((mode & 0xFFU) == IGH_CIA402_PH_COMM_MODE_CANOPEN) {
        return IGH_ECAN_OK;
    }
    if ((mode & 0xFFU) != IGH_CIA402_PH_COMM_MODE_ETHERCAT) {
        return IGH_ECAN_UNSUPPORTED;
    }

    rc = igh_cia402_sdo_write_verify_u8_timeout(cia,
                                                axis,
                                                IGH_CIA402_IDX_PH_COMM_MODE,
                                                0x00U,
                                                IGH_CIA402_PH_COMM_MODE_CANOPEN,
                                                timeout_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "switch communication mode to CANopen failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " %s\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
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
        fprintf(stderr,
                "communication mode verify failed: slave=%d ch=%u node=0x%02X mode=0x%02" PRIX32 "\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                mode & 0xFFU);
        return IGH_ECAN_FAIL;
    }
    return IGH_ECAN_OK;
}

static int configure_axis_csp_rpdo(igh_mcan_port_t *mcan,
                                   igh_cia402_context_t *cia,
                                   igh_cia402_axis_t *axis,
                                   uint32_t sdo_timeout_ms,
                                   unsigned settle_cycles,
                                   int verbose)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (verbose) {
        printf("[diag] RPDO remap begin slave=%d ch=%u node=0x%02X\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id);
    }
    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = igh_cia402_configure_csp_rpdo_timeout(cia, axis, sdo_timeout_ms);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if (!axis_sdo_needs_retry(rc, axis) ||
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            break;
        }
        printf("[diag] CSP RPDO configure retry slave=%d ch=%u node=0x%02X"
               " attempt=%u/%u rc=%d index=0x%04X:%02X cmd=0x%02X"
               " write_rc=%d read_rc=%d; send channel NMT start.\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command,
               axis->sdo_last_write_rc,
               axis->sdo_last_read_rc);
        if (send_axis_nmt_broadcast_and_flush(mcan,
                                              cia,
                                              axis,
                                              IGH_CIA402_NMT_START_REMOTE_NODE,
                                              settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "CSP RPDO configure failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X value=0x%08" PRIX32
                " write_rc=%d read_rc=%d abort=0x%08" PRIX32 "\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_response_value,
                axis->sdo_last_write_rc,
                axis->sdo_last_read_rc,
                axis->sdo_abort_code);
        return rc;
    }
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int configure_axis_csv_rpdo(igh_mcan_port_t *mcan,
                                   igh_cia402_context_t *cia,
                                   igh_cia402_axis_t *axis,
                                   uint32_t sdo_timeout_ms,
                                   unsigned settle_cycles,
                                   int verbose)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (verbose) {
        printf("[diag] CSV RPDO remap begin slave=%d ch=%u node=0x%02X\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id);
    }
    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = igh_cia402_configure_csv_rpdo_timeout(cia, axis, sdo_timeout_ms);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if (!axis_sdo_needs_retry(rc, axis) ||
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            break;
        }
        printf("[diag] CSV RPDO configure retry slave=%d ch=%u node=0x%02X"
               " attempt=%u/%u rc=%d index=0x%04X:%02X cmd=0x%02X"
               " write_rc=%d read_rc=%d; send channel NMT start.\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command,
               axis->sdo_last_write_rc,
               axis->sdo_last_read_rc);
        if (send_axis_nmt_broadcast_and_flush(mcan,
                                              cia,
                                              axis,
                                              IGH_CIA402_NMT_START_REMOTE_NODE,
                                              settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "CSV RPDO configure failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X value=0x%08" PRIX32
                " write_rc=%d read_rc=%d abort=0x%08" PRIX32 "\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_response_value,
                axis->sdo_last_write_rc,
                axis->sdo_last_read_rc,
                axis->sdo_abort_code);
        return rc;
    }
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int read_actual_position_axis(igh_mcan_port_t *mcan,
                                     igh_cia402_context_t *cia,
                                     igh_cia402_axis_t *axis,
                                     uint32_t sdo_timeout_ms,
                                     unsigned settle_cycles,
                                     int verbose)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (verbose) {
        printf("[diag] read-position begin slave=%d ch=%u node=0x%02X\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id);
    }
    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = igh_cia402_request_actual_position_timeout(cia, axis, sdo_timeout_ms);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if (!axis_sdo_needs_retry(rc, axis) ||
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            break;
        }
        printf("[diag] read-position retry slave=%d ch=%u node=0x%02X"
               " attempt=%u/%u rc=%d index=0x%04X:%02X cmd=0x%02X;"
               " send channel NMT start.\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command);
        if (send_axis_nmt_broadcast_and_flush(mcan,
                                              cia,
                                              axis,
                                              IGH_CIA402_NMT_START_REMOTE_NODE,
                                              settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "read position failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 " %s\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_abort_code,
                igh_cia402_sdo_abort_name(axis->sdo_abort_code));
        return rc;
    }
    if (verbose) {
        printf("[diag] read-position done slave=%d ch=%u node=0x%02X actual=%" PRId32 "\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               axis->actual_position);
    }
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int prime_csp_targets_to_actual(igh_mcan_port_t *mcan,
                                       igh_cia402_context_t *cia,
                                       unsigned settle_cycles,
                                       const char *stage)
{
    uint8_t sync_sent[ECAN_IGH_MAX_SLAVES][IGH_MCAN_CHANNEL_COUNT] = {{0}};
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;
    int rc = IGH_ECAN_OK;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (stage != NULL) {
        printf("[diag] %s prime CSP target_position from actual_position before enable\n", stage);
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);

    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        uint8_t data[IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN] = {0};
        int send_rc;

        if (!axis->enabled) {
            continue;
        }
        if (axis->actual_position_valid == 0U) {
            fprintf(stderr,
                    "target prime failed: slave=%d ch=%u node=0x%02X has no valid actual position\n",
                    axis->slave_pos,
                    axis->channel,
                    axis->node_id);
            rc = IGH_ECAN_NOT_READY;
            continue;
        }
        if (axis->slave_pos < 0 ||
            (size_t)axis->slave_pos >= ECAN_IGH_MAX_SLAVES ||
            axis->channel >= IGH_MCAN_CHANNEL_COUNT) {
            rc = IGH_ECAN_INVALID_ARG;
            continue;
        }

        axis->target_position = axis->actual_position;
        axis->rpdo1_payload_len = IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
        demo_put_u16_le(&data[0], IGH_CIA402_CONTROLWORD_SHUTDOWN);
        data[2] = IGH_CIA402_MODE_CSP;
        demo_put_i32_le(&data[3], axis->target_position);

        send_rc = igh_mcan_port_send_can(mcan,
                                         axis->slave_pos,
                                         axis->channel,
                                         IGH_CIA402_COB_RPDO1_BASE + axis->node_id,
                                         0,
                                         data,
                                         sizeof(data));
        if (send_rc != IGH_ECAN_OK && rc == IGH_ECAN_OK) {
            rc = send_rc;
        }
        sync_sent[(size_t)axis->slave_pos][axis->channel] = 1U;
    }

    for (size_t slave = 0U; slave < ECAN_IGH_MAX_SLAVES; slave++) {
        for (uint8_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
            igh_cia402_tx_frame_t sync_frame;
            int sync_rc;

            if (sync_sent[slave][ch] == 0U) {
                continue;
            }
            sync_rc = igh_cia402_build_sync_frame(cia, (int)slave, ch, &sync_frame);
            if (sync_rc == IGH_ECAN_OK) {
                sync_rc = igh_cia402_mcan_send_frame(mcan, &sync_frame);
            }
            if (sync_rc != IGH_ECAN_OK && rc == IGH_ECAN_OK) {
                rc = sync_rc;
            }
        }
    }

    igh_mcan_port_cycle_end(mcan);
    if (rc == IGH_ECAN_OK) {
        flush_cycles(mcan, cia, settle_cycles);
    }
    return rc;
}

static int prime_axis_csp_target_to_actual(igh_mcan_port_t *mcan,
                                           igh_cia402_context_t *cia,
                                           igh_cia402_axis_t *axis,
                                           unsigned settle_cycles,
                                           const char *stage)
{
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    igh_cia402_tx_frame_t sync_frame;
    uint8_t data[IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN] = {0};
    size_t frame_count = 0U;
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (!axis->enabled) {
        return IGH_ECAN_OK;
    }
    if (axis->slave_pos < 0 ||
        (size_t)axis->slave_pos >= ECAN_IGH_MAX_SLAVES ||
        axis->channel >= IGH_MCAN_CHANNEL_COUNT) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (stage != NULL) {
        printf("[diag] %s prime CSP target_position from actual_position"
               " slave=%d ch=%u node=0x%02X\n",
               stage,
               axis->slave_pos,
               axis->channel,
               axis->node_id);
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);

    if (axis->actual_position_valid == 0U) {
        igh_mcan_port_cycle_end(mcan);
        fprintf(stderr,
                "target prime failed: slave=%d ch=%u node=0x%02X has no valid actual position\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id);
        return IGH_ECAN_NOT_READY;
    }

    axis->target_position = axis->actual_position;
    axis->rpdo1_payload_len = IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
    demo_put_u16_le(&data[0], IGH_CIA402_CONTROLWORD_SHUTDOWN);
    data[2] = IGH_CIA402_MODE_CSP;
    demo_put_i32_le(&data[3], axis->target_position);

    rc = igh_mcan_port_send_can(mcan,
                                axis->slave_pos,
                                axis->channel,
                                IGH_CIA402_COB_RPDO1_BASE + axis->node_id,
                                0,
                                data,
                                sizeof(data));
    if (rc == IGH_ECAN_OK) {
        rc = igh_cia402_build_sync_frame(cia, axis->slave_pos, axis->channel, &sync_frame);
    }
    if (rc == IGH_ECAN_OK) {
        rc = igh_cia402_mcan_send_frame(mcan, &sync_frame);
    }

    igh_mcan_port_cycle_end(mcan);
    if (rc == IGH_ECAN_OK) {
        flush_cycles(mcan, cia, settle_cycles);
    }
    return rc;
}

static int set_axis_csp_mode(igh_mcan_port_t *mcan,
                             igh_cia402_context_t *cia,
                             igh_cia402_axis_t *axis,
                             uint32_t sdo_timeout_ms,
                             unsigned settle_cycles,
                             int verbose)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (verbose) {
        printf("[diag] csp-mode begin slave=%d ch=%u node=0x%02X\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id);
    }
    axis->rpdo1_payload_len = IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = igh_cia402_sdo_write_u8_timeout(cia,
                                             axis,
                                             IGH_CIA402_IDX_MODES_OF_OPERATION,
                                             0x00U,
                                             IGH_CIA402_MODE_CSP,
                                             sdo_timeout_ms);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if (!axis_sdo_needs_retry(rc, axis) ||
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            break;
        }
        printf("[diag] CSP mode retry slave=%d ch=%u node=0x%02X"
               " attempt=%u/%u rc=%d index=0x%04X:%02X cmd=0x%02X;"
               " send channel NMT start.\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command);
        if (send_axis_nmt_broadcast_and_flush(mcan,
                                              cia,
                                              axis,
                                              IGH_CIA402_NMT_START_REMOTE_NODE,
                                              settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (rc != IGH_ECAN_OK) {
        if (axis_sdo_needs_retry(rc, axis)) {
            printf("[diag] CSP mode write has no final SDO ack slave=%d ch=%u node=0x%02X; continue with stored/assumed mode.\n",
                   axis->slave_pos,
                   axis->channel,
                   axis->node_id);
            flush_cycles(mcan, cia, settle_cycles);
            return IGH_ECAN_OK;
        }
        fprintf(stderr,
                "CSP mode set failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 "\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_abort_code);
        return rc;
    }
    flush_cycles(mcan, cia, settle_cycles);
    {
        uint32_t mode = 0U;
        uint8_t size = 0U;
        int read_rc = igh_cia402_sdo_upload_u32_timeout(cia,
                                                        axis,
                                                        IGH_CIA402_IDX_MODES_OF_OPERATION,
                                                        IGH_CIA402_SUBIDX_VALUE,
                                                        &mode,
                                                        &size,
                                                        sdo_timeout_ms);
        if (read_rc == IGH_ECAN_OK) {
            if (size < 1U ||
                ((mode & IGH_CIA402_SDO_VALUE_MASK_U8) != IGH_CIA402_MODE_CSP)) {
                fprintf(stderr,
                        "CSP mode readback mismatch: slave=%d ch=%u node=0x%02X"
                        " value=0x%02" PRIX32 " size=%u\n",
                        axis->slave_pos,
                        axis->channel,
                        axis->node_id,
                        (uint32_t)(mode & IGH_CIA402_SDO_VALUE_MASK_U8),
                        size);
                return IGH_ECAN_FAIL;
            }
        } else {
            fprintf(stderr,
                    "[warn] CSP mode write acknowledged but readback failed:"
                    " slave=%d ch=%u node=0x%02X read_rc=%d"
                    " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 "\n",
                    axis->slave_pos,
                    axis->channel,
                    axis->node_id,
                    read_rc,
                    axis->sdo_response_index,
                    axis->sdo_response_subindex,
                    axis->sdo_response_command,
                    axis->sdo_abort_code);
        }
    }
    return rc;
}

static int set_axis_csv_mode(igh_mcan_port_t *mcan,
                             igh_cia402_context_t *cia,
                             igh_cia402_axis_t *axis,
                             uint8_t mode,
                             uint32_t sdo_timeout_ms,
                             unsigned settle_cycles,
                             int verbose)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if (verbose) {
        printf("[diag] csv-mode begin slave=%d ch=%u node=0x%02X mode=%u\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               mode);
    }
    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = igh_cia402_sdo_write_u8_timeout(cia,
                                             axis,
                                             IGH_CIA402_IDX_MODES_OF_OPERATION,
                                             0x00U,
                                             mode,
                                             sdo_timeout_ms);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if (!axis_sdo_needs_retry(rc, axis) ||
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            break;
        }
        printf("[diag] CSV mode retry slave=%d ch=%u node=0x%02X"
               " attempt=%u/%u rc=%d index=0x%04X:%02X cmd=0x%02X;"
               " send channel NMT start.\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command);
        if (send_axis_nmt_broadcast_and_flush(mcan,
                                              cia,
                                              axis,
                                              IGH_CIA402_NMT_START_REMOTE_NODE,
                                              settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr,
                "CSV mode set failed: slave=%d ch=%u node=0x%02X rc=%d"
                " index=0x%04X:%02X cmd=0x%02X abort=0x%08" PRIX32 "\n",
                axis->slave_pos,
                axis->channel,
                axis->node_id,
                rc,
                axis->sdo_response_index,
                axis->sdo_response_subindex,
                axis->sdo_response_command,
                axis->sdo_abort_code);
        return rc;
    }
    flush_cycles(mcan, cia, settle_cycles);
    return IGH_ECAN_OK;
}

static int startup_axis_transaction(igh_mcan_port_t *mcan,
                                    igh_cia402_context_t *cia,
                                    igh_cia402_axis_t *axis,
                                    const cia402_10axis_demo_config_t *config)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (axis == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (!axis->enabled) {
        return IGH_ECAN_OK;
    }

    printf("[diag] axis-startup begin slave=%d ch=%u node=0x%02X mode=%s\n",
           axis->slave_pos,
           axis->channel,
           axis->node_id,
           config->csv_pdo_mode ? "csv-rpdo-remap" :
           (config->skip_csp_config ? "stored-rpdo" : "csp-rpdo-remap"));

    rc = ensure_axis_canopen_mode(mcan, cia, axis, config->sdo_timeout_ms,
                                  config->nmt_start_settle_cycles);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    if (send_axis_nmt_broadcast_and_flush(mcan,
                                          cia,
                                          axis,
                                          IGH_CIA402_NMT_START_REMOTE_NODE,
                                          config->nmt_start_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }

    if (config->csv_sdo_mode != 0) {
        rc = set_axis_csv_mode(mcan,
                               cia,
                               axis,
                               config->csv_operation_mode,
                               config->sdo_timeout_ms,
                               config->configure_settle_cycles,
                               config->verbose);
    } else if (config->csv_pdo_mode != 0) {
        rc = configure_axis_csv_rpdo(mcan, cia, axis, config->sdo_timeout_ms,
                                     config->configure_settle_cycles,
                                     config->verbose);
    } else if (!config->skip_csp_config) {
        rc = configure_axis_csp_rpdo(mcan, cia, axis, config->sdo_timeout_ms,
                                     config->configure_settle_cycles,
                                     config->verbose);
    } else {
        rc = set_axis_csp_mode(mcan, cia, axis, config->sdo_timeout_ms,
                               config->configure_settle_cycles,
                               config->verbose);
    }
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    if (send_axis_nmt_broadcast_and_flush(mcan,
                                          cia,
                                          axis,
                                          IGH_CIA402_NMT_START_REMOTE_NODE,
                                          config->nmt_start_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }

    rc = read_actual_position_axis(mcan, cia, axis, config->sdo_timeout_ms,
                                   config->read_position_settle_cycles,
                                   config->verbose);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    if (config->csv_sdo_mode == 0 && config->csv_pdo_mode == 0) {
        rc = prime_axis_csp_target_to_actual(mcan,
                                             cia,
                                             axis,
                                             config->read_position_settle_cycles,
                                             "axis-pre-enable");
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }

    if (send_axis_nmt_broadcast_and_flush(mcan,
                                          cia,
                                          axis,
                                          IGH_CIA402_NMT_START_REMOTE_NODE,
                                          config->enable_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }

    rc = enable_axis_with_retry(mcan, cia, axis, config->sdo_timeout_ms,
                                config->enable_settle_cycles,
                                config->verbose);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    printf("[diag] axis-startup done slave=%d ch=%u node=0x%02X"
           " status=0x%04X state=%s actual=%" PRId32 "\n",
           axis->slave_pos,
           axis->channel,
           axis->node_id,
           axis->statusword,
           igh_cia402_statusword_state_name(axis->statusword),
           axis->actual_position);
    return IGH_ECAN_OK;
}

static int ensure_all_axes_canopen_mode_parallel(igh_mcan_port_t *mcan,
                                                 igh_cia402_context_t *cia,
                                                 const cia402_10axis_demo_config_t *config)
{
    int rc = IGH_ECAN_TIMEOUT;

    if ((mcan == NULL) || (cia == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_COMM_MODE_RETRIES;
         attempt++) {
        int need_switch = 0;

        rc = sdo_upload_all_axes_timeout(mcan,
                                         cia,
                                         IGH_CIA402_IDX_PH_COMM_MODE,
                                         0x00U,
                                         config->sdo_timeout_ms,
                                         "parallel-communication-mode");
        if (rc == IGH_ECAN_OK) {
            for (size_t i = 0U; i < cia->axis_count; i++) {
                igh_cia402_axis_t *axis = &cia->axes[i];
                uint32_t mode;

                if (!axis->enabled) {
                    continue;
                }
                mode = axis->sdo_response_value & 0xFFU;
                printf("[diag] communication-mode slave=%d ch=%u node=0x%02X"
                       " 0x2004:00=%" PRIu32 " (%s)\n",
                       axis->slave_pos,
                       axis->channel,
                       axis->node_id,
                       mode,
                       comm_mode_name(mode));
                if (mode == IGH_CIA402_PH_COMM_MODE_ETHERCAT) {
                    need_switch = 1;
                } else if (mode != IGH_CIA402_PH_COMM_MODE_CANOPEN) {
                    return IGH_ECAN_UNSUPPORTED;
                }
            }
            if (!need_switch) {
                return IGH_ECAN_OK;
            }

            rc = sdo_write_all_axes_timeout(mcan,
                                            cia,
                                            IGH_CIA402_SDO_WRITE_U8,
                                            IGH_CIA402_IDX_PH_COMM_MODE,
                                            0x00U,
                                            IGH_CIA402_PH_COMM_MODE_CANOPEN,
                                            config->sdo_timeout_ms,
                                            "parallel-switch-canopen");
            if (rc == IGH_ECAN_OK) {
                continue;
            }
        }

        if (attempt >= CIA402_10AXIS_DEMO_COMM_MODE_RETRIES &&
            rc == IGH_ECAN_TIMEOUT) {
            printf("[diag] 0x2004:00 unsupported/no-response on one or more axes;"
                   " assume CANopen mode for enabled axes.\n");
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_TIMEOUT) {
            return rc;
        }
        printf("[diag] 0x2004:00 no response on one or more axes"
               " attempt=%u/%u; send all channel NMT start and retry.\n",
               attempt + 1U,
               CIA402_10AXIS_DEMO_COMM_MODE_RETRIES);
        rc = send_enabled_channel_nmt_broadcasts_and_flush(mcan,
                                                           cia,
                                                           IGH_CIA402_NMT_START_REMOTE_NODE,
                                                           config->nmt_start_settle_cycles);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
    return rc;
}

static int set_all_axes_operation_mode_parallel(igh_mcan_port_t *mcan,
                                                igh_cia402_context_t *cia,
                                                const cia402_10axis_demo_config_t *config,
                                                uint8_t mode,
                                                const char *stage)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        if (cia->axes[i].enabled) {
            if (mode == IGH_CIA402_MODE_CSV) {
                cia->axes[i].rpdo1_payload_len = IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN;
            } else {
                cia->axes[i].rpdo1_payload_len = (mode == IGH_CIA402_MODE_CSP) ?
                                                 IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN :
                                                 0U;
            }
        }
    }

    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = sdo_write_all_axes_timeout(mcan,
                                        cia,
                                        IGH_CIA402_SDO_WRITE_U8,
                                        IGH_CIA402_IDX_MODES_OF_OPERATION,
                                        0x00U,
                                        mode,
                                        config->sdo_timeout_ms,
                                        stage);
        if (rc == IGH_ECAN_OK) {
            return IGH_ECAN_OK;
        }
        if (axis_sdo_needs_retry(rc, &cia->axes[0]) &&
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES &&
            mode == IGH_CIA402_MODE_CSP) {
            printf("[diag] operation-mode parallel stage=%s mode=%u has no final SDO ack;"
                   " continue with stored/assumed mode.\n",
                   stage,
                   mode);
            return IGH_ECAN_OK;
        }
        if (!axis_sdo_needs_retry(rc, &cia->axes[0]) ||
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            return rc;
        }
        printf("[diag] operation-mode parallel retry stage=%s mode=%u attempt=%u/%u rc=%d;"
               " send all channel NMT start.\n",
               stage,
               mode,
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc);
        rc = send_enabled_channel_nmt_broadcasts_and_flush(mcan,
                                                           cia,
                                                           IGH_CIA402_NMT_START_REMOTE_NODE,
                                                           config->configure_settle_cycles);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
    return IGH_ECAN_FAIL;
}

static int send_controlword_all_and_flush(igh_mcan_port_t *mcan,
                                          igh_cia402_context_t *cia,
                                          uint16_t controlword,
                                          unsigned settle_cycles)
{
    int rc = IGH_ECAN_OK;
    igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
    size_t frame_count = 0U;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    igh_mcan_port_cycle_begin(mcan);
    (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
    (void)process_mcan_frames(cia, frames, frame_count);
    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        int nmt_rc;
        int cw_rc;

        if (!axis->enabled) {
            continue;
        }
        nmt_rc = igh_cia402_send_nmt(cia, axis, IGH_CIA402_NMT_START_REMOTE_NODE);
        cw_rc = igh_cia402_sdo_write_u16(cia,
                                         axis,
                                         IGH_CIA402_IDX_CONTROLWORD,
                                         0x00U,
                                         controlword);
        if (rc == IGH_ECAN_OK && nmt_rc != IGH_ECAN_OK) {
            rc = nmt_rc;
        }
        if (rc == IGH_ECAN_OK && cw_rc != IGH_ECAN_OK) {
            rc = cw_rc;
        }
    }
    igh_mcan_port_cycle_end(mcan);
    flush_cycles(mcan, cia, settle_cycles);
    return rc;
}

static int all_enabled_axes_have_statusword(const igh_cia402_context_t *cia)
{
    if (cia == NULL) {
        return 0;
    }
    for (size_t i = 0U; i < cia->axis_count; i++) {
        if (cia->axes[i].enabled && cia->axes[i].statusword_valid == 0U) {
            return 0;
        }
    }
    return 1;
}

static int all_enabled_axes_operation_enabled(const igh_cia402_context_t *cia)
{
    if (cia == NULL) {
        return 0;
    }
    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];
        if (!axis->enabled) {
            continue;
        }
        if (axis->statusword_valid == 0U) {
            return 0;
        }
        if ((axis->statusword & IGH_CIA402_STATUSWORD_STATE_MASK) !=
            IGH_CIA402_STATUSWORD_OPERATION_ENABLED) {
            return 0;
        }
    }
    return 1;
}

static int request_statusword_all_timeout(igh_mcan_port_t *mcan,
                                          igh_cia402_context_t *cia,
                                          uint32_t timeout_ms)
{
    uint64_t start_ms;
    cia402_cycle_pacer_t pacer;
    int rc;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        if (cia->axes[i].enabled) {
            cia->axes[i].statusword_valid = 0U;
        }
    }
    rc = send_sdo_upload_all_axes(mcan, cia, IGH_CIA402_IDX_STATUSWORD, 0x00U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    start_ms = demo_monotonic_ms();
    pacer_init(&pacer, mcan->igh->cycle_us);
    for (;;) {
        if (all_enabled_axes_have_statusword(cia)) {
            return IGH_ECAN_OK;
        }
        for (size_t i = 0U; i < cia->axis_count; i++) {
            igh_cia402_axis_t *axis = &cia->axes[i];
            if (axis->enabled &&
                axis_sdo_wait_done(axis,
                                   IGH_CIA402_IDX_STATUSWORD,
                                   0x00U,
                                   CIA402_SDO_WAIT_UPLOAD) < 0) {
                print_sdo_wait_pending(cia,
                                       IGH_CIA402_IDX_STATUSWORD,
                                       0x00U,
                                       CIA402_SDO_WAIT_UPLOAD,
                                       "parallel-statusword");
                return IGH_ECAN_FAIL;
            }
        }
        if (demo_timeout_expired(start_ms, timeout_ms)) {
            print_sdo_wait_pending(cia,
                                   IGH_CIA402_IDX_STATUSWORD,
                                   0x00U,
                                   CIA402_SDO_WAIT_UPLOAD,
                                   "parallel-statusword");
            return IGH_ECAN_TIMEOUT;
        }
        run_one_cycle_io(mcan, cia);
        pacer_wait_next(&pacer);
    }
}

static int enable_all_axes_parallel(igh_mcan_port_t *mcan,
                                    igh_cia402_context_t *cia,
                                    const cia402_10axis_demo_config_t *config)
{
    int rc;
    int any_fault = 0;

    if ((mcan == NULL) || (cia == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = request_statusword_all_timeout(mcan, cia, config->sdo_timeout_ms);
    if (rc != IGH_ECAN_OK && rc != IGH_ECAN_TIMEOUT) {
        return rc;
    }
    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];
        if (axis->enabled &&
            axis->statusword_valid != 0U &&
            ((axis->statusword & IGH_CIA402_STATUSWORD_STATE_MASK) == 0x0008U)) {
            any_fault = 1;
        }
    }
    if (any_fault) {
        rc = send_controlword_all_and_flush(mcan,
                                            cia,
                                            IGH_CIA402_CONTROLWORD_FAULT_RESET,
                                            100U);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }

    rc = send_controlword_all_and_flush(mcan,
                                        cia,
                                        IGH_CIA402_CONTROLWORD_SHUTDOWN,
                                        20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = send_controlword_all_and_flush(mcan,
                                        cia,
                                        IGH_CIA402_CONTROLWORD_SWITCH_ON,
                                        20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = send_controlword_all_and_flush(mcan,
                                        cia,
                                        IGH_CIA402_CONTROLWORD_ENABLE_OP,
                                        20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = request_statusword_all_timeout(mcan, cia, config->sdo_timeout_ms);
    if (rc != IGH_ECAN_OK && rc != IGH_ECAN_TIMEOUT) {
        return rc;
    }
    if (rc == IGH_ECAN_TIMEOUT) {
        for (size_t i = 0U; i < cia->axis_count; i++) {
            igh_cia402_axis_t *axis = &cia->axes[i];

            if (axis->enabled &&
                axis->statusword_valid == 0U &&
                axis->emcy_valid == 0U &&
                ((axis->heartbeat_valid != 0U &&
                  axis->heartbeat == IGH_CIA402_HEARTBEAT_OPERATIONAL) ||
                 axis->actual_position_valid != 0U)) {
                axis->statusword = IGH_CIA402_STATUSWORD_OPERATION_ENABLED;
                axis->statusword_valid = 1U;
            }
        }
    }

    for (unsigned attempt = 0U; attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES; attempt++) {
        if (all_enabled_axes_operation_enabled(cia)) {
            flush_cycles(mcan, cia, config->enable_settle_cycles);
            rc = request_statusword_all_timeout(mcan, cia, config->sdo_timeout_ms);
            if (rc != IGH_ECAN_OK && rc != IGH_ECAN_TIMEOUT) {
                return rc;
            }
            if (all_enabled_axes_operation_enabled(cia)) {
                return IGH_ECAN_OK;
            }
            fprintf(stderr,
                    "enable post-check failed after settle; retry or fail before motion\n");
            if (attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
                break;
            }
            continue;
        }
        if (attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            break;
        }
        printf("[diag] enable-op retry attempt=%u/%u\n",
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES);
        rc = send_controlword_all_and_flush(mcan,
                                            cia,
                                            IGH_CIA402_CONTROLWORD_ENABLE_OP,
                                            config->enable_settle_cycles);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        rc = request_statusword_all_timeout(mcan, cia, config->sdo_timeout_ms);
        if (rc != IGH_ECAN_OK && rc != IGH_ECAN_TIMEOUT) {
            return rc;
        }
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];
        if (axis->enabled) {
            fprintf(stderr,
                    "enable failed: slave=%d ch=%u node=0x%02X"
                    " status_valid=%u status=0x%04X state=%s\n",
                    axis->slave_pos,
                    axis->channel,
                    axis->node_id,
                    axis->statusword_valid,
                    axis->statusword,
                    igh_cia402_statusword_state_name(axis->statusword));
        }
    }
    return IGH_ECAN_NOT_READY;
}

static int startup_all_axes_parallel_transactional(igh_mcan_port_t *mcan,
                                                   igh_cia402_context_t *cia,
                                                   const cia402_10axis_demo_config_t *config)
{
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    printf("[diag] startup mode=axis-parallel-transaction axes=%zu timeout=%" PRIu32
           "ms settle nmt=%u cfg=%u pos=%u enable=%u\n",
           count_enabled_axes(cia->axes, cia->axis_count),
           config->sdo_timeout_ms,
           config->nmt_start_settle_cycles,
           config->configure_settle_cycles,
           config->read_position_settle_cycles,
           config->enable_settle_cycles);
    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];
        if (axis->enabled) {
            printf("[diag] axis-startup begin slave=%d ch=%u node=0x%02X mode=%s\n",
                   axis->slave_pos,
                   axis->channel,
                   axis->node_id,
                   config->csv_pdo_mode ? "csv-rpdo-remap" :
                   (config->skip_csp_config ? "stored-rpdo" : "csp-rpdo-remap"));
        }
    }

    rc = ensure_all_axes_canopen_mode_parallel(mcan, cia, config);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    rc = send_enabled_channel_nmt_broadcasts_and_flush(mcan,
                                                       cia,
                                                       IGH_CIA402_NMT_START_REMOTE_NODE,
                                                       config->nmt_start_settle_cycles);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    if (config->csv_sdo_mode != 0) {
        rc = set_all_axes_operation_mode_parallel(mcan,
                                                  cia,
                                                  config,
                                                  config->csv_operation_mode,
                                                  "parallel-velocity-mode");
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        flush_cycles(mcan, cia, config->configure_settle_cycles);
    } else if (config->csv_pdo_mode != 0) {
        for (size_t i = 0U; i < cia->axis_count; i++) {
            igh_cia402_axis_t *axis = &cia->axes[i];

            if (!axis->enabled) {
                continue;
            }
            rc = configure_axis_csv_rpdo(mcan,
                                         cia,
                                         axis,
                                         config->sdo_timeout_ms,
                                         config->configure_settle_cycles,
                                         config->verbose);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    } else if (!config->skip_csp_config) {
        for (size_t i = 0U; i < cia->axis_count; i++) {
            igh_cia402_axis_t *axis = &cia->axes[i];

            if (!axis->enabled) {
                continue;
            }
            rc = configure_axis_csp_rpdo(mcan,
                                         cia,
                                         axis,
                                         config->sdo_timeout_ms,
                                         config->configure_settle_cycles,
                                         config->verbose);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    } else {
        rc = set_all_axes_operation_mode_parallel(mcan,
                                                  cia,
                                                  config,
                                                  IGH_CIA402_MODE_CSP,
                                                  "parallel-csp-mode");
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        flush_cycles(mcan, cia, config->configure_settle_cycles);
    }

    rc = send_enabled_channel_nmt_broadcasts_and_flush(mcan,
                                                       cia,
                                                       IGH_CIA402_NMT_START_REMOTE_NODE,
                                                       config->nmt_start_settle_cycles);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    for (unsigned attempt = 0U;
         attempt <= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES;
         attempt++) {
        rc = igh_cia402_request_actual_position_all_timeout(cia, config->sdo_timeout_ms);
        if (rc == IGH_ECAN_OK) {
            break;
        }
        if ((rc == IGH_ECAN_TIMEOUT || axis_sdo_needs_retry(rc, &cia->axes[0])) &&
            attempt >= CIA402_10AXIS_DEMO_CONFIGURE_RETRIES) {
            printf("[warn] enabled axes did not answer 0x6064 position request;"
                   " continue with target=0 for missing positions.\n");
            for (size_t i = 0U; i < cia->axis_count; i++) {
                igh_cia402_axis_t *axis = &cia->axes[i];

                if (axis->enabled && axis->actual_position_valid == 0U) {
                    axis->actual_position = 0;
                    axis->target_position = 0;
                    axis->actual_position_valid = 1U;
                }
            }
            break;
        }
        if (rc != IGH_ECAN_TIMEOUT && !axis_sdo_needs_retry(rc, &cia->axes[0])) {
            return rc;
        }
        printf("[diag] read-position parallel retry attempt=%u/%u rc=%d;"
               " send all channel NMT start.\n",
               attempt + 1U,
               CIA402_10AXIS_DEMO_CONFIGURE_RETRIES,
               rc);
        rc = send_enabled_channel_nmt_broadcasts_and_flush(mcan,
                                                           cia,
                                                           IGH_CIA402_NMT_START_REMOTE_NODE,
                                                           config->read_position_settle_cycles);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }

    rc = send_enabled_channel_nmt_broadcasts_and_flush(mcan,
                                                       cia,
                                                       IGH_CIA402_NMT_START_REMOTE_NODE,
                                                       config->enable_settle_cycles);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    if (config->csv_sdo_mode == 0 && config->csv_pdo_mode == 0) {
        rc = prime_csp_targets_to_actual(mcan,
                                         cia,
                                         config->read_position_settle_cycles,
                                         "parallel-pre-enable");
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
    rc = enable_all_axes_parallel(mcan, cia, config);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];
        if (!axis->enabled) {
            continue;
        }
        printf("[diag] axis-startup done slave=%d ch=%u node=0x%02X"
               " status=0x%04X state=%s actual=%" PRId32 "\n",
               axis->slave_pos,
               axis->channel,
               axis->node_id,
               axis->statusword,
               igh_cia402_statusword_state_name(axis->statusword),
               axis->actual_position);
    }
    return IGH_ECAN_OK;
}

static void print_axis_snapshot(const igh_cia402_axis_t *axes, size_t axis_count)
{
    for (size_t i = 0U; i < axis_count; i++) {
        const igh_cia402_axis_t *axis = &axes[i];
        if (!axis->enabled) {
            continue;
        }
        printf("axis slave=%d ch=%u node=0x%02X heartbeat_valid=%u heartbeat=0x%02X"
               " heartbeat_state=%s ready=%d"
               " status_valid=%u status=0x%04X state=%s actual_valid=%u"
               " actual=%" PRIi32 " target=%" PRIi32
               " velocity_valid=%u actual_velocity=%" PRIi32 " target_velocity=%" PRIi32
               " rpdo_len=%u mode_valid=%u mode=%d"
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
               axis->actual_velocity_valid,
               axis->actual_velocity,
               axis->target_velocity,
               axis->rpdo1_payload_len,
               axis->mode_display_valid,
               axis->mode_display,
               axis->sdo_response_index,
               axis->sdo_response_subindex,
               axis->sdo_response_command,
               axis->sdo_abort_code,
               igh_cia402_sdo_abort_name(axis->sdo_abort_code),
               axis->emcy_count,
               axis->emcy_code,
               axis->emcy_error_register);
    }
}

static void capture_axis_events(const igh_cia402_axis_t *axes,
                                size_t axis_count,
                                uint32_t *emcy_counts,
                                uint8_t *heartbeats)
{
    if ((axes == NULL) || (emcy_counts == NULL) || (heartbeats == NULL)) {
        return;
    }
    for (size_t i = 0U; i < axis_count; i++) {
        emcy_counts[i] = axes[i].emcy_count;
        heartbeats[i] = axes[i].heartbeat;
    }
}

static int axis_emcy_changed(const igh_cia402_axis_t *axes,
                             size_t axis_count,
                             const uint32_t *last_emcy_counts)
{
    if ((axes == NULL) || (last_emcy_counts == NULL)) {
        return 0;
    }
    for (size_t i = 0U; i < axis_count; i++) {
        if (axes[i].enabled && axes[i].emcy_count != last_emcy_counts[i]) {
            return 1;
        }
    }
    return 0;
}

static void print_axis_events(const igh_cia402_axis_t *axes,
                              size_t axis_count,
                              uint32_t *last_emcy_counts,
                              uint8_t *last_heartbeats,
                              unsigned cycle)
{
    if ((axes == NULL) || (last_emcy_counts == NULL) || (last_heartbeats == NULL)) {
        return;
    }
    for (size_t i = 0U; i < axis_count; i++) {
        const igh_cia402_axis_t *axis = &axes[i];
        if (!axis->enabled) {
            continue;
        }
        if (axis->emcy_count != last_emcy_counts[i]) {
            printf("[event] cycle=%u EMCY slave=%d ch=%u node=0x%02X count=%" PRIu32
                   " code=0x%04X err_reg=0x%02X data=%02X %02X %02X %02X %02X\n",
                   cycle,
                   axis->slave_pos,
                   axis->channel,
                   axis->node_id,
                   axis->emcy_count,
                   axis->emcy_code,
                   axis->emcy_error_register,
                   axis->emcy_data[0],
                   axis->emcy_data[1],
                   axis->emcy_data[2],
                   axis->emcy_data[3],
                   axis->emcy_data[4]);
            last_emcy_counts[i] = axis->emcy_count;
        }
        if (axis->heartbeat != last_heartbeats[i]) {
            printf("[event] cycle=%u heartbeat slave=%d ch=%u node=0x%02X 0x%02X(%s)->0x%02X(%s)\n",
                   cycle,
                   axis->slave_pos,
                   axis->channel,
                   axis->node_id,
                   last_heartbeats[i],
                   igh_cia402_heartbeat_state_name(last_heartbeats[i]),
                   axis->heartbeat,
                   igh_cia402_heartbeat_state_name(axis->heartbeat));
            last_heartbeats[i] = axis->heartbeat;
        }
    }
}

static int send_targets_with_diag(igh_mcan_port_t *mcan,
                                  igh_cia402_context_t *cia,
                                  size_t *ready_count,
                                  size_t *sent_count,
                                  size_t *skipped_count,
                                  cia402_target_trace_t *trace,
                                  size_t trace_count)
{
    int rc = IGH_ECAN_OK;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ready_count != NULL) {
        *ready_count = 0U;
    }
    if (sent_count != NULL) {
        *sent_count = 0U;
    }
    if (skipped_count != NULL) {
        *skipped_count = 0U;
    }
    if (trace != NULL) {
        memset(trace, 0, sizeof(trace[0]) * trace_count);
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        igh_cia402_target_step_t target_step;
        igh_cia402_tx_frame_t target_frame;
        int send_rc;

        if (!axis->enabled) {
            continue;
        }
        if (!axis_ready_for_motion(axis)) {
            if (skipped_count != NULL) {
                (*skipped_count)++;
            }
            continue;
        }
        if (ready_count != NULL) {
            (*ready_count)++;
        }
        send_rc = igh_cia402_prepare_target_step(axis, &target_step);
        if (send_rc != IGH_ECAN_OK) {
            fprintf(stderr,
                    "[diag] target-send-error stage=prepare axis_index=%zu"
                    " slave=%d ch=%u node=0x%02X rc=%d\n",
                    i,
                    axis->slave_pos,
                    axis->channel,
                    axis->node_id,
                    send_rc);
        }
        if (send_rc == IGH_ECAN_OK) {
            send_rc = igh_cia402_build_prepared_target_frame(cia,
                                                             axis,
                                                             &target_step,
                                                             &target_frame);
            if (send_rc != IGH_ECAN_OK) {
                fprintf(stderr,
                        "[diag] target-send-error stage=build axis_index=%zu"
                        " slave=%d ch=%u node=0x%02X rc=%d\n",
                        i,
                        axis->slave_pos,
                        axis->channel,
                        axis->node_id,
                        send_rc);
            }
        }
        if (send_rc == IGH_ECAN_OK) {
            send_rc = igh_cia402_mcan_send_frame(mcan, &target_frame);
            if (send_rc != IGH_ECAN_OK) {
                fprintf(stderr,
                        "[diag] target-send-error stage=send axis_index=%zu"
                        " slave=%d ch=%u node=0x%02X can_id=0x%03X len=%u rc=%d\n",
                        i,
                        axis->slave_pos,
                        axis->channel,
                        axis->node_id,
                        target_frame.frame.can_id,
                        target_frame.frame.data_len,
                        send_rc);
            }
        }
        if (send_rc == IGH_ECAN_OK) {
            send_rc = igh_cia402_set_target_position(axis, target_step.next_position);
            if (send_rc != IGH_ECAN_OK) {
                fprintf(stderr,
                        "[diag] target-send-error stage=set-target axis_index=%zu"
                        " slave=%d ch=%u node=0x%02X target=%" PRId32 " rc=%d\n",
                        i,
                        axis->slave_pos,
                        axis->channel,
                        axis->node_id,
                        target_step.next_position,
                        send_rc);
            }
        }
        if (send_rc == IGH_ECAN_OK) {
            if ((trace != NULL) && (i < trace_count)) {
                trace[i].sent = 1U;
                trace[i].data_len = target_step.data_len;
                trace[i].target_position = target_step.next_position;
                memcpy(trace[i].data, target_step.data, sizeof(trace[i].data));
            }
            if (sent_count != NULL) {
                (*sent_count)++;
            }
        } else if (rc == IGH_ECAN_OK) {
            rc = send_rc;
        }
    }
    return rc;
}

static int send_csv_velocities_with_diag(igh_mcan_port_t *mcan,
                                         igh_cia402_context_t *cia,
                                         int32_t target_velocity,
                                         size_t *ready_count,
                                         size_t *sent_count,
                                         size_t *skipped_count,
                                         cia402_target_trace_t *trace,
                                         size_t trace_count)
{
    int rc = IGH_ECAN_OK;

    if ((mcan == NULL) || (cia == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ready_count != NULL) {
        *ready_count = 0U;
    }
    if (sent_count != NULL) {
        *sent_count = 0U;
    }
    if (skipped_count != NULL) {
        *skipped_count = 0U;
    }
    if (trace != NULL) {
        memset(trace, 0, sizeof(trace[0]) * trace_count);
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        igh_cia402_axis_t *axis = &cia->axes[i];
        igh_cia402_tx_frame_t velocity_frame;
        int send_rc;

        if (!axis->enabled) {
            continue;
        }
        if (!axis_ready_for_motion(axis)) {
            if (skipped_count != NULL) {
                (*skipped_count)++;
            }
            continue;
        }
        if (ready_count != NULL) {
            (*ready_count)++;
        }
        send_rc = igh_cia402_build_velocity_frame(cia,
                                                  axis,
                                                  target_velocity,
                                                  &velocity_frame);
        if (send_rc != IGH_ECAN_OK) {
            fprintf(stderr,
                    "[diag] csv-velocity-send-error stage=build axis_index=%zu"
                    " slave=%d ch=%u node=0x%02X rc=%d\n",
                    i,
                    axis->slave_pos,
                    axis->channel,
                    axis->node_id,
                    send_rc);
        }
        if (send_rc == IGH_ECAN_OK) {
            send_rc = igh_cia402_mcan_send_frame(mcan, &velocity_frame);
            if (send_rc != IGH_ECAN_OK) {
                fprintf(stderr,
                        "[diag] csv-velocity-send-error stage=send axis_index=%zu"
                        " slave=%d ch=%u node=0x%02X can_id=0x%03X len=%u rc=%d\n",
                        i,
                        axis->slave_pos,
                        axis->channel,
                        axis->node_id,
                        velocity_frame.frame.can_id,
                        velocity_frame.frame.data_len,
                        send_rc);
            }
        }
        if (send_rc == IGH_ECAN_OK) {
            axis->target_velocity = target_velocity;
            if ((trace != NULL) && (i < trace_count)) {
                trace[i].sent = 1U;
                trace[i].data_len = velocity_frame.frame.data_len;
                trace[i].target_position = target_velocity;
                memcpy(trace[i].data, velocity_frame.frame.data, sizeof(trace[i].data));
            }
            if (sent_count != NULL) {
                (*sent_count)++;
            }
        } else if (rc == IGH_ECAN_OK) {
            rc = send_rc;
        }
    }
    return rc;
}

static int send_sync_for_sent_targets(igh_mcan_port_t *mcan,
                                      igh_cia402_context_t *cia,
                                      const cia402_target_trace_t *trace,
                                      size_t trace_count)
{
    uint8_t sync_sent[ECAN_IGH_MAX_SLAVES][IGH_MCAN_CHANNEL_COUNT] = {{0}};
    int rc = IGH_ECAN_OK;

    if ((mcan == NULL) || (cia == NULL) || (trace == NULL) ||
        trace_count < cia->axis_count) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t i = 0U; i < cia->axis_count; i++) {
        const igh_cia402_axis_t *axis = &cia->axes[i];

        if (trace[i].sent == 0U ||
            axis->slave_pos < 0 ||
            (size_t)axis->slave_pos >= ECAN_IGH_MAX_SLAVES ||
            axis->channel >= IGH_MCAN_CHANNEL_COUNT ||
            sync_sent[(size_t)axis->slave_pos][axis->channel] != 0U) {
            continue;
        }
        sync_sent[(size_t)axis->slave_pos][axis->channel] = 1U;
        igh_cia402_tx_frame_t sync_frame;
        int sync_rc = igh_cia402_build_sync_frame(cia,
                                                  axis->slave_pos,
                                                  axis->channel,
                                                  &sync_frame);
        if (sync_rc == IGH_ECAN_OK) {
            sync_rc = igh_cia402_mcan_send_frame(mcan, &sync_frame);
        }
        if (rc == IGH_ECAN_OK) {
            rc = sync_rc;
        }
    }
    return rc;
}

static int write_csv_velocity_all(igh_mcan_port_t *mcan,
                                  igh_cia402_context_t *cia,
                                  int32_t velocity,
                                  uint32_t timeout_ms,
                                  const char *stage)
{
    return sdo_write_all_axes_timeout(mcan,
                                      cia,
                                      IGH_CIA402_SDO_WRITE_U32,
                                      IGH_CIA402_IDX_TARGET_VELOCITY,
                                      IGH_CIA402_SUBIDX_VALUE,
                                      (uint32_t)velocity,
                                      timeout_ms,
                                      stage);
}

static int write_csv_velocity_all_nowait(igh_mcan_port_t *mcan,
                                         igh_cia402_context_t *cia,
                                         int32_t velocity,
                                         unsigned settle_cycles)
{
    int rc = send_sdo_write_all_axes(mcan,
                                     cia,
                                     IGH_CIA402_SDO_WRITE_U32,
                                     IGH_CIA402_IDX_TARGET_VELOCITY,
                                     IGH_CIA402_SUBIDX_VALUE,
                                     (uint32_t)velocity);

    if (rc == IGH_ECAN_OK) {
        flush_cycles(mcan, cia, settle_cycles);
    }
    return rc;
}

static void stop_csv_velocity_best_effort(igh_mcan_port_t *mcan,
                                          igh_cia402_context_t *cia,
                                          const cia402_10axis_demo_config_t *config)
{
    if ((mcan == NULL) || (cia == NULL) || (config == NULL)) {
        return;
    }
    for (unsigned i = 0U; i < 3U; i++) {
        int rc = write_csv_velocity_all_nowait(mcan, cia, 0, 20U);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "[warn] CSV target velocity stop send failed attempt=%u rc=%d\n",
                    i + 1U,
                    rc);
        }
    }
    {
        int rc = write_csv_velocity_all(mcan,
                                        cia,
                                        0,
                                        config->sdo_timeout_ms,
                                        "csv-target-velocity-stop");
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr,
                    "[warn] CSV target velocity stop ACK timeout/failed rc=%d; zero command was sent best-effort.\n",
                    rc);
        }
    }
}

static int run_csv_pdo_motion(igh_mcan_port_t *mcan,
                              igh_cia402_context_t *cia,
                              const cia402_10axis_demo_config_t *config,
                              igh_cia402_axis_t *axes,
                              size_t axis_count,
                              uint32_t *last_emcy_counts,
                              uint8_t *last_heartbeats)
{
    cia402_cycle_pacer_t motion_pacer;
    uint32_t ethercat_guard_misses = 0U;
    uint32_t ethercat_guard_total_misses = 0U;
    uint32_t ethercat_guard_max_streak = 0U;

    if ((mcan == NULL) || (cia == NULL) || (config == NULL) ||
        (axes == NULL) || (last_emcy_counts == NULL) || (last_heartbeats == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    printf("[diag] CSV-PDO velocity active velocity=%" PRId32 " cycles=%u\n",
           config->csv_target_velocity,
           config->motion_cycles);

    pacer_init(&motion_pacer, mcan->igh->cycle_us);
    for (unsigned loop = 0U; loop < config->motion_cycles; loop++) {
        igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
        size_t frame_count = 0U;
        cia402_target_trace_t velocity_trace[CIA402_10AXIS_DEMO_AXIS_MAX];
        size_t ready_count = 0U;
        size_t sent_count = 0U;
        size_t skipped_count = 0U;
        int motion_rc;

        igh_mcan_port_cycle_begin(mcan);
        if (!ethercat_ready_for_motion(mcan) ||
            ecan_igh_domain_is_complete(mcan->igh) == 0) {
            ethercat_guard_misses++;
            ethercat_guard_total_misses++;
            if (ethercat_guard_misses > ethercat_guard_max_streak) {
                ethercat_guard_max_streak = ethercat_guard_misses;
            }
            if (ethercat_guard_total_misses <= 3U ||
                (config->diag_every != 0U &&
                 (ethercat_guard_total_misses % config->diag_every) == 0U)) {
                print_ethercat_motion_guard(mcan,
                                            loop + 1U,
                                            config->ethercat_guard_fatal ? "fatal" : "warn");
            }
            igh_mcan_port_cycle_end(mcan);
            if (config->ethercat_guard_fatal &&
                (ethercat_guard_misses >= config->ethercat_guard_max_misses)) {
                return IGH_ECAN_FAIL;
            }
            pacer_wait_next(&motion_pacer);
            continue;
        }
        if (ethercat_guard_misses != 0U) {
            printf("[diag] ethercat-motion-recovered cycle=%u missed=%" PRIu32
                   " total_missed=%" PRIu32 " max_streak=%" PRIu32 "\n",
                   loop + 1U,
                   ethercat_guard_misses,
                   ethercat_guard_total_misses,
                   ethercat_guard_max_streak);
            ethercat_guard_misses = 0U;
        }
        (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
        (void)process_mcan_frames(cia, frames, frame_count);
        if (config->stop_on_emcy &&
            axis_emcy_changed(axes, axis_count, last_emcy_counts)) {
            print_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats, loop + 1U);
            igh_mcan_port_cycle_end(mcan);
            printf("[stage] stop-on-emcy cycle=%u\n", loop + 1U);
            break;
        }
        print_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats, loop + 1U);
        motion_rc = send_csv_velocities_with_diag(mcan,
                                                  cia,
                                                  config->csv_target_velocity,
                                                  &ready_count,
                                                  &sent_count,
                                                  &skipped_count,
                                                  velocity_trace,
                                                  axis_count);
        if (motion_rc == IGH_ECAN_OK) {
            motion_rc = verify_target_pdo_trace(mcan,
                                                axes,
                                                axis_count,
                                                velocity_trace,
                                                axis_count,
                                                loop + 1U);
        }
        if (motion_rc == IGH_ECAN_OK) {
            motion_rc = send_sync_for_sent_targets(mcan, cia, velocity_trace, axis_count);
        }
        if (motion_rc != IGH_ECAN_OK) {
            igh_mcan_port_cycle_abort(mcan);
            fprintf(stderr,
                    "csv-pdo motion send failed cycle=%u rc=%d ready=%zu sent=%zu skipped=%zu\n",
                    loop + 1U,
                    motion_rc,
                    ready_count,
                    sent_count,
                    skipped_count);
            print_axis_snapshot(axes, axis_count);
            print_mcan_tpdo_diag(mcan, "csv-pdo-motion-send-failed");
            return motion_rc;
        }
        igh_mcan_port_cycle_end(mcan);
        if ((config->diag_every != 0U) && (((loop + 1U) % config->diag_every) == 0U)) {
            printf("[diag] csv-pdo-motion cycle=%u/%u ready=%zu sent=%zu skipped=%zu"
                   " velocity=%" PRId32
                   " ethercat_missed_total=%" PRIu32
                   " ethercat_missed_max_streak=%" PRIu32 "\n",
                   loop + 1U,
                   config->motion_cycles,
                   ready_count,
                   sent_count,
                   skipped_count,
                   config->csv_target_velocity,
                   ethercat_guard_total_misses,
                   ethercat_guard_max_streak);
            if (sent_count == 0U) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "csv-pdo-motion-no-send");
            }
        }
        pacer_wait_next(&motion_pacer);
    }

    if (ethercat_guard_total_misses != 0U) {
        printf("[diag] ethercat-motion-guard-summary total_missed=%" PRIu32
               " max_streak=%" PRIu32 " mode=%s\n",
               ethercat_guard_total_misses,
               ethercat_guard_max_streak,
               config->ethercat_guard_fatal ? "fatal" : "warn");
    }
    (void)write_csv_velocity_all_nowait(mcan, cia, 0, 3U);
    if (config->traffic_fill_mode != 0) {
        print_stage("traffic-fill-complete");
        print_axis_snapshot(axes, axis_count);
    }
    return IGH_ECAN_OK;
}

static int run_csv_sdo_motion(igh_mcan_port_t *mcan,
                              igh_cia402_context_t *cia,
                              const cia402_10axis_demo_config_t *config,
                              igh_cia402_axis_t *axes,
                              size_t axis_count,
                              uint32_t *last_emcy_counts,
                              uint8_t *last_heartbeats)
{
    cia402_cycle_pacer_t motion_pacer;
    uint32_t ethercat_guard_misses = 0U;
    uint32_t ethercat_guard_total_misses = 0U;
    uint32_t ethercat_guard_max_streak = 0U;
    int rc;

    if ((mcan == NULL) || (cia == NULL) || (config == NULL) ||
        (axes == NULL) || (last_emcy_counts == NULL) || (last_heartbeats == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = write_csv_velocity_all(mcan,
                                cia,
                                config->csv_target_velocity,
                                config->sdo_timeout_ms,
                                "csv-target-velocity-start");
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "CSV target velocity start failed rc=%d velocity=%" PRId32 "\n",
                rc,
                config->csv_target_velocity);
        return rc;
    }
    printf("[diag] CSV-SDO velocity active velocity=%" PRId32 " cycles=%u refresh_cycles=%" PRIu32 "\n",
           config->csv_target_velocity,
           config->motion_cycles,
           config->csv_refresh_cycles);

    pacer_init(&motion_pacer, mcan->igh->cycle_us);
    for (unsigned loop = 0U; loop < config->motion_cycles; loop++) {
        igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
        size_t frame_count = 0U;

        igh_mcan_port_cycle_begin(mcan);
        if (!ethercat_ready_for_motion(mcan) ||
            ecan_igh_domain_is_complete(mcan->igh) == 0) {
            ethercat_guard_misses++;
            ethercat_guard_total_misses++;
            if (ethercat_guard_misses > ethercat_guard_max_streak) {
                ethercat_guard_max_streak = ethercat_guard_misses;
            }
            if (ethercat_guard_total_misses <= 3U ||
                (config->diag_every != 0U &&
                 (ethercat_guard_total_misses % config->diag_every) == 0U)) {
                print_ethercat_motion_guard(mcan,
                                            loop + 1U,
                                            config->ethercat_guard_fatal ? "fatal" : "warn");
            }
            igh_mcan_port_cycle_end(mcan);
            if (config->ethercat_guard_fatal &&
                (ethercat_guard_misses >= config->ethercat_guard_max_misses)) {
                stop_csv_velocity_best_effort(mcan, cia, config);
                return IGH_ECAN_FAIL;
            }
            pacer_wait_next(&motion_pacer);
            continue;
        }
        if (ethercat_guard_misses != 0U) {
            printf("[diag] ethercat-motion-recovered cycle=%u missed=%" PRIu32
                   " total_missed=%" PRIu32 " max_streak=%" PRIu32 "\n",
                   loop + 1U,
                   ethercat_guard_misses,
                   ethercat_guard_total_misses,
                   ethercat_guard_max_streak);
            ethercat_guard_misses = 0U;
        }
        (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
        (void)process_mcan_frames(cia, frames, frame_count);
        if (config->stop_on_emcy &&
            axis_emcy_changed(axes, axis_count, last_emcy_counts)) {
            print_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats, loop + 1U);
            igh_mcan_port_cycle_end(mcan);
            printf("[stage] stop-on-emcy cycle=%u\n", loop + 1U);
            break;
        }
        print_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats, loop + 1U);
        igh_mcan_port_cycle_end(mcan);

        if (config->csv_refresh_cycles != 0U &&
            (((loop + 1U) % config->csv_refresh_cycles) == 0U)) {
            int refresh_rc = write_csv_velocity_all_nowait(mcan,
                                                           cia,
                                                           config->csv_target_velocity,
                                                           1U);
            if (refresh_rc != IGH_ECAN_OK) {
                stop_csv_velocity_best_effort(mcan, cia, config);
                fprintf(stderr,
                        "CSV target velocity refresh failed cycle=%u rc=%d velocity=%" PRId32 "\n",
                        loop + 1U,
                        refresh_rc,
                        config->csv_target_velocity);
                return refresh_rc;
            }
        }
        if ((config->diag_every != 0U) && (((loop + 1U) % config->diag_every) == 0U)) {
            printf("[diag] csv-motion cycle=%u/%u velocity=%" PRId32
                   " ethercat_missed_total=%" PRIu32
                   " ethercat_missed_max_streak=%" PRIu32 "\n",
                   loop + 1U,
                   config->motion_cycles,
                   config->csv_target_velocity,
                   ethercat_guard_total_misses,
                   ethercat_guard_max_streak);
            print_axis_snapshot(axes, axis_count);
        }
        pacer_wait_next(&motion_pacer);
    }

    if (ethercat_guard_total_misses != 0U) {
        printf("[diag] ethercat-motion-guard-summary total_missed=%" PRIu32
               " max_streak=%" PRIu32 " mode=%s\n",
               ethercat_guard_total_misses,
               ethercat_guard_max_streak,
               config->ethercat_guard_fatal ? "fatal" : "warn");
    }
    stop_csv_velocity_best_effort(mcan, cia, config);
    return IGH_ECAN_OK;
}

static int run_10axis_demo(igh_mcan_port_t *mcan,
                           const cia402_10axis_demo_config_t *config)
{
    cia402_10axis_demo_config_t local_config;
    igh_cia402_context_t cia;
    igh_cia402_axis_t axes[CIA402_10AXIS_DEMO_AXIS_MAX];
    uint32_t last_emcy_counts[CIA402_10AXIS_DEMO_AXIS_MAX] = {0};
    uint8_t last_heartbeats[CIA402_10AXIS_DEMO_AXIS_MAX] = {0};
    size_t axis_count = 0U;
    int schedule_rc;
    uint32_t ethercat_guard_misses = 0U;
    uint32_t ethercat_guard_total_misses = 0U;
    uint32_t ethercat_guard_max_streak = 0U;
    igh_cia402_mcan_transport_t transport;

    if (mcan == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (config == NULL) {
        demo_default_config(&local_config);
        config = &local_config;
    }

    build_axes(axes, &axis_count, config->target_step, config);
    disable_axes_without_slave(axes, axis_count, mcan->igh->slave_count);
    igh_cia402_mcan_transport_init(&transport, mcan);
    igh_cia402_context_init(&cia, &transport.transport, axes, axis_count);
    schedule_rc = igh_cia402_set_scheduled_tx(&cia,
                                              config->scheduled_target,
                                              config->scheduled_target_offset_ns,
                                              config->scheduled_sync,
                                              config->scheduled_sync_offset_ns);
    if (schedule_rc != IGH_ECAN_OK) {
        fprintf(stderr, "invalid scheduled CiA402 configuration\n");
        return schedule_rc;
    }

    printf("[diag] enabled_axes=%zu disabled_axes=%zu ethercat_wait_op=%" PRIu32 "ms"
           " warmup=%" PRIu32 "/%" PRIu32
           " sdo_timeout=%" PRIu32 "ms diag_every=%" PRIu32
           " ethercat_guard=%s ethercat_guard_max_misses=%" PRIu32
           " motion_div=%" PRIu32 " motion_mode=%s velocity_mode=%u csv_velocity=%" PRId32
           " traffic_fill=%d pdo_remap=%s"
           " scheduled_target=%d target_offset_ns=%" PRIu32
           " scheduled_sync=%d sync_offset_ns=%" PRIu32
           " only_axis=%d\n",
           count_enabled_axes(axes, axis_count),
           axis_count - count_enabled_axes(axes, axis_count),
           config->wait_op_timeout_ms,
           config->warmup_cycles,
           config->warmup_stable_cycles,
           config->sdo_timeout_ms,
           config->diag_every,
           config->ethercat_guard_fatal ? "fatal" : "warn",
           config->ethercat_guard_max_misses,
           config->motion_divisor,
           config->csv_sdo_mode ? "csv-sdo" :
           (config->csv_pdo_mode ? "csv-rpdo" : "csp-rpdo"),
           config->csv_operation_mode,
           config->csv_target_velocity,
           config->traffic_fill_mode,
           config->skip_csp_config ? "skip" : "rpdo-verify",
           config->scheduled_target,
           config->scheduled_target_offset_ns,
           config->scheduled_sync,
           config->scheduled_sync_offset_ns,
           config->only_axis);
    print_axis_snapshot(axes, axis_count);
    if (config->traffic_fill_mode != 0) {
        print_stage("traffic-fill-ready");
        if (config->only_axis != 0 && axis_count > 1U) {
            int saved_enabled[CIA402_10AXIS_DEMO_AXIS_MAX];

            printf("[diag] traffic-fill starts runtime axis first, then uses remaining"
                   " selected topology slots as synthetic load.\n");
            for (size_t i = 0U; i < axis_count; i++) {
                saved_enabled[i] = axes[i].enabled;
                axes[i].enabled = (i == 0U) ? axes[i].enabled : 0;
            }
            if (!config->skip_nmt_reset) {
                print_stage("nmt-reset-primary");
                if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_RESET_NODE,
                                       config->nmt_reset_settle_cycles) != IGH_ECAN_OK) {
                    return IGH_ECAN_FAIL;
                }
            }
            print_stage("axis-startup-primary");
            if (config->csv_pdo_mode != 0) {
                printf("[diag] CSV-PDO startup remaps RPDO1=6040/6060/60FF"
                       " and TPDO1=606C/6041/6061; velocity is sent by RPDO and latched by SYNC.\n");
            } else if (config->csv_sdo_mode != 0) {
                printf("[diag] CSV-SDO startup sets 6060=%u and drives 60FF target velocity by SDO.\n",
                       config->csv_operation_mode);
            } else if (config->skip_csp_config) {
                printf("[diag] CSP startup uses stored RPDO1=6040/6060/607A mapping;"
                       " target is sent by RPDO and latched by SYNC.\n");
            } else {
                printf("[diag] CSP startup remaps RPDO1=6040/6060/607A;"
                       " target is sent by RPDO and latched by SYNC.\n");
            }
            if (((config->csv_sdo_mode != 0) ?
                 startup_axis_transaction(mcan, &cia, &axes[0], config) :
                 startup_all_axes_parallel_transactional(mcan, &cia, config)) != IGH_ECAN_OK) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "axis-startup-primary-failed");
                return IGH_ECAN_FAIL;
            }
            print_mcan_tpdo_diag(mcan, "after-axis-startup-primary");

            print_stage("nmt-start-primary");
            if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_START_REMOTE_NODE,
                                   config->nmt_start_settle_cycles) != IGH_ECAN_OK) {
                return IGH_ECAN_FAIL;
            }
            for (size_t i = 0U; i < axis_count; i++) {
                axes[i].enabled = saved_enabled[i];
            }
            if (config->csv_pdo_mode != 0) {
                prime_traffic_fill_axes_mode(&axes[1],
                                             axis_count - 1U,
                                             IGH_CIA402_MODE_CSV,
                                             config->csv_target_velocity);
            } else {
                prime_traffic_fill_axes(&axes[1], axis_count - 1U);
            }
        } else {
            if (config->csv_pdo_mode != 0) {
                prime_traffic_fill_axes_mode(axes,
                                             axis_count,
                                             IGH_CIA402_MODE_CSV,
                                             config->csv_target_velocity);
                printf("[diag] traffic-fill skips CiA402 startup and sends CSV RPDO/SYNC"
                       " for every enabled axis to simulate occupied motors.\n");
            } else {
                prime_traffic_fill_axes(axes, axis_count);
                printf("[diag] traffic-fill skips CiA402 startup and sends CSP RPDO/SYNC"
                       " for every enabled axis to simulate occupied motors.\n");
            }
        }
    } else {
        if (!config->skip_nmt_reset) {
            print_stage("nmt-reset");
            if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_RESET_NODE,
                                   config->nmt_reset_settle_cycles) != IGH_ECAN_OK) {
                return IGH_ECAN_FAIL;
            }
            if (config->verbose) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "after-nmt-reset");
            }
        }

        print_stage("axis-startup");
        if (config->csv_pdo_mode != 0) {
            printf("[diag] CSV-PDO startup remaps RPDO1=6040/6060/60FF"
                   " and TPDO1=606C/6041/6061; velocity is sent by RPDO and latched by SYNC.\n");
        } else if (config->csv_sdo_mode != 0) {
            printf("[diag] CSV-SDO startup sets 6060=%u and drives 60FF target velocity by SDO.\n",
                   config->csv_operation_mode);
        } else if (config->skip_csp_config) {
            printf("[diag] CSP startup uses stored RPDO1=6040/6060/607A mapping;"
                   " target is sent by RPDO and latched by SYNC.\n");
        } else {
            printf("[diag] CSP startup remaps RPDO1=6040/6060/607A;"
                   " target is sent by RPDO and latched by SYNC.\n");
        }
        print_axis_snapshot(axes, axis_count);
        if (config->csv_sdo_mode != 0 && axis_count == 1U) {
            if (startup_axis_transaction(mcan, &cia, &axes[0], config) != IGH_ECAN_OK) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "axis-startup-failed");
                return IGH_ECAN_FAIL;
            }
        } else if (startup_all_axes_parallel_transactional(mcan, &cia, config) != IGH_ECAN_OK) {
            print_axis_snapshot(axes, axis_count);
            print_mcan_tpdo_diag(mcan, "axis-startup-failed");
            return IGH_ECAN_FAIL;
        }
        print_mcan_tpdo_diag(mcan, "after-axis-startup");

        print_stage("nmt-start-all");
        if (send_nmt_and_flush(mcan, &cia, IGH_CIA402_NMT_START_REMOTE_NODE,
                               config->nmt_start_settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
    }

    print_axis_snapshot(axes, axis_count);
    capture_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats);
    print_stage("motion");
    if (config->csv_pdo_mode != 0) {
        int csv_rc = run_csv_pdo_motion(mcan,
                                        &cia,
                                        config,
                                        axes,
                                        axis_count,
                                        last_emcy_counts,
                                        last_heartbeats);
        if (csv_rc != IGH_ECAN_OK) {
            print_axis_snapshot(axes, axis_count);
            print_mcan_tpdo_diag(mcan, "csv-pdo-motion-failed");
            return csv_rc;
        }
        print_stage("read-position-final");
        (void)igh_cia402_request_actual_position_all_timeout(
            &cia,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
        if (stop_and_flush(mcan, &cia, config->stop_settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
        print_axis_snapshot(axes, axis_count);
        return IGH_ECAN_OK;
    }

    if (config->csv_sdo_mode != 0) {
        int csv_rc = run_csv_sdo_motion(mcan,
                                        &cia,
                                        config,
                                        axes,
                                        axis_count,
                                        last_emcy_counts,
                                        last_heartbeats);
        if (csv_rc != IGH_ECAN_OK) {
            print_axis_snapshot(axes, axis_count);
            print_mcan_tpdo_diag(mcan, "csv-motion-failed");
            return csv_rc;
        }
        print_stage("read-position-final");
        (void)igh_cia402_request_actual_position_all_timeout(
            &cia,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
        if (stop_and_flush(mcan, &cia, config->stop_settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
        print_axis_snapshot(axes, axis_count);
        return IGH_ECAN_OK;
    }
    cia402_cycle_pacer_t motion_pacer;
    pacer_init(&motion_pacer, mcan->igh->cycle_us);
    for (unsigned loop = 0U; loop < config->motion_cycles; loop++) {
        igh_mcan_rx_frame_t frames[CIA402_10AXIS_DEMO_RX_CAPACITY];
        size_t frame_count = 0U;
        cia402_target_trace_t target_trace[CIA402_10AXIS_DEMO_AXIS_MAX];
        size_t ready_count = 0U;
        size_t sent_count = 0U;
        size_t skipped_count = 0U;
        int motion_rc;

        igh_mcan_port_cycle_begin(mcan);
        if (!ethercat_ready_for_motion(mcan)) {
            ethercat_guard_misses++;
            ethercat_guard_total_misses++;
            if (ethercat_guard_misses > ethercat_guard_max_streak) {
                ethercat_guard_max_streak = ethercat_guard_misses;
            }
            if (ethercat_guard_misses == 1U ||
                (config->diag_every != 0U &&
                 (ethercat_guard_misses % config->diag_every) == 0U)) {
                print_ethercat_motion_guard(mcan,
                                            loop + 1U,
                                            config->ethercat_guard_fatal ? "fatal" : "warn");
                print_mcan_tpdo_diag(mcan, "ethercat-motion-hold");
            }
            igh_mcan_port_cycle_end(mcan);
            if (config->ethercat_guard_fatal &&
                (ethercat_guard_misses >= config->ethercat_guard_max_misses)) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "ethercat-motion-guard");
                return IGH_ECAN_FAIL;
            }
            pacer_wait_next(&motion_pacer);
            continue;
        }
        if (ecan_igh_domain_is_complete(mcan->igh) == 0) {
            ethercat_guard_misses++;
            ethercat_guard_total_misses++;
            if (ethercat_guard_misses > ethercat_guard_max_streak) {
                ethercat_guard_max_streak = ethercat_guard_misses;
            }
            if (ethercat_guard_total_misses <= 3U ||
                (config->diag_every != 0U &&
                 (ethercat_guard_total_misses % config->diag_every) == 0U)) {
                print_ethercat_motion_guard(mcan,
                                            loop + 1U,
                                            config->ethercat_guard_fatal ? "fatal" : "warn");
                print_mcan_tpdo_diag(mcan, "ethercat-motion-continue");
            }
            if (config->ethercat_guard_fatal &&
                (ethercat_guard_misses >= config->ethercat_guard_max_misses)) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "ethercat-motion-guard");
                igh_mcan_port_cycle_abort(mcan);
                return IGH_ECAN_FAIL;
            }
        } else if (ethercat_guard_misses != 0U) {
            if (ethercat_guard_misses > 1U ||
                ethercat_guard_total_misses <= 3U ||
                (config->diag_every != 0U &&
                 (ethercat_guard_total_misses % config->diag_every) == 0U)) {
                printf("[diag] ethercat-motion-recovered cycle=%u missed=%" PRIu32
                       " total_missed=%" PRIu32 " max_streak=%" PRIu32 "\n",
                       loop + 1U,
                       ethercat_guard_misses,
                       ethercat_guard_total_misses,
                       ethercat_guard_max_streak);
            }
            ethercat_guard_misses = 0U;
        }
        (void)igh_mcan_port_receive(mcan, frames, CIA402_10AXIS_DEMO_RX_CAPACITY, &frame_count);
        (void)process_mcan_frames(&cia, frames, frame_count);
        if (config->stop_on_emcy &&
            axis_emcy_changed(axes, axis_count, last_emcy_counts)) {
            print_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats, loop + 1U);
            igh_mcan_port_cycle_end(mcan);
            printf("[stage] stop-on-emcy cycle=%u\n", loop + 1U);
            break;
        }
        print_axis_events(axes, axis_count, last_emcy_counts, last_heartbeats, loop + 1U);
        if ((config->motion_divisor > 1U) &&
            (((loop + 1U) % config->motion_divisor) != 0U)) {
            igh_mcan_port_cycle_end(mcan);
            pacer_wait_next(&motion_pacer);
            continue;
        }
        motion_rc = send_targets_with_diag(mcan,
                                           &cia,
                                           &ready_count,
                                           &sent_count,
                                           &skipped_count,
                                           target_trace,
                                           axis_count);
        if (motion_rc == IGH_ECAN_OK) {
            motion_rc = verify_target_pdo_trace(mcan,
                                                axes,
                                                axis_count,
                                                target_trace,
                                                axis_count,
                                                loop + 1U);
        }
        if (motion_rc == IGH_ECAN_OK) {
            motion_rc = send_sync_for_sent_targets(mcan, &cia, target_trace, axis_count);
        }
        if (motion_rc != IGH_ECAN_OK) {
            igh_mcan_port_cycle_abort(mcan);
            fprintf(stderr, "motion send failed cycle=%u rc=%d ready=%zu sent=%zu skipped=%zu\n",
                    loop + 1U,
                    motion_rc,
                    ready_count,
                    sent_count,
                    skipped_count);
            print_axis_snapshot(axes, axis_count);
            print_mcan_tpdo_diag(mcan, "motion-send-failed");
            return motion_rc;
        }
        igh_mcan_port_cycle_end(mcan);
        if ((config->diag_every != 0U) && (((loop + 1U) % config->diag_every) == 0U)) {
            printf("[diag] motion cycle=%u/%u ready=%zu sent=%zu skipped=%zu"
                   " first_target=%" PRIi32
                   " ethercat_missed_total=%" PRIu32
                   " ethercat_missed_max_streak=%" PRIu32 "\n",
                   loop + 1U,
                   config->motion_cycles,
                   ready_count,
                   sent_count,
                   skipped_count,
                   axis_count > 0U ? axes[0].target_position : 0,
                   ethercat_guard_total_misses,
                   ethercat_guard_max_streak);
            if (sent_count == 0U) {
                print_axis_snapshot(axes, axis_count);
                print_mcan_tpdo_diag(mcan, "motion-no-send");
            }
        }
        pacer_wait_next(&motion_pacer);
    }

    if (ethercat_guard_misses != 0U) {
        printf("[diag] ethercat-motion-ended-with-hold missed=%" PRIu32
               " total_missed=%" PRIu32 " max_streak=%" PRIu32 "\n",
               ethercat_guard_misses,
               ethercat_guard_total_misses,
               ethercat_guard_max_streak);
    }
    if (ethercat_guard_total_misses != 0U) {
        printf("[diag] ethercat-motion-guard-summary total_missed=%" PRIu32
               " max_streak=%" PRIu32 " mode=%s\n",
               ethercat_guard_total_misses,
               ethercat_guard_max_streak,
               config->ethercat_guard_fatal ? "fatal" : "warn");
    }
    if (config->traffic_fill_mode != 0) {
        print_stage("traffic-fill-complete");
        print_stage("read-position-final");
        (void)igh_cia402_request_actual_position_all_timeout(
            &cia,
            cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
        if (stop_and_flush(mcan, &cia, config->stop_settle_cycles) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
        print_axis_snapshot(axes, axis_count);
        return IGH_ECAN_OK;
    }

    print_stage("read-position-final");
    (void)igh_cia402_request_actual_position_all_timeout(
        &cia,
        cycles_to_timeout_ms(mcan, config->read_position_settle_cycles));
    if (stop_and_flush(mcan, &cia, config->stop_settle_cycles) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    print_axis_snapshot(axes, axis_count);
    return IGH_ECAN_OK;
}

static void print_usage(const char *app)
{
    printf("Motion requires --enable-motor after checking all motor slots and mechanical safety.\n");
    printf("usage: %s [--slaves N] [--pdo-n 5+3-encoding] [--pdo-map a,b,c] [--cycle-us N] [--cycles N] [--step N]\n",
           app);
    printf("          [--wait-op-ms N] [--warmup-cycles N] [--warmup-stable-cycles N]\n");
    printf("          [--sdo-timeout-ms N] [--diag-every N] [--motion-div N]\n");
    printf("          [--ethercat-guard-max-misses N] [--ethercat-guard-fatal] [--ethercat-guard-warn]\n");
    printf("          [--verbose]\n");
    printf("          [--traffic-fill]\n");
    printf("          [--align4] [--profile N] [--nmt-reset] [--skip-nmt-reset] [--csp-config] [--skip-csp-config]\n");
    printf("          [--csv-pdo] [--csv-rpdo] [--csv-sdo] [--csp-rpdo] [--velocity N] [--velocity-mode N] [--velocity-refresh N]\n");
    printf("          [--axis-mask HEX] [--slots-classic-can] [--slots-canfd] [--slots-bitrate N] [--slots-data-bitrate N]\n");
    printf("          [--only-axis] [--axis-slave N] [--axis-channel N] [--axis-node N]\n");
    printf("          [--classic-can] [--canfd] [--bitrate N] [--data-bitrate N]\n");
    printf("          [--skip-sdo-config] [--stop-on-emcy] [--no-mlock]\n");
    printf("          [--scheduled-tx] [--scheduled-target] [--scheduled-sync] [--direct-tx]\n");
    printf("          [--schedule-target-offset-ns N] [--schedule-sync-offset-ns N]\n");
    printf("          [--dc-sync0] [--dc-cycle-us N] [--dc-shift-us N]\n");
    printf("default: pdo-map=%u,%u,%u chunks (%u,%u,%u bytes), cycle=%u us, motion-div=%u, demo=10-axis CiA402 CSP, pdo_remap=stored\n",
           (unsigned)CIA402_10AXIS_DEMO_DEFAULT_PDO_MULTIPLIER,
           (unsigned)CIA402_10AXIS_DEMO_DEFAULT_PDO_MULTIPLIER,
           (unsigned)CIA402_10AXIS_DEMO_THIRD_SLAVE_PDO_MULTIPLIER,
           (unsigned)ecan_igh_pdo_decode_bytes(CIA402_10AXIS_DEMO_DEFAULT_PDO_MULTIPLIER),
           (unsigned)ecan_igh_pdo_decode_bytes(CIA402_10AXIS_DEMO_DEFAULT_PDO_MULTIPLIER),
           (unsigned)ecan_igh_pdo_decode_bytes(CIA402_10AXIS_DEMO_THIRD_SLAVE_PDO_MULTIPLIER),
           IGH_ECAN_DEFAULT_CYCLE_US,
           CIA402_10AXIS_DEMO_DEFAULT_MOTION_DIVISOR);
}

static int parse_pdo_map_arg(ecan_igh_config_t *igh_config, const char *arg)
{
    const char *p = arg;

    if ((igh_config == NULL) || (arg == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t slave = 0U; slave < igh_config->slave_count; slave++) {
        char *end = NULL;
        unsigned long value;

        if (*p == '\0') {
            return IGH_ECAN_INVALID_ARG;
        }
        value = strtoul(p, &end, 0);
        if (end == p || value > UINT8_MAX ||
            !ecan_igh_pdo_size_encoding_is_valid((uint8_t)value) ||
            ecan_igh_config_set_slave_pdo_multiplier(igh_config, slave, (uint8_t)value) !=
                IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
        p = end;
        if (slave + 1U < igh_config->slave_count) {
            if (*p != ',') {
                return IGH_ECAN_INVALID_ARG;
            }
            p++;
        }
    }
    return (*p == '\0') ? IGH_ECAN_OK : IGH_ECAN_INVALID_ARG;
}

static int parse_args(int argc,
                      char **argv,
                      ecan_igh_config_t *igh_config,
                      igh_mcan_port_config_t *mcan_config,
                      cia402_10axis_demo_config_t *demo_config)
{
    if ((igh_config == NULL) || (mcan_config == NULL) || (demo_config == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--enable-motor") == 0) {
            demo_config->enable_motor = 1;
        } else if (strcmp(argv[i], "--slaves") == 0 && i + 1 < argc) {
            if (ecan_igh_config_set_slave_count(igh_config, igh_example_size(argv[++i])) !=
                IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--pdo-n") == 0 && i + 1 < argc) {
            if (ecan_igh_config_set_pdo_multiplier(igh_config, igh_example_u8(argv[++i])) !=
                IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--pdo-map") == 0 && i + 1 < argc) {
            if (parse_pdo_map_arg(igh_config, argv[++i]) != IGH_ECAN_OK) {
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
        } else if (strcmp(argv[i], "--motion-div") == 0 && i + 1 < argc) {
            demo_config->motion_divisor = igh_example_u32(argv[++i]);
            if (demo_config->motion_divisor == 0U) {
                return IGH_ECAN_INVALID_ARG;
            }
        } else if (strcmp(argv[i], "--ethercat-guard-max-misses") == 0 && i + 1 < argc) {
            demo_config->ethercat_guard_max_misses = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--ethercat-guard-fatal") == 0) {
            demo_config->ethercat_guard_fatal = 1;
        } else if (strcmp(argv[i], "--ethercat-guard-warn") == 0) {
            demo_config->ethercat_guard_fatal = 0;
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
        } else if (strcmp(argv[i], "--stop-on-emcy") == 0) {
            demo_config->stop_on_emcy = 1;
        } else if (strcmp(argv[i], "--no-mlock") == 0) {
            demo_config->lock_memory = 0;
        } else if (strcmp(argv[i], "--traffic-fill") == 0) {
            demo_config->traffic_fill_mode = 1;
        } else if (strcmp(argv[i], "--csv-pdo") == 0 ||
                   strcmp(argv[i], "--csv-rpdo") == 0) {
            demo_config->csv_pdo_mode = 1;
            demo_config->csv_sdo_mode = 0;
            demo_config->skip_csp_config = 0;
        } else if (strcmp(argv[i], "--csv-sdo") == 0) {
            demo_config->csv_sdo_mode = 1;
            demo_config->csv_pdo_mode = 0;
        } else if (strcmp(argv[i], "--csp-rpdo") == 0) {
            demo_config->csv_sdo_mode = 0;
            demo_config->csv_pdo_mode = 0;
        } else if (strcmp(argv[i], "--velocity") == 0 && i + 1 < argc) {
            demo_config->csv_target_velocity = igh_example_i32(argv[++i]);
        } else if (strcmp(argv[i], "--velocity-mode") == 0 && i + 1 < argc) {
            unsigned mode = igh_example_uint(argv[++i]);
            if (mode > UINT8_MAX) {
                return IGH_ECAN_INVALID_ARG;
            }
            demo_config->csv_operation_mode = (uint8_t)mode;
            demo_config->csv_sdo_mode = 1;
            demo_config->csv_pdo_mode = 0;
        } else if (strcmp(argv[i], "--velocity-refresh") == 0 && i + 1 < argc) {
            demo_config->csv_refresh_cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--axis-mask") == 0 && i + 1 < argc) {
            demo_config->axis_mask = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--slots-classic-can") == 0) {
            demo_config->slot_canfd_override_valid = 1;
            demo_config->slot_canfd_override = IGH_ECAN_CANFD_DISABLE;
        } else if (strcmp(argv[i], "--slots-canfd") == 0) {
            demo_config->slot_canfd_override_valid = 1;
            demo_config->slot_canfd_override = IGH_ECAN_CANFD_ENABLE;
        } else if (strcmp(argv[i], "--slots-bitrate") == 0 && i + 1 < argc) {
            demo_config->slot_canfd_override_valid = 1;
            demo_config->slot_nominal_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--slots-data-bitrate") == 0 && i + 1 < argc) {
            demo_config->slot_canfd_override_valid = 1;
            demo_config->slot_data_bitrate = igh_example_u32(argv[++i]);
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
        } else if (strcmp(argv[i], "--only-axis") == 0) {
            demo_config->only_axis = 1;
            demo_config->axis_canfd = IGH_ECAN_CANFD_DISABLE;
        } else if (strcmp(argv[i], "--axis-slave") == 0 && i + 1 < argc) {
            demo_config->axis_slave_pos = igh_example_size(argv[++i]);
            demo_config->only_axis = 1;
        } else if (strcmp(argv[i], "--axis-channel") == 0 && i + 1 < argc) {
            unsigned channel = igh_example_uint(argv[++i]);
            if (channel >= IGH_MCAN_CHANNEL_COUNT) {
                return IGH_ECAN_INVALID_ARG;
            }
            demo_config->axis_channel = (igh_ecan_mcan_channel_t)channel;
            demo_config->only_axis = 1;
        } else if (strcmp(argv[i], "--axis-node") == 0 && i + 1 < argc) {
            demo_config->axis_node_id = igh_example_u8(argv[++i]);
            demo_config->only_axis = 1;
        } else if (strcmp(argv[i], "--classic-can") == 0) {
            demo_config->axis_canfd = IGH_ECAN_CANFD_DISABLE;
            demo_config->only_axis = 1;
        } else if (strcmp(argv[i], "--canfd") == 0) {
            demo_config->axis_canfd = IGH_ECAN_CANFD_ENABLE;
            demo_config->only_axis = 1;
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            demo_config->axis_nominal_bitrate = igh_example_u32(argv[++i]);
            demo_config->only_axis = 1;
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            demo_config->axis_data_bitrate = igh_example_u32(argv[++i]);
            demo_config->only_axis = 1;
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
    if (demo_config->scheduled_target != 0 &&
        demo_config->scheduled_sync != 0 &&
        demo_config->scheduled_sync_offset_ns <= demo_config->scheduled_target_offset_ns) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (demo_config->warmup_cycles != 0U &&
        demo_config->warmup_stable_cycles > demo_config->warmup_cycles) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (demo_config->only_axis != 0 &&
        (demo_config->axis_node_id == 0U ||
         demo_config->axis_node_id > IGH_ECAN_CANOPEN_NODE_ID_MAX ||
         demo_config->axis_nominal_bitrate == 0U ||
         demo_config->axis_data_bitrate == 0U)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (demo_config->only_axis == 0 &&
        demo_config->axis_mask != 0U &&
        count_selected_motor_slots(demo_config) == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (demo_config->slot_canfd_override_valid != 0 &&
        (demo_config->slot_nominal_bitrate == 0U ||
         demo_config->slot_data_bitrate == 0U)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if ((demo_config->scheduled_target != 0 || demo_config->scheduled_sync != 0) &&
        igh_config->dc_assign_activate == 0U &&
        ecan_igh_config_enable_dc_sync0(igh_config, 0U, 0) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    return IGH_ECAN_OK;
}

int main(int argc, char **argv)
{
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    cia402_10axis_demo_config_t demo_config;
    size_t configured_axis_count;
    int result;

    setvbuf(stdout, NULL, _IOLBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    demo_default_config(&demo_config);
    if (ecan_igh_config_apply_realtime_profile(
            &igh_config, ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK ||
        apply_default_pdo_layout(&igh_config) != IGH_ECAN_OK) {
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    if (parse_args(argc, argv, &igh_config, &mcan_config, &demo_config) != IGH_ECAN_OK ||
        apply_10axis_mcan_port_config(&igh_config, &mcan_config, &demo_config) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    if (!demo_config.enable_motor) {
        fprintf(stderr, "Motion disabled: explicit --enable-motor is required; no master requested.\n");
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    if (demo_config.lock_memory != 0 &&
        lock_memory_for_realtime() != IGH_ECAN_OK) {
        return IGH_ECAN_EXIT_FAIL;
    }

    configured_axis_count = (demo_config.only_axis != 0) ?
                            1U :
                            count_selected_motor_slots(&demo_config);
    print_topology(&igh_config, &mcan_config, configured_axis_count, &demo_config);

    printf("ECAN IgH 10-axis demo: slaves=%zu pdo-map=",
           igh_config.slave_count);
    for (size_t slave = 0U; slave < igh_config.slave_count; slave++) {
        printf("%s%u", (slave == 0U) ? "" : ",",
               (unsigned)ecan_igh_config_get_slave_pdo_bytes(&igh_config, slave));
    }
    printf(" bytes cycle=%" PRIu32 " us axes=%u",
           igh_config.cycle_us,
           (unsigned)configured_axis_count);
    if (igh_config.dc_assign_activate != 0U) {
        uint32_t dc_cycle_ns = (igh_config.dc_sync0_cycle_ns != 0U) ?
                               igh_config.dc_sync0_cycle_ns :
                               igh_config.cycle_us * 1000U;
        printf(" dc=0x%04x sync0=%" PRIu32 "ns shift=%" PRId32 "ns",
               igh_config.dc_assign_activate,
               dc_cycle_ns,
               igh_config.dc_sync0_shift_ns);
    }
    printf("\n");

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
                                  "10axis") != IGH_ECAN_OK) {
        if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
            ecan_igh_print_runtime_diagnostics(&igh, 16U);
        }
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    result = run_10axis_demo(&mcan, &demo_config);
    if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
        ecan_igh_print_runtime_diagnostics(&igh, 16U);
    }
    ecan_igh_close(&igh);
    if (demo_config.lock_memory != 0) {
        (void)munlockall();
    }
    return (result == IGH_ECAN_OK) ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_FAIL;
}
