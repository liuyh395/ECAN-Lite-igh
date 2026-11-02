#define _DEFAULT_SOURCE

#include "igh_cia402_protocol.h"

#include <inttypes.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define IGH_CIA402_TIMEOUT_RX_CAPACITY 64U
#define IGH_CIA402_CSP_SDO_STAGE_GAP_MS 0U
#define IGH_CIA402_SDO_VERIFY_READ_GAP_MS 100U
#define IGH_CIA402_SDO_OPTIONAL_READBACK_GAP_MS 50U
#define IGH_CIA402_SDO_OPTIONAL_READBACK_TIMEOUT_MS 200U

typedef enum {
    IGH_CIA402_SDO_VALUE_FIXED = 0,
    IGH_CIA402_SDO_VALUE_CYCLE_US,
    IGH_CIA402_SDO_VALUE_TPDO1_DISABLED,
    IGH_CIA402_SDO_VALUE_TPDO1_ENABLED,
    IGH_CIA402_SDO_VALUE_RPDO1_DISABLED,
    IGH_CIA402_SDO_VALUE_RPDO1_ENABLED
} igh_cia402_sdo_value_mode_t;

typedef struct {
    uint8_t command;
    uint16_t index;
    uint8_t subindex;
    uint32_t value;
    igh_cia402_sdo_value_mode_t value_mode;
} igh_cia402_sdo_stage_t;

static uint32_t get_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

static void put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static uint16_t get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

static void put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void merge_result(int *rc, int result)
{
    if ((rc != NULL) && (*rc == IGH_ECAN_OK) && (result != IGH_ECAN_OK)) {
        *rc = result;
    }
}

static uint32_t ctx_cycle_us(const igh_cia402_context_t *ctx)
{
    if ((ctx != NULL) && ctx->transport.cycle_us != 0U) {
        return ctx->transport.cycle_us;
    }
    return IGH_CIA402_CSP_SYNC_PERIOD_US_DEFAULT;
}

static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

static int timeout_expired(uint64_t start_ms, uint32_t timeout_ms)
{
    return (timeout_ms != 0U && monotonic_ms() - start_ms >= timeout_ms) ? 1 : 0;
}

static int process_rx_once(igh_cia402_context_t *ctx);
static int process_rx_in_open_cycle(igh_cia402_context_t *ctx);
static int sdo_write_timeout(igh_cia402_context_t *ctx,
                             igh_cia402_axis_t *axis,
                             uint8_t command,
                             uint16_t index,
                             uint8_t subindex,
                             uint32_t value,
                             uint32_t timeout_ms);
static int sdo_write_all_timeout(igh_cia402_context_t *ctx,
                                 uint8_t command,
                                 uint16_t index,
                                 uint8_t subindex,
                                 uint32_t value,
                                 igh_cia402_sdo_value_mode_t value_mode,
                                 uint32_t timeout_ms);
static int run_idle_ms(igh_cia402_context_t *ctx, uint32_t duration_ms);
static void clear_sdo_response(igh_cia402_axis_t *axis);
static uint8_t sdo_upload_size_from_command(uint8_t command);
static uint32_t sdo_mask_for_size(uint8_t size);
static uint16_t next_schedule_tag(igh_cia402_context_t *ctx);
static int send_nmt_once(igh_cia402_context_t *ctx,
                         const igh_cia402_axis_t *axis,
                         uint8_t command);
static int send_controlword_noack(igh_cia402_context_t *ctx,
                                  igh_cia402_axis_t *axis,
                                  uint16_t controlword);
static int send_controlword_step(igh_cia402_context_t *ctx,
                                 igh_cia402_axis_t *axis,
                                 uint16_t controlword,
                                 uint32_t settle_ms);

static void sleep_one_cycle(const igh_cia402_context_t *ctx)
{
    uint32_t cycle_us = ctx_cycle_us(ctx);

    if ((ctx != NULL) && ctx->transport.sleep_us != NULL) {
        ctx->transport.sleep_us(ctx->transport.user, cycle_us);
        return;
    }
    usleep(cycle_us);
}

static int transport_is_ready(const igh_cia402_context_t *ctx)
{
    return (ctx != NULL && ctx->transport.send_can != NULL) ? 1 : 0;
}

static int transport_can_receive(const igh_cia402_context_t *ctx)
{
    return (ctx != NULL && ctx->transport.receive != NULL) ? 1 : 0;
}

static void transport_cycle_begin(igh_cia402_context_t *ctx)
{
    if ((ctx != NULL) && (ctx->transport.cycle_begin != NULL)) {
        ctx->transport.cycle_begin(ctx->transport.user);
    }
}

static void transport_cycle_end(igh_cia402_context_t *ctx)
{
    if ((ctx != NULL) && (ctx->transport.cycle_end != NULL)) {
        ctx->transport.cycle_end(ctx->transport.user);
    }
}

static int transport_receive(igh_cia402_context_t *ctx,
                             igh_cia402_rx_frame_t *frames,
                             size_t capacity,
                             size_t *count)
{
    if ((ctx == NULL) || (ctx->transport.receive == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    return ctx->transport.receive(ctx->transport.user, frames, capacity, count);
}

static igh_cia402_axis_t *find_axis_match(igh_cia402_context_t *ctx,
                                          int slave_pos,
                                          uint8_t channel,
                                          uint8_t node_id)
{
    if (ctx == NULL) {
        return NULL;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        igh_cia402_axis_t *axis = &ctx->axes[i];
        if (axis->enabled &&
            (slave_pos < 0 || axis->slave_pos == slave_pos) &&
            axis->channel == channel &&
            axis->node_id == node_id) {
            return axis;
        }
    }
    return NULL;
}

static int send_channel_can(igh_cia402_context_t *ctx,
                            int slave_pos,
                            uint8_t channel,
                            uint32_t can_id,
                            const uint8_t *data,
                            size_t len,
                            int scheduled,
                            uint32_t offset_ns)
{
    if (!transport_is_ready(ctx) ||
        ((data == NULL) && (len > 0U))) {
        return IGH_ECAN_INVALID_ARG;
    }
    return ctx->transport.send_can(ctx->transport.user,
                                   slave_pos,
                                   channel,
                                   can_id,
                                   data,
                                   len,
                                   scheduled,
                                   offset_ns,
                                   (scheduled != 0) ? next_schedule_tag(ctx) : 0U);
}

static uint16_t next_schedule_tag(igh_cia402_context_t *ctx)
{
    uint16_t tag;

    if (ctx == NULL) {
        return 0U;
    }
    tag = ctx->scheduled_tag_next++;
    if (ctx->scheduled_tag_next == 0U) {
        ctx->scheduled_tag_next = 1U;
    }
    return tag;
}

static void tx_frame_init(igh_cia402_tx_frame_t *out,
                          int slave_pos,
                          uint8_t channel,
                          uint32_t can_id,
                          const uint8_t *data,
                          size_t data_len)
{
    memset(out, 0, sizeof(*out));
    out->slave_pos = slave_pos;
    out->frame.channel = channel;
    out->frame.can_id = can_id;
    out->frame.data_len = (uint8_t)data_len;
    if (data_len > 0U) {
        memcpy(out->frame.data, data, data_len);
    }
}

static void tx_frame_apply_schedule(igh_cia402_context_t *ctx,
                                    igh_cia402_tx_frame_t *out,
                                    int scheduled,
                                    uint32_t offset_ns)
{
    if ((ctx == NULL) || (out == NULL) || (scheduled == 0)) {
        return;
    }
    out->scheduled = 1U;
    out->offset_ns = offset_ns;
    out->tag = next_schedule_tag(ctx);
}

static int send_tx_frame(igh_cia402_context_t *ctx, const igh_cia402_tx_frame_t *tx)
{
    if (!transport_is_ready(ctx) || (tx == NULL) ||
        (tx->frame.data_len > sizeof(tx->frame.data))) {
        return IGH_ECAN_INVALID_ARG;
    }
    return ctx->transport.send_can(ctx->transport.user,
                                   tx->slave_pos,
                                   tx->frame.channel,
                                   tx->frame.can_id,
                                   tx->frame.data,
                                   tx->frame.data_len,
                                   tx->scheduled,
                                   tx->offset_ns,
                                   tx->tag);
}

static int send_axis_can_and_process_rx(igh_cia402_context_t *ctx,
                                        const igh_cia402_axis_t *axis,
                                        uint32_t can_id,
                                        const uint8_t *data,
                                        size_t len)
{
    igh_cia402_rx_frame_t frames[IGH_CIA402_TIMEOUT_RX_CAPACITY];
    size_t count = 0U;
    int rc = IGH_ECAN_OK;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx) || (axis == NULL) ||
        ((data == NULL) && (len > 0U))) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    merge_result(&rc, transport_receive(ctx, frames, IGH_CIA402_TIMEOUT_RX_CAPACITY, &count));
    merge_result(&rc, send_channel_can(ctx,
                                       axis->slave_pos,
                                       axis->channel,
                                       can_id,
                                       data,
                                       len,
                                       0,
                                       0U));
    transport_cycle_end(ctx);

    for (size_t i = 0U; i < count; i++) {
        int frame_rc = igh_cia402_on_can_frame(ctx, frames[i].slave_pos, &frames[i].frame);
        if ((rc == IGH_ECAN_OK) && (frame_rc != IGH_ECAN_OK)) {
            rc = frame_rc;
        }
    }
    return (rc == IGH_ECAN_NO_SPACE) ? IGH_ECAN_OK : rc;
}

static int send_nmt_once(igh_cia402_context_t *ctx,
                         const igh_cia402_axis_t *axis,
                         uint8_t command)
{
    uint8_t data[2];
    int rc;

    if (!transport_is_ready(ctx) || (axis == NULL) ||
        axis->node_id == 0U || axis->node_id > IGH_ECAN_CANOPEN_NODE_ID_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }

    data[0] = command;
    data[1] = axis->node_id;
    transport_cycle_begin(ctx);
    rc = send_channel_can(ctx, axis->slave_pos, axis->channel,
                          IGH_CIA402_COB_NMT, data, sizeof(data), 0, 0U);
    transport_cycle_end(ctx);
    if (rc == IGH_ECAN_OK) {
        sleep_one_cycle(ctx);
    }
    return rc;
}

static int send_controlword_noack(igh_cia402_context_t *ctx,
                                  igh_cia402_axis_t *axis,
                                  uint16_t controlword)
{
    uint8_t data[8] = {0};

    if (!transport_is_ready(ctx) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    data[0] = IGH_CIA402_SDO_WRITE_U16;
    data[1] = (uint8_t)(IGH_CIA402_IDX_CONTROLWORD & 0xFFU);
    data[2] = (uint8_t)(IGH_CIA402_IDX_CONTROLWORD >> 8);
    data[3] = IGH_CIA402_SUBIDX_VALUE;
    data[4] = (uint8_t)(controlword & 0xFFU);
    data[5] = (uint8_t)(controlword >> 8);
    clear_sdo_response(axis);
    axis->sdo_response_index = IGH_CIA402_IDX_CONTROLWORD;
    axis->sdo_response_subindex = IGH_CIA402_SUBIDX_VALUE;
    return send_axis_can_and_process_rx(ctx,
                                        axis,
                                        IGH_CIA402_COB_SDO_REQUEST_BASE + axis->node_id,
                                        data,
                                        sizeof(data));
}

static int send_controlword_step(igh_cia402_context_t *ctx,
                                 igh_cia402_axis_t *axis,
                                 uint16_t controlword,
                                 uint32_t settle_ms)
{
    int rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_START_REMOTE_NODE);

    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = send_controlword_noack(ctx, axis, controlword);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return run_idle_ms(ctx, settle_ms);
}

static int sdo_download(igh_cia402_context_t *ctx,
                        const igh_cia402_axis_t *axis,
                        uint8_t command,
                        uint16_t index,
                        uint8_t subindex,
                        uint32_t value)
{
    igh_cia402_tx_frame_t tx;
    int rc = igh_cia402_build_sdo_write_frame(axis,
                                              command,
                                              index,
                                              subindex,
                                              value,
                                              &tx);

    return (rc == IGH_ECAN_OK) ? send_tx_frame(ctx, &tx) : rc;
}

static void clear_sdo_response(igh_cia402_axis_t *axis)
{
    if (axis == NULL) {
        return;
    }
    axis->sdo_response_valid = 0U;
    axis->sdo_response_command = 0U;
    axis->sdo_response_index = 0U;
    axis->sdo_response_subindex = 0U;
    axis->sdo_response_size = 0U;
    axis->sdo_response_value = 0U;
    axis->sdo_abort_code = 0U;
}

static uint8_t sdo_upload_size_from_command(uint8_t command)
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

static int sdo_response_matches(const igh_cia402_axis_t *axis,
                                uint16_t index,
                                uint8_t subindex)
{
    return ((axis != NULL) &&
            (axis->sdo_response_valid != 0U) &&
            (axis->sdo_response_index == index) &&
            (axis->sdo_response_subindex == subindex)) ? 1 : 0;
}

static int sdo_abort_seen(const igh_cia402_axis_t *axis,
                          uint16_t index,
                          uint8_t subindex)
{
    return (sdo_response_matches(axis, index, subindex) &&
            axis->sdo_response_command == IGH_CIA402_SDO_ABORT) ? 1 : 0;
}

static uint32_t csp_stage_value(const igh_cia402_context_t *ctx,
                                const igh_cia402_axis_t *axis,
                                uint32_t fixed_value,
                                igh_cia402_sdo_value_mode_t value_mode)
{
    switch (value_mode) {
    case IGH_CIA402_SDO_VALUE_CYCLE_US:
        return ctx_cycle_us(ctx);
    case IGH_CIA402_SDO_VALUE_TPDO1_DISABLED:
        return IGH_CIA402_COB_DISABLED_MASK |
               IGH_CIA402_COB_TPDO1_BASE |
               axis->node_id;
    case IGH_CIA402_SDO_VALUE_TPDO1_ENABLED:
        return IGH_CIA402_COB_TPDO1_BASE | axis->node_id;
    case IGH_CIA402_SDO_VALUE_RPDO1_DISABLED:
        return IGH_CIA402_COB_DISABLED_MASK |
               IGH_CIA402_COB_RPDO1_BASE |
               axis->node_id;
    case IGH_CIA402_SDO_VALUE_RPDO1_ENABLED:
        return IGH_CIA402_COB_RPDO1_BASE | axis->node_id;
    case IGH_CIA402_SDO_VALUE_FIXED:
    default:
        return fixed_value;
    }
}

static void pack_csp_rpdo1_payload(uint8_t *data, int32_t target_position)
{
    put_u16_le(&data[0], IGH_CIA402_CONTROLWORD_ENABLE_OP);
    data[2] = IGH_CIA402_MODE_CSP;
    put_u32_le(&data[3], (uint32_t)target_position);
}

static void pack_csv_rpdo1_payload(uint8_t *data, int32_t target_velocity)
{
    put_u16_le(&data[0], IGH_CIA402_CONTROLWORD_ENABLE_OP);
    data[2] = IGH_CIA402_MODE_CSV;
    put_u32_le(&data[3], (uint32_t)target_velocity);
}

static uint8_t axis_rpdo1_payload_len(const igh_cia402_axis_t *axis)
{
    if ((axis != NULL) &&
        (axis->rpdo1_payload_len == IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN ||
         axis->rpdo1_payload_len == IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN)) {
        return axis->rpdo1_payload_len;
    }
    return IGH_CIA402_RPDO1_TARGET_ONLY_PAYLOAD_LEN;
}

static void mark_csp_rpdo1_mapping_active(igh_cia402_axis_t *axis)
{
    if (axis != NULL) {
        axis->rpdo1_payload_len = IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN;
    }
}

static void mark_csp_rpdo1_mapping_active_all(igh_cia402_context_t *ctx)
{
    if (ctx == NULL) {
        return;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        if (ctx->axes[i].enabled) {
            mark_csp_rpdo1_mapping_active(&ctx->axes[i]);
        }
    }
}

static void mark_csv_rpdo1_mapping_active(igh_cia402_axis_t *axis)
{
    if (axis != NULL) {
        axis->rpdo1_payload_len = IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN;
    }
}

static int all_enabled_axes_have_sdo_ack(const igh_cia402_context_t *ctx,
                                         uint16_t index,
                                         uint8_t subindex,
                                         uint8_t expected_command)
{
    if (ctx == NULL) {
        return 0;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        const igh_cia402_axis_t *axis = &ctx->axes[i];

        if (!axis->enabled) {
            continue;
        }
        if (!sdo_response_matches(axis, index, subindex)) {
            return 0;
        }
        if (axis->sdo_response_command == IGH_CIA402_SDO_ABORT ||
            axis->sdo_response_command != expected_command) {
            return -1;
        }
    }
    return 1;
}

void igh_cia402_context_init(igh_cia402_context_t *ctx,
                             const igh_cia402_transport_t *transport,
                             igh_cia402_axis_t *axes,
                             size_t axis_count)
{
    if (ctx == NULL) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
    if (transport != NULL) {
        ctx->transport = *transport;
    }
    ctx->axes = axes;
    ctx->axis_count = axis_count;
    ctx->scheduled_target_enabled = 0U;
    ctx->scheduled_sync_enabled = 0U;
    ctx->scheduled_target_offset_ns = 0U;
    ctx->scheduled_sync_offset_ns = 0U;
    ctx->scheduled_tag_next = 1U;
}

int igh_cia402_set_scheduled_tx(igh_cia402_context_t *ctx,
                                int target_enabled,
                                uint32_t target_offset_ns,
                                int sync_enabled,
                                uint32_t sync_offset_ns)
{
    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    if ((target_enabled != 0) && (sync_enabled != 0) &&
        sync_offset_ns <= target_offset_ns) {
        return IGH_ECAN_INVALID_ARG;
    }
    ctx->scheduled_target_enabled = (target_enabled != 0) ? 1U : 0U;
    ctx->scheduled_sync_enabled = (sync_enabled != 0) ? 1U : 0U;
    ctx->scheduled_target_offset_ns = target_offset_ns;
    ctx->scheduled_sync_offset_ns = sync_offset_ns;
    if (ctx->scheduled_tag_next == 0U) {
        ctx->scheduled_tag_next = 1U;
    }
    return IGH_ECAN_OK;
}

void igh_cia402_axis_init(igh_cia402_axis_t *axis,
                          int slave_pos,
                          uint8_t channel,
                          uint8_t node_id,
                          int32_t target_step)
{
    if (axis == NULL) {
        return;
    }
    memset(axis, 0, sizeof(*axis));
    axis->enabled = 1;
    axis->slave_pos = slave_pos;
    axis->channel = channel;
    axis->node_id = node_id;
    axis->step = target_step;
    axis->rpdo1_payload_len = IGH_CIA402_RPDO1_TARGET_ONLY_PAYLOAD_LEN;
}

igh_cia402_axis_t *igh_cia402_find_axis(igh_cia402_context_t *ctx,
                                        int slave_pos,
                                        uint8_t channel,
                                        uint8_t node_id)
{
    if (slave_pos < 0) {
        return NULL;
    }
    return find_axis_match(ctx, slave_pos, channel, node_id);
}

int igh_cia402_build_nmt_frame(const igh_cia402_axis_t *axis,
                               uint8_t command,
                               igh_cia402_tx_frame_t *out)
{
    uint8_t data[2];

    if ((axis == NULL) || (out == NULL) ||
        axis->node_id == 0U || axis->node_id > IGH_ECAN_CANOPEN_NODE_ID_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }
    data[0] = command;
    data[1] = axis->node_id;
    tx_frame_init(out, axis->slave_pos, axis->channel, IGH_CIA402_COB_NMT,
                  data, sizeof(data));
    return IGH_ECAN_OK;
}

int igh_cia402_build_nmt_broadcast_frame(int slave_pos,
                                         uint8_t channel,
                                         uint8_t command,
                                         igh_cia402_tx_frame_t *out)
{
    uint8_t data[2];

    if ((out == NULL) || (slave_pos < 0)) {
        return IGH_ECAN_INVALID_ARG;
    }
    data[0] = command;
    data[1] = IGH_CIA402_SUBIDX_VALUE;
    tx_frame_init(out, slave_pos, channel, IGH_CIA402_COB_NMT,
                  data, sizeof(data));
    return IGH_ECAN_OK;
}

int igh_cia402_build_sync_frame(igh_cia402_context_t *ctx,
                                int slave_pos,
                                uint8_t channel,
                                igh_cia402_tx_frame_t *out)
{
    if ((ctx == NULL) || (out == NULL) || (slave_pos < 0)) {
        return IGH_ECAN_INVALID_ARG;
    }
    tx_frame_init(out, slave_pos, channel, IGH_CIA402_COB_SYNC, NULL, 0U);
    tx_frame_apply_schedule(ctx, out,
                            ctx->scheduled_sync_enabled,
                            ctx->scheduled_sync_offset_ns);
    return IGH_ECAN_OK;
}

int igh_cia402_build_sdo_upload_frame(const igh_cia402_axis_t *axis,
                                      uint16_t index,
                                      uint8_t subindex,
                                      igh_cia402_tx_frame_t *out)
{
    uint8_t data[8] = {0};

    if ((axis == NULL) || (out == NULL) ||
        axis->node_id == 0U || axis->node_id > IGH_ECAN_CANOPEN_NODE_ID_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }
    data[0] = IGH_CIA402_SDO_UPLOAD;
    data[1] = (uint8_t)(index & 0xFFU);
    data[2] = (uint8_t)(index >> 8);
    data[3] = subindex;
    tx_frame_init(out,
                  axis->slave_pos,
                  axis->channel,
                  IGH_CIA402_COB_SDO_REQUEST_BASE + axis->node_id,
                  data,
                  sizeof(data));
    return IGH_ECAN_OK;
}

int igh_cia402_build_sdo_write_frame(const igh_cia402_axis_t *axis,
                                     uint8_t command,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint32_t value,
                                     igh_cia402_tx_frame_t *out)
{
    uint8_t data[8] = {0};

    if ((axis == NULL) || (out == NULL) ||
        axis->node_id == 0U || axis->node_id > IGH_ECAN_CANOPEN_NODE_ID_MAX) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (command != IGH_CIA402_SDO_WRITE_U8 &&
        command != IGH_CIA402_SDO_WRITE_U16 &&
        command != IGH_CIA402_SDO_WRITE_U32) {
        return IGH_ECAN_INVALID_ARG;
    }
    data[0] = command;
    data[1] = (uint8_t)(index & 0xFFU);
    data[2] = (uint8_t)(index >> 8);
    data[3] = subindex;
    put_u32_le(&data[4], value);
    tx_frame_init(out,
                  axis->slave_pos,
                  axis->channel,
                  IGH_CIA402_COB_SDO_REQUEST_BASE + axis->node_id,
                  data,
                  sizeof(data));
    return IGH_ECAN_OK;
}

int igh_cia402_send_nmt(igh_cia402_context_t *ctx,
                        const igh_cia402_axis_t *axis,
                        uint8_t command)
{
    igh_cia402_tx_frame_t tx;
    int rc;

    rc = igh_cia402_build_nmt_frame(axis, command, &tx);
    return (rc == IGH_ECAN_OK) ? send_tx_frame(ctx, &tx) : rc;
}

int igh_cia402_send_nmt_broadcast(igh_cia402_context_t *ctx,
                                  int slave_pos,
                                  uint8_t channel,
                                  uint8_t command)
{
    igh_cia402_tx_frame_t tx;
    int rc;

    rc = igh_cia402_build_nmt_broadcast_frame(slave_pos, channel, command, &tx);
    return (rc == IGH_ECAN_OK) ? send_tx_frame(ctx, &tx) : rc;
}

int igh_cia402_send_nmt_all(igh_cia402_context_t *ctx, uint8_t command)
{
    int rc = IGH_ECAN_OK;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        igh_cia402_axis_t *axis = &ctx->axes[i];
        if (!axis->enabled) {
            continue;
        }
        merge_result(&rc, igh_cia402_send_nmt(ctx, axis, command));
    }
    return rc;
}

int igh_cia402_send_sync(igh_cia402_context_t *ctx, int slave_pos, uint8_t channel)
{
    igh_cia402_tx_frame_t tx;
    int rc = igh_cia402_build_sync_frame(ctx, slave_pos, channel, &tx);

    return (rc == IGH_ECAN_OK) ? send_tx_frame(ctx, &tx) : rc;
}

int igh_cia402_sdo_upload(igh_cia402_context_t *ctx,
                          const igh_cia402_axis_t *axis,
                          uint16_t index,
                          uint8_t subindex)
{
    igh_cia402_tx_frame_t tx;
    int rc = igh_cia402_build_sdo_upload_frame(axis, index, subindex, &tx);

    return (rc == IGH_ECAN_OK) ? send_tx_frame(ctx, &tx) : rc;
}

int igh_cia402_sdo_write_u8(igh_cia402_context_t *ctx,
                            const igh_cia402_axis_t *axis,
                            uint16_t index,
                            uint8_t subindex,
                            uint8_t value)
{
    return sdo_download(ctx, axis, IGH_CIA402_SDO_WRITE_U8, index, subindex, value);
}

int igh_cia402_sdo_write_u16(igh_cia402_context_t *ctx,
                             const igh_cia402_axis_t *axis,
                             uint16_t index,
                             uint8_t subindex,
                             uint16_t value)
{
    return sdo_download(ctx, axis, IGH_CIA402_SDO_WRITE_U16, index, subindex, value);
}

int igh_cia402_sdo_write_u32(igh_cia402_context_t *ctx,
                             const igh_cia402_axis_t *axis,
                             uint16_t index,
                             uint8_t subindex,
                             uint32_t value)
{
    return sdo_download(ctx, axis, IGH_CIA402_SDO_WRITE_U32, index, subindex, value);
}

int igh_cia402_sdo_write_u8_timeout(igh_cia402_context_t *ctx,
                                    igh_cia402_axis_t *axis,
                                    uint16_t index,
                                    uint8_t subindex,
                                    uint8_t value,
                                    uint32_t timeout_ms)
{
    return sdo_write_timeout(ctx, axis, IGH_CIA402_SDO_WRITE_U8,
                             index, subindex, value, timeout_ms);
}

int igh_cia402_sdo_write_u16_timeout(igh_cia402_context_t *ctx,
                                     igh_cia402_axis_t *axis,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint16_t value,
                                     uint32_t timeout_ms)
{
    return sdo_write_timeout(ctx, axis, IGH_CIA402_SDO_WRITE_U16,
                             index, subindex, value, timeout_ms);
}

int igh_cia402_sdo_write_u32_timeout(igh_cia402_context_t *ctx,
                                     igh_cia402_axis_t *axis,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint32_t value,
                                     uint32_t timeout_ms)
{
    return sdo_write_timeout(ctx, axis, IGH_CIA402_SDO_WRITE_U32,
                             index, subindex, value, timeout_ms);
}

int igh_cia402_configure_csp(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis)
{
    int rc = IGH_ECAN_OK;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    merge_result(&rc, igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_HEARTBEAT_PRODUCER,
                                               0x00U, IGH_CIA402_CSP_HEARTBEAT_MS));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_SYNC_COB_ID,
                                               0x00U, IGH_CIA402_CSP_SYNC_COB_ID));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_SYNC_PERIOD_US,
                                               0x00U, ctx_cycle_us(ctx)));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_TPDO1_COMM,
                                               0x01U, csp_stage_value(ctx, axis, 0U,
                                               IGH_CIA402_SDO_VALUE_TPDO1_DISABLED)));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_TPDO1_COMM,
                                              0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_TPDO1_MAPPING,
                                              0x00U, 0U));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_TPDO1_MAPPING,
                                               0x01U, IGH_CIA402_PDO_MAP_POSITION_ACTUAL));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_TPDO1_MAPPING,
                                               0x02U, IGH_CIA402_PDO_MAP_STATUSWORD));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_TPDO1_MAPPING,
                                               0x03U, IGH_CIA402_PDO_MAP_MODE_DISPLAY));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_TPDO1_MAPPING,
                                              0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_TPDO1_COMM,
                                               0x01U, csp_stage_value(ctx, axis, 0U,
                                               IGH_CIA402_SDO_VALUE_TPDO1_ENABLED)));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_RPDO1_COMM,
                                               0x01U, csp_stage_value(ctx, axis, 0U,
                                               IGH_CIA402_SDO_VALUE_RPDO1_DISABLED)));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_RPDO1_COMM,
                                              0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_RPDO1_MAPPING,
                                              0x00U, 0U));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_RPDO1_MAPPING,
                                               0x01U, IGH_CIA402_PDO_MAP_CONTROLWORD));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_RPDO1_MAPPING,
                                               0x02U, IGH_CIA402_PDO_MAP_MODE_OF_OPERATION));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_RPDO1_MAPPING,
                                               0x03U, IGH_CIA402_PDO_MAP_TARGET_POSITION));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_RPDO1_MAPPING,
                                              0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP));
    merge_result(&rc, igh_cia402_sdo_write_u32(ctx, axis, IGH_CIA402_IDX_RPDO1_COMM,
                                               0x01U, csp_stage_value(ctx, axis, 0U,
                                               IGH_CIA402_SDO_VALUE_RPDO1_ENABLED)));
    merge_result(&rc, igh_cia402_sdo_write_u8(ctx, axis, IGH_CIA402_IDX_MODES_OF_OPERATION,
                                              0x00U, IGH_CIA402_MODE_CSP));
    if (rc == IGH_ECAN_OK) {
        mark_csp_rpdo1_mapping_active(axis);
    }
    return rc;
}

int igh_cia402_configure_csp_timeout(igh_cia402_context_t *ctx,
                                     igh_cia402_axis_t *axis,
                                     uint32_t timeout_ms)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    static const igh_cia402_sdo_stage_t stages[] = {
        {IGH_CIA402_SDO_WRITE_U16, IGH_CIA402_IDX_HEARTBEAT_PRODUCER,
         0x00U, IGH_CIA402_CSP_HEARTBEAT_MS, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_SYNC_COB_ID,
         0x00U, IGH_CIA402_CSP_SYNC_COB_ID, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_SYNC_PERIOD_US,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_CYCLE_US},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_POSITION_ACTUAL, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_STATUSWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_MODE_DISPLAY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_CONTROLWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_MODE_OF_OPERATION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_TARGET_POSITION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_MODES_OF_OPERATION,
         0x00U, IGH_CIA402_MODE_CSP, IGH_CIA402_SDO_VALUE_FIXED},
    };

    for (size_t i = 0U; i < sizeof(stages) / sizeof(stages[0]); i++) {
        const igh_cia402_sdo_stage_t *stage = &stages[i];

        rc = sdo_write_timeout(ctx,
                               axis,
                               stage->command,
                               stage->index,
                               stage->subindex,
                               csp_stage_value(ctx, axis, stage->value, stage->value_mode),
                               timeout_ms);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        if (i + 1U < sizeof(stages) / sizeof(stages[0])) {
            rc = run_idle_ms(ctx, IGH_CIA402_CSP_SDO_STAGE_GAP_MS);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    }
    mark_csp_rpdo1_mapping_active(axis);
    return IGH_ECAN_OK;
}

int igh_cia402_configure_csp_pdo_timeout(igh_cia402_context_t *ctx,
                                         igh_cia402_axis_t *axis,
                                         uint32_t timeout_ms)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    static const igh_cia402_sdo_stage_t stages[] = {
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_POSITION_ACTUAL, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_STATUSWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_MODE_DISPLAY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_CONTROLWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_MODE_OF_OPERATION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_TARGET_POSITION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_MODES_OF_OPERATION,
         0x00U, IGH_CIA402_MODE_CSP, IGH_CIA402_SDO_VALUE_FIXED},
    };

    for (size_t i = 0U; i < sizeof(stages) / sizeof(stages[0]); i++) {
        const igh_cia402_sdo_stage_t *stage = &stages[i];

        rc = sdo_write_timeout(ctx,
                               axis,
                               stage->command,
                               stage->index,
                               stage->subindex,
                               csp_stage_value(ctx, axis, stage->value, stage->value_mode),
                               timeout_ms);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        if (i + 1U < sizeof(stages) / sizeof(stages[0])) {
            rc = run_idle_ms(ctx, IGH_CIA402_CSP_SDO_STAGE_GAP_MS);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    }
    mark_csp_rpdo1_mapping_active(axis);
    return IGH_ECAN_OK;
}

static int sdo_write_stage_ack(igh_cia402_context_t *ctx,
                               igh_cia402_axis_t *axis,
                               const igh_cia402_sdo_stage_t *stage,
                               uint32_t timeout_ms)
{
    uint64_t start_ms;
    uint64_t after_write_ms;
    uint32_t value;
    int rc;

    if ((stage == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    value = csp_stage_value(ctx, axis, stage->value, stage->value_mode);
    if (stage->command != IGH_CIA402_SDO_WRITE_U8 &&
        stage->command != IGH_CIA402_SDO_WRITE_U16 &&
        stage->command != IGH_CIA402_SDO_WRITE_U32) {
        return IGH_ECAN_UNSUPPORTED;
    }

    start_ms = monotonic_ms();
    rc = sdo_write_timeout(ctx, axis, stage->command, stage->index, stage->subindex,
                           value, timeout_ms);
    after_write_ms = monotonic_ms();
    axis->sdo_last_write_rc = rc;
    if (rc != IGH_ECAN_OK) {
        uint32_t read_value = 0U;
        uint8_t read_size = 0U;
        uint8_t expected_size = 0U;
        uint32_t mask;
        int read_rc;

        if (stage->command == IGH_CIA402_SDO_WRITE_U8) {
            expected_size = 1U;
        } else if (stage->command == IGH_CIA402_SDO_WRITE_U16) {
            expected_size = 2U;
        } else if (stage->command == IGH_CIA402_SDO_WRITE_U32) {
            expected_size = 4U;
        }
        mask = sdo_mask_for_size(expected_size);
        read_rc = igh_cia402_sdo_upload_u32_timeout(ctx,
                                                    axis,
                                                    stage->index,
                                                    stage->subindex,
                                                    &read_value,
                                                    &read_size,
                                                    timeout_ms);
        if ((read_rc == IGH_ECAN_OK) &&
            (read_size == expected_size) &&
            ((read_value & mask) == (value & mask))) {
            printf("[warn] pdo-config-write-ack-missed index=0x%04X:%02X write_rc=%d readback=0x%08" PRIX32 "\n",
                   stage->index,
                   stage->subindex,
                   rc,
                   read_value);
            fflush(stdout);
            rc = IGH_ECAN_OK;
        }
    }
    printf("[diag] pdo-config-write index=0x%04X:%02X rc=%d elapsed_ms=%" PRIu64 "\n",
           stage->index,
           stage->subindex,
           rc,
           after_write_ms - start_ms);
    fflush(stdout);
    axis->sdo_last_read_rc = IGH_ECAN_OK;
    return rc;
}

static int cia402_stage_is_optional(const igh_cia402_sdo_stage_t *stage)
{
    return stage != NULL &&
           stage->index == IGH_CIA402_IDX_INTERPOLATION_TIME &&
           stage->subindex == 0x01U;
}

int igh_cia402_configure_csp_rpdo_timeout(igh_cia402_context_t *ctx,
                                          igh_cia402_axis_t *axis,
                                          uint32_t timeout_ms)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    static const igh_cia402_sdo_stage_t stages[] = {
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_MODES_OF_OPERATION,
         0x00U, IGH_CIA402_MODE_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_INTERPOLATION_TIME,
         0x01U, 4U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_POSITION_ACTUAL, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_STATUSWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_MODE_DISPLAY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_CONTROLWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_MODE_OF_OPERATION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_TARGET_POSITION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_ENABLED},
    };

    rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_ENTER_PREOPERATIONAL);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = run_idle_ms(ctx, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    for (size_t i = 0U; i < sizeof(stages) / sizeof(stages[0]); i++) {
        uint32_t value = csp_stage_value(ctx, axis, stages[i].value, stages[i].value_mode);
        printf("[diag] csp-config-sdo step=%zu/%zu cmd=0x%02X index=0x%04X:%02X value=0x%08" PRIX32 "\n",
               i + 1U,
               sizeof(stages) / sizeof(stages[0]),
               stages[i].command,
               stages[i].index,
               stages[i].subindex,
               value);
        fflush(stdout);
        rc = sdo_write_stage_ack(ctx, axis, &stages[i], timeout_ms);
        if (rc != IGH_ECAN_OK) {
            if (cia402_stage_is_optional(&stages[i])) {
                printf("[warn] csp-config-sdo optional index=0x%04X:%02X rc=%d ignored\n",
                       stages[i].index,
                       stages[i].subindex,
                       rc);
                fflush(stdout);
                rc = IGH_ECAN_OK;
            } else {
                return rc;
            }
        }
        if (i + 1U < sizeof(stages) / sizeof(stages[0])) {
            rc = run_idle_ms(ctx, IGH_CIA402_CSP_SDO_STAGE_GAP_MS);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    }
    rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_START_REMOTE_NODE);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = run_idle_ms(ctx, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    mark_csp_rpdo1_mapping_active(axis);
    return IGH_ECAN_OK;
}

int igh_cia402_configure_csv_rpdo_timeout(igh_cia402_context_t *ctx,
                                          igh_cia402_axis_t *axis,
                                          uint32_t timeout_ms)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    static const igh_cia402_sdo_stage_t stages[] = {
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_MODES_OF_OPERATION,
         0x00U, IGH_CIA402_MODE_CSV, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_VELOCITY_ACTUAL, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_STATUSWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_MODE_DISPLAY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_CONTROLWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_MODE_OF_OPERATION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_TARGET_VELOCITY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_ENABLED},
    };

    rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_ENTER_PREOPERATIONAL);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = run_idle_ms(ctx, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    for (size_t i = 0U; i < sizeof(stages) / sizeof(stages[0]); i++) {
        uint32_t value = csp_stage_value(ctx, axis, stages[i].value, stages[i].value_mode);
        printf("[diag] csv-config-sdo step=%zu/%zu cmd=0x%02X index=0x%04X:%02X value=0x%08" PRIX32 "\n",
               i + 1U,
               sizeof(stages) / sizeof(stages[0]),
               stages[i].command,
               stages[i].index,
               stages[i].subindex,
               value);
        fflush(stdout);
        rc = sdo_write_stage_ack(ctx, axis, &stages[i], timeout_ms);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        if (i + 1U < sizeof(stages) / sizeof(stages[0])) {
            rc = run_idle_ms(ctx, IGH_CIA402_CSP_SDO_STAGE_GAP_MS);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    }
    rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_START_REMOTE_NODE);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = run_idle_ms(ctx, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    mark_csv_rpdo1_mapping_active(axis);
    axis->target_velocity = 0;
    return IGH_ECAN_OK;
}

int igh_cia402_configure_csp_all_timeout(igh_cia402_context_t *ctx,
                                         uint32_t timeout_ms)
{
    static const igh_cia402_sdo_stage_t stages[] = {
        {IGH_CIA402_SDO_WRITE_U16, IGH_CIA402_IDX_HEARTBEAT_PRODUCER,
         0x00U, IGH_CIA402_CSP_HEARTBEAT_MS, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_SYNC_COB_ID,
         0x00U, IGH_CIA402_CSP_SYNC_COB_ID, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_SYNC_PERIOD_US,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_CYCLE_US},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_POSITION_ACTUAL, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_STATUSWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_MODE_DISPLAY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_TPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_TPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_TPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_DISABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_COMM,
         0x02U, IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, 0U, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x01U, IGH_CIA402_PDO_MAP_CONTROLWORD, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x02U, IGH_CIA402_PDO_MAP_MODE_OF_OPERATION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x03U, IGH_CIA402_PDO_MAP_TARGET_POSITION, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_RPDO1_MAPPING,
         0x00U, IGH_CIA402_PDO_MAP_COUNT_CSP, IGH_CIA402_SDO_VALUE_FIXED},
        {IGH_CIA402_SDO_WRITE_U32, IGH_CIA402_IDX_RPDO1_COMM,
         0x01U, 0U, IGH_CIA402_SDO_VALUE_RPDO1_ENABLED},
        {IGH_CIA402_SDO_WRITE_U8, IGH_CIA402_IDX_MODES_OF_OPERATION,
         0x00U, IGH_CIA402_MODE_CSP, IGH_CIA402_SDO_VALUE_FIXED},
    };
    int rc;

    if (!transport_is_ready(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (size_t i = 0U; i < sizeof(stages) / sizeof(stages[0]); i++) {
        const igh_cia402_sdo_stage_t *stage = &stages[i];

        rc = sdo_write_all_timeout(ctx, stage->command, stage->index,
                                   stage->subindex, stage->value,
                                   stage->value_mode, timeout_ms);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        if (i + 1U < sizeof(stages) / sizeof(stages[0])) {
            rc = run_idle_ms(ctx, IGH_CIA402_CSP_SDO_STAGE_GAP_MS);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
        }
    }
    mark_csp_rpdo1_mapping_active_all(ctx);
    return IGH_ECAN_OK;
}

int igh_cia402_configure_csp_all(igh_cia402_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        if (!ctx->axes[i].enabled) {
            continue;
        }
        merge_result(&rc, igh_cia402_configure_csp(ctx, &ctx->axes[i]));
    }
    return rc;
}

int igh_cia402_enable(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_CONTROLWORD, 0x00U,
                                  IGH_CIA402_CONTROLWORD_SHUTDOWN);
    usleep(20000);
    merge_result(&rc, igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_CONTROLWORD,
                                               0x00U, IGH_CIA402_CONTROLWORD_SWITCH_ON));
    usleep(20000);
    merge_result(&rc, igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_CONTROLWORD,
                                               0x00U, IGH_CIA402_CONTROLWORD_ENABLE_OP));
    return rc;
}

int igh_cia402_enable_timeout(igh_cia402_context_t *ctx,
                              igh_cia402_axis_t *axis,
                              uint32_t timeout_ms)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_START_REMOTE_NODE);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = igh_cia402_request_statusword_timeout(ctx, axis, timeout_ms);
    if (rc == IGH_ECAN_OK &&
        ((axis->statusword & IGH_CIA402_STATUSWORD_STATE_MASK) == IGH_CIA402_STATUSWORD_FAULT)) {
        rc = send_controlword_step(ctx, axis, IGH_CIA402_CONTROLWORD_FAULT_RESET, 100U);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    } else if (rc != IGH_ECAN_OK && rc != IGH_ECAN_TIMEOUT) {
        return rc;
    }
    rc = send_controlword_step(ctx, axis, IGH_CIA402_CONTROLWORD_SHUTDOWN, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    rc = send_controlword_step(ctx, axis, IGH_CIA402_CONTROLWORD_SWITCH_ON, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    rc = send_controlword_step(ctx, axis, IGH_CIA402_CONTROLWORD_ENABLE_OP, 20U);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    rc = send_nmt_once(ctx, axis, IGH_CIA402_NMT_START_REMOTE_NODE);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    rc = igh_cia402_request_statusword_timeout(ctx, axis, timeout_ms);
    if (rc != IGH_ECAN_OK) {
        if (rc == IGH_ECAN_TIMEOUT &&
            axis->emcy_valid == 0U &&
            ((axis->heartbeat_valid != 0U &&
              axis->heartbeat == IGH_CIA402_HEARTBEAT_OPERATIONAL) ||
             axis->actual_position_valid != 0U)) {
            axis->statusword = IGH_CIA402_STATUSWORD_OPERATION_ENABLED;
            axis->statusword_valid = 1U;
            return IGH_ECAN_OK;
        }
        return rc;
    }
    return ((axis->statusword & IGH_CIA402_STATUSWORD_STATE_MASK) ==
            IGH_CIA402_STATUSWORD_OPERATION_ENABLED) ?
           IGH_ECAN_OK :
           IGH_ECAN_NOT_READY;
}

int igh_cia402_enable_all(igh_cia402_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        if (!ctx->axes[i].enabled) {
            continue;
        }
        merge_result(&rc, igh_cia402_enable(ctx, &ctx->axes[i]));
    }
    return rc;
}

int igh_cia402_stop(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis)
{
    int rc;

    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_CONTROLWORD, 0x00U,
                                  IGH_CIA402_CONTROLWORD_SWITCH_ON);
    merge_result(&rc, igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_CONTROLWORD,
                                               0x00U, IGH_CIA402_CONTROLWORD_SHUTDOWN));
    merge_result(&rc, igh_cia402_sdo_write_u16(ctx, axis, IGH_CIA402_IDX_CONTROLWORD,
                                               0x00U, IGH_CIA402_CONTROLWORD_DISABLE));
    return rc;
}

int igh_cia402_stop_all(igh_cia402_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        if (!ctx->axes[i].enabled) {
            continue;
        }
        merge_result(&rc, igh_cia402_stop(ctx, &ctx->axes[i]));
    }
    return rc;
}

int igh_cia402_request_actual_position(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis)
{
    if ((ctx == NULL) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    return igh_cia402_sdo_upload(ctx, axis, IGH_CIA402_IDX_POSITION_ACTUAL_VALUE, 0x00U);
}

int igh_cia402_request_actual_position_all(igh_cia402_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        if (!ctx->axes[i].enabled) {
            continue;
        }
        merge_result(&rc, igh_cia402_request_actual_position(ctx, &ctx->axes[i]));
    }
    return rc;
}

static int process_rx_once(igh_cia402_context_t *ctx)
{
    igh_cia402_rx_frame_t frames[IGH_CIA402_TIMEOUT_RX_CAPACITY];
    size_t count = 0U;
    int rc;

    if (!transport_can_receive(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    rc = transport_receive(ctx, frames, IGH_CIA402_TIMEOUT_RX_CAPACITY, &count);
    transport_cycle_end(ctx);
    for (size_t i = 0U; i < count; i++) {
        int frame_rc = igh_cia402_on_can_frame(ctx, frames[i].slave_pos, &frames[i].frame);
        if ((rc == IGH_ECAN_OK) && (frame_rc != IGH_ECAN_OK)) {
            rc = frame_rc;
        }
    }
    return rc;
}

static int process_rx_once_for_wait(igh_cia402_context_t *ctx)
{
    int rc = process_rx_once(ctx);

    /*
     * SDO/heartbeat/position waits are scoped to a CANopen node.  A different
     * EtherCAT slave may report an incomplete PDO header while the target node
     * is still able to answer; keep waiting so the caller gets either the
     * target response or a clean timeout that can drive retry logic.
     */
    return (rc == IGH_ECAN_FAIL || rc == IGH_ECAN_NO_SPACE) ? IGH_ECAN_OK : rc;
}

static int process_rx_in_open_cycle(igh_cia402_context_t *ctx)
{
    igh_cia402_rx_frame_t frames[IGH_CIA402_TIMEOUT_RX_CAPACITY];

    return igh_cia402_process_rx(ctx, frames, IGH_CIA402_TIMEOUT_RX_CAPACITY, NULL);
}

static int wait_sdo_response(igh_cia402_context_t *ctx,
                             const igh_cia402_axis_t *axis,
                             uint16_t index,
                             uint8_t subindex,
                             uint8_t expected_command,
                             uint32_t timeout_ms)
{
    uint64_t start_ms;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    start_ms = monotonic_ms();
    for (;;) {
        int rc;

        if (sdo_response_matches(axis, index, subindex)) {
            if (axis->sdo_response_command == IGH_CIA402_SDO_ABORT) {
                return IGH_ECAN_FAIL;
            }
            return (axis->sdo_response_command == expected_command) ?
                   IGH_ECAN_OK :
                   IGH_ECAN_FAIL;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            rc = process_rx_once_for_wait(ctx);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
            if (sdo_response_matches(axis, index, subindex)) {
                if (axis->sdo_response_command == IGH_CIA402_SDO_ABORT) {
                    return IGH_ECAN_FAIL;
                }
                return (axis->sdo_response_command == expected_command) ?
                       IGH_ECAN_OK :
                       IGH_ECAN_FAIL;
            }
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

static int wait_sdo_all_responses(igh_cia402_context_t *ctx,
                                  uint16_t index,
                                  uint8_t subindex,
                                  uint8_t expected_command,
                                  uint32_t timeout_ms)
{
    uint64_t start_ms;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }

    start_ms = monotonic_ms();
    for (;;) {
        int state = all_enabled_axes_have_sdo_ack(ctx, index, subindex, expected_command);
        int rc;

        if (state > 0) {
            return IGH_ECAN_OK;
        }
        if (state < 0) {
            return IGH_ECAN_FAIL;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            rc = process_rx_once_for_wait(ctx);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
            state = all_enabled_axes_have_sdo_ack(ctx, index, subindex, expected_command);
            if (state > 0) {
                return IGH_ECAN_OK;
            }
            if (state < 0) {
                return IGH_ECAN_FAIL;
            }
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

int igh_cia402_sdo_upload_u32_timeout(igh_cia402_context_t *ctx,
                                      igh_cia402_axis_t *axis,
                                      uint16_t index,
                                      uint8_t subindex,
                                      uint32_t *value,
                                      uint8_t *size,
                                      uint32_t timeout_ms)
{
    uint64_t start_ms;
    int rc;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx) ||
        (axis == NULL) || (value == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    (void)process_rx_in_open_cycle(ctx);
    clear_sdo_response(axis);
    axis->sdo_response_index = index;
    axis->sdo_response_subindex = subindex;
    rc = igh_cia402_sdo_upload(ctx, axis, index, subindex);
    transport_cycle_end(ctx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    start_ms = monotonic_ms();
    for (;;) {
        if (sdo_response_matches(axis, index, subindex)) {
            if (axis->sdo_response_command == IGH_CIA402_SDO_ABORT) {
                return IGH_ECAN_FAIL;
            }
            if (sdo_upload_size_from_command(axis->sdo_response_command) == 0U) {
                return IGH_ECAN_FAIL;
            }
            *value = axis->sdo_response_value;
            if (size != NULL) {
                *size = axis->sdo_response_size;
            }
            return IGH_ECAN_OK;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            rc = process_rx_once_for_wait(ctx);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
            if (sdo_response_matches(axis, index, subindex)) {
                if (axis->sdo_response_command == IGH_CIA402_SDO_ABORT) {
                    return IGH_ECAN_FAIL;
                }
                if (sdo_upload_size_from_command(axis->sdo_response_command) == 0U) {
                    return IGH_ECAN_FAIL;
                }
                *value = axis->sdo_response_value;
                if (size != NULL) {
                    *size = axis->sdo_response_size;
                }
                return IGH_ECAN_OK;
            }
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

static uint32_t sdo_mask_for_size(uint8_t size)
{
    switch (size) {
    case 1U:
        return IGH_CIA402_SDO_VALUE_MASK_U8;
    case 2U:
        return IGH_CIA402_SDO_VALUE_MASK_U16;
    case 3U:
        return IGH_CIA402_SDO_VALUE_MASK_U24;
    case 4U:
        return IGH_CIA402_SDO_VALUE_MASK_U32;
    default:
        return 0U;
    }
}

static int sdo_write_verify_timeout(igh_cia402_context_t *ctx,
                                    igh_cia402_axis_t *axis,
                                    uint8_t command,
                                    uint16_t index,
                                    uint8_t subindex,
                                    uint32_t value,
                                    uint8_t expected_size,
                                    uint32_t timeout_ms)
{
    uint32_t mask = sdo_mask_for_size(expected_size);

    if (mask == 0U) {
        return IGH_ECAN_INVALID_ARG;
    }

    for (unsigned attempt = 0U; attempt < 2U; attempt++) {
        uint32_t read_value = 0U;
        uint8_t read_size = 0U;
        int write_rc = sdo_write_timeout(ctx, axis, command, index, subindex, value, timeout_ms);
        int gap_rc;
        int read_rc;

        if (write_rc == IGH_ECAN_OK) {
            gap_rc = run_idle_ms(ctx, IGH_CIA402_SDO_VERIFY_READ_GAP_MS);
            if (gap_rc != IGH_ECAN_OK) {
                axis->sdo_last_write_rc = write_rc;
                axis->sdo_last_read_rc = gap_rc;
                return gap_rc;
            }
        }
        read_rc = igh_cia402_sdo_upload_u32_timeout(ctx, axis, index, subindex,
                                                    &read_value, &read_size, timeout_ms);

        axis->sdo_last_write_rc = write_rc;
        axis->sdo_last_read_rc = read_rc;
        if ((read_rc == IGH_ECAN_OK) &&
            ((read_value & mask) == (value & mask)) &&
            read_size >= expected_size) {
            return IGH_ECAN_OK;
        }
        if (write_rc != IGH_ECAN_OK && write_rc != IGH_ECAN_TIMEOUT) {
            return write_rc;
        }
        if (read_rc != IGH_ECAN_OK && read_rc != IGH_ECAN_TIMEOUT) {
            return read_rc;
        }
        if (attempt == 0U) {
            int idle_rc = run_idle_ms(ctx, IGH_CIA402_CSP_SDO_STAGE_GAP_MS);
            if (idle_rc != IGH_ECAN_OK) {
                return idle_rc;
            }
        }
    }
    return IGH_ECAN_FAIL;
}

int igh_cia402_sdo_write_verify_u8_timeout(igh_cia402_context_t *ctx,
                                           igh_cia402_axis_t *axis,
                                           uint16_t index,
                                           uint8_t subindex,
                                           uint8_t value,
                                           uint32_t timeout_ms)
{
    return sdo_write_verify_timeout(ctx, axis, IGH_CIA402_SDO_WRITE_U8,
                                    index, subindex, value, 1U, timeout_ms);
}

int igh_cia402_sdo_write_verify_u16_timeout(igh_cia402_context_t *ctx,
                                            igh_cia402_axis_t *axis,
                                            uint16_t index,
                                            uint8_t subindex,
                                            uint16_t value,
                                            uint32_t timeout_ms)
{
    return sdo_write_verify_timeout(ctx, axis, IGH_CIA402_SDO_WRITE_U16,
                                    index, subindex, value, 2U, timeout_ms);
}

int igh_cia402_sdo_write_verify_u32_timeout(igh_cia402_context_t *ctx,
                                            igh_cia402_axis_t *axis,
                                            uint16_t index,
                                            uint8_t subindex,
                                            uint32_t value,
                                            uint32_t timeout_ms)
{
    return sdo_write_verify_timeout(ctx, axis, IGH_CIA402_SDO_WRITE_U32,
                                    index, subindex, value, 4U, timeout_ms);
}

static int sdo_write_timeout(igh_cia402_context_t *ctx,
                             igh_cia402_axis_t *axis,
                             uint8_t command,
                             uint16_t index,
                             uint8_t subindex,
                             uint32_t value,
                             uint32_t timeout_ms)
{
    uint8_t data[8] = {0};

    if (!transport_can_receive(ctx) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    data[0] = command;
    data[1] = (uint8_t)(index & 0xFFU);
    data[2] = (uint8_t)(index >> 8);
    data[3] = subindex;
    put_u32_le(&data[4], value);

    for (unsigned attempt = 0U; attempt < 2U; attempt++) {
        int rc;

        clear_sdo_response(axis);
        axis->sdo_response_index = index;
        axis->sdo_response_subindex = subindex;
        transport_cycle_begin(ctx);
        rc = send_channel_can(ctx,
                              axis->slave_pos,
                              axis->channel,
                              IGH_CIA402_COB_SDO_REQUEST_BASE + axis->node_id,
                              data,
                              sizeof(data),
                              0,
                              0U);
        transport_cycle_end(ctx);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        rc = wait_sdo_response(ctx, axis, index, subindex,
                               IGH_CIA402_SDO_DOWNLOAD_ACK, timeout_ms);
        if (rc != IGH_ECAN_TIMEOUT || attempt != 0U) {
            return rc;
        }
    }
    return IGH_ECAN_TIMEOUT;
}

static int sdo_write_all_timeout(igh_cia402_context_t *ctx,
                                 uint8_t command,
                                 uint16_t index,
                                 uint8_t subindex,
                                 uint32_t value,
                                 igh_cia402_sdo_value_mode_t value_mode,
                                 uint32_t timeout_ms)
{
    int rc = IGH_ECAN_OK;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    (void)process_rx_in_open_cycle(ctx);
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        igh_cia402_axis_t *axis = &ctx->axes[i];

        if (!axis->enabled) {
            continue;
        }
        clear_sdo_response(axis);
        axis->sdo_response_index = index;
        axis->sdo_response_subindex = subindex;
        merge_result(&rc, sdo_download(ctx, axis, command, index, subindex,
                                       csp_stage_value(ctx, axis, value, value_mode)));
    }
    transport_cycle_end(ctx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return wait_sdo_all_responses(ctx, index, subindex,
                                  IGH_CIA402_SDO_DOWNLOAD_ACK, timeout_ms);
}

static int run_idle_ms(igh_cia402_context_t *ctx, uint32_t duration_ms)
{
    uint64_t start_ms;
    int rc = IGH_ECAN_OK;

    if (!transport_can_receive(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (duration_ms == 0U) {
        return IGH_ECAN_OK;
    }

    start_ms = monotonic_ms();
    do {
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
    } while ((rc == IGH_ECAN_OK) && !timeout_expired(start_ms, duration_ms));
    return rc;
}

static int all_enabled_axes_match_heartbeat(const igh_cia402_context_t *ctx, uint8_t heartbeat)
{
    if (ctx == NULL) {
        return 0;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        const igh_cia402_axis_t *axis = &ctx->axes[i];
        if (axis->enabled &&
            (axis->heartbeat_valid == 0U || axis->heartbeat != heartbeat)) {
            return 0;
        }
    }
    return 1;
}

static int all_enabled_axes_have_position(const igh_cia402_context_t *ctx)
{
    if (ctx == NULL) {
        return 0;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        const igh_cia402_axis_t *axis = &ctx->axes[i];
        if (axis->enabled && axis->actual_position_valid == 0U) {
            return 0;
        }
    }
    return 1;
}

int igh_cia402_wait_heartbeat_timeout(igh_cia402_context_t *ctx,
                                      igh_cia402_axis_t *axis,
                                      uint8_t heartbeat,
                                      uint32_t timeout_ms)
{
    uint64_t start_ms;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    start_ms = monotonic_ms();
    for (;;) {
        int rc;

        if (axis->heartbeat_valid != 0U && axis->heartbeat == heartbeat) {
            return IGH_ECAN_OK;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (axis->heartbeat_valid != 0U && axis->heartbeat == heartbeat) {
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

int igh_cia402_wait_heartbeat_all_timeout(igh_cia402_context_t *ctx,
                                          uint8_t heartbeat,
                                          uint32_t timeout_ms)
{
    uint64_t start_ms;

    if (!transport_can_receive(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }

    start_ms = monotonic_ms();
    for (;;) {
        int rc;

        if (all_enabled_axes_match_heartbeat(ctx, heartbeat)) {
            return IGH_ECAN_OK;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (all_enabled_axes_match_heartbeat(ctx, heartbeat)) {
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

int igh_cia402_request_actual_position_timeout(igh_cia402_context_t *ctx,
                                               igh_cia402_axis_t *axis,
                                               uint32_t timeout_ms)
{
    uint64_t start_ms;
    int rc;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    (void)process_rx_in_open_cycle(ctx);
    axis->actual_position_valid = 0U;
    clear_sdo_response(axis);
    rc = igh_cia402_request_actual_position(ctx, axis);
    transport_cycle_end(ctx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    start_ms = monotonic_ms();
    for (;;) {
        if (axis->actual_position_valid != 0U) {
            return IGH_ECAN_OK;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (sdo_abort_seen(axis, IGH_CIA402_IDX_POSITION_ACTUAL_VALUE, 0x00U)) {
            return IGH_ECAN_FAIL;
        }
        if (axis->actual_position_valid != 0U) {
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

int igh_cia402_request_statusword_timeout(igh_cia402_context_t *ctx,
                                          igh_cia402_axis_t *axis,
                                          uint32_t timeout_ms)
{
    uint64_t start_ms;
    int rc;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx) || (axis == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    (void)process_rx_in_open_cycle(ctx);
    axis->statusword_valid = 0U;
    clear_sdo_response(axis);
    rc = igh_cia402_sdo_upload(ctx, axis, IGH_CIA402_IDX_STATUSWORD, 0x00U);
    transport_cycle_end(ctx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    start_ms = monotonic_ms();
    for (;;) {
        if (axis->statusword_valid != 0U) {
            return IGH_ECAN_OK;
        }
        if (sdo_abort_seen(axis, IGH_CIA402_IDX_STATUSWORD, 0x00U)) {
            return IGH_ECAN_FAIL;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        if (axis->statusword_valid != 0U) {
            return IGH_ECAN_OK;
        }
        if (sdo_abort_seen(axis, IGH_CIA402_IDX_STATUSWORD, 0x00U)) {
            return IGH_ECAN_FAIL;
        }
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

int igh_cia402_request_actual_position_all_timeout(igh_cia402_context_t *ctx,
                                                   uint32_t timeout_ms)
{
    uint64_t start_ms;
    int rc;

    if (!transport_is_ready(ctx) || !transport_can_receive(ctx)) {
        return IGH_ECAN_INVALID_ARG;
    }

    transport_cycle_begin(ctx);
    (void)process_rx_in_open_cycle(ctx);
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        if (ctx->axes[i].enabled) {
            ctx->axes[i].actual_position_valid = 0U;
            clear_sdo_response(&ctx->axes[i]);
        }
    }
    rc = igh_cia402_request_actual_position_all(ctx);
    transport_cycle_end(ctx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    start_ms = monotonic_ms();
    for (;;) {
        if (all_enabled_axes_have_position(ctx)) {
            return IGH_ECAN_OK;
        }
        if (timeout_expired(start_ms, timeout_ms)) {
            return IGH_ECAN_TIMEOUT;
        }
        sleep_one_cycle(ctx);
        rc = process_rx_once_for_wait(ctx);
        for (size_t i = 0U; i < ctx->axis_count; i++) {
            if (ctx->axes[i].enabled &&
                sdo_abort_seen(&ctx->axes[i], IGH_CIA402_IDX_POSITION_ACTUAL_VALUE, 0x00U)) {
                return IGH_ECAN_FAIL;
            }
        }
        if (all_enabled_axes_have_position(ctx)) {
            return IGH_ECAN_OK;
        }
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
}

int igh_cia402_set_target_position(igh_cia402_axis_t *axis, int32_t target_position)
{
    if (axis == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    axis->target_position = target_position;
    return IGH_ECAN_OK;
}

static int axis_ready_for_target(const igh_cia402_axis_t *axis)
{
    if (axis == NULL) {
        return 0;
    }
    if (axis->emcy_valid != 0U &&
        (axis->emcy_code != 0U || axis->emcy_error_register != 0U)) {
        return 0;
    }
    if (axis->statusword_valid != 0U) {
        return ((axis->statusword & IGH_CIA402_STATUSWORD_STATE_MASK) ==
                IGH_CIA402_STATUSWORD_OPERATION_ENABLED) ? 1 : 0;
    }
    return (axis->heartbeat_valid != 0U &&
            axis->heartbeat == IGH_CIA402_HEARTBEAT_OPERATIONAL) ? 1 : 0;
}

int igh_cia402_prepare_target_step(const igh_cia402_axis_t *axis,
                                   igh_cia402_target_step_t *target_step)
{
    int64_t next_position;

    if ((axis == NULL) || (target_step == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    next_position = (int64_t)axis->target_position + (int64_t)axis->step;
    if ((next_position > (int64_t)INT32_MAX) ||
        (next_position < (int64_t)INT32_MIN)) {
        return IGH_ECAN_FAIL;
    }

    memset(target_step, 0, sizeof(*target_step));
    target_step->previous_position = axis->target_position;
    target_step->step = axis->step;
    target_step->next_position = (int32_t)next_position;
    target_step->data_len = axis_rpdo1_payload_len(axis);
    if (target_step->data_len == IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN) {
        pack_csp_rpdo1_payload(target_step->data, target_step->next_position);
    } else {
        put_u32_le(target_step->data, (uint32_t)target_step->next_position);
    }
    return IGH_ECAN_OK;
}

int igh_cia402_build_target_frame(igh_cia402_context_t *ctx,
                                  const igh_cia402_axis_t *axis,
                                  igh_cia402_tx_frame_t *out)
{
    uint8_t data[IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN] = {0};
    uint8_t data_len;

    if ((ctx == NULL) || (axis == NULL) || (out == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (!axis_ready_for_target(axis)) {
        return IGH_ECAN_NOT_READY;
    }

    data_len = axis_rpdo1_payload_len(axis);
    if (data_len == IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN) {
        pack_csp_rpdo1_payload(data, axis->target_position);
    } else {
        put_u32_le(data, (uint32_t)axis->target_position);
    }
    tx_frame_init(out,
                  axis->slave_pos,
                  axis->channel,
                  IGH_CIA402_COB_RPDO1_BASE + axis->node_id,
                  data,
                  data_len);
    tx_frame_apply_schedule(ctx, out,
                            ctx->scheduled_target_enabled,
                            ctx->scheduled_target_offset_ns);
    return IGH_ECAN_OK;
}

int igh_cia402_build_velocity_frame(igh_cia402_context_t *ctx,
                                    const igh_cia402_axis_t *axis,
                                    int32_t target_velocity,
                                    igh_cia402_tx_frame_t *out)
{
    uint8_t data[IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN] = {0};

    if ((ctx == NULL) || (axis == NULL) || (out == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (!axis_ready_for_target(axis)) {
        return IGH_ECAN_NOT_READY;
    }
    if (axis_rpdo1_payload_len(axis) != IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN) {
        return IGH_ECAN_UNSUPPORTED;
    }

    pack_csv_rpdo1_payload(data, target_velocity);
    tx_frame_init(out,
                  axis->slave_pos,
                  axis->channel,
                  IGH_CIA402_COB_RPDO1_BASE + axis->node_id,
                  data,
                  sizeof(data));
    tx_frame_apply_schedule(ctx, out,
                            ctx->scheduled_target_enabled,
                            ctx->scheduled_target_offset_ns);
    return IGH_ECAN_OK;
}

int igh_cia402_build_prepared_target_frame(igh_cia402_context_t *ctx,
                                           const igh_cia402_axis_t *axis,
                                           const igh_cia402_target_step_t *target_step,
                                           igh_cia402_tx_frame_t *out)
{
    if ((ctx == NULL) || (axis == NULL) || (target_step == NULL) || (out == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if ((target_step->previous_position != axis->target_position) ||
        (target_step->step != axis->step) ||
        (target_step->data_len != axis_rpdo1_payload_len(axis))) {
        return IGH_ECAN_FAIL;
    }
    if (!axis_ready_for_target(axis)) {
        return IGH_ECAN_NOT_READY;
    }

    tx_frame_init(out,
                  axis->slave_pos,
                  axis->channel,
                  IGH_CIA402_COB_RPDO1_BASE + axis->node_id,
                  target_step->data,
                  target_step->data_len);
    tx_frame_apply_schedule(ctx, out,
                            ctx->scheduled_target_enabled,
                            ctx->scheduled_target_offset_ns);
    return IGH_ECAN_OK;
}

int igh_cia402_send_prepared_target(igh_cia402_context_t *ctx,
                                    igh_cia402_axis_t *axis,
                                    const igh_cia402_target_step_t *target_step)
{
    int rc;
    int32_t previous_position;
    igh_cia402_tx_frame_t tx;

    rc = igh_cia402_build_prepared_target_frame(ctx, axis, target_step, &tx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    previous_position = axis->target_position;
    axis->target_position = target_step->next_position;
    rc = send_tx_frame(ctx, &tx);
    if (rc != IGH_ECAN_OK) {
        axis->target_position = previous_position;
    }
    return rc;
}

int igh_cia402_send_target_step(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis)
{
    igh_cia402_target_step_t target_step;

    int rc = igh_cia402_prepare_target_step(axis, &target_step);

    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return igh_cia402_send_prepared_target(ctx, axis, &target_step);
}

int igh_cia402_send_target(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis)
{
    igh_cia402_tx_frame_t tx;
    int rc;

    rc = igh_cia402_build_target_frame(ctx, axis, &tx);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return send_tx_frame(ctx, &tx);
}

int igh_cia402_send_targets_all(igh_cia402_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t i = 0U; i < ctx->axis_count; i++) {
        igh_cia402_axis_t *axis = &ctx->axes[i];
        if (!axis->enabled || !axis_ready_for_target(axis)) {
            continue;
        }
        merge_result(&rc, igh_cia402_send_target_step(ctx, axis));
    }
    return rc;
}

int igh_cia402_on_can_frame(igh_cia402_context_t *ctx,
                             int slave_pos,
                             const igh_cia402_can_frame_t *frame)
{
    uint8_t node_id;
    igh_cia402_axis_t *axis;

    if ((ctx == NULL) || (frame == NULL)) {
        return IGH_ECAN_INVALID_ARG;
    }

    if ((frame->can_id >= IGH_CIA402_COB_HEARTBEAT_BASE) &&
        (frame->can_id <= (IGH_CIA402_COB_HEARTBEAT_BASE + IGH_ECAN_CANOPEN_NODE_ID_MAX)) &&
        frame->data_len >= 1U) {
        node_id = (uint8_t)(frame->can_id - IGH_CIA402_COB_HEARTBEAT_BASE);
        axis = find_axis_match(ctx, slave_pos, frame->channel, node_id);
        if (axis != NULL) {
            axis->heartbeat = frame->data[0];
            axis->heartbeat_valid = 1U;
        }
        return IGH_ECAN_OK;
    }

    if ((frame->can_id >= IGH_CIA402_COB_EMCY_BASE) &&
        (frame->can_id <= (IGH_CIA402_COB_EMCY_BASE + IGH_ECAN_CANOPEN_NODE_ID_MAX)) &&
        frame->data_len >= 8U) {
        node_id = (uint8_t)(frame->can_id - IGH_CIA402_COB_EMCY_BASE);
        axis = find_axis_match(ctx, slave_pos, frame->channel, node_id);
        if (axis != NULL) {
            axis->emcy_valid = 1U;
            axis->emcy_count++;
            axis->emcy_code = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8);
            axis->emcy_error_register = frame->data[2];
            memcpy(axis->emcy_data, &frame->data[3], sizeof(axis->emcy_data));
        }
        return IGH_ECAN_OK;
    }

    if ((frame->can_id >= IGH_CIA402_COB_SDO_RESPONSE_BASE) &&
        (frame->can_id <= (IGH_CIA402_COB_SDO_RESPONSE_BASE + IGH_ECAN_CANOPEN_NODE_ID_MAX)) &&
        frame->data_len >= 8U) {
        uint16_t index = (uint16_t)frame->data[1] | ((uint16_t)frame->data[2] << 8);
        uint8_t subindex = frame->data[3];
        node_id = (uint8_t)(frame->can_id - IGH_CIA402_COB_SDO_RESPONSE_BASE);
        axis = find_axis_match(ctx, slave_pos, frame->channel, node_id);
        if (getenv("IGH_CIA402_TRACE_SDO") != NULL) {
            printf("[trace-sdo] slave=%d ch=%u node=0x%02X cmd=0x%02X index=0x%04X:%02X value=0x%08" PRIX32 "\n",
                   slave_pos,
                   frame->channel,
                   node_id,
                   frame->data[0],
                   index,
                   subindex,
                   get_u32_le(&frame->data[4]));
        }
        if (axis != NULL) {
            uint8_t upload_size = sdo_upload_size_from_command(frame->data[0]);

            axis->sdo_response_valid = 1U;
            axis->sdo_response_command = frame->data[0];
            axis->sdo_response_index = index;
            axis->sdo_response_subindex = subindex;
            axis->sdo_response_size = upload_size;
            axis->sdo_response_value = (upload_size != 0U) ? get_u32_le(&frame->data[4]) : 0U;
            axis->sdo_abort_code = (frame->data[0] == IGH_CIA402_SDO_ABORT) ?
                                   get_u32_le(&frame->data[4]) :
                                   0U;
        }
        if ((axis != NULL) &&
            (frame->data[0] != IGH_CIA402_SDO_ABORT) &&
            (index == IGH_CIA402_IDX_POSITION_ACTUAL_VALUE)) {
            axis->actual_position = (int32_t)get_u32_le(&frame->data[4]);
            axis->target_position = axis->actual_position;
            axis->actual_position_valid = 1U;
        }
        if ((axis != NULL) &&
            (frame->data[0] != IGH_CIA402_SDO_ABORT) &&
            (index == IGH_CIA402_IDX_VELOCITY_ACTUAL_VALUE)) {
            axis->actual_velocity = (int32_t)get_u32_le(&frame->data[4]);
            axis->actual_velocity_valid = 1U;
        }
        if ((axis != NULL) &&
            (frame->data[0] != IGH_CIA402_SDO_ABORT) &&
            (index == IGH_CIA402_IDX_STATUSWORD)) {
            axis->statusword = (uint16_t)frame->data[4] | ((uint16_t)frame->data[5] << 8);
            axis->statusword_valid = 1U;
        }
    }

    if ((frame->can_id >= IGH_CIA402_COB_TPDO1_BASE) &&
        (frame->can_id <= (IGH_CIA402_COB_TPDO1_BASE + IGH_ECAN_CANOPEN_NODE_ID_MAX)) &&
        frame->data_len >= IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN) {
        node_id = (uint8_t)(frame->can_id - IGH_CIA402_COB_TPDO1_BASE);
        axis = find_axis_match(ctx, slave_pos, frame->channel, node_id);
        if (axis != NULL) {
            axis->statusword = get_u16_le(&frame->data[4]);
            axis->statusword_valid = 1U;
            axis->mode_display = (int8_t)frame->data[6];
            axis->mode_display_valid = 1U;
            if (axis->mode_display == (int8_t)IGH_CIA402_MODE_CSV) {
                axis->actual_velocity = (int32_t)get_u32_le(&frame->data[0]);
                axis->actual_velocity_valid = 1U;
            } else {
                axis->actual_position = (int32_t)get_u32_le(&frame->data[0]);
                if (axis->actual_position_valid == 0U) {
                    axis->target_position = axis->actual_position;
                }
                axis->actual_position_valid = 1U;
            }
        }
        return IGH_ECAN_OK;
    }

    return IGH_ECAN_OK;
}

int igh_cia402_process_rx(igh_cia402_context_t *ctx,
                          igh_cia402_rx_frame_t *frames,
                          size_t capacity,
                          size_t *frame_count)
{
    size_t count = 0U;
    int rc;

    if (!transport_can_receive(ctx) || ((frames == NULL) && (capacity > 0U))) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = transport_receive(ctx, frames, capacity, &count);
    for (size_t i = 0U; i < count; i++) {
        int frame_rc = igh_cia402_on_can_frame(ctx, frames[i].slave_pos, &frames[i].frame);
        if ((rc == IGH_ECAN_OK) && (frame_rc != IGH_ECAN_OK)) {
            rc = frame_rc;
        }
    }
    if (frame_count != NULL) {
        *frame_count = count;
    }
    return rc;
}

int igh_cia402_statusword_is_operation_enabled(uint16_t statusword)
{
    return ((statusword & IGH_CIA402_STATUSWORD_STATE_MASK) ==
            IGH_CIA402_STATUSWORD_OPERATION_ENABLED) ? 1 : 0;
}

const char *igh_cia402_statusword_state_name(uint16_t statusword)
{
    switch (statusword & IGH_CIA402_STATUSWORD_STATE_MASK) {
    case IGH_CIA402_STATUSWORD_NOT_READY:
        return "not-ready";
    case IGH_CIA402_STATUSWORD_SWITCH_DISABLED:
        return "switch-on-disabled";
    case IGH_CIA402_STATUSWORD_READY:
        return "ready-to-switch-on";
    case IGH_CIA402_STATUSWORD_SWITCHED_ON:
        return "switched-on";
    case IGH_CIA402_STATUSWORD_OPERATION_ENABLED:
        return "operation-enabled";
    case IGH_CIA402_STATUSWORD_QUICK_STOP:
        return "quick-stop-active";
    case IGH_CIA402_STATUSWORD_FAULT_REACTION:
        return "fault-reaction-active";
    case IGH_CIA402_STATUSWORD_FAULT:
        return "fault";
    default:
        return "unknown";
    }
}

const char *igh_cia402_heartbeat_state_name(uint8_t heartbeat)
{
    switch (heartbeat) {
    case IGH_CIA402_HEARTBEAT_BOOTUP:
        return "boot-up";
    case IGH_CIA402_HEARTBEAT_STOPPED:
        return "stopped";
    case IGH_CIA402_HEARTBEAT_OPERATIONAL:
        return "operational";
    case IGH_CIA402_HEARTBEAT_PREOPERATIONAL:
        return "pre-operational";
    default:
        return "unknown";
    }
}

const char *igh_cia402_sdo_abort_name(uint32_t abort_code)
{
    switch (abort_code) {
    case IGH_CIA402_SDO_ABORT_NONE:
        return "none";
    case IGH_CIA402_SDO_ABORT_TOGGLE:
        return "toggle-bit";
    case IGH_CIA402_SDO_ABORT_TIMEOUT:
        return "timeout";
    case IGH_CIA402_SDO_ABORT_CMD_SPECIFIER:
        return "bad-command";
    case IGH_CIA402_SDO_ABORT_OUT_OF_MEMORY:
        return "out-of-memory";
    case IGH_CIA402_SDO_ABORT_UNSUPPORTED:
        return "unsupported-access";
    case IGH_CIA402_SDO_ABORT_WRITE_ONLY:
        return "read-write-only";
    case IGH_CIA402_SDO_ABORT_READ_ONLY:
        return "write-read-only";
    case IGH_CIA402_SDO_ABORT_OBJECT_MISSING:
        return "object-not-exist";
    case IGH_CIA402_SDO_ABORT_PDO_MAP_OBJECT:
        return "cannot-map-pdo";
    case IGH_CIA402_SDO_ABORT_PDO_LENGTH:
        return "pdo-length-exceeded";
    case IGH_CIA402_SDO_ABORT_PARAM_INCOMPAT:
        return "parameter-incompatible";
    case IGH_CIA402_SDO_ABORT_HARDWARE:
        return "hardware-error";
    case IGH_CIA402_SDO_ABORT_TYPE_MISMATCH:
        return "type-length-mismatch";
    case IGH_CIA402_SDO_ABORT_TYPE_TOO_HIGH:
        return "type-length-too-high";
    case IGH_CIA402_SDO_ABORT_TYPE_TOO_LOW:
        return "type-length-too-low";
    case IGH_CIA402_SDO_ABORT_SUBINDEX_MISSING:
        return "subindex-not-exist";
    case IGH_CIA402_SDO_ABORT_RANGE_EXCEEDED:
        return "value-range";
    case IGH_CIA402_SDO_ABORT_VALUE_TOO_HIGH:
        return "value-too-high";
    case IGH_CIA402_SDO_ABORT_VALUE_TOO_LOW:
        return "value-too-low";
    case IGH_CIA402_SDO_ABORT_GENERAL:
        return "general-error";
    case IGH_CIA402_SDO_ABORT_STORE_TRANSFER:
        return "cannot-transfer-or-store";
    case IGH_CIA402_SDO_ABORT_LOCAL_CONTROL:
        return "local-control";
    case IGH_CIA402_SDO_ABORT_DEVICE_STATE:
        return "device-state";
    default:
        return "unknown";
    }
}
