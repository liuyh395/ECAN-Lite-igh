#include "igh_mcan_pdo_internal.h"

#include <stdint.h>
#include <string.h>
#include <time.h>

/* PDO writer                                                                  */
/* -------------------------------------------------------------------------- */

static uint16_t pdo_writer_initial_sequence(const igh_mcan_pdo_writer_t *writer,
                                            const uint8_t *data)
{
    uintptr_t addr_mix = (uintptr_t)writer ^ ((uintptr_t)data >> 4);
    uint64_t mix = (uint64_t)addr_mix;

    /* The initial sequence is only a stale-image discriminator, not a
     * cryptographic nonce. C89 time()/clock() keep the portable codec free of
     * POSIX-only clock_gettime dependencies.
     */
    mix ^= (uint64_t)time(NULL);
    mix ^= (uint64_t)clock() << 16;
    mix ^= mix >> 32;
    mix ^= mix >> 16;
    return (uint16_t)(mix & 0xFFFFU);
}

/* Bind a writer to one PDO output buffer and remember its layout options. */
void igh_mcan_pdo_writer_init(igh_mcan_pdo_writer_t *writer,
                               uint8_t *data,
                               size_t size,
                               uint8_t channel_mask,
                               uint8_t encapsulation_profile,
                               uint8_t option_flags)
{
    if (writer == NULL) {
        return;
    }

    memset(writer, 0, sizeof(*writer));
    writer->data = data;
    writer->size = size;
    writer->channel_mask = channel_mask & IGH_ECAN_MCAN_CHANNEL_MASK_ALL;
    writer->encapsulation_profile = encapsulation_profile;
    writer->option_flags = option_flags;
    writer->header_bytes = (uint8_t)igh_mcan_pdo_header_bytes_for_options(option_flags);
    writer->packet_sequence = pdo_writer_initial_sequence(writer, data);
}

/* Start a new output cycle by clearing payload space and advancing sequence. */
void igh_mcan_pdo_writer_begin(igh_mcan_pdo_writer_t *writer)
{
    size_t clear_bytes;

    if ((writer == NULL) || (writer->data == NULL) || !igh_mcan_pdo_size_is_valid(writer->size)) {
        return;
    }

    if ((writer->option_flags & IGH_MCAN_OPT_PAYLOAD_ONLY) != 0U) {
        clear_bytes = writer->header_bytes;
    } else {
        clear_bytes = writer->last_clear_bytes;
        if (clear_bytes < writer->header_bytes) {
            clear_bytes = writer->header_bytes;
        }
    }
    if (clear_bytes > writer->size) {
        clear_bytes = writer->size;
    }
    memset(writer->data, 0, clear_bytes);
    writer->sequence_wrapped = (writer->packet_sequence == UINT16_MAX) ? 1U : 0U;
    writer->packet_sequence++;
    writer->frame_bytes = 0U;
    writer->frame_count = 0U;
    writer->overflow_sat = 0U;
}

/* Encode one CAN/CAN FD frame into the active PDO payload. */
int igh_mcan_pdo_writer_append_raw(igh_mcan_pdo_writer_t *writer,
                                   uint8_t channel,
                                   uint32_t can_id,
                                   uint8_t dlc,
                                   int is_extended,
                                   int is_canfd,
                                   int brs,
                                   int x_bit,
                                   uint8_t tx_tsf,
                                   uint8_t schedule_format,
                                   uint8_t schedule_flags,
                                   uint16_t schedule_tag,
                                   uint32_t schedule_offset_ns,
                                   const uint8_t *payload,
                                   size_t payload_len)
{
    uint16_t ctrl = 0U;
    size_t id_len;
    size_t tsf_len;
    size_t data_len;
    size_t copy_len;
    size_t raw_len;
    size_t packet_len;
    size_t offset;
    uint32_t max_id;

    if ((writer == NULL) || (writer->data == NULL) ||
        ((payload == NULL) && (payload_len > 0U)) ||
        !igh_mcan_pdo_size_is_valid(writer->size)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (channel >= IGH_MCAN_CHANNEL_COUNT ||
        ((writer->channel_mask & IGH_ECAN_BIT_U8(channel)) == 0U)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if ((!is_canfd && (brs != 0 ||
                       dlc > IGH_ECAN_CLASSIC_CAN_MAX_DLEN ||
                       payload_len > IGH_ECAN_CLASSIC_CAN_MAX_DLEN)) ||
        (is_canfd && (dlc > 15U || payload_len > IGH_ECAN_CANFD_MAX_DLEN))) {
        return IGH_ECAN_INVALID_ARG;
    }
    max_id = (is_extended != 0) ? IGH_ECAN_CAN_EXT_ID_MAX : IGH_ECAN_CAN_STD_ID_MAX;
    if (can_id > max_id) {
        return IGH_ECAN_INVALID_ARG;
    }
    if ((tx_tsf != IGH_MCAN_TSF_NONE) && (tx_tsf != IGH_MCAN_TSF_FULL64)) {
        return IGH_ECAN_INVALID_ARG;
    }

    id_len = (is_extended != 0) ? 4U : 2U;
    tsf_len = (tx_tsf == IGH_MCAN_TSF_FULL64) ? 8U : 0U;
    data_len = (!is_canfd && x_bit) ? 0U : igh_mcan_dlc_to_len(dlc);
    raw_len = 2U + id_len + tsf_len + data_len;
    packet_len = igh_mcan_pdo_align_packet_len(writer->option_flags, raw_len);

    if ((size_t)writer->header_bytes + writer->frame_bytes + packet_len > writer->size) {
        if (writer->overflow_sat < 0xFFU) {
            writer->overflow_sat++;
        }
        return IGH_ECAN_NO_SPACE;
    }

    offset = writer->header_bytes + writer->frame_bytes;
    ctrl |= (uint16_t)(channel & IGH_MCAN_CTRL_CH_MASK);
    ctrl |= (uint16_t)((dlc & 0x0FU) << IGH_MCAN_CTRL_DLC_SHIFT);
    ctrl |= (is_extended != 0) ? IGH_MCAN_CTRL_IDE_MASK : 0U;
    ctrl |= (is_canfd != 0) ? IGH_MCAN_CTRL_FDF_MASK : 0U;
    ctrl |= (brs != 0) ? IGH_MCAN_CTRL_BRS_MASK : 0U;
    ctrl |= (x_bit != 0) ? IGH_MCAN_CTRL_X_MASK : 0U;
    ctrl |= (uint16_t)((tx_tsf & 0x03U) << IGH_MCAN_CTRL_TSF_SHIFT);
    ctrl |= (uint16_t)(IGH_MCAN_KIND_CAN_FRAME << IGH_MCAN_CTRL_KIND_SHIFT);
    igh_mcan_pdo_put_u16_le(&writer->data[offset], ctrl);
    offset += 2U;

    if (is_extended != 0) {
        igh_mcan_pdo_put_u32_le(&writer->data[offset], can_id & IGH_ECAN_CAN_EXT_ID_MAX);
        offset += 4U;
    } else {
        igh_mcan_pdo_put_u16_le(&writer->data[offset], (uint16_t)(can_id & IGH_ECAN_CAN_STD_ID_MAX));
        offset += 2U;
    }
    if (tx_tsf == IGH_MCAN_TSF_FULL64) {
        writer->data[offset] = schedule_format;
        writer->data[offset + 1U] = schedule_flags;
        igh_mcan_pdo_put_u16_le(&writer->data[offset + 2U], schedule_tag);
        igh_mcan_pdo_put_u32_le(&writer->data[offset + 4U], schedule_offset_ns);
        offset += 8U;
    }

    if (data_len > 0U) {
        copy_len = (payload_len < data_len) ? payload_len : data_len;
        if (copy_len > 0U) {
            memcpy(&writer->data[offset], payload, copy_len);
        }
        if (copy_len < data_len) {
            memset(&writer->data[offset + copy_len], 0, data_len - copy_len);
        }
        offset += data_len;
    }
    if (packet_len > raw_len) {
        memset(&writer->data[offset], 0, packet_len - raw_len);
    }
    writer->frame_bytes = (uint16_t)(writer->frame_bytes + packet_len);
    if (writer->frame_count < 0xFFU) {
        writer->frame_count++;
    }
    return IGH_ECAN_OK;
}

/* Encode one frame object into the active PDO payload. */
int igh_mcan_pdo_writer_append(igh_mcan_pdo_writer_t *writer,
                                const igh_mcan_frame_t *frame)
{
    if (frame == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    return igh_mcan_pdo_writer_append_raw(writer,
                                          frame->channel,
                                          frame->can_id,
                                          frame->dlc,
                                          frame->is_extended,
                                          frame->is_canfd,
                                          frame->brs,
                                          frame->is_canfd ? frame->esi : frame->rtr,
                                          frame->tx_tsf,
                                          frame->schedule_format,
                                          frame->schedule_flags,
                                          frame->schedule_tag,
                                          frame->schedule_offset_ns,
                                          frame->data,
                                          frame->data_len);
}

/* Seal the PDO header so the receiver can verify sequence, length, and CRC. */
void igh_mcan_pdo_writer_finish(igh_mcan_pdo_writer_t *writer)
{
    uint8_t flags = 0U;
    uint8_t crc_alg;
    uint16_t payload_crc;

    if ((writer == NULL) || (writer->data == NULL) || !igh_mcan_pdo_size_is_valid(writer->size)) {
        return;
    }

    if (writer->overflow_sat != 0U) {
        flags |= IGH_MCAN_HDR_FLAG_STREAM_TRUNCATED | IGH_MCAN_HDR_FLAG_OVERFLOW_VALID;
    }
    if (writer->sequence_wrapped != 0U) {
        flags |= IGH_MCAN_HDR_FLAG_SEQ_WRAP;
    }

    igh_mcan_pdo_put_u16_le(&writer->data[IGH_MCAN_HDR_SEQ_END_OFFSET],
               (uint16_t)(writer->packet_sequence ^ 0xFFFFU));

    crc_alg = ((writer->option_flags & IGH_MCAN_OPT_NO_CRC16) != 0U) ?
              IGH_MCAN_CRC_ALG_NONE : IGH_MCAN_CRC_ALG_CCITT_FALSE;
    payload_crc = (crc_alg == IGH_MCAN_CRC_ALG_CCITT_FALSE) ?
                  igh_mcan_pdo_calc_payload_crc(crc_alg,
                                               &writer->data[writer->header_bytes],
                                               writer->frame_bytes) : 0U;

    writer->data[IGH_MCAN_HDR_VER_IHL_OFFSET] =
        ((writer->option_flags & IGH_MCAN_OPT_HEADER_V2) != 0U) ?
        IGH_MCAN_HDR_V2_VER_IHL : IGH_MCAN_HDR_V1_VER_IHL;
    writer->data[IGH_MCAN_HDR_FLAGS_OFFSET] = flags;
    igh_mcan_pdo_put_u16_le(&writer->data[IGH_MCAN_HDR_SEQ_BEGIN_OFFSET], writer->packet_sequence);
    igh_mcan_pdo_put_u16_le(&writer->data[IGH_MCAN_HDR_PAYLOAD_LEN_OFFSET], writer->frame_bytes);
    igh_mcan_pdo_put_u16_le(&writer->data[IGH_MCAN_HDR_PAYLOAD_CRC_OFFSET], payload_crc);
    igh_mcan_pdo_put_u16_le(&writer->data[IGH_MCAN_HDR_LAYOUT_ID_OFFSET],
               igh_mcan_pdo_make_layout_id(writer->size, writer->channel_mask,
                              writer->encapsulation_profile,
                              writer->option_flags));
    if ((writer->option_flags & IGH_MCAN_OPT_HEADER_V2) == 0U) {
        writer->data[IGH_MCAN_HDR_CHANNEL_MASK_OFFSET] = writer->channel_mask;
        writer->data[IGH_MCAN_HDR_FRAME_COUNT_OFFSET] = writer->frame_count;
        writer->data[IGH_MCAN_HDR_OVERFLOW_SAT_OFFSET] = writer->overflow_sat;
        writer->data[IGH_MCAN_HDR_CRC_ALG_OFFSET] = crc_alg;
    }
    writer->last_clear_bytes = (uint16_t)(((writer->option_flags & IGH_MCAN_OPT_PAYLOAD_ONLY) != 0U) ?
                                          writer->header_bytes :
                                          (writer->header_bytes + writer->frame_bytes));
    igh_mcan_pdo_write_barrier();
    igh_mcan_pdo_put_u16_le(&writer->data[IGH_MCAN_HDR_SEQ_END_OFFSET], writer->packet_sequence);
}

