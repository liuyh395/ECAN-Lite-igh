#include "igh_mcan_pdo_internal.h"

#include <string.h>

typedef struct {
    igh_mcan_frame_t frame;
    uint16_t ctrl;
    uint8_t ch;
    uint8_t dlc;
    uint8_t tsf;
    uint8_t kind;
    size_t packet_start;
    size_t packet_end;
    size_t data_window_len;
    size_t real_data_len;
} pdo_packet_view_t;

/* Read the PDO header and reject channel-mask or payload-CRC mismatches early. */
static int parse_header_and_crc(const uint8_t *data,
                                size_t size,
                                uint8_t expected_channel_mask,
                                uint8_t option_flags,
                                igh_mcan_pdo_header_t *header,
                                igh_mcan_pdo_parse_diag_t *diag)
{
    size_t header_bytes;
    uint16_t payload_crc;

    if ((data == NULL) || (header == NULL) || size < 12U) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_ARG, 0U, 0U, 0U, NULL,
                          0U, 0U, 0U, 0U);
    }
    if (igh_mcan_pdo_read_header(data, size, header) != IGH_ECAN_OK) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER, 0U, 0U, 0U, NULL,
                          0U, 0U, 0U, 0U);
    }
    if (igh_mcan_pdo_header_is_empty(header)) {
        return IGH_ECAN_OK;
    }
    if (((header->version == IGH_MCAN_HDR_VERSION_V2) !=
         ((option_flags & IGH_MCAN_OPT_HEADER_V2) != 0U))) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER,
                          0U, 0U, 0U, header, 0U, 0U, 0U, 0U);
    }
    if (header->version == IGH_MCAN_HDR_VERSION_V2) {
        header->channel_mask = expected_channel_mask;
        header->crc_alg = ((option_flags & IGH_MCAN_OPT_NO_CRC16) != 0U) ?
                          IGH_MCAN_CRC_ALG_NONE : IGH_MCAN_CRC_ALG_CCITT_FALSE;
    } else if ((((option_flags & IGH_MCAN_OPT_NO_CRC16) != 0U) &&
                ((header->crc_alg != IGH_MCAN_CRC_ALG_NONE) ||
                 (header->payload_crc16 != 0U))) ||
               (((option_flags & IGH_MCAN_OPT_NO_CRC16) == 0U) &&
                (header->crc_alg != IGH_MCAN_CRC_ALG_CCITT_FALSE))) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER,
                          0U, 0U, 0U, header, 0U, 0U, 0U, 0U);
    }
    if ((header->channel_mask == 0U) ||
        ((header->channel_mask & ~expected_channel_mask) != 0U)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_CHANNEL_MASK,
                          header->header_bytes, 0U, 0U, header,
                          0U, 0U, 0U, 0U);
    }

    header_bytes = (size_t)header->ihl_words * 4U;
    if ((option_flags & IGH_MCAN_OPT_NO_CRC16) != 0U) {
        if (header->payload_crc16 != 0U) {
            return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER,
                                           0U, 0U, 0U, header, 0U, 0U, 0U, 0U);
        }
    } else {
        payload_crc = igh_mcan_pdo_calc_payload_crc(
            header->crc_alg, &data[header_bytes], header->payload_len);
        if (payload_crc != header->payload_crc16) {
            return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER,
                                           0U, 0U, 0U, header, 0U, 0U, 0U, 0U);
        }
    }
    return IGH_ECAN_OK;
}

/* Re-read the header after CRC validation to catch concurrent writer updates. */
static int header_is_stable(const uint8_t *data,
                            size_t size,
                            const igh_mcan_pdo_header_t *header)
{
    igh_mcan_pdo_header_t after;

    if ((header == NULL) || igh_mcan_pdo_read_header(data, size, &after) != IGH_ECAN_OK) {
        return 0;
    }
    if (!(after.version == header->version &&
           after.ihl_words == header->ihl_words &&
           after.flags == header->flags &&
           after.seq_begin == header->seq_begin &&
           after.seq_end == header->seq_end &&
           after.payload_len == header->payload_len &&
           after.payload_crc16 == header->payload_crc16 &&
           after.layout_id == header->layout_id)) {
        return 0;
    }
    for (size_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
        if (after.channel_status[ch] != header->channel_status[ch]) {
            return 0;
        }
    }
    return (header->version == IGH_MCAN_HDR_VERSION_V2) ||
           (after.channel_mask == header->channel_mask &&
            after.frame_count == header->frame_count &&
            after.overflow_sat == header->overflow_sat &&
            after.crc_alg == header->crc_alg);
}

/* Decode the compact per-frame control word into a packet view. */
static void packet_decode_ctrl(pdo_packet_view_t *packet, const uint8_t *data, size_t offset)
{
    memset(packet, 0, sizeof(*packet));
    packet->packet_start = offset;
    packet->ctrl = igh_mcan_pdo_get_u16_le(&data[offset]);
    packet->ch = (uint8_t)(packet->ctrl & IGH_MCAN_CTRL_CH_MASK);
    packet->dlc = (uint8_t)((packet->ctrl & IGH_MCAN_CTRL_DLC_MASK) >>
                            IGH_MCAN_CTRL_DLC_SHIFT);
    packet->tsf = (uint8_t)((packet->ctrl & IGH_MCAN_CTRL_TSF_MASK) >>
                            IGH_MCAN_CTRL_TSF_SHIFT);
    packet->kind = (uint8_t)((packet->ctrl & IGH_MCAN_CTRL_KIND_MASK) >>
                             IGH_MCAN_CTRL_KIND_SHIFT);
}

/* Validate channel, timestamp, and profile-specific control bits. */
static int packet_validate_ctrl(const pdo_packet_view_t *packet,
                                const igh_mcan_pdo_header_t *header,
                                uint8_t expected_channel_mask,
                                uint8_t encapsulation_profile,
                                igh_mcan_pdo_parse_diag_t *diag)
{
    uint8_t channel_bit = IGH_ECAN_BIT_U8(packet->ch);

    if (packet->kind != IGH_MCAN_KIND_CAN_FRAME) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_KIND, packet->packet_start, 0U,
                          packet->ctrl, header, packet->ch, packet->dlc,
                          packet->tsf, packet->kind);
    }
    if (packet->ch >= IGH_MCAN_CHANNEL_COUNT ||
        ((expected_channel_mask & channel_bit) == 0U) ||
        ((header->channel_mask & channel_bit) == 0U)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_CHANNEL, packet->packet_start, 0U,
                          packet->ctrl, header, packet->ch, packet->dlc,
                          packet->tsf, packet->kind);
    }
    if (!igh_mcan_pdo_profile_allows_tsf(encapsulation_profile, packet->tsf)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_TSF, packet->packet_start, 0U,
                          packet->ctrl, header, packet->ch, packet->dlc,
                          packet->tsf, packet->kind);
    }
    if ((packet->ctrl & IGH_MCAN_CTRL_D64_MASK) != 0U &&
        !igh_mcan_pdo_profile_allows_d64(encapsulation_profile)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_D64, packet->packet_start, 0U,
                          packet->ctrl, header, packet->ch, packet->dlc,
                          packet->tsf, packet->kind);
    }
    return IGH_ECAN_OK;
}

/* Derive frame shape and packet bounds before reading variable-length fields. */
static int packet_prepare_frame(pdo_packet_view_t *packet,
                                const igh_mcan_pdo_header_t *header,
                                size_t end,
                                uint8_t option_flags,
                                igh_mcan_pdo_parse_diag_t *diag)
{
    size_t id_len;
    size_t raw_len;
    size_t packet_len;

    packet->frame.channel = packet->ch;
    packet->frame.dlc = packet->dlc;
    packet->frame.is_extended = ((packet->ctrl & IGH_MCAN_CTRL_IDE_MASK) != 0U);
    packet->frame.is_canfd = ((packet->ctrl & IGH_MCAN_CTRL_FDF_MASK) != 0U);
    packet->frame.brs = ((packet->ctrl & IGH_MCAN_CTRL_BRS_MASK) != 0U);
    if (packet->frame.is_canfd) {
        packet->frame.esi = ((packet->ctrl & IGH_MCAN_CTRL_X_MASK) != 0U);
    } else {
        packet->frame.rtr = ((packet->ctrl & IGH_MCAN_CTRL_X_MASK) != 0U);
        if (packet->frame.brs ||
            packet->dlc > IGH_ECAN_CLASSIC_CAN_MAX_DLEN ||
            (packet->ctrl & IGH_MCAN_CTRL_D64_MASK) != 0U) {
            return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_CLASSIC,
                              packet->packet_start, 0U, packet->ctrl, header,
                              packet->ch, packet->dlc, packet->tsf, packet->kind);
        }
    }

    id_len = packet->frame.is_extended ? 4U : 2U;
    packet->real_data_len = packet->frame.rtr ? 0U : igh_mcan_dlc_to_len(packet->dlc);
    packet->data_window_len = ((packet->ctrl & IGH_MCAN_CTRL_D64_MASK) != 0U) ?
                              IGH_ECAN_CANFD_MAX_DLEN :
                              packet->real_data_len;
    raw_len = 2U + id_len + igh_mcan_pdo_tsf_len(packet->tsf) + packet->data_window_len;
    packet_len = igh_mcan_pdo_align_packet_len(option_flags, raw_len);
    packet->packet_end = packet->packet_start + packet_len;
    if (packet->packet_end > end) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_TRUNCATED,
                          packet->packet_start, packet->packet_end, packet->ctrl,
                          header, packet->ch, packet->dlc, packet->tsf, packet->kind);
    }
    return IGH_ECAN_OK;
}

/* Decode and validate one packet descriptor at the current payload offset. */
static int packet_read_at(const uint8_t *data,
                          size_t offset,
                          size_t end,
                          const igh_mcan_pdo_header_t *header,
                          uint8_t expected_channel_mask,
                          uint8_t encapsulation_profile,
                          uint8_t option_flags,
                          pdo_packet_view_t *packet,
                          igh_mcan_pdo_parse_diag_t *diag)
{
    int rc;

    if (offset + 2U > end) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_SHORT_CTRL, offset, end,
                          0U, header, 0U, 0U, 0U, 0U);
    }

    packet_decode_ctrl(packet, data, offset);
    rc = packet_validate_ctrl(packet, header, expected_channel_mask,
                              encapsulation_profile, diag);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    return packet_prepare_frame(packet, header, end, option_flags, diag);
}

/* Copy CAN identifier and payload bytes from the packet view into a frame. */
static void packet_copy_body(const uint8_t *data, pdo_packet_view_t *packet)
{
    size_t offset = packet->packet_start + 2U;

    if (packet->frame.is_extended) {
        packet->frame.can_id = igh_mcan_pdo_get_u32_le(&data[offset]) & IGH_ECAN_CAN_EXT_ID_MAX;
        offset += 4U;
    } else {
        packet->frame.can_id = igh_mcan_pdo_get_u16_le(&data[offset]) & IGH_ECAN_CAN_STD_ID_MAX;
        offset += 2U;
    }

    /* 原样交付时间字段；单位由已确认的固件契约决定，不猜测旧版本语义。 */
    packet->frame.rx_tsf = packet->tsf;
    size_t timestamp_bytes = igh_mcan_pdo_tsf_len(packet->tsf);
    memset(packet->frame.rx_timestamp_bytes, 0, sizeof(packet->frame.rx_timestamp_bytes));
    if (timestamp_bytes != 0U) {
        memcpy(packet->frame.rx_timestamp_bytes, &data[offset], timestamp_bytes);
    }
    offset += timestamp_bytes;
    if (packet->real_data_len > 0U) {
        memcpy(packet->frame.data, &data[offset], packet->real_data_len);
    }
    packet->frame.data_len = (uint8_t)packet->real_data_len;
}

/* Enforce zero-filled alignment padding so stale payload bytes are visible. */
static int packet_padding_is_zero(const uint8_t *data,
                                  const pdo_packet_view_t *packet,
                                  const igh_mcan_pdo_header_t *header,
                                  igh_mcan_pdo_parse_diag_t *diag)
{
    size_t offset = packet->packet_start + 2U;

    offset += packet->frame.is_extended ? 4U : 2U;
    offset += igh_mcan_pdo_tsf_len(packet->tsf);
    offset += packet->data_window_len;

    while (offset < packet->packet_end) {
        if (data[offset] != 0U) {
            return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_PADDING,
                              offset, packet->packet_end, packet->ctrl, header,
                              packet->ch, packet->dlc, packet->tsf, packet->kind);
        }
        offset++;
    }
    return IGH_ECAN_OK;
}

/* Parse every frame in a PDO buffer and report detailed diagnostics on failure. */
int igh_mcan_pdo_parse_frames_ex(const uint8_t *data,
                                  size_t size,
                                  uint8_t expected_channel_mask,
                                  uint8_t encapsulation_profile,
                                  uint8_t option_flags,
                                  igh_mcan_frame_cb_t cb,
                                  void *user,
                                  igh_mcan_pdo_header_t *out_header,
                                  igh_mcan_pdo_parse_diag_t *diag)
{
    igh_mcan_pdo_header_t header;
    uint8_t snapshot[IGH_MCAN_PDO_MAX_BYTES];
    size_t offset;
    size_t end;
    size_t header_bytes;
    uint8_t parsed_frames = 0U;
    int rc;

    if (diag != NULL) {
        memset(diag, 0, sizeof(*diag));
    }

    /* The caller owns the process image while it is copied. Callbacks may then
     * reuse that image without changing the already validated packet. */
    if ((data == NULL) || size > sizeof(snapshot)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_ARG,
                                       0U, 0U, 0U, NULL, 0U, 0U, 0U, 0U);
    }
    memcpy(snapshot, data, size);
    data = snapshot;

    rc = parse_header_and_crc(data, size, expected_channel_mask, option_flags,
                              &header, diag);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }
    if (out_header != NULL) {
        *out_header = header;
    }
    if (igh_mcan_pdo_header_is_empty(&header)) {
        return IGH_ECAN_OK;
    }
    if (!header_is_stable(data, size, &header)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER, 0U, 0U, 0U, &header,
                          0U, 0U, 0U, 0U);
    }

    header_bytes = (size_t)header.ihl_words * 4U;
    offset = header_bytes;
    end = offset + header.payload_len;
    while (offset < end) {
        pdo_packet_view_t packet;

        rc = packet_read_at(data, offset, end, &header, expected_channel_mask,
                            encapsulation_profile, option_flags, &packet, diag);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        rc = packet_padding_is_zero(data, &packet, &header, diag);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
        offset = packet.packet_end;
        if (parsed_frames < 0xFFU) {
            parsed_frames++;
        }
    }
    if ((header.version == IGH_MCAN_HDR_VERSION_V1) &&
        (parsed_frames != header.frame_count)) {
        return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_BAD_HEADER, 0U, end, 0U, &header,
                          0U, 0U, 0U, 0U);
    }
    if (header.version == IGH_MCAN_HDR_VERSION_V2) {
        /* V2 removes frame_count from the wire; expose the derived count to callers. */
        header.frame_count = parsed_frames;
    }
    /* No application side effects until the ENTIRE immutable stream passes.
     * A callback's own failure can still leave earlier callback side effects. */
    if (cb != NULL) {
        offset = header_bytes;
        while (offset < end) {
            pdo_packet_view_t packet;
            rc = packet_read_at(data, offset, end, &header, expected_channel_mask,
                                 encapsulation_profile, option_flags, &packet, diag);
            if (rc != IGH_ECAN_OK) {
                return rc;
            }
            packet_copy_body(data, &packet);
            if (cb(&packet.frame, &header, user) != IGH_ECAN_OK) {
                return igh_mcan_pdo_parse_fail(diag, IGH_MCAN_PDO_PARSE_CALLBACK_FAIL,
                                              packet.packet_start, packet.packet_end,
                                              packet.ctrl, &header, packet.ch, packet.dlc,
                                              packet.tsf, packet.kind);
            }
            offset = packet.packet_end;
        }
    }
    if (out_header != NULL) {
        *out_header = header;
    }

    return IGH_ECAN_OK;
}

/* Parse every frame in a PDO buffer without exposing diagnostics. */
int igh_mcan_pdo_parse_frames(const uint8_t *data,
                               size_t size,
                               uint8_t expected_channel_mask,
                               uint8_t encapsulation_profile,
                               uint8_t option_flags,
                               igh_mcan_frame_cb_t cb,
                               void *user,
                               igh_mcan_pdo_header_t *out_header)
{
    return igh_mcan_pdo_parse_frames_ex(data, size, expected_channel_mask,
                                         encapsulation_profile, option_flags,
                                         cb, user, out_header, NULL);
}
