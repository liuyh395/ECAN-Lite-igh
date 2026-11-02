#include "igh_mcan_pdo_internal.h"

#include <stdbool.h>
#include <string.h>

static const uint8_t s_dlc_to_len[16] = {
    0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U
};

/* -------------------------------------------------------------------------- */
/* Byte order and CRC helpers                                                  */
/* -------------------------------------------------------------------------- */

/* Decode a 16-bit little-endian field from PDO memory. */
uint16_t igh_mcan_pdo_get_u16_le(const uint8_t *src)
{
    return (uint16_t)src[0] | ((uint16_t)src[1] << 8);
}

/* Decode a 32-bit little-endian field from PDO memory. */
uint32_t igh_mcan_pdo_get_u32_le(const uint8_t *src)
{
    return (uint32_t)src[0] |
           ((uint32_t)src[1] << 8) |
           ((uint32_t)src[2] << 16) |
           ((uint32_t)src[3] << 24);
}

/* Encode a 16-bit little-endian field into PDO memory. */
void igh_mcan_pdo_put_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/* Encode a 32-bit little-endian field into PDO memory. */
void igh_mcan_pdo_put_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

static const uint16_t crc16_ccitt_false_table[256] = {
    0x0000U, 0x1021U, 0x2042U, 0x3063U, 0x4084U, 0x50A5U, 0x60C6U, 0x70E7U,
    0x8108U, 0x9129U, 0xA14AU, 0xB16BU, 0xC18CU, 0xD1ADU, 0xE1CEU, 0xF1EFU,
    0x1231U, 0x0210U, 0x3273U, 0x2252U, 0x52B5U, 0x4294U, 0x72F7U, 0x62D6U,
    0x9339U, 0x8318U, 0xB37BU, 0xA35AU, 0xD3BDU, 0xC39CU, 0xF3FFU, 0xE3DEU,
    0x2462U, 0x3443U, 0x0420U, 0x1401U, 0x64E6U, 0x74C7U, 0x44A4U, 0x5485U,
    0xA56AU, 0xB54BU, 0x8528U, 0x9509U, 0xE5EEU, 0xF5CFU, 0xC5ACU, 0xD58DU,
    0x3653U, 0x2672U, 0x1611U, 0x0630U, 0x76D7U, 0x66F6U, 0x5695U, 0x46B4U,
    0xB75BU, 0xA77AU, 0x9719U, 0x8738U, 0xF7DFU, 0xE7FEU, 0xD79DU, 0xC7BCU,
    0x48C4U, 0x58E5U, 0x6886U, 0x78A7U, 0x0840U, 0x1861U, 0x2802U, 0x3823U,
    0xC9CCU, 0xD9EDU, 0xE98EU, 0xF9AFU, 0x8948U, 0x9969U, 0xA90AU, 0xB92BU,
    0x5AF5U, 0x4AD4U, 0x7AB7U, 0x6A96U, 0x1A71U, 0x0A50U, 0x3A33U, 0x2A12U,
    0xDBFDU, 0xCBDCU, 0xFBBFU, 0xEB9EU, 0x9B79U, 0x8B58U, 0xBB3BU, 0xAB1AU,
    0x6CA6U, 0x7C87U, 0x4CE4U, 0x5CC5U, 0x2C22U, 0x3C03U, 0x0C60U, 0x1C41U,
    0xEDAEU, 0xFD8FU, 0xCDECU, 0xDDCDU, 0xAD2AU, 0xBD0BU, 0x8D68U, 0x9D49U,
    0x7E97U, 0x6EB6U, 0x5ED5U, 0x4EF4U, 0x3E13U, 0x2E32U, 0x1E51U, 0x0E70U,
    0xFF9FU, 0xEFBEU, 0xDFDDU, 0xCFFCU, 0xBF1BU, 0xAF3AU, 0x9F59U, 0x8F78U,
    0x9188U, 0x81A9U, 0xB1CAU, 0xA1EBU, 0xD10CU, 0xC12DU, 0xF14EU, 0xE16FU,
    0x1080U, 0x00A1U, 0x30C2U, 0x20E3U, 0x5004U, 0x4025U, 0x7046U, 0x6067U,
    0x83B9U, 0x9398U, 0xA3FBU, 0xB3DAU, 0xC33DU, 0xD31CU, 0xE37FU, 0xF35EU,
    0x02B1U, 0x1290U, 0x22F3U, 0x32D2U, 0x4235U, 0x5214U, 0x6277U, 0x7256U,
    0xB5EAU, 0xA5CBU, 0x95A8U, 0x8589U, 0xF56EU, 0xE54FU, 0xD52CU, 0xC50DU,
    0x34E2U, 0x24C3U, 0x14A0U, 0x0481U, 0x7466U, 0x6447U, 0x5424U, 0x4405U,
    0xA7DBU, 0xB7FAU, 0x8799U, 0x97B8U, 0xE75FU, 0xF77EU, 0xC71DU, 0xD73CU,
    0x26D3U, 0x36F2U, 0x0691U, 0x16B0U, 0x6657U, 0x7676U, 0x4615U, 0x5634U,
    0xD94CU, 0xC96DU, 0xF90EU, 0xE92FU, 0x99C8U, 0x89E9U, 0xB98AU, 0xA9ABU,
    0x5844U, 0x4865U, 0x7806U, 0x6827U, 0x18C0U, 0x08E1U, 0x3882U, 0x28A3U,
    0xCB7DU, 0xDB5CU, 0xEB3FU, 0xFB1EU, 0x8BF9U, 0x9BD8U, 0xABBBU, 0xBB9AU,
    0x4A75U, 0x5A54U, 0x6A37U, 0x7A16U, 0x0AF1U, 0x1AD0U, 0x2AB3U, 0x3A92U,
    0xFD2EU, 0xED0FU, 0xDD6CU, 0xCD4DU, 0xBDAAU, 0xAD8BU, 0x9DE8U, 0x8DC9U,
    0x7C26U, 0x6C07U, 0x5C64U, 0x4C45U, 0x3CA2U, 0x2C83U, 0x1CE0U, 0x0CC1U,
    0xEF1FU, 0xFF3EU, 0xCF5DU, 0xDF7CU, 0xAF9BU, 0xBFBAU, 0x8FD9U, 0x9FF8U,
    0x6E17U, 0x7E36U, 0x4E55U, 0x5E74U, 0x2E93U, 0x3EB2U, 0x0ED1U, 0x1EF0U,
};

/* Calculate CRC-16/CCITT-FALSE for the PDO payload bytes. */
static uint16_t crc16_ccitt_false(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;

    for (size_t i = 0U; i < len; i++) {
        uint8_t index = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ crc16_ccitt_false_table[index]);
    }

    return crc;
}

/* Dispatch the configured PDO payload CRC algorithm. */
uint16_t igh_mcan_pdo_calc_payload_crc(uint8_t crc_alg, const uint8_t *data, size_t len)
{
    if (crc_alg == IGH_MCAN_CRC_ALG_CCITT_FALSE) {
        return crc16_ccitt_false(data, len);
    }

    return 0U;
}

/* Publish a completed PDO header after payload writes reach memory. */
void igh_mcan_pdo_write_barrier(void)
{
#if defined(__GNUC__)
    __sync_synchronize();
#endif
}

/* -------------------------------------------------------------------------- */
/* PDO layout helpers                                                          */
/* -------------------------------------------------------------------------- */

/* Check that a PDO buffer size matches the ECAN layout unit rules. */
int igh_mcan_pdo_size_is_valid(size_t size)
{
    return size >= (IGH_MCAN_PDO_SIZE_UNIT * IGH_MCAN_PDO_MULTIPLIER_MIN) &&
           size <= IGH_MCAN_PDO_MAX_BYTES &&
           (size % IGH_MCAN_PDO_FINE_SIZE_UNIT) == 0U;
}

size_t igh_mcan_pdo_header_bytes_for_options(uint8_t option_flags)
{
    return ((option_flags & IGH_MCAN_OPT_HEADER_V2) != 0U) ?
           (size_t) IGH_MCAN_PDO_HEADER_V2_BYTES :
           (size_t) IGH_MCAN_PDO_HEADER_V1_BYTES;
}

/* Validate whether an encapsulation profile accepts a timestamp format. */
int igh_mcan_pdo_profile_allows_tsf(uint8_t profile, uint8_t tsf)
{
    if (tsf == IGH_MCAN_TSF_NONE) {
        return true;
    }
    if (tsf == IGH_MCAN_TSF_FULL64) {
        return true;
    }
    return (tsf == IGH_MCAN_TSF_DELTA16) && ((profile == 1U) || (profile == 5U));
}

/* Validate whether an encapsulation profile accepts fixed 64-byte data windows. */
int igh_mcan_pdo_profile_allows_d64(uint8_t profile)
{
    return (profile == 4U) || (profile == 5U);
}

/* Apply optional 4-byte packet alignment to one encoded CAN frame. */
size_t igh_mcan_pdo_align_packet_len(uint8_t option_flags, size_t len)
{
    if ((option_flags & IGH_MCAN_OPT_ALIGN4) == 0U) {
        return len;
    }
    return (len + 3U) & ~(size_t)3U;
}

/* Build the compact layout signature stored in the PDO header. */
uint16_t igh_mcan_pdo_make_layout_id(size_t pdo_size,
                               uint8_t channel_mask,
                               uint8_t encapsulation_profile,
                               uint8_t option_flags)
{
    uint8_t multiplier = (uint8_t)(pdo_size / IGH_MCAN_PDO_SIZE_UNIT);
    uint8_t fine = (uint8_t)((pdo_size % IGH_MCAN_PDO_SIZE_UNIT) /
                             IGH_MCAN_PDO_FINE_SIZE_UNIT);
    uint32_t layout_word = ((uint32_t)(multiplier - 1U) & 0x1FUL)
                         | (((uint32_t)channel_mask &
                             (uint32_t)IGH_ECAN_MCAN_CHANNEL_MASK_ALL) << 5)
                         | (((uint32_t)encapsulation_profile & 0x3FUL) << 13)
                         | (((uint32_t)option_flags & 0xFFUL) << 21)
                         | ((uint32_t)fine << 29);

    return (uint16_t)(layout_word ^ (layout_word >> 16));
}

/* Return the encoded byte count of a timestamp/schedule field. */
size_t igh_mcan_pdo_tsf_len(uint8_t tsf)
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

/* Fill parse diagnostics consistently before returning a parser failure. */
int igh_mcan_pdo_parse_fail(igh_mcan_pdo_parse_diag_t *diag,
                      igh_mcan_pdo_parse_error_t code,
                      size_t offset,
                      size_t packet_end,
                      uint16_t ctrl,
                      const igh_mcan_pdo_header_t *header,
                      uint8_t channel,
                      uint8_t dlc,
                      uint8_t tsf,
                      uint8_t kind)
{
    if (diag != NULL) {
        memset(diag, 0, sizeof(*diag));
        diag->code = code;
        diag->offset = offset;
        diag->packet_end = packet_end;
        diag->ctrl = ctrl;
        diag->channel = channel;
        diag->dlc = dlc;
        diag->tsf = tsf;
        diag->kind = kind;
        if (header != NULL) {
            diag->version = header->version;
            diag->ihl_words = header->ihl_words;
            diag->header_flags = header->flags;
            diag->packet_sequence = header->seq_begin;
            diag->frame_bytes = header->payload_len;
            diag->payload_crc16 = header->payload_crc16;
            diag->layout_id = header->layout_id;
            diag->header_channel_mask = header->channel_mask;
            diag->crc_alg = header->crc_alg;
        }
    }

    return IGH_ECAN_FAIL;
}

/* Treat an all-zero header as an empty PDO cycle instead of malformed data. */
int igh_mcan_pdo_header_is_empty(const igh_mcan_pdo_header_t *header)
{
    if (header == NULL) {
        return false;
    }
    if (header->version != 0U ||
        header->ihl_words != 0U ||
        header->flags != 0U ||
        header->seq_begin != 0U ||
        header->seq_end != 0U ||
        header->payload_len != 0U ||
        header->payload_crc16 != 0U ||
        header->layout_id != 0U ||
        header->channel_mask != 0U ||
        header->frame_count != 0U ||
        header->overflow_sat != 0U ||
        header->crc_alg != 0U) {
        return false;
    }
    for (size_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
        if (header->channel_status[ch] != 0U) {
            return false;
        }
    }
    return true;
}

/* -------------------------------------------------------------------------- */
/* Public DLC helpers                                                          */
/* -------------------------------------------------------------------------- */

/* Convert a CAN/CAN FD DLC code into its payload byte capacity. */
uint8_t igh_mcan_dlc_to_len(uint8_t dlc)
{
    return s_dlc_to_len[(dlc < 16U) ? dlc : 15U];
}

/* Select the smallest DLC that can carry the requested payload length. */
uint8_t igh_mcan_len_to_dlc(size_t len, int canfd)
{
    if (!canfd && len > 8U) {
        len = 8U;
    }
    for (uint8_t dlc = 0U; dlc < 16U; dlc++) {
        if (len <= s_dlc_to_len[dlc]) {
            return dlc;
        }
    }
    return 15U;
}

/* Decode and validate the fixed ECAN MCAN PDO header. */
int igh_mcan_pdo_read_header(const uint8_t *data,
                              size_t size,
                              igh_mcan_pdo_header_t *header)
{
    uint8_t ver_ihl;
    size_t header_bytes;

    if ((data == NULL) || (header == NULL) || size < 12U) {
        return IGH_ECAN_INVALID_ARG;
    }

    memset(header, 0, sizeof(*header));
    ver_ihl = data[IGH_MCAN_HDR_VER_IHL_OFFSET];
    header->version = (uint8_t)(ver_ihl >> 4U);
    header->ihl_words = (uint8_t)(ver_ihl & 0x0FU);
    header->flags = data[IGH_MCAN_HDR_FLAGS_OFFSET];
    header->seq_begin = igh_mcan_pdo_get_u16_le(&data[IGH_MCAN_HDR_SEQ_BEGIN_OFFSET]);
    header->seq_end = igh_mcan_pdo_get_u16_le(&data[IGH_MCAN_HDR_SEQ_END_OFFSET]);
    header->payload_len = igh_mcan_pdo_get_u16_le(&data[IGH_MCAN_HDR_PAYLOAD_LEN_OFFSET]);
    header->payload_crc16 = igh_mcan_pdo_get_u16_le(&data[IGH_MCAN_HDR_PAYLOAD_CRC_OFFSET]);
    header->layout_id = igh_mcan_pdo_get_u16_le(&data[IGH_MCAN_HDR_LAYOUT_ID_OFFSET]);
    if (igh_mcan_pdo_header_is_empty(header)) {
        return IGH_ECAN_OK;
    }
    header_bytes = (size_t)header->ihl_words * 4U;
    if (header_bytes > size ||
        header->seq_begin != header->seq_end ||
        (size_t)header->payload_len > size - header_bytes) {
        return IGH_ECAN_FAIL;
    }
    header->header_bytes = (uint8_t)header_bytes;
    if ((header->version == IGH_MCAN_HDR_VERSION_V1) &&
        (header->ihl_words == IGH_MCAN_HDR_V1_IHL_WORDS)) {
        header->channel_mask = data[IGH_MCAN_HDR_CHANNEL_MASK_OFFSET];
        header->frame_count = data[IGH_MCAN_HDR_FRAME_COUNT_OFFSET];
        header->overflow_sat = data[IGH_MCAN_HDR_OVERFLOW_SAT_OFFSET];
        header->crc_alg = data[IGH_MCAN_HDR_CRC_ALG_OFFSET];
        for (size_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
            header->channel_status[ch] =
                igh_mcan_pdo_get_u16_le(&data[IGH_MCAN_HDR_CH_STATUS_OFFSET + ch * 2U]);
        }
        if ((header->crc_alg != IGH_MCAN_CRC_ALG_CCITT_FALSE) &&
            ((header->crc_alg != IGH_MCAN_CRC_ALG_NONE) ||
             (header->payload_crc16 != 0U))) {
            return IGH_ECAN_FAIL;
        }
    } else if ((header->version == IGH_MCAN_HDR_VERSION_V2) &&
               (header->ihl_words == IGH_MCAN_HDR_V2_IHL_WORDS)) {
        header->overflow_sat =
            ((header->flags & IGH_MCAN_HDR_FLAG_OVERFLOW_OCCURRED) != 0U) ? 1U : 0U;
        for (size_t ch = 0U; ch < IGH_MCAN_CHANNEL_COUNT; ch++) {
            header->channel_status[ch] = data[IGH_MCAN_HDR_V2_CH_STATUS_OFFSET + ch];
        }
    } else {
        return IGH_ECAN_FAIL;
    }

    return IGH_ECAN_OK;
}

/* Convert parser error codes into stable diagnostic strings. */
const char *igh_mcan_pdo_parse_error_name(igh_mcan_pdo_parse_error_t code)
{
    switch (code) {
    case IGH_MCAN_PDO_PARSE_OK:
        return "ok";
    case IGH_MCAN_PDO_PARSE_BAD_ARG:
        return "bad-arg";
    case IGH_MCAN_PDO_PARSE_BAD_HEADER:
        return "bad-header";
    case IGH_MCAN_PDO_PARSE_BAD_CHANNEL_MASK:
        return "bad-channel-mask";
    case IGH_MCAN_PDO_PARSE_SHORT_CTRL:
        return "short-ctrl";
    case IGH_MCAN_PDO_PARSE_BAD_KIND:
        return "bad-kind";
    case IGH_MCAN_PDO_PARSE_BAD_CHANNEL:
        return "bad-channel";
    case IGH_MCAN_PDO_PARSE_BAD_TSF:
        return "bad-tsf";
    case IGH_MCAN_PDO_PARSE_BAD_D64:
        return "bad-d64";
    case IGH_MCAN_PDO_PARSE_BAD_CLASSIC:
        return "bad-classic";
    case IGH_MCAN_PDO_PARSE_TRUNCATED:
        return "truncated";
    case IGH_MCAN_PDO_PARSE_BAD_PADDING:
        return "bad-padding";
    case IGH_MCAN_PDO_PARSE_CALLBACK_FAIL:
        return "callback-fail";
    default:
        return "unknown";
    }
}
