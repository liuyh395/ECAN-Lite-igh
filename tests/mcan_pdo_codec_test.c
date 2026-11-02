#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "igh_mcan_pdo_codec.h"

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        return 1; \
    } \
} while (0)

typedef struct {
    igh_mcan_frame_t frames[4];
    size_t count;
} capture_t;

static int test_product_layout(void)
{
    uint8_t image[128];
    uint8_t fine_image[144];
    igh_mcan_pdo_writer_t writer;
    igh_mcan_pdo_header_t header;
    size_t expected_v1_header_bytes =
        (IGH_MCAN_CHANNEL_COUNT == 8U) ? 32U : 24U;
    size_t expected_v2_header_bytes =
        (IGH_MCAN_CHANNEL_COUNT == 8U) ? 20U : 16U;
    uint8_t expected_channel_mask =
        (IGH_MCAN_CHANNEL_COUNT == 8U) ? 0xFFU : 0x0FU;

    CHECK((size_t)IGH_MCAN_PDO_HEADER_V1_BYTES == expected_v1_header_bytes);
    CHECK((size_t)IGH_MCAN_PDO_HEADER_V2_BYTES == expected_v2_header_bytes);
    CHECK((uint8_t)IGH_ECAN_MCAN_CHANNEL_MASK_ALL == expected_channel_mask);

    memset(image, 0, sizeof(image));
    igh_mcan_pdo_writer_init(&writer, image, sizeof(image), 0xFFU, 0U,
                              IGH_MCAN_OPT_DEFAULT);
    CHECK(writer.channel_mask == IGH_ECAN_MCAN_CHANNEL_MASK_ALL);
    igh_mcan_pdo_writer_begin(&writer);
    igh_mcan_pdo_writer_finish(&writer);
    CHECK(igh_mcan_pdo_read_header(image, sizeof(image), &header) == IGH_ECAN_OK);
    CHECK(header.version == IGH_MCAN_HDR_VERSION_V1);
    CHECK(header.ihl_words == IGH_MCAN_HDR_V1_IHL_WORDS);
    CHECK(header.header_bytes == IGH_MCAN_PDO_HEADER_V1_BYTES);
    CHECK(header.channel_mask == IGH_ECAN_MCAN_CHANNEL_MASK_ALL);
    CHECK(header.payload_crc16 == 0U);
    CHECK(header.crc_alg == IGH_MCAN_CRC_ALG_NONE);

    memset(fine_image, 0, sizeof(fine_image));
    igh_mcan_pdo_writer_init(&writer, fine_image, sizeof(fine_image), 0x0FU, 0U,
                              IGH_MCAN_OPT_DEFAULT);
    igh_mcan_pdo_writer_begin(&writer);
    igh_mcan_pdo_writer_finish(&writer);
    CHECK(igh_mcan_pdo_read_header(fine_image, sizeof(fine_image), &header) == IGH_ECAN_OK);
    CHECK(header.layout_id == 0x2160U);
    return 0;
}

static int capture_frame(const igh_mcan_frame_t *frame,
                         const igh_mcan_pdo_header_t *header,
                         void *user)
{
    capture_t *capture = (capture_t *)user;

    (void)header;
    if (capture->count >= 4U) {
        return -1;
    }
    capture->frames[capture->count++] = *frame;
    return 0;
}

static int test_round_trip_and_crc(void)
{
    uint8_t image[384];
    uint8_t corrupted[384];
    igh_mcan_pdo_writer_t writer;
    igh_mcan_pdo_header_t header;
    igh_mcan_pdo_parse_diag_t diag;
    igh_mcan_frame_t classic;
    igh_mcan_frame_t fd;
    capture_t capture;

    memset(image, 0xA5, sizeof(image));
    memset(&classic, 0, sizeof(classic));
    memset(&fd, 0, sizeof(fd));
    memset(&capture, 0, sizeof(capture));
    classic.channel = 0U;
    classic.can_id = 0x123U;
    classic.dlc = 8U;
    classic.data_len = 8U;
    for (size_t i = 0U; i < classic.data_len; i++) {
        classic.data[i] = (uint8_t)(0x10U + i);
    }
    fd.channel = (uint8_t)(IGH_MCAN_CHANNEL_COUNT - 1U);
    fd.can_id = 0x18FF1234U;
    fd.is_extended = 1U;
    fd.is_canfd = 1U;
    fd.brs = 1U;
    fd.dlc = 10U;
    fd.data_len = 16U;
    for (size_t i = 0U; i < fd.data_len; i++) {
        fd.data[i] = (uint8_t)(0x80U + i);
    }

    igh_mcan_pdo_writer_init(&writer, image, sizeof(image),
                              IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U, 0U);
    igh_mcan_pdo_writer_begin(&writer);
    CHECK(igh_mcan_pdo_writer_append(&writer, &classic) == IGH_ECAN_OK);
    CHECK(igh_mcan_pdo_writer_append(&writer, &fd) == IGH_ECAN_OK);
    igh_mcan_pdo_writer_finish(&writer);

    CHECK(igh_mcan_pdo_parse_frames_ex(image, sizeof(image),
                                        IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U, 0U,
                                        capture_frame, &capture, &header, &diag) == IGH_ECAN_OK);
    CHECK(capture.count == 2U);
    CHECK(header.frame_count == 2U);
    CHECK(capture.frames[0].can_id == classic.can_id);
    CHECK(memcmp(capture.frames[0].data, classic.data, classic.data_len) == 0);
    CHECK(capture.frames[1].can_id == fd.can_id);
    CHECK(capture.frames[1].is_canfd == 1U && capture.frames[1].brs == 1U);
    CHECK(memcmp(capture.frames[1].data, fd.data, fd.data_len) == 0);

    memcpy(corrupted, image, sizeof(corrupted));
    corrupted[IGH_MCAN_PDO_HEADER_V1_BYTES + 3U] ^= 0x01U;
    memset(&diag, 0, sizeof(diag));
    CHECK(igh_mcan_pdo_parse_frames_ex(corrupted, sizeof(corrupted),
                                       IGH_ECAN_MCAN_CHANNEL_MASK_ALL,
                                       0U, 0U, NULL, NULL, NULL, &diag) != IGH_ECAN_OK);
    CHECK(diag.code != IGH_MCAN_PDO_PARSE_OK);
    return 0;
}

static int test_v2_round_trip_and_crc_modes(void)
{
    uint8_t image[128];
    uint8_t corrupted[128];
    igh_mcan_pdo_writer_t writer;
    igh_mcan_pdo_header_t header;
    igh_mcan_pdo_parse_diag_t diag;
    igh_mcan_frame_t frame;
    capture_t capture;

    memset(&frame, 0, sizeof(frame));
    frame.channel = 0U;
    frame.can_id = 0x321U;
    frame.is_canfd = 1U;
    frame.brs = 1U;
    frame.dlc = 8U;
    frame.data_len = 8U;
    memset(frame.data, 0x5AU, frame.data_len);

    memset(image, 0, sizeof(image));
    memset(&capture, 0, sizeof(capture));
    igh_mcan_pdo_writer_init(&writer, image, sizeof(image),
                              IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U,
                              IGH_MCAN_OPT_V2_NO_CRC16);
    CHECK(writer.header_bytes == IGH_MCAN_PDO_HEADER_V2_BYTES);
    igh_mcan_pdo_writer_begin(&writer);
    CHECK(igh_mcan_pdo_writer_append(&writer, &frame) == IGH_ECAN_OK);
    igh_mcan_pdo_writer_finish(&writer);
    CHECK(igh_mcan_pdo_read_header(image, sizeof(image), &header) == IGH_ECAN_OK);
    CHECK(header.version == IGH_MCAN_HDR_VERSION_V2);
    CHECK(header.ihl_words == IGH_MCAN_HDR_V2_IHL_WORDS);
    CHECK(header.header_bytes == IGH_MCAN_PDO_HEADER_V2_BYTES);
    CHECK(header.payload_crc16 == 0U);
    CHECK(header.frame_count == 0U);
    CHECK(igh_mcan_pdo_parse_frames_ex(image, sizeof(image),
                                        IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U,
                                        IGH_MCAN_OPT_V2_NO_CRC16, capture_frame,
                                        &capture, &header, &diag) == IGH_ECAN_OK);
    CHECK(capture.count == 1U);
    CHECK(header.frame_count == 1U);
    CHECK(header.crc_alg == IGH_MCAN_CRC_ALG_NONE);
    CHECK(header.channel_mask == IGH_ECAN_MCAN_CHANNEL_MASK_ALL);

    memset(image, 0, sizeof(image));
    igh_mcan_pdo_writer_init(&writer, image, sizeof(image),
                              IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U,
                              IGH_MCAN_OPT_HEADER_V2);
    igh_mcan_pdo_writer_begin(&writer);
    CHECK(igh_mcan_pdo_writer_append(&writer, &frame) == IGH_ECAN_OK);
    igh_mcan_pdo_writer_finish(&writer);
    memcpy(corrupted, image, sizeof(corrupted));
    corrupted[IGH_MCAN_PDO_HEADER_V2_BYTES + 3U] ^= 0x01U;
    CHECK(igh_mcan_pdo_parse_frames_ex(corrupted, sizeof(corrupted),
                                       IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U,
                                       IGH_MCAN_OPT_HEADER_V2, NULL, NULL,
                                       NULL, &diag) != IGH_ECAN_OK);
    return 0;
}

static int test_wrap_and_capacity(void)
{
    uint8_t image[128];
    igh_mcan_pdo_writer_t writer;
    igh_mcan_pdo_header_t header;
    igh_mcan_frame_t frame;
    int append_rc = IGH_ECAN_OK;

    memset(image, 0, sizeof(image));
    memset(&frame, 0, sizeof(frame));
    frame.channel = 0U;
    frame.can_id = 0x321U;
    frame.is_canfd = 1U;
    frame.brs = 1U;
    frame.dlc = 15U;
    frame.data_len = 64U;

    igh_mcan_pdo_writer_init(&writer, image, sizeof(image), 0x01U, 0U, 0U);
    writer.packet_sequence = UINT16_MAX;
    igh_mcan_pdo_writer_begin(&writer);
    CHECK(writer.packet_sequence == 0U);
    CHECK(igh_mcan_pdo_writer_append(&writer, &frame) == IGH_ECAN_OK);
    while (append_rc == IGH_ECAN_OK) {
        append_rc = igh_mcan_pdo_writer_append(&writer, &frame);
    }
    CHECK(append_rc == IGH_ECAN_NO_SPACE);
    igh_mcan_pdo_writer_finish(&writer);
    CHECK(igh_mcan_pdo_read_header(image, sizeof(image), &header) == IGH_ECAN_OK);
    CHECK((header.flags & IGH_MCAN_HDR_FLAG_SEQ_WRAP) != 0U);
    CHECK((header.flags & IGH_MCAN_HDR_FLAG_STREAM_TRUNCATED) != 0U);
    CHECK(header.overflow_sat != 0U);
    return 0;
}

typedef struct {
    uint8_t *image;
    capture_t capture;
} snapshot_capture_t;

static int mutate_original_frame(const igh_mcan_frame_t *frame,
                                 const igh_mcan_pdo_header_t *header, void *user)
{
    snapshot_capture_t *state = (snapshot_capture_t *)user;
    memset(state->image, 0xFF, 128U);
    return capture_frame(frame, header, &state->capture);
}

static int test_validate_before_delivery(void)
{
    for (unsigned v2 = 0U; v2 < 2U; v2++) {
        for (unsigned crc = 0U; crc < 2U; crc++) {
            uint8_t image[128], corrupt[128];
            uint8_t options = (uint8_t)((v2 ? IGH_MCAN_OPT_HEADER_V2 : 0U) |
                                        (crc ? 0U : IGH_MCAN_OPT_NO_CRC16));
            size_t header_size = v2 ? IGH_MCAN_PDO_HEADER_V2_BYTES : IGH_MCAN_PDO_HEADER_V1_BYTES;
            igh_mcan_pdo_writer_t writer;
            igh_mcan_frame_t frame = {0};
            igh_mcan_pdo_header_t header;
            capture_t capture = {0};
            frame.can_id = 301U; frame.dlc = frame.data_len = 8U;
            frame.is_canfd = frame.brs = 1U;
            igh_mcan_pdo_writer_init(&writer, image, sizeof(image), 1U, 0U, options);
            igh_mcan_pdo_writer_begin(&writer);
            CHECK(igh_mcan_pdo_writer_append(&writer, &frame) == IGH_ECAN_OK);
            frame.can_id = 302U;
            CHECK(igh_mcan_pdo_writer_append(&writer, &frame) == IGH_ECAN_OK);
            /* Invalid KIND in the second frame; finish recalculates a VALID CRC. */
            image[header_size + 12U + 1U] |= 0x20U;
            igh_mcan_pdo_writer_finish(&writer);
            CHECK(igh_mcan_pdo_parse_frames(image, sizeof(image), 1U, 0U, options,
                                            capture_frame, &capture, &header) != IGH_ECAN_OK);
            CHECK(capture.count == 0U);
            image[header_size + 12U + 1U] &= (uint8_t)(0xffU ^ 0x20U);
            igh_mcan_pdo_writer_finish(&writer);
            memcpy(corrupt, image, sizeof(image));
            if (!v2) {
                corrupt[13] = 3U; /* Declared count disagrees with complete stream. */
                CHECK(igh_mcan_pdo_parse_frames(corrupt, sizeof(corrupt), 1U, 0U, options,
                                                capture_frame, &capture, NULL) != IGH_ECAN_OK);
                CHECK(capture.count == 0U);
            }
            snapshot_capture_t state = {0};
            state.image = image;
            CHECK(igh_mcan_pdo_parse_frames(image, sizeof(image), 1U, 0U, options,
                                            mutate_original_frame, &state, NULL) == IGH_ECAN_OK);
            CHECK(state.capture.count == 2U);
            CHECK(state.capture.frames[0].can_id == 301U);
            CHECK(state.capture.frames[1].can_id == 302U);
        }
    }
    return 0;
}

static int test_rx_timestamp_delivery(void)
{
    for (unsigned v2=0; v2<2; v2++) for (unsigned crc=0; crc<2; crc++) {
        for (unsigned invalid=0; invalid<2; invalid++) {
            uint8_t image[128]={0};
            uint8_t options=(v2?IGH_MCAN_OPT_HEADER_V2:0U)|(crc?0U:IGH_MCAN_OPT_NO_CRC16);
            igh_mcan_pdo_writer_t writer;
            igh_mcan_frame_t frame={0};capture_t capture={0};
            frame.can_id=0x321;frame.is_canfd=1;frame.brs=1;frame.dlc=8;frame.data_len=8;
            for(unsigned i=0;i<8;i++)frame.data[i]=(uint8_t)(0xa0+i);
            igh_mcan_pdo_writer_init(&writer,image,sizeof(image),1U,1U,options);
            igh_mcan_pdo_writer_begin(&writer);
            CHECK(igh_mcan_pdo_writer_append(&writer,&frame)==IGH_ECAN_OK);
            size_t start=writer.header_bytes;
            memmove(image+start+6,image+start+4,8);
            image[start+1]|=8U; /* TSF=1, Ctrl bit11 */
            image[start+4]=invalid?0xff:0x34;image[start+5]=invalid?0xff:0x12;
            writer.frame_bytes+=2;igh_mcan_pdo_writer_finish(&writer);
            CHECK(igh_mcan_pdo_parse_frames(image,sizeof(image),1U,1U,options,capture_frame,&capture,NULL)==IGH_ECAN_OK);
            CHECK(capture.count==1&&capture.frames[0].rx_tsf==1);
            CHECK(capture.frames[0].rx_timestamp_bytes[0]==(invalid?0xff:0x34));
            CHECK(capture.frames[0].rx_timestamp_bytes[1]==(invalid?0xff:0x12));
            for(unsigned i=2;i<8;i++)CHECK(capture.frames[0].rx_timestamp_bytes[i]==0);
            CHECK(memcmp(frame.data,capture.frames[0].data,8)==0);
        }
    }
    return 0;
}

int main(void)
{
    CHECK(test_rx_timestamp_delivery() == 0);
    CHECK(test_product_layout() == 0);
    CHECK(test_round_trip_and_crc() == 0);
    CHECK(test_v2_round_trip_and_crc_modes() == 0);
    CHECK(test_wrap_and_capacity() == 0);
    CHECK(test_validate_before_delivery() == 0);
    puts("mcan_pdo_codec_test: PASS");
    return 0;
}
