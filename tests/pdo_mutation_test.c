#include "igh_mcan_pdo_codec.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "mutation check line %d\n", __LINE__); return 1; } } while (0)
static uint32_t random_state = 0xECAD1234U;
static uint32_t next_random(void)
{
    random_state ^= random_state << 13;
    random_state ^= random_state >> 17;
    random_state ^= random_state << 5;
    return random_state;
}

static int on_frame(const igh_mcan_frame_t *frame, const igh_mcan_pdo_header_t *header, void *user)
{
    size_t *count = user;
    (void)header;
    if (frame->channel >= IGH_MCAN_CHANNEL_COUNT || frame->data_len > 64U) abort();
    ++*count;
    return IGH_ECAN_OK;
}

int main(void)
{
    uint8_t seed[384];
    const uint8_t options[] = {0U, IGH_MCAN_OPT_NO_CRC16, IGH_MCAN_OPT_HEADER_V2,
                              IGH_MCAN_OPT_HEADER_V2 | IGH_MCAN_OPT_NO_CRC16};
    for (size_t mode = 0; mode < sizeof(options); ++mode) {
        igh_mcan_pdo_writer_t writer;
        igh_mcan_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.channel = IGH_MCAN_CHANNEL_COUNT - 1U;
        frame.can_id = 0x123U;
        frame.dlc = 15U;
        frame.data_len = 64U;
        frame.is_canfd = 1U;
        frame.brs = 1U;
        memset(frame.data, 0x5A, sizeof(frame.data));
        memset(seed, 0, sizeof(seed));
        igh_mcan_pdo_writer_init(&writer, seed, sizeof(seed), IGH_ECAN_MCAN_CHANNEL_MASK_ALL, 0U, options[mode]);
        igh_mcan_pdo_writer_begin(&writer);
        CHECK(igh_mcan_pdo_writer_append(&writer, &frame) == IGH_ECAN_OK);
        igh_mcan_pdo_writer_finish(&writer);
        /* 容量标志不表示本包正文损坏；V1/V2、CRC 开关均应交付完整帧。
         * 模拟正文已冻结、后续条目无法装入的合法状态，不改变正文长度。 */
        {
            size_t callbacks = 0U;
            igh_mcan_pdo_header_t header;
            igh_mcan_pdo_parse_diag_t diag;
            writer.overflow_sat = 1U;
            igh_mcan_pdo_writer_finish(&writer);
            CHECK(igh_mcan_pdo_parse_frames_ex(seed, sizeof(seed), IGH_ECAN_MCAN_CHANNEL_MASK_ALL,
                                               0U, options[mode], on_frame, &callbacks, &header, &diag)
                  == IGH_ECAN_OK);
            CHECK(callbacks == 1U);
            CHECK((header.flags & (IGH_MCAN_HDR_FLAG_STREAM_TRUNCATED | IGH_MCAN_HDR_FLAG_OVERFLOW_VALID))
                  == (IGH_MCAN_HDR_FLAG_STREAM_TRUNCATED | IGH_MCAN_HDR_FLAG_OVERFLOW_VALID));
            if ((options[mode] & IGH_MCAN_OPT_HEADER_V2) == 0U) {
                CHECK(header.frame_count == 1U && header.overflow_sat == 1U);
            }
            writer.overflow_sat = 0U;
            igh_mcan_pdo_writer_finish(&writer);
        }
        for (unsigned iteration = 0; iteration < 5000U; ++iteration) {
            size_t size = iteration % 3U == 0U ? next_random() % (IGH_MCAN_PDO_MAX_BYTES + 2U) : sizeof(seed);
            uint8_t *input = malloc(size ? size : 1U);
            uint8_t *before = malloc(size ? size : 1U);
            size_t callbacks = 0U;
            igh_mcan_pdo_parse_diag_t diag;
            int result;
            CHECK(input && before);
            memset(input, 0, size);
            memcpy(input, seed, size < sizeof(seed) ? size : sizeof(seed));
            if (iteration % 5U != 0U && size != 0U) {
                for (unsigned flip = 0; flip < 3U; ++flip)
                    input[next_random() % size] ^= (uint8_t)(1U << (next_random() % 8U));
            }
            memcpy(before, input, size);
            result = igh_mcan_pdo_parse_frames_ex(input, size, IGH_ECAN_MCAN_CHANNEL_MASK_ALL,
                                                 0U, options[mode], on_frame, &callbacks, NULL, &diag);
            CHECK(memcmp(before, input, size) == 0);
            CHECK(result == IGH_ECAN_OK || callbacks == 0U);
            CHECK(callbacks <= IGH_MCAN_PDO_MAX_BYTES / 4U);
            free(before);
            free(input);
        }
    }
    printf("PDO mutation: 20000 cases, V1/V2, CRC on/off, channels=%u\n", IGH_MCAN_CHANNEL_COUNT);
    return 0;
}
