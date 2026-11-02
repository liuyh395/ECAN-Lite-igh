/*
 * ECAN-Lite4 MCAN PDO codec.
 *
 * This header owns the dynamic PDO wire format, CAN/CANFD frame model,
 * PDO writer, header reader, and parser diagnostics.
 */

#ifndef IGH_MCAN_PDO_CODEC_H
#define IGH_MCAN_PDO_CODEC_H

#include <stddef.h>
#include <stdint.h>

#include "igh_ecan_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PDO codec is portable: these constants intentionally do not depend on IgH. */
#define IGH_MCAN_PDO_SIZE_UNIT             128U
#define IGH_MCAN_PDO_FINE_SIZE_UNIT        16U
#define IGH_MCAN_PDO_MULTIPLIER_MIN        1U
#define IGH_MCAN_PDO_MULTIPLIER_MAX        10U
#define IGH_MCAN_PDO_FINE_MULTIPLIER_MAX   7U
#define IGH_MCAN_PDO_DEFAULT_MULTIPLIER    IGH_ECAN_DEFAULT_PDO_MULTIPLIER
#define IGH_MCAN_PDO_MAX_BYTES             \
    (IGH_MCAN_PDO_SIZE_UNIT * IGH_MCAN_PDO_MULTIPLIER_MAX + \
     IGH_MCAN_PDO_FINE_SIZE_UNIT * IGH_MCAN_PDO_FINE_MULTIPLIER_MAX)
#define IGH_MCAN_PDO_DEFAULT_BYTES         (IGH_MCAN_PDO_SIZE_UNIT * IGH_MCAN_PDO_DEFAULT_MULTIPLIER)

/* V1: 16B common + 2B/channel; V2: 12B common + 1B/channel. */
#define IGH_MCAN_PDO_HEADER_V1_BYTES       (16U + (IGH_MCAN_CHANNEL_COUNT * 2U))
#define IGH_MCAN_PDO_HEADER_V2_BYTES       (12U + IGH_MCAN_CHANNEL_COUNT)
#define IGH_MCAN_PDO_HEADER_BYTES          IGH_MCAN_PDO_HEADER_V1_BYTES

/* 24B 固定头部字段偏移。 */
#define IGH_MCAN_HDR_VER_IHL_OFFSET        0U
#define IGH_MCAN_HDR_FLAGS_OFFSET          1U
#define IGH_MCAN_HDR_SEQ_BEGIN_OFFSET      2U
#define IGH_MCAN_HDR_SEQ_END_OFFSET        4U
#define IGH_MCAN_HDR_PAYLOAD_LEN_OFFSET    6U
#define IGH_MCAN_HDR_PAYLOAD_CRC_OFFSET    8U
#define IGH_MCAN_HDR_LAYOUT_ID_OFFSET      10U
#define IGH_MCAN_HDR_CHANNEL_MASK_OFFSET   12U
#define IGH_MCAN_HDR_FRAME_COUNT_OFFSET    13U
#define IGH_MCAN_HDR_OVERFLOW_SAT_OFFSET   14U
#define IGH_MCAN_HDR_CRC_ALG_OFFSET        15U
#define IGH_MCAN_HDR_CH_STATUS_OFFSET      16U
#define IGH_MCAN_HDR_V2_CH_STATUS_OFFSET   12U

#define IGH_MCAN_HDR_VERSION_V1            1U
#define IGH_MCAN_HDR_VERSION_V2            2U
#define IGH_MCAN_HDR_V1_IHL_WORDS          (IGH_MCAN_PDO_HEADER_V1_BYTES / 4U)
#define IGH_MCAN_HDR_V2_IHL_WORDS          (IGH_MCAN_PDO_HEADER_V2_BYTES / 4U)
#define IGH_MCAN_HDR_V1_VER_IHL            ((uint8_t)((IGH_MCAN_HDR_VERSION_V1 << 4U) | IGH_MCAN_HDR_V1_IHL_WORDS))
#define IGH_MCAN_HDR_V2_VER_IHL            ((uint8_t)((IGH_MCAN_HDR_VERSION_V2 << 4U) | IGH_MCAN_HDR_V2_IHL_WORDS))

#define IGH_MCAN_CRC_ALG_NONE              0U
#define IGH_MCAN_CRC_ALG_CCITT_FALSE       1U

/* 固定头部 flags：用于快速感知整包状态，详细原因通过 SDO 查询。 */
#define IGH_MCAN_HDR_FLAG_STREAM_TRUNCATED  (1U << 0)
#define IGH_MCAN_HDR_FLAG_ANY_CH_ERROR      (1U << 1)
#define IGH_MCAN_HDR_FLAG_RESERVED_BIT2     (1U << 2)
#define IGH_MCAN_HDR_FLAG_STATUS_CHANGED    (1U << 3)
#define IGH_MCAN_HDR_FLAG_SEQ_WRAP          (1U << 4)
#define IGH_MCAN_HDR_FLAG_OVERFLOW_VALID    (1U << 5)
#define IGH_MCAN_HDR_FLAG_OVERFLOW_OCCURRED (1U << 5)
#define IGH_MCAN_HDR_FLAG_DETAIL_PENDING    (1U << 6)

/* 2B 帧控制字：从该字段可确定后续 ID、时间戳、Data 的长度与含义。 */
#define IGH_MCAN_CTRL_CH_MASK              0x0007U
#define IGH_MCAN_CTRL_DLC_SHIFT            3U
#define IGH_MCAN_CTRL_DLC_MASK             0x0078U
#define IGH_MCAN_CTRL_IDE_MASK             0x0080U
#define IGH_MCAN_CTRL_FDF_MASK             0x0100U
#define IGH_MCAN_CTRL_BRS_MASK             0x0200U
#define IGH_MCAN_CTRL_X_MASK               0x0400U
#define IGH_MCAN_CTRL_TSF_SHIFT            11U
#define IGH_MCAN_CTRL_TSF_MASK             0x1800U
#define IGH_MCAN_CTRL_KIND_SHIFT           13U
#define IGH_MCAN_CTRL_KIND_MASK            0x6000U
#define IGH_MCAN_CTRL_D64_MASK             0x8000U

/* 帧内时间戳编码，是否允许由 SDO Encapsulation profile 决定。 */
#define IGH_MCAN_TSF_NONE                  0U
#define IGH_MCAN_TSF_DELTA16               1U
#define IGH_MCAN_TSF_DELTA32               2U
#define IGH_MCAN_TSF_FULL64                3U

#define IGH_MCAN_SCHED_FORMAT_SYNC_PHASE_V1 1U
#define IGH_MCAN_SCHED_FLAGS_V1_SUPPORTED  0U

/* 帧封装包类型。第一版主机 demo 只生成 CAN_FRAME。 */
#define IGH_MCAN_KIND_CAN_FRAME            0U
#define IGH_MCAN_KIND_RESERVED1            1U
#define IGH_MCAN_KIND_STREAM_META          2U
#define IGH_MCAN_KIND_ESCAPE               3U

/* PDO profile option flags，与 0x8010/0x8011 SI7 对应。 */
#define IGH_MCAN_OPT_RESERVED_BIT0         (1U << 0)
#define IGH_MCAN_OPT_ALIGN4                (1U << 1)
#define IGH_MCAN_OPT_NO_CRC16              (1U << 2)
#define IGH_MCAN_OPT_APP_BYPASS            (1U << 3)
#define IGH_MCAN_OPT_PAYLOAD_ONLY          (1U << 4)
#define IGH_MCAN_OPT_HEADER_V2             (1U << 5)
#define IGH_MCAN_OPT_DEFAULT               IGH_MCAN_OPT_NO_CRC16
#define IGH_MCAN_OPT_V2_NO_CRC16           (IGH_MCAN_OPT_HEADER_V2 | IGH_MCAN_OPT_NO_CRC16)

#define IGH_MCAN_PDO_FEATURE_RESERVED_BIT0      (1UL << 0)
#define IGH_MCAN_PDO_FEATURE_TSF_DELTA16        (1UL << 1)
#define IGH_MCAN_PDO_FEATURE_TSF_DELTA32        (1UL << 2)
#define IGH_MCAN_PDO_FEATURE_TSF_FULL64         (1UL << 3)
#define IGH_MCAN_PDO_FEATURE_D64                (1UL << 4)
#define IGH_MCAN_PDO_FEATURE_ALIGN4             (1UL << 5)
#define IGH_MCAN_PDO_FEATURE_STREAM_CRC32       (1UL << 6)
#define IGH_MCAN_PDO_FEATURE_FINE_SIZE_16       (1UL << 7)
#define IGH_MCAN_PDO_FEATURE_RX_TSF_SCHEDULE_TX (1UL << 8)
#define IGH_MCAN_PDO_FEATURE_NO_CRC16           (1UL << 9)
#define IGH_MCAN_PDO_FEATURE_APP_BYPASS         (1UL << 10)
#define IGH_MCAN_PDO_FEATURE_PAYLOAD_ONLY       (1UL << 11)
#define IGH_MCAN_PDO_FEATURE_FINE_BYPASS        (1UL << 12)
#define IGH_MCAN_PDO_FEATURE_HEADER_V2           (1UL << 13)

/* 主机侧 CAN/CANFD 帧抽象，协议层只关心 CAN 语义，不绑定驱动。 */
typedef struct {
    uint8_t channel;
    uint8_t dlc;
    uint8_t data_len;
    uint8_t is_extended;
    uint8_t is_canfd;
    uint8_t brs;
    uint8_t rtr;
    uint8_t esi;
    uint8_t tx_tsf;
    uint8_t schedule_format;
    uint8_t schedule_flags;
    uint16_t schedule_tag;
    uint32_t schedule_offset_ns;
    uint32_t can_id;
    uint8_t data[64];
    /* 接收元数据：按线格式原样保留，不依据版本号自动解释时间单位。
     * rx_tsf=0 时字节全零；其余有效长度为2/4/8B。修改结构后须整体重新编译。 */
    uint8_t rx_tsf;
    uint8_t rx_timestamp_bytes[8];
} igh_mcan_frame_t;

/* 解析后的 PDO 头：V1 为 24B/32B，V2 为 16B/20B（4路/8路）。 */
typedef struct {
    uint8_t version;
    uint8_t ihl_words;
    uint8_t flags;
    uint16_t seq_begin;
    uint16_t seq_end;
    uint16_t payload_len;
    uint16_t payload_crc16;
    uint16_t layout_id;
    uint8_t channel_mask;
    uint8_t frame_count; /* 本映像内帧数；不是接收累计量。 */
    /* 设备上行：预组容量事件通常为0/1，不是丢帧数；V2由容量位归一化为0/1。
     * 不能与主站下行writer的追加失败次数混用，也不能据此跳过有效正文。 */
    uint8_t overflow_sat;
    uint8_t crc_alg;
    uint8_t header_bytes;
    /* 发布前状态：RX pending包含本包选择；TX pending不含未触发的DC预装。 */
    uint16_t channel_status[IGH_MCAN_CHANNEL_COUNT];
} igh_mcan_pdo_header_t;

/* PDO 解析错误码：用于把 parse_errors 定位到具体字段。 */
typedef enum {
    IGH_MCAN_PDO_PARSE_OK = 0,
    IGH_MCAN_PDO_PARSE_BAD_ARG,
    IGH_MCAN_PDO_PARSE_BAD_HEADER,
    IGH_MCAN_PDO_PARSE_BAD_CHANNEL_MASK,
    IGH_MCAN_PDO_PARSE_SHORT_CTRL,
    IGH_MCAN_PDO_PARSE_BAD_KIND,
    IGH_MCAN_PDO_PARSE_BAD_CHANNEL,
    IGH_MCAN_PDO_PARSE_BAD_TSF,
    IGH_MCAN_PDO_PARSE_BAD_D64,
    IGH_MCAN_PDO_PARSE_BAD_CLASSIC,
    IGH_MCAN_PDO_PARSE_TRUNCATED,
    IGH_MCAN_PDO_PARSE_BAD_PADDING,
    IGH_MCAN_PDO_PARSE_CALLBACK_FAIL
} igh_mcan_pdo_parse_error_t;

/* 解析诊断快照，失败时记录最近一个异常包的位置和关键控制字段。 */
typedef struct {
    igh_mcan_pdo_parse_error_t code;
    size_t offset;
    size_t packet_end;
    uint16_t ctrl;
    uint16_t packet_sequence;
    uint16_t frame_bytes;
    uint16_t payload_crc16;
    uint16_t layout_id;
    uint8_t version;
    uint8_t ihl_words;
    uint8_t header_flags;
    uint8_t header_channel_mask;
    uint8_t crc_alg;
    uint8_t channel;
    uint8_t dlc;
    uint8_t tsf;
    uint8_t kind;
} igh_mcan_pdo_parse_diag_t;

/* PDO 写入器状态：每个 EtherCAT 从站、每个周期独立使用。 */
typedef struct {
    uint8_t *data;
    size_t size;
    uint8_t channel_mask;
    uint8_t encapsulation_profile;
    uint8_t option_flags;
    uint8_t header_bytes;
    uint16_t packet_sequence;
    uint16_t frame_bytes;
    uint16_t last_clear_bytes;
    uint8_t frame_count;
    uint8_t overflow_sat;
    uint8_t sequence_wrapped;
} igh_mcan_pdo_writer_t;

/* PDO 解析回调。返回非 0 会终止本次解析。 */
typedef int (*igh_mcan_frame_cb_t)(const igh_mcan_frame_t *frame,
                                    const igh_mcan_pdo_header_t *header,
                                    void *user);

/* -------------------------------------------------------------------------- */
/* PDO codec API                                                               */
/* -------------------------------------------------------------------------- */

/* 将真实数据长度转换为 CAN/CANFD DLC。classic CAN 会自动限制到 8B。 */
uint8_t igh_mcan_len_to_dlc(size_t len, int canfd);

/* 将 DLC 转换为真实数据窗口长度。非法 DLC 按 15 处理为 64B。 */
uint8_t igh_mcan_dlc_to_len(uint8_t dlc);

/* Resolve the fixed-header size selected by OptionFlags bit5. */
size_t igh_mcan_pdo_header_bytes_for_options(uint8_t option_flags);

/* 初始化 PDO 写入器，不清空 PDO 数据区。 */
void igh_mcan_pdo_writer_init(igh_mcan_pdo_writer_t *writer,
                               uint8_t *data,
                               size_t size,
                               uint8_t channel_mask,
                               uint8_t encapsulation_profile,
                               uint8_t option_flags);

/* 开始一个 PDO 周期：清空数据区并递增 packet sequence。 */
void igh_mcan_pdo_writer_begin(igh_mcan_pdo_writer_t *writer);

/* 追加一帧 CAN/CANFD，返回 IGH_ECAN_OK 或负数错误码。 */
int igh_mcan_pdo_writer_append(igh_mcan_pdo_writer_t *writer,
                                const igh_mcan_frame_t *frame);

/* 结束一个 PDO 周期：按所选 V1/V2 格式回填头部和可选 CRC。 */
void igh_mcan_pdo_writer_finish(igh_mcan_pdo_writer_t *writer);

/* 仅读取固定头部，不解析帧封装区。V2 不携带 frame_count，该字段返回 0；
 * 使用 parse_frames_ex 的输出或回调数量获取已验证的实际帧数。 */
int igh_mcan_pdo_read_header(const uint8_t *data,
                              size_t size,
                              igh_mcan_pdo_header_t *header);

/* 将解析错误码转换为短字符串，便于测试日志输出。 */
const char *igh_mcan_pdo_parse_error_name(igh_mcan_pdo_parse_error_t code);

/* 解析完整 PDO，并在失败时输出诊断信息。
 * 调用方须在入口复制期间独占 data；函数在内部快照完整校验通过后才调用 cb。
 * 格式/CRC/帧数错误不交付任何帧。cb 自身返回错误时，已执行的回调无法撤销。
 * 回调可以修改原始 data，不影响本次已经验证的快照。 */
int igh_mcan_pdo_parse_frames_ex(const uint8_t *data,
                                  size_t size,
                                  uint8_t expected_channel_mask,
                                  uint8_t encapsulation_profile,
                                  uint8_t option_flags,
                                  igh_mcan_frame_cb_t cb,
                                  void *user,
                                  igh_mcan_pdo_header_t *out_header,
                                  igh_mcan_pdo_parse_diag_t *diag);

/* 解析完整 PDO 帧封装区，严格校验长度、padding、通道掩码和 profile 能力。 */
int igh_mcan_pdo_parse_frames(const uint8_t *data,
                               size_t size,
                               uint8_t expected_channel_mask,
                               uint8_t encapsulation_profile,
                               uint8_t option_flags,
                               igh_mcan_frame_cb_t cb,
                               void *user,
                               igh_mcan_pdo_header_t *out_header);


#ifdef __cplusplus
}
#endif

#endif /* IGH_MCAN_PDO_CODEC_H */
