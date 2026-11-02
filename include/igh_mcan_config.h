/*
 * ECAN-Lite4 MCAN channel and startup-SDO configuration.
 */

#ifndef IGH_MCAN_CONFIG_H
#define IGH_MCAN_CONFIG_H

#include "igh_adapter.h"
#include "igh_mcan_pdo_codec.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 单个 ECAN 从站的 MCAN 配置。 */
typedef struct {
    uint8_t channel_mask;
    uint8_t canfd_mask;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t nominal_sp_min_x10;
    uint16_t nominal_sp_max_x10;
    uint16_t data_sp_min_x10;
    uint16_t data_sp_max_x10;
    uint32_t channel_policy[IGH_MCAN_CHANNEL_COUNT];
    uint8_t datapath_rx_budget_override_mask;
    uint8_t datapath_tx_budget_override_mask;
    uint8_t datapath_rx_budget_max[IGH_MCAN_CHANNEL_COUNT];
    uint8_t datapath_tx_budget_max[IGH_MCAN_CHANNEL_COUNT];
} igh_mcan_slave_config_t;

/* MCAN port 配置。与 EtherCAT 寻址解耦，只描述 CAN/CANFD 能力。 */
typedef struct {
    uint8_t encapsulation_profile;
    uint8_t option_flags;
    uint8_t tx_budget_policy;
    uint8_t rx_budget_policy;
    igh_mcan_slave_config_t slaves[ECAN_IGH_MAX_SLAVES];
} igh_mcan_port_config_t;

/* 0xF000公开设备能力。F000:05=公开版本高字，F000:06=构建号。 */
#define IGH_MCAN_OD_DEVICE_OBJECTS              0xF000U
#define IGH_MCAN_OD_FW_VERSION_MAJOR_WORD       0x05U
#define IGH_MCAN_OD_FW_VERSION_BUILD_WORD       0x06U
#define IGH_MCAN_OD_PDO_FEATURE_FLAGS           0x10U
#define IGH_MCAN_FW_PROTOCOL_V2_MIN_MAJOR        2U
#define IGH_MCAN_FW_PROTOCOL_V2_MIN_MAJOR_WORD   0x0200U

typedef struct {
    uint8_t major;
    uint8_t minor;
    uint16_t build;
    uint16_t public_version_word;
    uint32_t pdo_feature_flags;
    uint8_t protocol_v2;
    uint8_t header_v2;
    uint8_t fine_size_16;
} igh_mcan_firmware_capabilities_t;

/* MCAN 通道配置对象：CAN0=0x8001，八通道产品依次到 CAN7=0x8008。 */
#define IGH_MCAN_OD_CHANNEL_BASE               0x8001U
#define IGH_MCAN_OD_CHANNEL_APPLY              IGH_ECAN_OD_SUBINDEX_APPLY
#define IGH_MCAN_OD_CHANNEL_DATA_POLICY        0x06U
#define IGH_MCAN_OD_CHANNEL_TIMING_MODE        0x07U
#define IGH_MCAN_OD_CHANNEL_NODE_MODE          0x08U
/* Source-compatible legacy name; value 1 maps to restricted/monitor mode. */
#define IGH_MCAN_OD_CHANNEL_MONITOR_ENABLE     IGH_MCAN_OD_CHANNEL_NODE_MODE
#define IGH_MCAN_NODE_MODE_NORMAL              0U
#define IGH_MCAN_NODE_MODE_RESTRICTED          1U
#define IGH_MCAN_NODE_MODE_INTERNAL_LOOPBACK   2U
#define IGH_MCAN_NODE_MODE_EXTERNAL_LOOPBACK   3U
#define IGH_MCAN_OD_CHANNEL_CANFD_ENABLE       0x0FU
#define IGH_MCAN_OD_CHANNEL_TDC_ENABLE         0x10U
#define IGH_MCAN_OD_CHANNEL_NOMINAL_BITRATE    0x13U
#define IGH_MCAN_OD_CHANNEL_DATA_BITRATE       0x14U
#define IGH_MCAN_OD_CHANNEL_NOMINAL_SP_MIN_X10 0x15U
#define IGH_MCAN_OD_CHANNEL_NOMINAL_SP_MAX_X10 0x16U
#define IGH_MCAN_OD_CHANNEL_DATA_SP_MIN_X10    0x17U
#define IGH_MCAN_OD_CHANNEL_DATA_SP_MAX_X10    0x18U
#define IGH_MCAN_OD_CHANNEL_NOM_BRP            0x19U
#define IGH_MCAN_OD_CHANNEL_NOM_TSEG1          0x1AU
#define IGH_MCAN_OD_CHANNEL_NOM_TSEG2          0x1BU
#define IGH_MCAN_OD_CHANNEL_NOM_SJW            0x1CU
#define IGH_MCAN_OD_CHANNEL_DATA_BRP           0x1DU
#define IGH_MCAN_OD_CHANNEL_DATA_TSEG1         0x1EU
#define IGH_MCAN_OD_CHANNEL_DATA_TSEG2         0x1FU
#define IGH_MCAN_OD_CHANNEL_DATA_SJW           0x20U
#define IGH_MCAN_OD_CHANNEL_TDC_OFFSET         0x21U
#define IGH_MCAN_OD_CHANNEL_TDC_FILTER         0x22U
#define IGH_MCAN_OD_CHANNEL_STD_NONMATCH       0x24U
#define IGH_MCAN_OD_CHANNEL_EXT_NONMATCH       0x25U
#define IGH_MCAN_OD_CHANNEL_REJECT_REMOTE_STD  0x26U
#define IGH_MCAN_OD_CHANNEL_REJECT_REMOTE_EXT  0x27U
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_INDEX   0x2BU
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_CMD     0x2DU
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_COUNT   0x31U
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_TYPE    0x32U
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_ACTION  0x33U
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_SYNC    0x34U
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_ID1     0x35U
#define IGH_MCAN_OD_CHANNEL_STD_FILTER_ID2     0x36U
#define IGH_MCAN_OD_CHANNEL_EXT_FILTER_COUNT   0x37U
#define IGH_MCAN_OD_CHANNEL_RXFIFO0_MODE       0x3DU
#define IGH_MCAN_OD_CHANNEL_RXFIFO0_ELEMS      0x3EU
#define IGH_MCAN_OD_CHANNEL_RXFIFO1_MODE       0x3FU
#define IGH_MCAN_OD_CHANNEL_RXFIFO1_ELEMS      0x40U
#define IGH_MCAN_OD_CHANNEL_RXFIFO0_DATA_SIZE  0x41U
#define IGH_MCAN_OD_CHANNEL_RXFIFO1_DATA_SIZE  0x42U
#define IGH_MCAN_OD_CHANNEL_RXBUF_ELEMS        0x43U
#define IGH_MCAN_OD_CHANNEL_RXBUF_DATA_SIZE    0x44U
#define IGH_MCAN_OD_CHANNEL_TX_FIFO_MODE       0x45U
#define IGH_MCAN_OD_CHANNEL_TX_FIFO_ELEMS      0x46U
#define IGH_MCAN_OD_CHANNEL_TX_DEDICATED_ELEMS 0x47U
#define IGH_MCAN_OD_CHANNEL_TX_DATA_SIZE       0x48U
#define IGH_MCAN_OD_CHANNEL_TX_EVENT_ELEMS     0x49U

/* 0x8021..0x8024: explicit per-channel data-path control. These entries take
 * precedence over the legacy 0x8001..0x8004:06 policy after Apply is written.
 */
#define IGH_MCAN_OD_DATAPATH_CHANNEL_BASE      0x8021U
#define IGH_MCAN_OD_DATAPATH_ENABLE            0x01U
#define IGH_MCAN_OD_DATAPATH_APPLY             0x02U
#define IGH_MCAN_OD_DATAPATH_RX_BUDGET_MAX     0x05U
#define IGH_MCAN_OD_DATAPATH_TX_BUDGET_MAX     0x0CU
#define IGH_MCAN_DATAPATH_BUDGET_MAX           8U

/* ECAN-Lite4 PDO profile 对象：TxPDO=0x8010，RxPDO=0x8011。 */
#define IGH_MCAN_OD_TXPDO_PROFILE              0x8010U
#define IGH_MCAN_OD_RXPDO_PROFILE              0x8011U
#define IGH_MCAN_OD_PROFILE_APPLY              IGH_ECAN_OD_SUBINDEX_APPLY
#define IGH_MCAN_OD_PROFILE_STATUS             0x02U
#define IGH_MCAN_OD_PROFILE_PDO_MULTIPLIER     0x03U
#define IGH_MCAN_OD_PROFILE_CHANNEL_MASK       0x04U
#define IGH_MCAN_OD_PROFILE_ENCAPSULATION      0x05U
#define IGH_MCAN_OD_PROFILE_BUDGET             0x06U
#define IGH_MCAN_OD_PROFILE_OPTION_FLAGS       0x07U

/* 0x8010/0x8011:06 BudgetPolicy: low bits keep the policy, high bits are
 * diagnostic bypass selectors. They intentionally live outside OptionFlags
 * because OptionFlags is already nearly full in the shipped object dictionary.
 */
#define IGH_MCAN_BUDGET_POLICY_MASK            0x03U
#define IGH_MCAN_BUDGET_POLICY_BALANCED        0x00U
#define IGH_MCAN_BUDGET_POLICY_CONSERVATIVE    0x01U
#define IGH_MCAN_BUDGET_POLICY_PERFORMANCE     0x02U
#define IGH_MCAN_BUDGET_POLICY_ADAPTIVE        0x03U
#define IGH_MCAN_RX_BYPASS_STAGE               (1U << 2)
#define IGH_MCAN_RX_BYPASS_PARSE               (1U << 3)
#define IGH_MCAN_RX_BYPASS_TX_RING             (1U << 4)
#define IGH_MCAN_RX_BYPASS_MCAN_WRITE          (1U << 5)
#define IGH_MCAN_TX_BYPASS_MCAN_READ           (1U << 2)
#define IGH_MCAN_TX_BYPASS_BUILD               (1U << 3)
#define IGH_MCAN_TX_BYPASS_COPY                (1U << 4)
#define IGH_MCAN_TX_BUDGET_ALLOWED_MASK        (IGH_MCAN_BUDGET_POLICY_MASK | \
                                                IGH_MCAN_TX_BYPASS_MCAN_READ | \
                                                IGH_MCAN_TX_BYPASS_BUILD | \
                                                IGH_MCAN_TX_BYPASS_COPY)
#define IGH_MCAN_RX_BUDGET_ALLOWED_MASK        (IGH_MCAN_BUDGET_POLICY_MASK | \
                                                IGH_MCAN_RX_BYPASS_STAGE | \
                                                IGH_MCAN_RX_BYPASS_PARSE | \
                                                IGH_MCAN_RX_BYPASS_TX_RING | \
                                                IGH_MCAN_RX_BYPASS_MCAN_WRITE)

/* 0x8001..0x8004:06 per-channel data-path policy. */
#define IGH_MCAN_CH_POLICY_RX_BUDGET_MAX_SHIFT 0U
#define IGH_MCAN_CH_POLICY_RX_BUDGET_MAX_MASK  (0x0FUL << IGH_MCAN_CH_POLICY_RX_BUDGET_MAX_SHIFT)
#define IGH_MCAN_CH_POLICY_RX_POLICY_SHIFT     4U
#define IGH_MCAN_CH_POLICY_RX_POLICY_MASK      (0x03UL << IGH_MCAN_CH_POLICY_RX_POLICY_SHIFT)
#define IGH_MCAN_CH_POLICY_RX_DEPTH_SHIFT      6U
#define IGH_MCAN_CH_POLICY_RX_DEPTH_MASK       (0x03UL << IGH_MCAN_CH_POLICY_RX_DEPTH_SHIFT)
#define IGH_MCAN_CH_POLICY_RX_DROP_SHIFT       8U
#define IGH_MCAN_CH_POLICY_RX_DROP_MASK        (0x03UL << IGH_MCAN_CH_POLICY_RX_DROP_SHIFT)
#define IGH_MCAN_CH_POLICY_CRITICAL_SHIFT      10U
#define IGH_MCAN_CH_POLICY_CRITICAL_MASK       (0x03UL << IGH_MCAN_CH_POLICY_CRITICAL_SHIFT)
#define IGH_MCAN_CH_POLICY_ALLOWED_MASK        (IGH_MCAN_CH_POLICY_RX_BUDGET_MAX_MASK | \
                                                IGH_MCAN_CH_POLICY_RX_POLICY_MASK | \
                                                IGH_MCAN_CH_POLICY_RX_DEPTH_MASK | \
                                                IGH_MCAN_CH_POLICY_RX_DROP_MASK | \
                                                IGH_MCAN_CH_POLICY_CRITICAL_MASK)
#define IGH_MCAN_CH_RX_POLICY_BALANCED         0U
#define IGH_MCAN_CH_RX_POLICY_CONSERVATIVE     1U
#define IGH_MCAN_CH_RX_POLICY_PERFORMANCE      2U
#define IGH_MCAN_CH_RX_POLICY_ADAPTIVE         3U
#define IGH_MCAN_CH_RX_DEPTH_FULL              0U
#define IGH_MCAN_CH_RX_DEPTH_8                 1U
#define IGH_MCAN_CH_RX_DEPTH_16                2U
#define IGH_MCAN_CH_RX_DEPTH_24                3U
#define IGH_MCAN_CH_RX_DROP_OLDEST             0U
#define IGH_MCAN_CH_RX_DROP_NEWEST             1U
#define IGH_MCAN_CH_RX_DROP_PRESERVE_CRITICAL  2U
#define IGH_MCAN_CH_CRITICAL_NONE              0U
#define IGH_MCAN_CH_CRITICAL_CANOPEN_SAFETY    1U
#define IGH_MCAN_CH_CRITICAL_CANOPEN_ALL       2U

/* MCAN 过滤器编辑窗口值。 */
#define IGH_MCAN_FILTER_NONMATCH_REJECT        2U
#define IGH_MCAN_FILTER_CMD_SAVE_CURRENT       2U
#define IGH_MCAN_FILTER_TYPE_CLASSIC           2U
#define IGH_MCAN_FILTER_ACTION_RXFIFO0         1U

/* 0x8001..0x8004:07 位时序模式：0=按波特率自动求解，1=使用低层 BRP/TSEG/SJW。 */
#define IGH_MCAN_BIT_TIMING_AUTO               0U
#define IGH_MCAN_BIT_TIMING_MANUAL             1U

/* -------------------------------------------------------------------------- */
/* MCAN configuration API                                                      */
/* -------------------------------------------------------------------------- */

/* 填充 MCAN 默认配置：V1 PDO + NO_CRC16，4 路 classic CAN，1Mbps。 */
void igh_mcan_port_default_config(igh_mcan_port_config_t *config);

/* 解码0xF000:05/:06/:16，并同时执行V2版本门槛和单项能力检查。 */
int igh_mcan_firmware_capabilities_decode(
    uint16_t public_version_word,
    uint16_t build_word,
    uint32_t pdo_feature_flags,
    igh_mcan_firmware_capabilities_t *capabilities);

/* 仅判断公开固件大版本是否属于V2代际。 */
int igh_mcan_firmware_supports_protocol_v2(uint16_t public_version_word);

/*
 * 显式选择 PDO 线协议。默认配置始终为 V1 + NO_CRC16；选择 V2 时必须传入
 * 已由 0xF000:05/:06/:16 解码且声明 HEADER_V2 的能力。disable_crc16=1
 * 选择高性能的 V2 + NO_CRC16，值为 0 时保留 CRC16。
 */
int igh_mcan_port_select_pdo_protocol(
    igh_mcan_port_config_t *config,
    const igh_mcan_firmware_capabilities_t *capabilities,
    uint8_t protocol_version,
    int disable_crc16);

/* 设置一个从站启用的 MCAN 通道掩码和 CAN FD 通道掩码。 */
int igh_mcan_port_set_channel_mask(igh_mcan_port_config_t *config,
                                   size_t slave_pos,
                                   uint8_t channel_mask,
                                   uint8_t canfd_mask);

/* 设置一个从站所有启用通道共用的仲裁段/数据段波特率。 */
int igh_mcan_port_set_bitrate(igh_mcan_port_config_t *config,
                              size_t slave_pos,
                              uint32_t nominal_bitrate,
                              uint32_t data_bitrate);

int igh_mcan_port_set_sample_point_range(igh_mcan_port_config_t *config,
                                         size_t slave_pos,
                                         uint16_t nominal_min_x10,
                                         uint16_t nominal_max_x10,
                                         uint16_t data_min_x10,
                                         uint16_t data_max_x10);

/* 启用一个 classic CAN 通道，并更新该从站仲裁段波特率。 */
int igh_mcan_port_set_classic_channel(igh_mcan_port_config_t *config,
                                      size_t slave_pos,
                                      uint8_t channel,
                                      uint32_t nominal_bitrate);

/* 启用一个 CAN FD 通道，并更新该从站仲裁段和数据段波特率。 */
int igh_mcan_port_set_canfd_channel(igh_mcan_port_config_t *config,
                                    size_t slave_pos,
                                    uint8_t channel,
                                    uint32_t nominal_bitrate,
                                    uint32_t data_bitrate);

/* 关闭一个 MCAN 通道；为保持 PDO 有效，最后一路不会被全部关闭。 */
int igh_mcan_port_disable_channel(igh_mcan_port_config_t *config,
                                  size_t slave_pos,
                                  uint8_t channel);

int igh_mcan_port_set_channel_policy(igh_mcan_port_config_t *config,
                                     size_t slave_pos,
                                     uint8_t channel,
                                     uint32_t policy_word);

/* Override the explicit 0x8021+channel receive-drain or transmit-write budget.
 * Values are firmware service budgets per channel and cycle, in the range
 * 0..8. The override is queued only after this setter is called; a default
 * configuration therefore keeps the firmware's own values unchanged.
 */
int igh_mcan_port_set_channel_rx_drain_budget(igh_mcan_port_config_t *config,
                                              size_t slave_pos,
                                              uint8_t channel,
                                              uint8_t budget_max);

int igh_mcan_port_set_channel_tx_write_budget(igh_mcan_port_config_t *config,
                                              size_t slave_pos,
                                              uint8_t channel,
                                              uint8_t budget_max);

/* 将默认启动所需的 PDO profile、通道模式和波特率配置追加到 IgH 启动 SDO 队列。 */
int igh_mcan_port_apply_config(ecan_igh_config_t *igh_config,
                               const igh_mcan_port_config_t *mcan_config);


#ifdef __cplusplus
}
#endif

#endif /* IGH_MCAN_CONFIG_H */
