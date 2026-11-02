/*
 * IgH ECAN 主站通用定义。
 *
 * 约定：
 * - 返回值统一使用 int，0 表示成功，负数表示错误。
 * - 公共默认值集中在这里，避免 demo、协议层和适配层各自硬编码。
 */

#ifndef IGH_ECAN_COMMON_H
#define IGH_ECAN_COMMON_H

#include <stddef.h>
#include <stdint.h>

/* ECAN-Lite remains the default; eight-channel host builds select 8 in CMake. */
#ifndef IGH_MCAN_CHANNEL_COUNT
#define IGH_MCAN_CHANNEL_COUNT             4U
#endif
#if (IGH_MCAN_CHANNEL_COUNT != 4U) && (IGH_MCAN_CHANNEL_COUNT != 8U)
#error "IGH_MCAN_CHANNEL_COUNT must be 4 or 8"
#endif

/* 通用开关枚举，用于 demo 拓扑和简单配置项。 */
typedef enum {
    IGH_ECAN_DISABLE = 0,
    IGH_ECAN_ENABLE = 1
} igh_ecan_enable_t;

/* MCAN 通道枚举。四通道产品使用 CH0..CH3，八通道产品使用 CH0..CH7。 */
typedef enum {
    IGH_ECAN_MCAN_CH0 = 0,
    IGH_ECAN_MCAN_CH1 = 1,
    IGH_ECAN_MCAN_CH2 = 2,
    IGH_ECAN_MCAN_CH3 = 3,
    IGH_ECAN_MCAN_CH4 = 4,
    IGH_ECAN_MCAN_CH5 = 5,
    IGH_ECAN_MCAN_CH6 = 6,
    IGH_ECAN_MCAN_CH7 = 7
} igh_ecan_mcan_channel_t;

/* 当前目标产品的全部 MCAN 通道掩码。 */
#define IGH_ECAN_MCAN_CHANNEL_MASK_ALL     ((uint8_t)((1UL << IGH_MCAN_CHANNEL_COUNT) - 1UL))

/* CANFD 模式枚举。 */
typedef enum {
    IGH_ECAN_CANFD_DISABLE = 0,
    IGH_ECAN_CANFD_ENABLE = 1
} igh_ecan_canfd_mode_t;

/* 通用返回值：函数执行成功。 */
#define IGH_ECAN_OK                       (0)

/* 通用返回值：未细分的失败。 */
#define IGH_ECAN_FAIL                     (-1)

/* 通用返回值：PDO 帧区或发送队列空间不足。 */
#define IGH_ECAN_NO_SPACE                 (-2)

/* 通用返回值：参数为空、越界或组合非法。 */
#define IGH_ECAN_INVALID_ARG              (-3)

/* 通用返回值：对象尚未初始化或运行状态不满足要求。 */
#define IGH_ECAN_NOT_READY                (-4)

/* 通用返回值：等待状态超时。 */
#define IGH_ECAN_TIMEOUT                  (-5)

/* 通用返回值：当前 profile、选项或运行环境不支持。 */
#define IGH_ECAN_UNSUPPORTED              (-6)

/* 主程序退出码：成功。 */
#define IGH_ECAN_EXIT_OK                  (0)

/* 主程序退出码：运行阶段失败。 */
#define IGH_ECAN_EXIT_FAIL                (1)

/* 主程序退出码：命令行参数错误。 */
#define IGH_ECAN_EXIT_ARG_ERROR           (2)

/* 常用 bit 生成宏，返回 uint8_t，适合通道掩码。 */
#define IGH_ECAN_BIT_U8(bit)              ((uint8_t)(1U << (bit)))

/* 静态数组元素个数。 */
#define IGH_ECAN_ARRAY_SIZE(array)        (sizeof(array) / sizeof((array)[0]))

/* CAN/CANopen 通用范围。 */
#define IGH_ECAN_CAN_STD_ID_MAX           0x7FFUL
#define IGH_ECAN_CAN_EXT_ID_MAX           0x1FFFFFFFUL
#define IGH_ECAN_CANOPEN_NODE_ID_MAX      0x7FU
#define IGH_ECAN_CLASSIC_CAN_MAX_DLEN     8U
#define IGH_ECAN_CANFD_MAX_DLEN           64U

/* 对象字典常用子项。 */
#define IGH_ECAN_OD_SUBINDEX_VALUE        0x00U
#define IGH_ECAN_OD_SUBINDEX_APPLY        0x01U
#define IGH_ECAN_OD_APPLY_COMMAND         1U

/* 默认 EtherCAT 周期：1ms。 */
#define IGH_ECAN_DEFAULT_CYCLE_US         (1000U)

/* 默认 CAN 仲裁段波特率：1Mbit/s。 */
#define IGH_ECAN_DEFAULT_CAN_BITRATE      (1000000UL)

/* 默认 CAN FD 数据段波特率：5Mbit/s。 */
#define IGH_ECAN_DEFAULT_CANFD_BITRATE    (5000000UL)

/* 默认 CAN/CAN FD 采样点约束，单位为 0.1%。 */
#define IGH_ECAN_DEFAULT_CAN20_SP_MIN_X10 (750U)
#define IGH_ECAN_DEFAULT_CAN20_SP_MAX_X10 (875U)
#define IGH_ECAN_DEFAULT_CANFD_SP_MIN_X10 (750U)
#define IGH_ECAN_DEFAULT_CANFD_SP_MAX_X10 (875U)

/* 1M 仲裁段低层位时序：80MHz MCAN clock，16TQ，采样点 87.5%。 */
#define IGH_ECAN_DEFAULT_NOM_BRP          (5U)
#define IGH_ECAN_DEFAULT_NOM_TSEG1        (13U)
#define IGH_ECAN_DEFAULT_NOM_TSEG2        (2U)
#define IGH_ECAN_DEFAULT_NOM_SJW          (1U)

/* 5M 数据段低层位时序：80MHz MCAN clock，16TQ，采样点 87.5%。 */
#define IGH_ECAN_DEFAULT_DATA_BRP         (1U)
#define IGH_ECAN_DEFAULT_DATA_TSEG1       (13U)
#define IGH_ECAN_DEFAULT_DATA_TSEG2       (2U)
#define IGH_ECAN_DEFAULT_DATA_SJW         (1U)

/* 默认启用 CAN FD TDC，offset/filter 为 0 时由固件按当前数据段时序自动计算。 */
#define IGH_ECAN_DEFAULT_CANFD_TDC_ENABLE  (1U)
#define IGH_ECAN_DEFAULT_CANFD_TDC_OFFSET  (0U)
#define IGH_ECAN_DEFAULT_CANFD_TDC_FILTER  (0U)

/* HPM MCAN data field size 枚举值。 */
#define IGH_ECAN_MCAN_DATA_SIZE_8_BYTES   (0U)
#define IGH_ECAN_MCAN_DATA_SIZE_64_BYTES  (7U)

/* HPM MCAN 全局过滤策略：未匹配帧进入 RX FIFO0，适合通用 CAN 网关默认接收。 */
#define IGH_ECAN_MCAN_ACCEPT_NON_MATCH_RXFIFO0 (0U)

/* HPM MCAN FIFO 阻塞模式。 */
#define IGH_ECAN_MCAN_FIFO_BLOCKING       (0U)

/* HPM MCAN TX FIFO/Queue 模式：0=FIFO, 1=Queue。 */
#define IGH_ECAN_MCAN_TX_FIFO_MODE        (0U)

/*
 * MCAN 默认 Message RAM 资源，和固件默认划分保持一致。
 * 数据面只使用 TX FIFO；不要把 dedicated TX buffer 与 TX FIFO 混合，
 * 否则 SDK 的连续 TXFIFO put-index 路径会产生错误 backpressure。
 */
#if IGH_MCAN_CHANNEL_COUNT == 8U
/* HPM6E 8CH: 8 controllers share the 32 KiB AHB message RAM, 4 KiB each. */
#define IGH_ECAN_MCAN_CLASSIC_RXFIFO0_DEFAULT (32U)
#define IGH_ECAN_MCAN_CLASSIC_RXFIFO1_DEFAULT (16U)
#define IGH_ECAN_MCAN_CLASSIC_RXBUF_DEFAULT   (0U)
#define IGH_ECAN_MCAN_CLASSIC_TXBUF_DEFAULT   (0U)
#define IGH_ECAN_MCAN_CLASSIC_TXFIFO_DEFAULT  (16U)
#define IGH_ECAN_MCAN_CLASSIC_TXEVT_DEFAULT   (16U)
#define IGH_ECAN_MCAN_CANFD_RXFIFO0_DEFAULT   (16U)
#define IGH_ECAN_MCAN_CANFD_RXFIFO1_DEFAULT   (4U)
#define IGH_ECAN_MCAN_CANFD_RXBUF_DEFAULT     (4U)
#define IGH_ECAN_MCAN_CANFD_TXBUF_DEFAULT     (0U)
#define IGH_ECAN_MCAN_CANFD_TXFIFO_DEFAULT    (8U)
#define IGH_ECAN_MCAN_CANFD_TXEVT_DEFAULT     (8U)
#else
#define IGH_ECAN_MCAN_CLASSIC_RXFIFO0_DEFAULT (64U)
#define IGH_ECAN_MCAN_CLASSIC_RXFIFO1_DEFAULT (64U)
#define IGH_ECAN_MCAN_CLASSIC_RXBUF_DEFAULT   (0U)
#define IGH_ECAN_MCAN_CLASSIC_TXBUF_DEFAULT   (0U)
#define IGH_ECAN_MCAN_CLASSIC_TXFIFO_DEFAULT  (32U)
#define IGH_ECAN_MCAN_CLASSIC_TXEVT_DEFAULT   (32U)
#define IGH_ECAN_MCAN_CANFD_RXFIFO0_DEFAULT   (48U)
#define IGH_ECAN_MCAN_CANFD_RXFIFO1_DEFAULT   (12U)
#define IGH_ECAN_MCAN_CANFD_RXBUF_DEFAULT     (12U)
#define IGH_ECAN_MCAN_CANFD_TXBUF_DEFAULT     (0U)
#define IGH_ECAN_MCAN_CANFD_TXFIFO_DEFAULT    (32U)
#define IGH_ECAN_MCAN_CANFD_TXEVT_DEFAULT     (32U)
#endif

/* 默认 PDO 总字节数：256B。 */
#define IGH_ECAN_DEFAULT_PDO_BYTES        (256U)

/* 默认 PDO 倍数：256 / 128 = 2。 */
#define IGH_ECAN_DEFAULT_PDO_MULTIPLIER   (2U)

/* 默认 demo 运行周期数，保持旧版 10 轴示例的长时间位置环行为。 */
#define IGH_ECAN_DEFAULT_PROFILE_CYCLES   (1200000U)

#endif /* IGH_ECAN_COMMON_H */
