#ifndef IGH_CIA402_PROTOCOL_H
#define IGH_CIA402_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "igh_ecan_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/* CiA402 demo 默认使用 CSP 模式，也可切换到 CSV 速度模式。 */
#define IGH_CIA402_MODE_CSP                    8U
#define IGH_CIA402_MODE_CSV                    9U

/* CANopen NMT 命令。 */
#define IGH_CIA402_NMT_START_REMOTE_NODE       0x01U
#define IGH_CIA402_NMT_ENTER_PREOPERATIONAL    0x80U
#define IGH_CIA402_NMT_RESET_NODE              0x81U

/* CANopen COB-ID 基址。 */
#define IGH_CIA402_COB_DISABLED_MASK           0x80000000UL
#define IGH_CIA402_COB_NMT                     0x000U
#define IGH_CIA402_COB_SYNC                    0x080U
#define IGH_CIA402_COB_EMCY_BASE               0x080U
#define IGH_CIA402_COB_TPDO1_BASE              0x180U
#define IGH_CIA402_COB_RPDO1_BASE              0x200U
#define IGH_CIA402_COB_SDO_RESPONSE_BASE       0x580U
#define IGH_CIA402_COB_SDO_REQUEST_BASE        0x600U
#define IGH_CIA402_COB_HEARTBEAT_BASE          0x700U

/* CiA402/CANopen 常用对象索引。 */
#define IGH_CIA402_IDX_ERROR_REGISTER          0x1001U
#define IGH_CIA402_IDX_SYNC_COB_ID             0x1005U
#define IGH_CIA402_IDX_SYNC_PERIOD_US          0x1006U
#define IGH_CIA402_IDX_HEARTBEAT_PRODUCER      0x1017U
#define IGH_CIA402_IDX_SYNC_COUNTER_OVERFLOW   0x1019U
/* PH 系列电机通信模式：0=EtherCAT，1=CANopen。 */
#define IGH_CIA402_IDX_PH_COMM_MODE            0x2004U
#define IGH_CIA402_IDX_RPDO1_COMM              0x1400U
#define IGH_CIA402_IDX_RPDO1_MAPPING           0x1600U
#define IGH_CIA402_IDX_TPDO1_COMM              0x1800U
#define IGH_CIA402_IDX_TPDO1_MAPPING           0x1A00U
#define IGH_CIA402_IDX_CONTROLWORD             0x6040U
#define IGH_CIA402_IDX_STATUSWORD              0x6041U
#define IGH_CIA402_IDX_MODES_OF_OPERATION      0x6060U
#define IGH_CIA402_IDX_MODES_OF_OPERATION_DISPLAY 0x6061U
#define IGH_CIA402_IDX_POSITION_ACTUAL_VALUE   0x6064U
#define IGH_CIA402_IDX_VELOCITY_ACTUAL_VALUE   0x606CU
#define IGH_CIA402_IDX_TARGET_POSITION         0x607AU
#define IGH_CIA402_IDX_TARGET_VELOCITY         0x60FFU
#define IGH_CIA402_IDX_INTERPOLATION_TIME      0x60C2U

/* CANopen 通用子项与 PDO 通信/映射子项。 */
#define IGH_CIA402_SUBIDX_VALUE                IGH_ECAN_OD_SUBINDEX_VALUE
#define IGH_CIA402_SUBIDX_COB_ID               0x01U
#define IGH_CIA402_SUBIDX_TRANSMISSION_TYPE    0x02U
#define IGH_CIA402_SUBIDX_PDO_MAP_COUNT        IGH_ECAN_OD_SUBINDEX_VALUE
#define IGH_CIA402_SUBIDX_PDO_MAP_ENTRY1       0x01U
#define IGH_CIA402_SUBIDX_PDO_MAP_ENTRY2       0x02U
#define IGH_CIA402_SUBIDX_PDO_MAP_ENTRY3       0x03U

/* PDO 传输类型：1 表示每个 SYNC 周期更新一次，匹配当前电机 CSP 启动表。 */
#define IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY 0x01U
/* 兼容旧调用命名，语义已经改为每个 SYNC 周期一次。 */
#define IGH_CIA402_RPDO_TRANSMISSION_SYNC      IGH_CIA402_PDO_TRANSMISSION_SYNC_EVERY
/* 0xFF 表示异步事件驱动，保留给用户自定义场景。 */
#define IGH_CIA402_RPDO_TRANSMISSION_ASYNC     0xFFU

/* 电机 CSP 启动表使用的同步参数与 PDO mapping。 */
#define IGH_CIA402_CSP_HEARTBEAT_MS            1000U
#define IGH_CIA402_CSP_SYNC_COB_ID             IGH_CIA402_COB_SYNC
#define IGH_CIA402_CSP_SYNC_PERIOD_US_DEFAULT  1000UL
#define IGH_CIA402_RPDO1_TARGET_ONLY_PAYLOAD_LEN 4U
#define IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN       7U
#define IGH_CIA402_RPDO1_CSV_PAYLOAD_LEN       7U

/* PDO mapping entry 低 8bit 为长度，bit31..16 为 index，bit15..8 为 subindex。 */
#define IGH_CIA402_PDO_MAP_ENTRY(index, sub, bits) \
    ((((uint32_t)(index)) << 16) | (((uint32_t)(sub)) << 8) | ((uint32_t)(bits)))
#define IGH_CIA402_PDO_MAP_COUNT_CSP           3U
#define IGH_CIA402_PDO_MAP_CONTROLWORD \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_CONTROLWORD, IGH_CIA402_SUBIDX_VALUE, 0x10U)
#define IGH_CIA402_PDO_MAP_MODE_OF_OPERATION \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_MODES_OF_OPERATION, IGH_CIA402_SUBIDX_VALUE, 0x08U)
#define IGH_CIA402_PDO_MAP_TARGET_POSITION \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_TARGET_POSITION, IGH_CIA402_SUBIDX_VALUE, 0x20U)
#define IGH_CIA402_PDO_MAP_TARGET_VELOCITY \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_TARGET_VELOCITY, IGH_CIA402_SUBIDX_VALUE, 0x20U)
#define IGH_CIA402_PDO_MAP_POSITION_ACTUAL \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_POSITION_ACTUAL_VALUE, IGH_CIA402_SUBIDX_VALUE, 0x20U)
#define IGH_CIA402_PDO_MAP_VELOCITY_ACTUAL \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_VELOCITY_ACTUAL_VALUE, IGH_CIA402_SUBIDX_VALUE, 0x20U)
#define IGH_CIA402_PDO_MAP_STATUSWORD \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_STATUSWORD, IGH_CIA402_SUBIDX_VALUE, 0x10U)
#define IGH_CIA402_PDO_MAP_MODE_DISPLAY \
    IGH_CIA402_PDO_MAP_ENTRY(IGH_CIA402_IDX_MODES_OF_OPERATION_DISPLAY, IGH_CIA402_SUBIDX_VALUE, 0x08U)

/* CiA402 控制字，按 Shutdown -> Switch on -> Enable operation 流程使用。 */
#define IGH_CIA402_CONTROLWORD_SHUTDOWN        0x06U
#define IGH_CIA402_CONTROLWORD_SWITCH_ON       0x07U
#define IGH_CIA402_CONTROLWORD_ENABLE_OP       0x0FU
#define IGH_CIA402_CONTROLWORD_DISABLE         0x00U
#define IGH_CIA402_CONTROLWORD_FAULT_RESET     0x80U

/* CiA402 statusword 状态位掩码。0x0027 表示 Operation enabled。 */
#define IGH_CIA402_STATUSWORD_STATE_MASK       0x006FU
#define IGH_CIA402_STATUSWORD_OPERATION_ENABLED 0x0027U
#define IGH_CIA402_STATUSWORD_NOT_READY        0x0000U
#define IGH_CIA402_STATUSWORD_SWITCH_DISABLED  0x0040U
#define IGH_CIA402_STATUSWORD_READY            0x0021U
#define IGH_CIA402_STATUSWORD_SWITCHED_ON      0x0023U
#define IGH_CIA402_STATUSWORD_QUICK_STOP       0x0007U
#define IGH_CIA402_STATUSWORD_FAULT_REACTION   0x000FU
#define IGH_CIA402_STATUSWORD_FAULT            0x0008U

/* CANopen SDO expedited 命令字。 */
#define IGH_CIA402_SDO_UPLOAD                  0x40U
#define IGH_CIA402_SDO_UPLOAD_RESP_U8          0x4FU
#define IGH_CIA402_SDO_UPLOAD_RESP_U16         0x4BU
#define IGH_CIA402_SDO_UPLOAD_RESP_U24         0x47U
#define IGH_CIA402_SDO_UPLOAD_RESP_U32         0x43U
#define IGH_CIA402_SDO_WRITE_U8                0x2FU
#define IGH_CIA402_SDO_WRITE_U16               0x2BU
#define IGH_CIA402_SDO_WRITE_U32               0x23U
#define IGH_CIA402_SDO_DOWNLOAD_ACK            0x60U
#define IGH_CIA402_SDO_ABORT                   0x80U

/* 心跳 0x05 表示节点处于 Operational。 */
#define IGH_CIA402_HEARTBEAT_BOOTUP            0x00U
#define IGH_CIA402_HEARTBEAT_STOPPED           0x04U
#define IGH_CIA402_HEARTBEAT_OPERATIONAL       0x05U
#define IGH_CIA402_HEARTBEAT_PREOPERATIONAL    0x7FU

/* PH 系列电机 0x2004:00 通信模式。 */
#define IGH_CIA402_PH_COMM_MODE_ETHERCAT       0U
#define IGH_CIA402_PH_COMM_MODE_CANOPEN        1U

/* CANopen SDO abort codes used by diagnostics. */
#define IGH_CIA402_SDO_ABORT_NONE              0x00000000UL
#define IGH_CIA402_SDO_ABORT_TOGGLE            0x05030000UL
#define IGH_CIA402_SDO_ABORT_TIMEOUT           0x05040000UL
#define IGH_CIA402_SDO_ABORT_CMD_SPECIFIER     0x05040001UL
#define IGH_CIA402_SDO_ABORT_OUT_OF_MEMORY     0x05040005UL
#define IGH_CIA402_SDO_ABORT_UNSUPPORTED       0x06010000UL
#define IGH_CIA402_SDO_ABORT_WRITE_ONLY        0x06010001UL
#define IGH_CIA402_SDO_ABORT_READ_ONLY         0x06010002UL
#define IGH_CIA402_SDO_ABORT_OBJECT_MISSING    0x06020000UL
#define IGH_CIA402_SDO_ABORT_PDO_MAP_OBJECT    0x06040041UL
#define IGH_CIA402_SDO_ABORT_PDO_LENGTH        0x06040042UL
#define IGH_CIA402_SDO_ABORT_PARAM_INCOMPAT    0x06040043UL
#define IGH_CIA402_SDO_ABORT_HARDWARE          0x06060000UL
#define IGH_CIA402_SDO_ABORT_TYPE_MISMATCH     0x06070010UL
#define IGH_CIA402_SDO_ABORT_TYPE_TOO_HIGH     0x06070012UL
#define IGH_CIA402_SDO_ABORT_TYPE_TOO_LOW      0x06070013UL
#define IGH_CIA402_SDO_ABORT_SUBINDEX_MISSING  0x06090011UL
#define IGH_CIA402_SDO_ABORT_RANGE_EXCEEDED    0x06090030UL
#define IGH_CIA402_SDO_ABORT_VALUE_TOO_HIGH    0x06090031UL
#define IGH_CIA402_SDO_ABORT_VALUE_TOO_LOW     0x06090032UL
#define IGH_CIA402_SDO_ABORT_GENERAL           0x08000000UL
#define IGH_CIA402_SDO_ABORT_STORE_TRANSFER    0x08000020UL
#define IGH_CIA402_SDO_ABORT_LOCAL_CONTROL     0x08000021UL
#define IGH_CIA402_SDO_ABORT_DEVICE_STATE      0x08000022UL

/* Masks for expedited SDO response payload sizes. */
#define IGH_CIA402_SDO_VALUE_MASK_U8           0x000000FFUL
#define IGH_CIA402_SDO_VALUE_MASK_U16          0x0000FFFFUL
#define IGH_CIA402_SDO_VALUE_MASK_U24          0x00FFFFFFUL
#define IGH_CIA402_SDO_VALUE_MASK_U32          0xFFFFFFFFUL

typedef struct {
    int enabled;
    int slave_pos;
    uint8_t channel;
    uint8_t node_id;
    uint8_t heartbeat;
    uint8_t heartbeat_valid;
    uint8_t actual_position_valid;
    uint8_t actual_velocity_valid;
    uint8_t statusword_valid;
    uint8_t sdo_response_valid;
    uint8_t sdo_response_command;
    uint16_t sdo_response_index;
    uint8_t sdo_response_subindex;
    uint8_t sdo_response_size;
    uint8_t emcy_valid;
    uint8_t emcy_error_register;
    uint8_t mode_display_valid;
    int8_t mode_display;
    uint8_t rpdo1_payload_len;
    uint8_t emcy_data[5];
    uint32_t sdo_abort_code;
    uint32_t sdo_response_value;
    int sdo_last_write_rc;
    int sdo_last_read_rc;
    uint32_t emcy_count;
    int32_t step;
    int32_t target_position;
    int32_t actual_position;
    int32_t target_velocity;
    int32_t actual_velocity;
    uint16_t emcy_code;
    uint16_t statusword;
} igh_cia402_axis_t;

/* CiA402 只使用 classic CAN 帧；底层可以是 MCAN、SocketCAN、UCAN 或其它 CAN transport。 */
typedef struct {
    uint8_t channel;
    uint8_t data_len;
    uint8_t is_extended;
    uint32_t can_id;
    uint8_t data[8];
} igh_cia402_can_frame_t;

typedef struct {
    int slave_pos;
    igh_cia402_can_frame_t frame;
} igh_cia402_rx_frame_t;

typedef struct {
    int slave_pos;
    igh_cia402_can_frame_t frame;
    uint8_t scheduled;
    uint32_t offset_ns;
    uint16_t tag;
} igh_cia402_tx_frame_t;

typedef int (*igh_cia402_send_can_fn)(void *user,
                                      int slave_pos,
                                      uint8_t channel,
                                      uint32_t can_id,
                                      const uint8_t *data,
                                      size_t data_len,
                                      int scheduled,
                                      uint32_t offset_ns,
                                      uint16_t tag);
typedef int (*igh_cia402_receive_fn)(void *user,
                                     igh_cia402_rx_frame_t *frames,
                                     size_t capacity,
                                     size_t *count);
typedef void (*igh_cia402_cycle_fn)(void *user);
typedef void (*igh_cia402_sleep_us_fn)(void *user, uint32_t sleep_us);

typedef struct {
    void *user;
    igh_cia402_send_can_fn send_can;
    igh_cia402_receive_fn receive;
    igh_cia402_cycle_fn cycle_begin;
    igh_cia402_cycle_fn cycle_end;
    igh_cia402_cycle_fn cycle_abort;
    igh_cia402_sleep_us_fn sleep_us;
    uint32_t cycle_us;
} igh_cia402_transport_t;

/* CiA402 协议上下文：只绑定 CAN transport 和轴表，不依赖具体主站或网关。 */
typedef struct {
    igh_cia402_transport_t transport;
    igh_cia402_axis_t *axes;
    size_t axis_count;
    uint8_t scheduled_target_enabled;
    uint8_t scheduled_sync_enabled;
    uint32_t scheduled_target_offset_ns;
    uint32_t scheduled_sync_offset_ns;
    uint16_t scheduled_tag_next;
} igh_cia402_context_t;

/* CSP 目标位置双参数：数值结果和最终 CAN payload 同时保存，供需要诊断回看 payload 的 demo 使用。 */
typedef struct {
    int32_t previous_position;
    int32_t step;
    int32_t next_position;
    uint8_t data_len;
    uint8_t data[IGH_CIA402_RPDO1_CSP_PAYLOAD_LEN];
} igh_cia402_target_step_t;

/* 初始化 CiA402 协议上下文并绑定外部 CAN transport。 */
void igh_cia402_context_init(igh_cia402_context_t *ctx,
                             const igh_cia402_transport_t *transport,
                             igh_cia402_axis_t *axes,
                             size_t axis_count);

/* 设置目标位置帧和 SYNC 帧是否使用同步相位调度发送。 */
int igh_cia402_set_scheduled_tx(igh_cia402_context_t *ctx,
                                int target_enabled,
                                uint32_t target_offset_ns,
                                int sync_enabled,
                                uint32_t sync_offset_ns);

/* 初始化一个轴的从站、通道、节点号和目标位置步进。 */
void igh_cia402_axis_init(igh_cia402_axis_t *axis,
                          int slave_pos,
                          uint8_t channel,
                          uint8_t node_id,
                          int32_t target_step);

/* 按从站、通道和节点号查找轴配置。 */
igh_cia402_axis_t *igh_cia402_find_axis(igh_cia402_context_t *ctx,
                                        int slave_pos,
                                        uint8_t channel,
                                        uint8_t node_id);

/* 向单个轴发送 NMT 命令。 */
int igh_cia402_send_nmt(igh_cia402_context_t *ctx,
                        const igh_cia402_axis_t *axis,
                        uint8_t command);

/* 向一个从站通道广播 NMT 命令。 */
int igh_cia402_send_nmt_broadcast(igh_cia402_context_t *ctx,
                                  int slave_pos,
                                  uint8_t channel,
                                  uint8_t command);

/* 向上下文中的全部轴发送 NMT 命令。 */
int igh_cia402_send_nmt_all(igh_cia402_context_t *ctx, uint8_t command);

/* 向指定从站通道发送 CANopen SYNC 帧。 */
int igh_cia402_send_sync(igh_cia402_context_t *ctx, int slave_pos, uint8_t channel);

/* 扁平集成入口：只构造 CANopen/CiA402 帧，不触碰底层 CAN transport。 */
int igh_cia402_build_nmt_frame(const igh_cia402_axis_t *axis,
                               uint8_t command,
                               igh_cia402_tx_frame_t *out);
int igh_cia402_build_nmt_broadcast_frame(int slave_pos,
                                         uint8_t channel,
                                         uint8_t command,
                                         igh_cia402_tx_frame_t *out);
int igh_cia402_build_sync_frame(igh_cia402_context_t *ctx,
                                int slave_pos,
                                uint8_t channel,
                                igh_cia402_tx_frame_t *out);
int igh_cia402_build_sdo_upload_frame(const igh_cia402_axis_t *axis,
                                      uint16_t index,
                                      uint8_t subindex,
                                      igh_cia402_tx_frame_t *out);
int igh_cia402_build_sdo_write_frame(const igh_cia402_axis_t *axis,
                                     uint8_t command,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint32_t value,
                                     igh_cia402_tx_frame_t *out);

/* 发送一帧 SDO upload 请求，不等待响应。 */
int igh_cia402_sdo_upload(igh_cia402_context_t *ctx,
                          const igh_cia402_axis_t *axis,
                          uint16_t index,
                          uint8_t subindex);

/* 发送一帧 8bit expedited SDO download，不等待响应。 */
int igh_cia402_sdo_write_u8(igh_cia402_context_t *ctx,
                            const igh_cia402_axis_t *axis,
                            uint16_t index,
                            uint8_t subindex,
                            uint8_t value);

/* 发送一帧 16bit expedited SDO download，不等待响应。 */
int igh_cia402_sdo_write_u16(igh_cia402_context_t *ctx,
                             const igh_cia402_axis_t *axis,
                             uint16_t index,
                             uint8_t subindex,
                             uint16_t value);

/* 发送一帧 32bit expedited SDO download，不等待响应。 */
int igh_cia402_sdo_write_u32(igh_cia402_context_t *ctx,
                             const igh_cia402_axis_t *axis,
                             uint16_t index,
                             uint8_t subindex,
                             uint32_t value);

/* 写 8bit SDO 并在指定超时内等待 download ack。 */
int igh_cia402_sdo_write_u8_timeout(igh_cia402_context_t *ctx,
                                    igh_cia402_axis_t *axis,
                                    uint16_t index,
                                    uint8_t subindex,
                                    uint8_t value,
                                    uint32_t timeout_ms);

/* 写 16bit SDO 并在指定超时内等待 download ack。 */
int igh_cia402_sdo_write_u16_timeout(igh_cia402_context_t *ctx,
                                     igh_cia402_axis_t *axis,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint16_t value,
                                     uint32_t timeout_ms);

/* 写 32bit SDO 并在指定超时内等待 download ack。 */
int igh_cia402_sdo_write_u32_timeout(igh_cia402_context_t *ctx,
                                     igh_cia402_axis_t *axis,
                                     uint16_t index,
                                     uint8_t subindex,
                                     uint32_t value,
                                     uint32_t timeout_ms);

/* 读取 32bit 以内的 expedited SDO 响应，并返回实际长度。 */
int igh_cia402_sdo_upload_u32_timeout(igh_cia402_context_t *ctx,
                                      igh_cia402_axis_t *axis,
                                      uint16_t index,
                                      uint8_t subindex,
                                      uint32_t *value,
                                      uint8_t *size,
                                      uint32_t timeout_ms);

/* 写 8bit SDO 后回读校验，避免设备静默忽略配置。 */
int igh_cia402_sdo_write_verify_u8_timeout(igh_cia402_context_t *ctx,
                                           igh_cia402_axis_t *axis,
                                           uint16_t index,
                                           uint8_t subindex,
                                           uint8_t value,
                                           uint32_t timeout_ms);

/* 写 16bit SDO 后回读校验，避免设备静默忽略配置。 */
int igh_cia402_sdo_write_verify_u16_timeout(igh_cia402_context_t *ctx,
                                            igh_cia402_axis_t *axis,
                                            uint16_t index,
                                            uint8_t subindex,
                                            uint16_t value,
                                            uint32_t timeout_ms);

/* 写 32bit SDO 后回读校验，避免设备静默忽略配置。 */
int igh_cia402_sdo_write_verify_u32_timeout(igh_cia402_context_t *ctx,
                                            igh_cia402_axis_t *axis,
                                            uint16_t index,
                                            uint8_t subindex,
                                            uint32_t value,
                                            uint32_t timeout_ms);

/* 使用默认超时配置单个轴 CSP 参数和 PDO。 */
int igh_cia402_configure_csp(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis);

/* 使用默认超时配置全部轴 CSP 参数和 PDO。 */
int igh_cia402_configure_csp_all(igh_cia402_context_t *ctx);

/* 配置单个轴完整 CSP 启动参数。 */
int igh_cia402_configure_csp_timeout(igh_cia402_context_t *ctx,
                                     igh_cia402_axis_t *axis,
                                     uint32_t timeout_ms);

/* 配置单个轴 TPDO/RPDO 通信参数和映射表。 */
int igh_cia402_configure_csp_pdo_timeout(igh_cia402_context_t *ctx,
                                         igh_cia402_axis_t *axis,
                                         uint32_t timeout_ms);

/* 按现场流程配置单个轴 CSP 同步参数和 TPDO/RPDO 映射。 */
int igh_cia402_configure_csp_rpdo_timeout(igh_cia402_context_t *ctx,
                                          igh_cia402_axis_t *axis,
                                          uint32_t timeout_ms);

/* 按现场流程配置单个轴 CSV 速度模式和 TPDO/RPDO 映射。 */
int igh_cia402_configure_csv_rpdo_timeout(igh_cia402_context_t *ctx,
                                          igh_cia402_axis_t *axis,
                                          uint32_t timeout_ms);

/* 配置上下文中的全部轴完整 CSP 启动参数。 */
int igh_cia402_configure_csp_all_timeout(igh_cia402_context_t *ctx,
                                         uint32_t timeout_ms);

/* 使用默认超时使能单个轴。 */
int igh_cia402_enable(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis);

/* 使用默认超时使能全部轴。 */
int igh_cia402_enable_all(igh_cia402_context_t *ctx);

/* 按 CiA402 状态机流程使能单个轴到 Operation enabled。 */
int igh_cia402_enable_timeout(igh_cia402_context_t *ctx,
                              igh_cia402_axis_t *axis,
                              uint32_t timeout_ms);

/* 发送 Disable controlword 停止单个轴。 */
int igh_cia402_stop(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis);

/* 对全部轴发送 Disable controlword。 */
int igh_cia402_stop_all(igh_cia402_context_t *ctx);

/* 请求单个轴实际位置，不等待响应。 */
int igh_cia402_request_actual_position(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis);

/* 请求全部轴实际位置，不等待响应。 */
int igh_cia402_request_actual_position_all(igh_cia402_context_t *ctx);

/* 请求并等待单个轴 statusword 响应。 */
int igh_cia402_request_statusword_timeout(igh_cia402_context_t *ctx,
                                          igh_cia402_axis_t *axis,
                                          uint32_t timeout_ms);

/* 下列 timeout API 会主动执行 transport 周期，需在应用周期 begin/end 外调用。timeout_ms 为 0 表示一直等待。 */
int igh_cia402_wait_heartbeat_timeout(igh_cia402_context_t *ctx,
                                      igh_cia402_axis_t *axis,
                                      uint8_t heartbeat,
                                      uint32_t timeout_ms);
int igh_cia402_wait_heartbeat_all_timeout(igh_cia402_context_t *ctx,
                                          uint8_t heartbeat,
                                          uint32_t timeout_ms);
int igh_cia402_request_actual_position_timeout(igh_cia402_context_t *ctx,
                                               igh_cia402_axis_t *axis,
                                               uint32_t timeout_ms);
int igh_cia402_request_actual_position_all_timeout(igh_cia402_context_t *ctx,
                                                   uint32_t timeout_ms);

/* 更新轴的目标位置缓存。 */
int igh_cia402_set_target_position(igh_cia402_axis_t *axis, int32_t target_position);

/* 根据当前实际位置和 step 计算下一次目标位置 payload。 */
int igh_cia402_prepare_target_step(const igh_cia402_axis_t *axis,
                                   igh_cia402_target_step_t *target_step);

/* 根据轴当前 target_position 构造 RPDO1 目标位置帧。 */
int igh_cia402_build_target_frame(igh_cia402_context_t *ctx,
                                  const igh_cia402_axis_t *axis,
                                  igh_cia402_tx_frame_t *out);

/* 根据目标速度构造 CSV RPDO1 帧：6040 + 6060 + 60FF。 */
int igh_cia402_build_velocity_frame(igh_cia402_context_t *ctx,
                                    const igh_cia402_axis_t *axis,
                                    int32_t target_velocity,
                                    igh_cia402_tx_frame_t *out);

/* 根据预计算 target_step 构造 RPDO1 目标位置帧。 */
int igh_cia402_build_prepared_target_frame(igh_cia402_context_t *ctx,
                                           const igh_cia402_axis_t *axis,
                                           const igh_cia402_target_step_t *target_step,
                                           igh_cia402_tx_frame_t *out);

/* 发送预计算的目标位置帧，并更新轴的 target_position。 */
int igh_cia402_send_prepared_target(igh_cia402_context_t *ctx,
                                    igh_cia402_axis_t *axis,
                                    const igh_cia402_target_step_t *target_step);

/* 发送轴当前 target_position。 */
int igh_cia402_send_target(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis);

/* 按 step 计算并发送下一次目标位置。 */
int igh_cia402_send_target_step(igh_cia402_context_t *ctx, igh_cia402_axis_t *axis);

/* 对全部轴发送当前目标位置。 */
int igh_cia402_send_targets_all(igh_cia402_context_t *ctx);

/* 收集当前 transport 周期收到的 CAN 帧并更新 CiA402 轴状态。 */
int igh_cia402_process_rx(igh_cia402_context_t *ctx,
                          igh_cia402_rx_frame_t *frames,
                          size_t capacity,
                          size_t *frame_count);

/* 把单帧 CANopen/CiA402 响应喂给协议层，更新对应轴状态。 */
int igh_cia402_on_can_frame(igh_cia402_context_t *ctx,
                             int slave_pos,
                             const igh_cia402_can_frame_t *frame);

/* 判断 statusword 是否处于 Operation enabled。 */
int igh_cia402_statusword_is_operation_enabled(uint16_t statusword);

/* 把 statusword 转成人可读状态名。 */
const char *igh_cia402_statusword_state_name(uint16_t statusword);

/* 把 heartbeat 状态值转成人可读状态名。 */
const char *igh_cia402_heartbeat_state_name(uint8_t heartbeat);

/* 把 SDO abort code 转成人可读短文本。 */
const char *igh_cia402_sdo_abort_name(uint32_t abort_code);

#ifdef __cplusplus
}
#endif

#endif /* IGH_CIA402_PROTOCOL_H */
