/*
 * ECAN-Lite4 MCAN port runtime.
 *
 * Runtime responsibilities are binding an initialized IgH context, managing
 * EtherCAT cycles, sending CAN/CANFD frames, and parsing received frames.
 */

#ifndef IGH_MCAN_PORT_H
#define IGH_MCAN_PORT_H

#include "igh_mcan_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 用户接收帧：包含来源从站、CAN/CANFD 内容，以及该 PDO 周期固定头快照。 */
typedef struct {
    int slave_pos;
    igh_mcan_frame_t frame;
    igh_mcan_pdo_header_t header;
} igh_mcan_rx_frame_t;

typedef struct {
    uint32_t cycles_completed;
    uint32_t consecutive_stable_cycles;
    uint32_t begin_errors;
    uint32_t receive_errors;
    uint32_t no_space_events;
    uint32_t end_errors;
    size_t last_rx_count;
    size_t max_rx_count;
} igh_mcan_port_warmup_result_t;

/* 绑定到 IgH raw PDO 的 MCAN port 运行上下文。 */
typedef struct {
    ecan_igh_context_t *igh;
    igh_mcan_port_config_t config;
    igh_mcan_pdo_writer_t writers[ECAN_IGH_MAX_SLAVES];
    uint8_t last_rx_header_valid[ECAN_IGH_MAX_SLAVES];
    igh_mcan_pdo_header_t last_rx_header[ECAN_IGH_MAX_SLAVES];
    uint8_t cycle_active;
    uint8_t last_parse_valid;
    size_t last_parse_slave;
    igh_mcan_pdo_parse_diag_t last_parse_diag;
} igh_mcan_port_t;

/* -------------------------------------------------------------------------- */
/* MCAN port runtime API                                                       */
/* -------------------------------------------------------------------------- */

/* 绑定已初始化的 IgH 上下文。必须在 ecan_igh_init() 成功后调用。 */
int igh_mcan_port_init(igh_mcan_port_t *port,
                       ecan_igh_context_t *igh,
                       const igh_mcan_port_config_t *config);

/* 开始一个 MCAN/EtherCAT 周期，失败时返回错误码。 */
int igh_mcan_port_cycle_begin_checked(igh_mcan_port_t *port);

/* 尝试开始一个 MCAN/EtherCAT 周期；若当前已有活动周期则返回错误。 */
int igh_mcan_port_cycle_try_begin(igh_mcan_port_t *port);

/* 结束当前周期，完成 PDO 封包并发送到 EtherCAT。 */
int igh_mcan_port_cycle_end_checked(igh_mcan_port_t *port);

/* 放弃当前周期，清理本周期写入状态。 */
int igh_mcan_port_cycle_abort_checked(igh_mcan_port_t *port);

/* 兼容旧 demo 的无返回值周期开始接口。 */
void igh_mcan_port_cycle_begin(igh_mcan_port_t *port);

/* 兼容旧 demo 的无返回值周期结束接口。 */
void igh_mcan_port_cycle_end(igh_mcan_port_t *port);

/* 兼容旧 demo 的无返回值周期放弃接口。 */
void igh_mcan_port_cycle_abort(igh_mcan_port_t *port);

/* 在当前周期追加一帧 classic CAN。 */
int igh_mcan_port_send_can(igh_mcan_port_t *port,
                           int slave_pos,
                           uint8_t channel,
                           uint32_t can_id,
                           int is_extended,
                           const uint8_t *data,
                           size_t data_len);

/* 在当前周期追加一帧 CAN FD。 */
int igh_mcan_port_send_canfd(igh_mcan_port_t *port,
                             int slave_pos,
                             uint8_t channel,
                             uint32_t can_id,
                             int is_extended,
                             int brs,
                             const uint8_t *data,
                             size_t data_len);

/* 在当前周期追加一帧带同步相位偏移的 classic CAN。 */
int igh_mcan_port_schedule_can(igh_mcan_port_t *port,
                               int slave_pos,
                               uint8_t channel,
                               uint32_t can_id,
                               int is_extended,
                               const uint8_t *data,
                               size_t data_len,
                               uint32_t offset_ns,
                               uint16_t tag);

/* 在当前周期追加一帧带同步相位偏移的 CAN FD。 */
int igh_mcan_port_schedule_canfd(igh_mcan_port_t *port,
                                 int slave_pos,
                                 uint8_t channel,
                                 uint32_t can_id,
                                 int is_extended,
                                 int brs,
                                 const uint8_t *data,
                                 size_t data_len,
                                 uint32_t offset_ns,
                                 uint16_t tag);

/* 解析所有从站 TxPDO 中的 CAN/CANFD 帧，返回本次收到的帧数。 */
int igh_mcan_port_receive(igh_mcan_port_t *port,
                          igh_mcan_rx_frame_t *rx_frames,
                          size_t rx_capacity,
                          size_t *rx_count);

/* 空跑一个周期并读取 TxPDO。用于 OP 后 warm-up，避免正式运动/压力数据从不稳定周期开始。 */
int igh_mcan_port_warmup_cycle_checked(igh_mcan_port_t *port,
                                       igh_mcan_rx_frame_t *rx_frames,
                                       size_t rx_capacity,
                                       igh_mcan_port_warmup_result_t *result);

/* 获取最近一次解析到的指定从站 TxPDO 固定头。 */
int igh_mcan_port_get_tpdo_header(igh_mcan_port_t *port,
                                  size_t slave_pos,
                                  igh_mcan_pdo_header_t *out_header);

/* 获取最近一次 PDO 解析错误诊断，返回 NULL 表示暂无错误。 */
const igh_mcan_pdo_parse_diag_t *igh_mcan_port_get_last_parse_error(
    const igh_mcan_port_t *port,
    size_t *slave_index);

#ifdef __cplusplus
}
#endif

#endif /* IGH_MCAN_PORT_H */
