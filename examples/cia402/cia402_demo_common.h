#ifndef IGH_CIA402_DEMO_COMMON_H
#define IGH_CIA402_DEMO_COMMON_H

#include <stddef.h>
#include <stdint.h>

#include "igh_ecan_common.h"

/*
 * CiA402 示例默认把 ECAN-Lite4 的 MCAN 通道配置为 CANFD-capable。
 * CANopen NMT/SDO/RPDO 帧仍按 classic CAN 帧发送，CANFD 只表示网关通道使用
 * 1M+5M timing、64B message RAM 和 FD-capable 控制器模式。
 */
#define IGH_CIA402_DEMO_DEFAULT_CANFD      IGH_ECAN_CANFD_ENABLE

typedef struct {
    igh_ecan_enable_t enable;
    size_t slave_pos;
    igh_ecan_mcan_channel_t mcan_channel;
    uint8_t node_id;
    igh_ecan_canfd_mode_t canfd;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
} igh_cia402_demo_motor_slot_t;

#endif /* IGH_CIA402_DEMO_COMMON_H */
