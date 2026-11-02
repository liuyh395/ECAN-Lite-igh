/* SPDX-License-Identifier: MIT */
#ifndef DM_ROBOT_H
#define DM_ROBOT_H

#include "dm_motor_protocol.h"
#include "igh_mcan_port.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DM_ROBOT_MOTOR_COUNT 12U
#define DM_ROBOT_MOTORS_PER_CAN 3U
#define DM_ROBOT_RX_CAPACITY 64U

typedef struct {
    uint8_t channel;
    uint8_t motor_id;
    uint32_t command_id;
    uint32_t feedback_id;
    uint8_t extended_id;
    dm_motor_limits_t limits;
} dm_robot_motor_config_t;

typedef struct {
    dm_robot_motor_config_t config;
    dm_motor_command_t command;
    dm_motor_feedback_t feedback;
    uint64_t commands_sent;
    uint64_t feedback_received;
    uint8_t has_feedback;
} dm_robot_motor_t;

typedef struct {
    size_t slave_pos;
    dm_robot_motor_t motors[DM_ROBOT_MOTOR_COUNT];
    uint64_t cycles_completed;
} dm_robot_t;

/* 默认映射：CAN0..CAN3，每个通道三台电机，ID 1..3。
 * 反馈CAN ID仅为示例，必须替换为驱动器对应的实际值。 */
void dm_robot_default_config(dm_robot_motor_config_t configs[DM_ROBOT_MOTOR_COUNT]);
int dm_robot_init(dm_robot_t *robot, size_t slave_pos,
                  const dm_robot_motor_config_t configs[DM_ROBOT_MOTOR_COUNT]);

/* 将全部四个通道配置为经典CAN模式（CAN FD掩码始终为0）。 */
int dm_robot_apply_classic_can_config(ecan_igh_config_t *igh_config,
                                      igh_mcan_port_config_t *mcan_config,
                                      size_t slave_pos, uint32_t bitrate);

/* 命令索引采用通道 * 3 + 电机槽位；默认电机槽位对应编号1..3。 */
int dm_robot_submit(dm_robot_t *robot, size_t motor_index,
                    const dm_motor_command_t *command);
/* 在更新命令集之前验证全部十二个值。 */
int dm_robot_submit_all(dm_robot_t *robot,
                        const dm_motor_command_t commands[DM_ROBOT_MOTOR_COUNT]);

/* 一次定时EtherCAT周期：开始，接收反馈，发送12条指令，结束。
 * 调用方负责周期时序控制，应先初始化并启动EtherCAT端口。 */
int dm_robot_cycle(dm_robot_t *robot, igh_mcan_port_t *port,
                   size_t *received_count);

/* 在单个EtherCAT周期内向全部12台电机发送一条特殊控制字。 */
int dm_robot_send_control_all(dm_robot_t *robot, igh_mcan_port_t *port,
                              dm_motor_control_t control);

#ifdef __cplusplus
}
#endif
#endif
