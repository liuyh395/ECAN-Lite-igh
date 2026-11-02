/* SPDX-License-Identifier: MIT */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "dm_motor_protocol.h"
#include "igh_mcan_port.h"

#define DM_MOTOR_MANAGER_CAPACITY 12U

typedef struct {
    uint8_t channel;
    uint8_t motor_id;
    uint32_t command_id;
    uint32_t feedback_id;
    int extended_id;
    dm_motor_limits_t limits;
} dm_motor_config_t;

typedef struct {
    dm_motor_config_t config;
    dm_motor_command_t command;
    dm_motor_feedback_t feedback;
    uint64_t commands_sent;
    uint64_t feedback_received;
    uint64_t feedback_timeouts;
    uint64_t malformed_feedback;
    uint8_t enabled;
    uint8_t has_feedback;
} dm_motor_t;

typedef struct {
    size_t slave_pos;
    size_t count;
    dm_motor_t motors[DM_MOTOR_MANAGER_CAPACITY];
} dm_motor_manager_t;

void dm_motor_default_config(dm_motor_config_t *config, uint8_t channel,
                             uint8_t motor_id);
int dm_motor_init(dm_motor_t *motor, const dm_motor_config_t *config);
int dm_motor_send_control(dm_motor_t *motor, igh_mcan_port_t *port,
                          size_t slave_pos, dm_motor_control_t control);
int dm_motor_send_command(dm_motor_t *motor, igh_mcan_port_t *port,
                          size_t slave_pos);
int dm_motor_accept_feedback(dm_motor_t *motor,
                             const igh_mcan_rx_frame_t *frame);

void dm_motor_manager_default_config(dm_motor_config_t configs[DM_MOTOR_MANAGER_CAPACITY]);
int dm_motor_manager_init(dm_motor_manager_t *manager, size_t slave_pos,
                          const dm_motor_config_t *configs, size_t count);
int dm_motor_manager_receive(dm_motor_manager_t *manager,
                             const igh_mcan_rx_frame_t *frames, size_t count);
int dm_motor_manager_send(dm_motor_manager_t *manager, igh_mcan_port_t *port);

#endif
