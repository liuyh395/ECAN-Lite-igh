/* SPDX-License-Identifier: MIT */
#ifndef DM_MOTOR_PROTOCOL_H
#define DM_MOTOR_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define DM_MOTOR_PAYLOAD_SIZE 8U

typedef struct {
    float position_min, position_max;
    float velocity_min, velocity_max;
    float kp_min, kp_max;
    float kd_min, kd_max;
    float torque_min, torque_max;
} dm_motor_limits_t;

typedef struct {
    float position, velocity, kp, kd, torque;
} dm_motor_command_t;

typedef struct {
    uint8_t motor_id;
    uint8_t error_state;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    float position, velocity, torque;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
} dm_motor_feedback_t;

typedef enum {
    DM_MOTOR_CONTROL_NONE = 0,
    DM_MOTOR_CONTROL_ENABLE,
    DM_MOTOR_CONTROL_DISABLE,
    DM_MOTOR_CONTROL_ZERO_POSITION,
    DM_MOTOR_CONTROL_CLEAR_ERROR
} dm_motor_control_t;

void dm_motor_default_limits(dm_motor_limits_t *limits);
int dm_motor_limits_valid(const dm_motor_limits_t *limits);
int dm_motor_pack_command(const dm_motor_limits_t *limits,
                          const dm_motor_command_t *command,
                          uint8_t payload[DM_MOTOR_PAYLOAD_SIZE]);
int dm_motor_unpack_feedback(const dm_motor_limits_t *limits,
                             const uint8_t payload[DM_MOTOR_PAYLOAD_SIZE],
                             dm_motor_feedback_t *feedback);
void dm_motor_pack_control(dm_motor_control_t control,
                            uint8_t payload[DM_MOTOR_PAYLOAD_SIZE]);
dm_motor_control_t dm_motor_detect_control(
    const uint8_t payload[DM_MOTOR_PAYLOAD_SIZE]);

#endif
