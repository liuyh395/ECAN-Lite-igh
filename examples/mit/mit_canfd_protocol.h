/* SPDX-License-Identifier: MIT */
#ifndef MIT_CANFD_PROTOCOL_H
#define MIT_CANFD_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MIT_CANFD_PAYLOAD_SIZE 8U

typedef struct {
    float position_min;
    float position_max;
    float velocity_min;
    float velocity_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float torque_min;
    float torque_max;
} mit_canfd_limits_t;

typedef struct {
    float position;
    float velocity;
    float kp;
    float kd;
    float torque;
} mit_canfd_command_t;

typedef struct {
    uint8_t node_id;
    float position;
    float velocity;
    float torque;
    uint8_t status;
    uint8_t temperature_c;
} mit_canfd_feedback_t;

typedef enum {
    MIT_CANFD_CONTROL_NONE = 0,
    MIT_CANFD_CONTROL_ENABLE,
    MIT_CANFD_CONTROL_DISABLE,
    MIT_CANFD_CONTROL_ZERO_POSITION
} mit_canfd_control_t;

/* Common AK-series ranges. Override them for the actual motor before use. */
void mit_canfd_default_limits(mit_canfd_limits_t *limits);

int mit_canfd_limits_valid(const mit_canfd_limits_t *limits);
int mit_canfd_pack_command(const mit_canfd_limits_t *limits,
                           const mit_canfd_command_t *command,
                           uint8_t payload[MIT_CANFD_PAYLOAD_SIZE]);
int mit_canfd_unpack_command(const mit_canfd_limits_t *limits,
                             const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE],
                             mit_canfd_command_t *command);
int mit_canfd_pack_feedback(const mit_canfd_limits_t *limits,
                            const mit_canfd_feedback_t *feedback,
                            uint8_t payload[MIT_CANFD_PAYLOAD_SIZE]);
int mit_canfd_unpack_feedback(const mit_canfd_limits_t *limits,
                              const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE],
                              size_t payload_size,
                              mit_canfd_feedback_t *feedback);
void mit_canfd_pack_control(mit_canfd_control_t control,
                            uint8_t payload[MIT_CANFD_PAYLOAD_SIZE]);
mit_canfd_control_t mit_canfd_detect_control(
    const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* MIT_CANFD_PROTOCOL_H */
