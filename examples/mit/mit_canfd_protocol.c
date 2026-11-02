/* SPDX-License-Identifier: MIT */
#include "mit_canfd_protocol.h"

#include <math.h>
#include <string.h>

static uint32_t float_to_uint(float value, float min, float max, unsigned bits)
{
    const uint32_t span = (1U << bits) - 1U;
    double scaled;

    if (value < min) {
        value = min;
    } else if (value > max) {
        value = max;
    }
    scaled = ((double)value - (double)min) * (double)span /
             ((double)max - (double)min);
    return (uint32_t)(scaled + 0.5);
}

static float uint_to_float(uint32_t value, float min, float max, unsigned bits)
{
    const uint32_t span = (1U << bits) - 1U;

    return (float)((double)value * ((double)max - (double)min) /
                   (double)span + (double)min);
}

void mit_canfd_default_limits(mit_canfd_limits_t *limits)
{
    if (limits == NULL) {
        return;
    }
    limits->position_min = -12.5F;
    limits->position_max = 12.5F;
    limits->velocity_min = -45.0F;
    limits->velocity_max = 45.0F;
    limits->kp_min = 0.0F;
    limits->kp_max = 500.0F;
    limits->kd_min = 0.0F;
    limits->kd_max = 5.0F;
    limits->torque_min = -18.0F;
    limits->torque_max = 18.0F;
}

int mit_canfd_limits_valid(const mit_canfd_limits_t *limits)
{
    if (limits == NULL) {
        return 0;
    }
    return isfinite(limits->position_min) && isfinite(limits->position_max) &&
           isfinite(limits->velocity_min) && isfinite(limits->velocity_max) &&
           isfinite(limits->kp_min) && isfinite(limits->kp_max) &&
           isfinite(limits->kd_min) && isfinite(limits->kd_max) &&
           isfinite(limits->torque_min) && isfinite(limits->torque_max) &&
           limits->position_min < limits->position_max &&
           limits->velocity_min < limits->velocity_max &&
           limits->kp_min < limits->kp_max && limits->kd_min < limits->kd_max &&
           limits->torque_min < limits->torque_max;
}

int mit_canfd_pack_command(const mit_canfd_limits_t *limits,
                           const mit_canfd_command_t *command,
                           uint8_t payload[MIT_CANFD_PAYLOAD_SIZE])
{
    uint32_t p;
    uint32_t v;
    uint32_t kp;
    uint32_t kd;
    uint32_t torque;

    if (!mit_canfd_limits_valid(limits) || command == NULL || payload == NULL ||
        !isfinite(command->position) || !isfinite(command->velocity) ||
        !isfinite(command->kp) || !isfinite(command->kd) ||
        !isfinite(command->torque)) {
        return -1;
    }
    p = float_to_uint(command->position, limits->position_min,
                      limits->position_max, 16U);
    v = float_to_uint(command->velocity, limits->velocity_min,
                      limits->velocity_max, 12U);
    kp = float_to_uint(command->kp, limits->kp_min, limits->kp_max, 12U);
    kd = float_to_uint(command->kd, limits->kd_min, limits->kd_max, 12U);
    torque = float_to_uint(command->torque, limits->torque_min,
                           limits->torque_max, 12U);

    payload[0] = (uint8_t)(p >> 8);
    payload[1] = (uint8_t)p;
    payload[2] = (uint8_t)(v >> 4);
    payload[3] = (uint8_t)(((v & 0x0FU) << 4) | (kp >> 8));
    payload[4] = (uint8_t)kp;
    payload[5] = (uint8_t)(kd >> 4);
    payload[6] = (uint8_t)(((kd & 0x0FU) << 4) | (torque >> 8));
    payload[7] = (uint8_t)torque;
    return 0;
}

int mit_canfd_unpack_command(const mit_canfd_limits_t *limits,
                             const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE],
                             mit_canfd_command_t *command)
{
    uint32_t p;
    uint32_t v;
    uint32_t kp;
    uint32_t kd;
    uint32_t torque;

    if (!mit_canfd_limits_valid(limits) || payload == NULL || command == NULL) {
        return -1;
    }
    p = ((uint32_t)payload[0] << 8) | payload[1];
    v = ((uint32_t)payload[2] << 4) | (payload[3] >> 4);
    kp = ((uint32_t)(payload[3] & 0x0FU) << 8) | payload[4];
    kd = ((uint32_t)payload[5] << 4) | (payload[6] >> 4);
    torque = ((uint32_t)(payload[6] & 0x0FU) << 8) | payload[7];

    command->position = uint_to_float(p, limits->position_min,
                                      limits->position_max, 16U);
    command->velocity = uint_to_float(v, limits->velocity_min,
                                      limits->velocity_max, 12U);
    command->kp = uint_to_float(kp, limits->kp_min, limits->kp_max, 12U);
    command->kd = uint_to_float(kd, limits->kd_min, limits->kd_max, 12U);
    command->torque = uint_to_float(torque, limits->torque_min,
                                    limits->torque_max, 12U);
    return 0;
}

int mit_canfd_pack_feedback(const mit_canfd_limits_t *limits,
                            const mit_canfd_feedback_t *feedback,
                            uint8_t payload[MIT_CANFD_PAYLOAD_SIZE])
{
    uint32_t p;
    uint32_t v;
    uint32_t torque;

    if (!mit_canfd_limits_valid(limits) || feedback == NULL || payload == NULL ||
        !isfinite(feedback->position) || !isfinite(feedback->velocity) ||
        !isfinite(feedback->torque)) {
        return -1;
    }
    p = float_to_uint(feedback->position, limits->position_min,
                      limits->position_max, 16U);
    v = float_to_uint(feedback->velocity, limits->velocity_min,
                      limits->velocity_max, 12U);
    torque = float_to_uint(feedback->torque, limits->torque_min,
                           limits->torque_max, 12U);
    payload[0] = feedback->node_id;
    payload[1] = (uint8_t)(p >> 8);
    payload[2] = (uint8_t)p;
    payload[3] = (uint8_t)(v >> 4);
    payload[4] = (uint8_t)(((v & 0x0FU) << 4) | (torque >> 8));
    payload[5] = (uint8_t)torque;
    payload[6] = feedback->status;
    payload[7] = feedback->temperature_c;
    return 0;
}

int mit_canfd_unpack_feedback(const mit_canfd_limits_t *limits,
                              const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE],
                              size_t payload_size,
                              mit_canfd_feedback_t *feedback)
{
    uint32_t p;
    uint32_t v;
    uint32_t torque;

    if (!mit_canfd_limits_valid(limits) || payload == NULL || feedback == NULL ||
        payload_size < 6U) {
        return -1;
    }
    p = ((uint32_t)payload[1] << 8) | payload[2];
    v = ((uint32_t)payload[3] << 4) | (payload[4] >> 4);
    torque = ((uint32_t)(payload[4] & 0x0FU) << 8) | payload[5];
    feedback->node_id = payload[0];
    feedback->position = uint_to_float(p, limits->position_min,
                                       limits->position_max, 16U);
    feedback->velocity = uint_to_float(v, limits->velocity_min,
                                       limits->velocity_max, 12U);
    feedback->torque = uint_to_float(torque, limits->torque_min,
                                     limits->torque_max, 12U);
    feedback->status = payload_size >= 7U ? payload[6] : 0U;
    feedback->temperature_c = payload_size >= 8U ? payload[7] : 0U;
    return 0;
}

void mit_canfd_pack_control(mit_canfd_control_t control,
                            uint8_t payload[MIT_CANFD_PAYLOAD_SIZE])
{
    uint8_t opcode = 0U;

    if (payload == NULL) {
        return;
    }
    memset(payload, 0xFF, MIT_CANFD_PAYLOAD_SIZE);
    if (control == MIT_CANFD_CONTROL_ENABLE) {
        opcode = 0xFCU;
    } else if (control == MIT_CANFD_CONTROL_DISABLE) {
        opcode = 0xFDU;
    } else if (control == MIT_CANFD_CONTROL_ZERO_POSITION) {
        opcode = 0xFEU;
    }
    payload[7] = opcode;
}

mit_canfd_control_t mit_canfd_detect_control(
    const uint8_t payload[MIT_CANFD_PAYLOAD_SIZE])
{
    if (payload == NULL) {
        return MIT_CANFD_CONTROL_NONE;
    }
    for (size_t i = 0U; i < 7U; i++) {
        if (payload[i] != 0xFFU) {
            return MIT_CANFD_CONTROL_NONE;
        }
    }
    if (payload[7] == 0xFCU) {
        return MIT_CANFD_CONTROL_ENABLE;
    }
    if (payload[7] == 0xFDU) {
        return MIT_CANFD_CONTROL_DISABLE;
    }
    if (payload[7] == 0xFEU) {
        return MIT_CANFD_CONTROL_ZERO_POSITION;
    }
    return MIT_CANFD_CONTROL_NONE;
}
