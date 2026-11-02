/* SPDX-License-Identifier: MIT */
#include "dm_motor_protocol.h"

#include <math.h>
#include <string.h>

static uint32_t float_to_uint(float value, float min, float max, unsigned bits)
{
    const uint32_t span = (1U << bits) - 1U;
    double scaled;
    if (value < min) value = min;
    if (value > max) value = max;
    scaled = ((double)value - min) * span / ((double)max - min);
    return (uint32_t)(scaled + 0.5);
}

static float uint_to_float(uint32_t value, float min, float max, unsigned bits)
{
    const uint32_t span = (1U << bits) - 1U;
    return (float)((double)value * ((double)max - min) / span + min);
}

void dm_motor_default_limits(dm_motor_limits_t *limits)
{
    if (limits == NULL) return;
    limits->position_min = -12.5F; limits->position_max = 12.5F;
    limits->velocity_min = -45.0F; limits->velocity_max = 45.0F;
    limits->kp_min = 0.0F; limits->kp_max = 500.0F;
    limits->kd_min = 0.0F; limits->kd_max = 5.0F;
    limits->torque_min = -18.0F; limits->torque_max = 18.0F;
}

int dm_motor_limits_valid(const dm_motor_limits_t *l)
{
    return l != NULL && isfinite(l->position_min) && isfinite(l->position_max) &&
           isfinite(l->velocity_min) && isfinite(l->velocity_max) &&
           isfinite(l->kp_min) && isfinite(l->kp_max) &&
           isfinite(l->kd_min) && isfinite(l->kd_max) &&
           isfinite(l->torque_min) && isfinite(l->torque_max) &&
           l->position_min < l->position_max && l->velocity_min < l->velocity_max &&
           l->kp_min < l->kp_max && l->kd_min < l->kd_max &&
           l->torque_min < l->torque_max;
}

int dm_motor_pack_command(const dm_motor_limits_t *l, const dm_motor_command_t *c,
                          uint8_t p[DM_MOTOR_PAYLOAD_SIZE])
{
    uint32_t pos, vel, kp, kd, torque;
    if (!dm_motor_limits_valid(l) || c == NULL || p == NULL ||
        !isfinite(c->position) || !isfinite(c->velocity) || !isfinite(c->kp) ||
        !isfinite(c->kd) || !isfinite(c->torque)) return -1;
    pos = float_to_uint(c->position, l->position_min, l->position_max, 16U);
    vel = float_to_uint(c->velocity, l->velocity_min, l->velocity_max, 12U);
    kp = float_to_uint(c->kp, l->kp_min, l->kp_max, 12U);
    kd = float_to_uint(c->kd, l->kd_min, l->kd_max, 12U);
    torque = float_to_uint(c->torque, l->torque_min, l->torque_max, 12U);
    p[0] = (uint8_t)(pos >> 8); p[1] = (uint8_t)pos;
    p[2] = (uint8_t)(vel >> 4); p[3] = (uint8_t)((vel << 4) | (kp >> 8));
    p[4] = (uint8_t)kp; p[5] = (uint8_t)(kd >> 4);
    p[6] = (uint8_t)((kd << 4) | (torque >> 8)); p[7] = (uint8_t)torque;
    return 0;
}

int dm_motor_unpack_feedback(const dm_motor_limits_t *l,
                             const uint8_t p[DM_MOTOR_PAYLOAD_SIZE],
                             dm_motor_feedback_t *f)
{
    if (!dm_motor_limits_valid(l) || p == NULL || f == NULL) return -1;
    f->motor_id = (uint8_t)(p[0] & 0x0FU);
    f->error_state = (uint8_t)(p[0] >> 4);
    f->position_raw = (uint16_t)(((uint16_t)p[1] << 8) | p[2]);
    f->velocity_raw = (uint16_t)(((uint16_t)p[3] << 4) | (p[4] >> 4));
    f->torque_raw = (uint16_t)((((uint16_t)p[4] & 0x0FU) << 8) | p[5]);
    f->position = uint_to_float(f->position_raw, l->position_min, l->position_max, 16U);
    f->velocity = uint_to_float(f->velocity_raw, l->velocity_min, l->velocity_max, 12U);
    f->torque = uint_to_float(f->torque_raw, l->torque_min, l->torque_max, 12U);
    f->mos_temperature = p[6]; f->rotor_temperature = p[7];
    return 0;
}

void dm_motor_pack_control(dm_motor_control_t control, uint8_t p[DM_MOTOR_PAYLOAD_SIZE])
{
    uint8_t opcode = 0U;
    if (p == NULL) return;
    memset(p, 0xFF, DM_MOTOR_PAYLOAD_SIZE);
    if (control == DM_MOTOR_CONTROL_ENABLE) opcode = 0xFCU;
    else if (control == DM_MOTOR_CONTROL_DISABLE) opcode = 0xFDU;
    else if (control == DM_MOTOR_CONTROL_ZERO_POSITION) opcode = 0xFEU;
    else if (control == DM_MOTOR_CONTROL_CLEAR_ERROR) opcode = 0xFBU;
    p[7] = opcode;
}

dm_motor_control_t dm_motor_detect_control(const uint8_t p[DM_MOTOR_PAYLOAD_SIZE])
{
    if (p == NULL) return DM_MOTOR_CONTROL_NONE;
    for (size_t i = 0; i < 7U; i++) if (p[i] != 0xFFU) return DM_MOTOR_CONTROL_NONE;
    if (p[7] == 0xFCU) return DM_MOTOR_CONTROL_ENABLE;
    if (p[7] == 0xFDU) return DM_MOTOR_CONTROL_DISABLE;
    if (p[7] == 0xFEU) return DM_MOTOR_CONTROL_ZERO_POSITION;
    if (p[7] == 0xFBU) return DM_MOTOR_CONTROL_CLEAR_ERROR;
    return DM_MOTOR_CONTROL_NONE;
}
