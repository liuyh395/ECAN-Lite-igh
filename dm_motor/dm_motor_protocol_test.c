/* SPDX-License-Identifier: MIT */
#include "dm_motor_protocol.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "check failed: %s\n", #x); return 1; } } while (0)

int main(void)
{
    dm_motor_limits_t limits;
    dm_motor_command_t command = {1.25F, -7.0F, 123.0F, 1.25F, -3.0F};
    dm_motor_feedback_t feedback;
    uint8_t p[DM_MOTOR_PAYLOAD_SIZE];
    dm_motor_default_limits(&limits);
    CHECK(dm_motor_limits_valid(&limits));
    CHECK(dm_motor_pack_command(&limits, &command, p) == 0);
    CHECK(p[0] == 0x8CU && p[1] == 0xCCU);
    { const uint8_t fb[8] = {0x21U, 0x80U, 0x00U, 0x80U, 0x00U, 0x80U, 55U, 66U};
      CHECK(dm_motor_unpack_feedback(&limits, fb, &feedback) == 0); }
    CHECK(feedback.motor_id == 1U && feedback.error_state == 2U);
    CHECK(feedback.mos_temperature == 55U && feedback.rotor_temperature == 66U);
    dm_motor_pack_control(DM_MOTOR_CONTROL_CLEAR_ERROR, p);
    CHECK(dm_motor_detect_control(p) == DM_MOTOR_CONTROL_CLEAR_ERROR);
    CHECK(fabsf(feedback.position) < 0.001F);
    puts("DM motor protocol tests passed");
    return 0;
}
