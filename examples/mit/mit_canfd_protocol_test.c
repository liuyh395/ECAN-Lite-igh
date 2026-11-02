/* SPDX-License-Identifier: MIT */
#include "mit_canfd_protocol.h"

#include <math.h>
#include <stdio.h>

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        return 1; \
    } \
} while (0)

static int expect_near(float actual, float expected, float tolerance)
{
    return fabsf(actual - expected) <= tolerance;
}

int main(void)
{
    mit_canfd_limits_t limits;
    mit_canfd_command_t command = {1.25F, -7.0F, 123.0F, 1.25F, -3.0F};
    mit_canfd_command_t decoded = {0};
    mit_canfd_feedback_t feedback = {11U, -2.5F, 9.0F, 4.0F, 0x03U, 37U};
    mit_canfd_feedback_t decoded_feedback = {0};
    uint8_t payload[MIT_CANFD_PAYLOAD_SIZE];

    mit_canfd_default_limits(&limits);
    CHECK(mit_canfd_limits_valid(&limits));
    CHECK(mit_canfd_pack_command(&limits, &command, payload) == 0);
    CHECK(mit_canfd_detect_control(payload) == MIT_CANFD_CONTROL_NONE);
    CHECK(mit_canfd_unpack_command(&limits, payload, &decoded) == 0);
    CHECK(expect_near(decoded.position, command.position, 0.001F));
    CHECK(expect_near(decoded.velocity, command.velocity, 0.03F));
    CHECK(expect_near(decoded.kp, command.kp, 0.13F));
    CHECK(expect_near(decoded.kd, command.kd, 0.002F));
    CHECK(expect_near(decoded.torque, command.torque, 0.01F));

    CHECK(mit_canfd_pack_feedback(&limits, &feedback, payload) == 0);
    CHECK(mit_canfd_unpack_feedback(&limits, payload, sizeof(payload),
                                    &decoded_feedback) == 0);
    CHECK(decoded_feedback.node_id == feedback.node_id);
    CHECK(decoded_feedback.status == feedback.status);
    CHECK(decoded_feedback.temperature_c == feedback.temperature_c);
    CHECK(expect_near(decoded_feedback.position, feedback.position, 0.001F));
    CHECK(expect_near(decoded_feedback.velocity, feedback.velocity, 0.03F));
    CHECK(expect_near(decoded_feedback.torque, feedback.torque, 0.01F));

    mit_canfd_pack_control(MIT_CANFD_CONTROL_ENABLE, payload);
    CHECK(mit_canfd_detect_control(payload) == MIT_CANFD_CONTROL_ENABLE);
    mit_canfd_pack_control(MIT_CANFD_CONTROL_DISABLE, payload);
    CHECK(mit_canfd_detect_control(payload) == MIT_CANFD_CONTROL_DISABLE);
    mit_canfd_pack_control(MIT_CANFD_CONTROL_ZERO_POSITION, payload);
    CHECK(mit_canfd_detect_control(payload) == MIT_CANFD_CONTROL_ZERO_POSITION);

    puts("MIT CAN FD protocol tests passed");
    return 0;
}
