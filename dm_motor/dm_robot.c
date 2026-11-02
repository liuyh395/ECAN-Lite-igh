/* SPDX-License-Identifier: MIT */
#include "dm_robot.h"

#include <math.h>
#include <string.h>

static int command_valid(const dm_motor_command_t *command)
{
    return command != NULL && isfinite(command->position) &&
           isfinite(command->velocity) && isfinite(command->kp) &&
           isfinite(command->kd) && isfinite(command->torque);
}

void dm_robot_default_config(dm_robot_motor_config_t configs[DM_ROBOT_MOTOR_COUNT])
{
    if (configs == NULL) return;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        uint8_t channel = (uint8_t)(i / DM_ROBOT_MOTORS_PER_CAN);
        uint8_t id = (uint8_t)(i % DM_ROBOT_MOTORS_PER_CAN + 1U);
        configs[i].channel = channel;
        configs[i].motor_id = id;
        configs[i].command_id = id;
        /* Placeholder convention only; edit each entry for the actual motor. */
        configs[i].feedback_id = 0x101U + (uint32_t)i;
        configs[i].extended_id = 0U;
        dm_motor_default_limits(&configs[i].limits);
    }
}

static int config_valid(const dm_robot_motor_config_t *c)
{
    uint32_t max_id;
    if (c == NULL || c->channel >= IGH_MCAN_CHANNEL_COUNT ||
        c->motor_id > 0x0FU || c->extended_id > 1U ||
        !dm_motor_limits_valid(&c->limits)) return 0;
    max_id = c->extended_id ? 0x1FFFFFFFU : 0x7FFU;
    return c->command_id <= max_id && c->feedback_id <= max_id;
}

int dm_robot_init(dm_robot_t *robot, size_t slave_pos,
                  const dm_robot_motor_config_t configs[DM_ROBOT_MOTOR_COUNT])
{
    if (robot == NULL || configs == NULL) return -1;
    memset(robot, 0, sizeof(*robot));
    robot->slave_pos = slave_pos;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        if (!config_valid(&configs[i])) return -1;
        robot->motors[i].config = configs[i];
    }
    return 0;
}

int dm_robot_apply_classic_can_config(ecan_igh_config_t *igh_config,
                                      igh_mcan_port_config_t *mcan_config,
                                      size_t slave_pos, uint32_t bitrate)
{
    if (igh_config == NULL || mcan_config == NULL || bitrate == 0U) return -1;
    if (igh_mcan_port_set_channel_mask(mcan_config, slave_pos, 0x0FU, 0x00U) != IGH_ECAN_OK ||
        igh_mcan_port_set_bitrate(mcan_config, slave_pos, bitrate, bitrate) != IGH_ECAN_OK ||
        igh_mcan_port_apply_config(igh_config, mcan_config) != IGH_ECAN_OK) return -1;
    return 0;
}

int dm_robot_submit(dm_robot_t *robot, size_t index,
                    const dm_motor_command_t *command)
{
    if (robot == NULL || index >= DM_ROBOT_MOTOR_COUNT ||
        !command_valid(command)) return -1;
    robot->motors[index].command = *command;
    return 0;
}

int dm_robot_submit_all(dm_robot_t *robot,
                        const dm_motor_command_t commands[DM_ROBOT_MOTOR_COUNT])
{
    if (robot == NULL || commands == NULL) return -1;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i)
        if (!command_valid(&commands[i])) return -1;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i)
        robot->motors[i].command = commands[i];
    return 0;
}

static int send_one(dm_robot_t *robot, igh_mcan_port_t *port, size_t index)
{
    dm_robot_motor_t *motor = &robot->motors[index];
    uint8_t payload[DM_MOTOR_PAYLOAD_SIZE];
    int rc;
    if (dm_motor_pack_command(&motor->config.limits, &motor->command, payload) != 0)
        return -1;
    rc = igh_mcan_port_send_can(port, (int)robot->slave_pos,
                                motor->config.channel, motor->config.command_id,
                                motor->config.extended_id, payload, sizeof(payload));
    if (rc == IGH_ECAN_OK) motor->commands_sent++;
    return rc;
}

static int process_feedback(dm_robot_t *robot,
                            const igh_mcan_rx_frame_t *frames, size_t count)
{
    int accepted = 0;
    for (size_t f = 0; f < count; ++f) {
        if (frames[f].slave_pos != (int)robot->slave_pos ||
            frames[f].frame.is_canfd ||
            frames[f].frame.data_len != DM_MOTOR_PAYLOAD_SIZE) continue;
        for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
            dm_robot_motor_t *m = &robot->motors[i];
            dm_motor_feedback_t feedback;
            if (frames[f].frame.channel != m->config.channel ||
                frames[f].frame.can_id != m->config.feedback_id ||
                frames[f].frame.is_extended != m->config.extended_id ||
                dm_motor_unpack_feedback(&m->config.limits,
                                         frames[f].frame.data, &feedback) != 0 ||
                feedback.motor_id != m->config.motor_id) continue;
            m->feedback = feedback;
            m->feedback_received++;
            m->has_feedback = 1U;
            accepted++;
            break;
        }
    }
    return accepted;
}

int dm_robot_cycle(dm_robot_t *robot, igh_mcan_port_t *port,
                   size_t *received_count)
{
    igh_mcan_rx_frame_t frames[DM_ROBOT_RX_CAPACITY];
    size_t count = 0U;
    int rc;
    if (robot == NULL || port == NULL) return -1;
    rc = igh_mcan_port_cycle_begin_checked(port);
    if (rc != IGH_ECAN_OK) return rc;
    rc = igh_mcan_port_receive(port, frames, DM_ROBOT_RX_CAPACITY, &count);
    if (rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) {
        (void)igh_mcan_port_cycle_abort_checked(port);
        return rc;
    }
    (void)process_feedback(robot, frames, count);
    if (received_count != NULL) *received_count = count;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        rc = send_one(robot, port, i);
        if (rc != IGH_ECAN_OK) {
            (void)igh_mcan_port_cycle_abort_checked(port);
            return rc;
        }
    }
    rc = igh_mcan_port_cycle_end_checked(port);
    if (rc == IGH_ECAN_OK) robot->cycles_completed++;
    return rc;
}

int dm_robot_send_control_all(dm_robot_t *robot, igh_mcan_port_t *port,
                              dm_motor_control_t control)
{
    uint8_t payload[DM_MOTOR_PAYLOAD_SIZE];
    int rc;
    if (robot == NULL || port == NULL || control == DM_MOTOR_CONTROL_NONE) return -1;
    dm_motor_pack_control(control, payload);
    rc = igh_mcan_port_cycle_begin_checked(port);
    if (rc != IGH_ECAN_OK) return rc;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        const dm_robot_motor_config_t *c = &robot->motors[i].config;
        rc = igh_mcan_port_send_can(port, (int)robot->slave_pos, c->channel,
                                    c->command_id, c->extended_id,
                                    payload, sizeof(payload));
        if (rc != IGH_ECAN_OK) {
            (void)igh_mcan_port_cycle_abort_checked(port);
            return rc;
        }
    }
    return igh_mcan_port_cycle_end_checked(port);
}
