/* SPDX-License-Identifier: MIT */
#include "dm_motor.h"

#include <string.h>

void dm_motor_default_config(dm_motor_config_t *c, uint8_t channel, uint8_t id)
{
    if (c == NULL) return;
    memset(c, 0, sizeof(*c));
    c->channel = channel; c->motor_id = id;
    c->command_id = id; c->feedback_id = id;
    dm_motor_default_limits(&c->limits);
}

int dm_motor_init(dm_motor_t *m, const dm_motor_config_t *c)
{
    if (m == NULL || c == NULL || c->channel >= IGH_MCAN_CHANNEL_COUNT ||
        c->motor_id > 0x0FU || c->command_id > (c->extended_id ? 0x1FFFFFFFU : 0x7FFU) ||
        c->feedback_id > (c->extended_id ? 0x1FFFFFFFU : 0x7FFU) ||
        !dm_motor_limits_valid(&c->limits)) return -1;
    memset(m, 0, sizeof(*m)); m->config = *c;
    return 0;
}

int dm_motor_send_control(dm_motor_t *m, igh_mcan_port_t *p, size_t slave,
                          dm_motor_control_t control)
{
    uint8_t payload[DM_MOTOR_PAYLOAD_SIZE];
    if (m == NULL || p == NULL || control == DM_MOTOR_CONTROL_NONE) return -1;
    dm_motor_pack_control(control, payload);
    return igh_mcan_port_send_can(p, (int)slave, m->config.channel,
                                  m->config.command_id, m->config.extended_id,
                                  payload, sizeof(payload));
}

int dm_motor_send_command(dm_motor_t *m, igh_mcan_port_t *p, size_t slave)
{
    uint8_t payload[DM_MOTOR_PAYLOAD_SIZE];
    int rc;
    if (m == NULL || p == NULL || dm_motor_pack_command(&m->config.limits,
                                                         &m->command, payload) != 0) return -1;
    rc = igh_mcan_port_send_can(p, (int)slave, m->config.channel,
                                m->config.command_id, m->config.extended_id,
                                payload, sizeof(payload));
    if (rc == IGH_ECAN_OK) m->commands_sent++;
    return rc;
}

int dm_motor_accept_feedback(dm_motor_t *m, const igh_mcan_rx_frame_t *r)
{
    if (m == NULL || r == NULL) return -1;
    if (r->frame.is_canfd || r->frame.data_len != DM_MOTOR_PAYLOAD_SIZE ||
        r->frame.channel != m->config.channel ||
        r->frame.can_id != m->config.feedback_id ||
        r->frame.is_extended != m->config.extended_id) return 1;
    if (dm_motor_unpack_feedback(&m->config.limits, r->frame.data, &m->feedback) != 0) {
        m->malformed_feedback++; return -1;
    }
    if (m->feedback.motor_id != m->config.motor_id) return 1;
    m->feedback_received++; m->has_feedback = 1U;
    return 0;
}

void dm_motor_manager_default_config(dm_motor_config_t configs[DM_MOTOR_MANAGER_CAPACITY])
{
    if (configs == NULL) return;
    for (size_t i = 0; i < DM_MOTOR_MANAGER_CAPACITY; i++)
        dm_motor_default_config(&configs[i], (uint8_t)(i / 3U), (uint8_t)(i % 3U + 1U));
}

int dm_motor_manager_init(dm_motor_manager_t *manager, size_t slave,
                          const dm_motor_config_t *configs, size_t count)
{
    if (manager == NULL || configs == NULL || count == 0U ||
        count > DM_MOTOR_MANAGER_CAPACITY) return -1;
    memset(manager, 0, sizeof(*manager)); manager->slave_pos = slave;
    for (size_t i = 0; i < count; i++)
        if (dm_motor_init(&manager->motors[i], &configs[i]) != 0) return -1;
    manager->count = count; return 0;
}

int dm_motor_manager_receive(dm_motor_manager_t *manager,
                             const igh_mcan_rx_frame_t *frames, size_t count)
{
    int accepted = 0;
    if (manager == NULL || (frames == NULL && count != 0U)) return -1;
    for (size_t i = 0; i < count; i++) {
        if (frames[i].slave_pos != (int)manager->slave_pos) continue;
        for (size_t j = 0; j < manager->count; j++)
            if (dm_motor_accept_feedback(&manager->motors[j], &frames[i]) == 0) {
                accepted++; break;
            }
    }
    return accepted;
}

int dm_motor_manager_send(dm_motor_manager_t *manager, igh_mcan_port_t *port)
{
    if (manager == NULL || port == NULL) return -1;
    for (size_t i = 0; i < manager->count; i++) {
        int rc = dm_motor_send_command(&manager->motors[i], port, manager->slave_pos);
        if (rc != IGH_ECAN_OK) return rc;
    }
    return IGH_ECAN_OK;
}
