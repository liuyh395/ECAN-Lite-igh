/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include "igh_250us_pipeline.h"

#include <stdlib.h>
#include <string.h>

typedef struct {
    igh_250us_pipeline_config_t config;
    ecan_igh_context_t igh;
    igh_mcan_port_t mcan;
    igh_rt_periodic_clock_t clock;
    igh_250us_pipeline_stats_t stats;
    uint64_t next_cycle;
    uint64_t application_start_ns;
    uint8_t active_channel;
    uint8_t cycle_active;
    uint8_t igh_open;
} pipeline_impl_t;

/* Open is a process-start operation; keep diagnostics C99-compatible. */
static igh_250us_pipeline_open_stage_t last_open_stage;
static igh_250us_pipeline_open_diag_t last_open_diag;

void igh_250us_pipeline_get_last_open_diag(igh_250us_pipeline_open_diag_t *diag)
{
    if (diag != NULL) *diag = last_open_diag;
}

static void capture_open_diag(pipeline_impl_t *impl, int result)
{
    memset(&last_open_diag, 0, sizeof(last_open_diag));
    last_open_diag.stage = last_open_stage;
    last_open_diag.result = result;
    if (impl == NULL || impl->igh_open == 0U) return;
    last_open_diag.domain_complete = ecan_igh_domain_is_complete(&impl->igh) ? 1U : 0U;
    last_open_diag.all_operational = ecan_igh_slaves_are_operational(&impl->igh) ? 1U : 0U;
    last_open_diag.working_counter = impl->igh.domain_working_counter;
    last_open_diag.expected_working_counter = impl->igh.domain_expected_working_counter;
    last_open_diag.slave_count = impl->igh.slave_count;
    for (size_t slave = 0U; slave < impl->igh.slave_count; slave++) {
        last_open_diag.slave_al_state[slave] = ecan_igh_slave_al_state(&impl->igh, slave);
    }
    (void)ecan_igh_get_sched_stats(&impl->igh, &last_open_diag.ethercat);
    {
        ecan_igh_diag_event_snapshot_t events;
        memset(&events, 0, sizeof(events));
        if (ecan_igh_get_diag_events(&impl->igh, 0U, 1U, &events) == IGH_ECAN_OK &&
            events.count != 0U) {
            last_open_diag.last_event = events.events[events.count - 1U];
            last_open_diag.last_event_valid = 1U;
        }
    }
}

igh_250us_pipeline_open_stage_t igh_250us_pipeline_last_open_stage(void)
{
    return last_open_stage;
}

const char *igh_250us_pipeline_open_stage_name(igh_250us_pipeline_open_stage_t stage)
{
    switch (stage) {
    case IGH_250US_PIPELINE_OPEN_NONE: return "none";
    case IGH_250US_PIPELINE_OPEN_RUNTIME_TUNING: return "runtime-tuning";
    case IGH_250US_PIPELINE_OPEN_STACK: return "stack";
    case IGH_250US_PIPELINE_OPEN_CLOCK: return "clock";
    case IGH_250US_PIPELINE_OPEN_WARMUP: return "warmup";
    case IGH_250US_PIPELINE_OPEN_NOT_OPERATIONAL: return "not-operational";
    case IGH_250US_PIPELINE_OPEN_DIAGNOSTICS: return "diagnostics";
    default: return "unknown";
    }
}

static int config_valid(const igh_250us_pipeline_config_t *config)
{
    uint8_t phase_mask = 0U;

    if (config == NULL) return 0;
    for (size_t phase = 0U; phase < IGH_250US_PIPELINE_PHASE_COUNT; phase++) {
        uint8_t channel = config->phase_channels[phase];
        if (channel >= IGH_250US_PIPELINE_PHASE_COUNT ||
            (phase_mask & (uint8_t)(1U << channel)) != 0U) return 0;
        phase_mask = (uint8_t)(phase_mask | (uint8_t)(1U << channel));
    }
    return config->slave_count > 0U &&
           config->slave_count <= ECAN_IGH_MAX_SLAVES &&
           config->channel_mask != 0U &&
           (config->channel_mask & (uint8_t)~IGH_250US_PIPELINE_CHANNEL_MASK) == 0U &&
           phase_mask == IGH_250US_PIPELINE_CHANNEL_MASK &&
           ecan_igh_pdo_size_encoding_is_valid(config->rxpdo_multiplier) &&
           ecan_igh_pdo_size_encoding_is_valid(config->txpdo_multiplier) &&
           config->nominal_bitrate != 0U && config->data_bitrate != 0U &&
           config->nominal_sample_point_x10 >= 500U &&
           config->nominal_sample_point_x10 <= 950U &&
           config->data_sample_point_x10 >= 500U &&
           config->data_sample_point_x10 <= 950U &&
           config->mode == IGH_250US_PIPELINE_SM &&
           config->stage_timing <= 1U &&
           config->rt_priority >= 0 && config->rt_priority <= 99;
}

void igh_250us_pipeline_default_config(igh_250us_pipeline_config_t *config)
{
    if (config == NULL) return;
    memset(config, 0, sizeof(*config));
    config->slave_count = 1U;
    config->channel_mask = IGH_250US_PIPELINE_CHANNEL_MASK;
    for (uint8_t phase = 0U; phase < IGH_250US_PIPELINE_PHASE_COUNT; phase++) {
        config->phase_channels[phase] = phase;
    }
    config->rxpdo_multiplier = ECAN_RT_SM_RXPDO_MULTIPLIER;
    config->txpdo_multiplier = ECAN_RT_SM_TXPDO_MULTIPLIER;
    config->nominal_bitrate = 1000000U;
    config->data_bitrate = 5000000U;
    config->nominal_sample_point_x10 = 875U;
    config->data_sample_point_x10 = 875U;
    config->mode = IGH_250US_PIPELINE_SM;
    config->dc_sync0_shift_us = 0;
    config->wait_op_ms = 10000U;
    /* Zero selects the validated SM startup baseline: 4000 * 250us = 1s. */
    config->warmup_cycles = 0U;
    config->spin_us = 20U;
    config->lock_memory = 1U;
    config->stage_timing = 0U;
    config->rt_priority = 80;
    config->cpu = -1;
}

int igh_250us_pipeline_config_set_phase_order(igh_250us_pipeline_config_t *config,
                                              uint8_t phase0,
                                              uint8_t phase1,
                                              uint8_t phase2,
                                              uint8_t phase3)
{
    const uint8_t order[IGH_250US_PIPELINE_PHASE_COUNT] = {
        phase0, phase1, phase2, phase3
    };
    uint8_t mask = 0U;

    if (config == NULL) return IGH_ECAN_INVALID_ARG;
    for (size_t phase = 0U; phase < IGH_250US_PIPELINE_PHASE_COUNT; phase++) {
        if (order[phase] >= IGH_250US_PIPELINE_PHASE_COUNT ||
            (mask & (uint8_t)(1U << order[phase])) != 0U) {
            return IGH_ECAN_INVALID_ARG;
        }
        mask = (uint8_t)(mask | (uint8_t)(1U << order[phase]));
    }
    memcpy(config->phase_channels, order, sizeof(order));
    return IGH_ECAN_OK;
}

static int configure_stack(pipeline_impl_t *impl)
{
    ecan_igh_config_t igh_config;
    igh_mcan_port_config_t mcan_config;
    const igh_250us_pipeline_config_t *cfg = &impl->config;

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (ecan_igh_config_apply_realtime_profile(
            &igh_config, ECAN_IGH_PROFILE_SM_250US_1MS) != IGH_ECAN_OK ||
        ecan_igh_config_set_slave_count(&igh_config, cfg->slave_count) != IGH_ECAN_OK ||
        ecan_igh_config_set_direction_pdo_multiplier(
            &igh_config, cfg->rxpdo_multiplier, cfg->txpdo_multiplier) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t slave = 0U; slave < cfg->slave_count; slave++) {
        if (igh_mcan_port_set_channel_mask(&mcan_config, slave,
                                           cfg->channel_mask,
                                           cfg->channel_mask) != IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(&mcan_config, slave,
                                      cfg->nominal_bitrate,
                                      cfg->data_bitrate) != IGH_ECAN_OK ||
            igh_mcan_port_set_sample_point_range(
                &mcan_config, slave,
                cfg->nominal_sample_point_x10, cfg->nominal_sample_point_x10,
                cfg->data_sample_point_x10, cfg->data_sample_point_x10) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    if (igh_mcan_port_apply_config(&igh_config, &mcan_config) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ecan_igh_init(&impl->igh, &igh_config) != IGH_ECAN_OK) return IGH_ECAN_FAIL;
    impl->igh_open = 1U;
    if (igh_mcan_port_init(&impl->mcan, &impl->igh, &mcan_config) != IGH_ECAN_OK ||
        ecan_igh_wait_op_timeout(&impl->igh, cfg->wait_op_ms) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    return IGH_ECAN_OK;
}

int igh_250us_pipeline_open(igh_250us_pipeline_t *pipeline,
                            const igh_250us_pipeline_config_t *config)
{
    pipeline_impl_t *impl;
    uint32_t warmup_cycles;
    uint32_t stable_cycles = 0U;
    int rc;

    last_open_stage = IGH_250US_PIPELINE_OPEN_NONE;
    memset(&last_open_diag, 0, sizeof(last_open_diag));
    if (pipeline == NULL || pipeline->impl != NULL || !config_valid(config)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (igh_rt_apply_runtime_tuning_checked(config->lock_memory,
                                            config->rt_priority,
                                            config->cpu) != 0) {
        last_open_stage = IGH_250US_PIPELINE_OPEN_RUNTIME_TUNING;
        capture_open_diag(NULL, IGH_ECAN_FAIL);
        return IGH_ECAN_FAIL;
    }
    impl = calloc(1U, sizeof(*impl));
    if (impl == NULL) return IGH_ECAN_FAIL;
    impl->config = *config;
    pipeline->impl = impl;
    rc = configure_stack(impl);
    if (rc != IGH_ECAN_OK) {
        last_open_stage = IGH_250US_PIPELINE_OPEN_STACK;
        capture_open_diag(impl, rc);
        igh_250us_pipeline_close(pipeline);
        return rc;
    }
    if (igh_rt_periodic_clock_start_checked(&impl->clock,
                                             IGH_250US_PIPELINE_CYCLE_US,
                                             config->spin_us, 0U) != 0) {
        last_open_stage = IGH_250US_PIPELINE_OPEN_CLOCK;
        capture_open_diag(impl, IGH_ECAN_FAIL);
        igh_250us_pipeline_close(pipeline);
        return IGH_ECAN_FAIL;
    }
    warmup_cycles = config->warmup_cycles;
    if (warmup_cycles == 0U) {
        warmup_cycles = 4000U;
    }
    for (uint32_t i = 0U; i < warmup_cycles; i++) {
        igh_mcan_rx_frame_t scratch[128];
        igh_mcan_port_warmup_result_t warmup;
        if ((impl->igh.dc_assign_activate == 0U ? igh_rt_monotonic_ns() :
             igh_rt_periodic_clock_wait(&impl->clock)) == 0U) {
            last_open_stage = IGH_250US_PIPELINE_OPEN_WARMUP;
            capture_open_diag(impl, IGH_ECAN_FAIL);
            igh_250us_pipeline_close(pipeline);
            return IGH_ECAN_FAIL;
        }
        memset(&warmup, 0, sizeof(warmup));
        rc = igh_mcan_port_warmup_cycle_checked(
            &impl->mcan, scratch, sizeof(scratch) / sizeof(scratch[0]), &warmup);
        if (rc != IGH_ECAN_OK) {
            last_open_stage = IGH_250US_PIPELINE_OPEN_WARMUP;
            capture_open_diag(impl, rc);
            igh_250us_pipeline_close(pipeline);
            return rc;
        }
        if (ecan_igh_domain_is_complete(&impl->igh) &&
            ecan_igh_slaves_are_operational(&impl->igh)) {
            stable_cycles++;
        } else {
            stable_cycles = 0U;
        }
    }
    if (stable_cycles < 100U || !ecan_igh_domain_is_complete(&impl->igh) ||
        !ecan_igh_slaves_are_operational(&impl->igh)) {
        last_open_stage = IGH_250US_PIPELINE_OPEN_NOT_OPERATIONAL;
        capture_open_diag(impl, IGH_ECAN_NOT_READY);
        igh_250us_pipeline_close(pipeline);
        return IGH_ECAN_NOT_READY;
    }
    if (ecan_igh_reset_runtime_diagnostics(&impl->igh) != IGH_ECAN_OK) {
        last_open_stage = IGH_250US_PIPELINE_OPEN_DIAGNOSTICS;
        capture_open_diag(impl, IGH_ECAN_FAIL);
        igh_250us_pipeline_close(pipeline);
        return IGH_ECAN_FAIL;
    }
    memset(&impl->stats, 0, sizeof(impl->stats));
    igh_rt_period_stats_reset(&impl->stats.period);
    if (igh_rt_periodic_clock_start_checked(&impl->clock,
                                            IGH_250US_PIPELINE_CYCLE_US,
                                            config->spin_us, 0U) != 0) {
        last_open_stage = IGH_250US_PIPELINE_OPEN_CLOCK;
        capture_open_diag(impl, IGH_ECAN_FAIL);
        igh_250us_pipeline_close(pipeline);
        return IGH_ECAN_FAIL;
    }
    return IGH_ECAN_OK;
}

int igh_250us_pipeline_cycle_begin(igh_250us_pipeline_t *pipeline,
                                   igh_mcan_rx_frame_t *rx_frames,
                                   size_t rx_capacity,
                                   igh_250us_pipeline_cycle_t *cycle)
{
    pipeline_impl_t *impl;
    size_t rx_count = 0U;
    uint64_t now_ns;
    uint64_t stage_ns = 0U;
    uint64_t elapsed_ns;
    int rc;

    if (pipeline == NULL || pipeline->impl == NULL || cycle == NULL ||
        (rx_capacity != 0U && rx_frames == NULL)) return IGH_ECAN_INVALID_ARG;
    impl = pipeline->impl;
    if (impl->cycle_active != 0U) return IGH_ECAN_NOT_READY;
    /* SM 的发送和输入交付均由适配器定时；不再叠加第二条周期时间线。
     * DC 保留既有相位等待。统计仍采样实际 begin，不清除迟到事件。 */
    now_ns = impl->igh.dc_assign_activate == 0U ? igh_rt_monotonic_ns() :
        igh_rt_periodic_clock_wait(&impl->clock);
    if (now_ns == 0U) return IGH_ECAN_FAIL;
    igh_rt_period_stats_sample(&impl->stats.period, now_ns, impl->clock.cycle_ns);
    if (impl->config.stage_timing != 0U) stage_ns = igh_rt_monotonic_ns();
    rc = igh_mcan_port_cycle_begin_checked(&impl->mcan);
    if (impl->config.stage_timing != 0U) {
        elapsed_ns = igh_rt_monotonic_ns() - stage_ns;
        impl->stats.transport_begin_total_ns += elapsed_ns;
        if (elapsed_ns > impl->stats.transport_begin_max_ns) {
            impl->stats.transport_begin_max_ns = elapsed_ns;
        }
    }
    if (rc != IGH_ECAN_OK) {
        impl->stats.receive_errors++;
        return rc;
    }
    impl->cycle_active = 1U;
    if (impl->config.stage_timing != 0U) stage_ns = igh_rt_monotonic_ns();
    rc = igh_mcan_port_receive(&impl->mcan, rx_frames, rx_capacity, &rx_count);
    if (impl->config.stage_timing != 0U) {
        elapsed_ns = igh_rt_monotonic_ns() - stage_ns;
        impl->stats.rx_parse_total_ns += elapsed_ns;
        if (elapsed_ns > impl->stats.rx_parse_max_ns) {
            impl->stats.rx_parse_max_ns = elapsed_ns;
        }
        impl->application_start_ns = igh_rt_monotonic_ns();
    }
    if (rc == IGH_ECAN_NO_SPACE) impl->stats.no_space_events++;
    else if (rc != IGH_ECAN_OK) impl->stats.receive_errors++;
    impl->active_channel = impl->config.phase_channels[
        impl->next_cycle % IGH_250US_PIPELINE_PHASE_COUNT];
    cycle->index = impl->next_cycle;
    cycle->wake_time_ns = now_ns;
    cycle->active_channel = impl->active_channel;
    cycle->rx_count = rx_count;
    impl->stats.rx_frames += rx_count;
    return rc;
}

int igh_250us_pipeline_send_canfd(igh_250us_pipeline_t *pipeline,
                                  int slave_pos, uint32_t can_id,
                                  int is_extended, int brs,
                                  const uint8_t *data, size_t data_len)
{
    pipeline_impl_t *impl;
    int rc;
    if (pipeline == NULL || pipeline->impl == NULL) return IGH_ECAN_INVALID_ARG;
    impl = pipeline->impl;
    if (impl->cycle_active == 0U) return IGH_ECAN_NOT_READY;
    rc = igh_mcan_port_send_canfd(&impl->mcan, slave_pos, impl->active_channel,
                                  can_id, is_extended, brs, data, data_len);
    if (rc != IGH_ECAN_OK) impl->stats.send_errors++;
    else impl->stats.tx_frames++;
    return rc;
}

int igh_250us_pipeline_send_can(igh_250us_pipeline_t *pipeline,
                                int slave_pos, uint32_t can_id,
                                int is_extended,
                                const uint8_t *data, size_t data_len)
{
    pipeline_impl_t *impl;
    int rc;
    if (pipeline == NULL || pipeline->impl == NULL) return IGH_ECAN_INVALID_ARG;
    impl = pipeline->impl;
    if (impl->cycle_active == 0U) return IGH_ECAN_NOT_READY;
    rc = igh_mcan_port_send_can(&impl->mcan, slave_pos, impl->active_channel,
                                can_id, is_extended, data, data_len);
    if (rc != IGH_ECAN_OK) impl->stats.send_errors++;
    else impl->stats.tx_frames++;
    return rc;
}

int igh_250us_pipeline_cycle_end(igh_250us_pipeline_t *pipeline)
{
    pipeline_impl_t *impl;
    uint64_t stage_ns = 0U;
    uint64_t elapsed_ns;
    int rc;
    if (pipeline == NULL || pipeline->impl == NULL) return IGH_ECAN_INVALID_ARG;
    impl = pipeline->impl;
    if (impl->cycle_active == 0U) return IGH_ECAN_NOT_READY;
    if (impl->config.stage_timing != 0U) {
        stage_ns = igh_rt_monotonic_ns();
        elapsed_ns = stage_ns - impl->application_start_ns;
        impl->stats.application_total_ns += elapsed_ns;
        if (elapsed_ns > impl->stats.application_max_ns) {
            impl->stats.application_max_ns = elapsed_ns;
        }
    }
    rc = igh_mcan_port_cycle_end_checked(&impl->mcan);
    if (impl->config.stage_timing != 0U) {
        elapsed_ns = igh_rt_monotonic_ns() - stage_ns;
        impl->stats.publish_total_ns += elapsed_ns;
        if (elapsed_ns > impl->stats.publish_max_ns) {
            impl->stats.publish_max_ns = elapsed_ns;
        }
        impl->stats.stage_samples++;
    }
    impl->cycle_active = 0U;
    if (rc != IGH_ECAN_OK) {
        impl->stats.send_errors++;
        if (impl->mcan.cycle_active != 0U) {
            (void)igh_mcan_port_cycle_abort_checked(&impl->mcan);
        }
    }
    else {
        impl->stats.cycles++;
        impl->next_cycle++;
    }
    return rc;
}

int igh_250us_pipeline_cycle_abort(igh_250us_pipeline_t *pipeline)
{
    pipeline_impl_t *impl;
    int rc;

    if (pipeline == NULL || pipeline->impl == NULL) return IGH_ECAN_INVALID_ARG;
    impl = pipeline->impl;
    if (impl->cycle_active == 0U) return IGH_ECAN_NOT_READY;
    rc = igh_mcan_port_cycle_abort_checked(&impl->mcan);
    impl->cycle_active = 0U;
    return rc;
}

int igh_250us_pipeline_get_stats(const igh_250us_pipeline_t *pipeline,
                                 igh_250us_pipeline_stats_t *stats)
{
    const pipeline_impl_t *impl;
    if (pipeline == NULL || pipeline->impl == NULL || stats == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    impl = pipeline->impl;
    *stats = impl->stats;
    stats->clock_resyncs = impl->clock.resyncs;
    return ecan_igh_get_sched_stats(&impl->igh, &stats->ethercat);
}

void igh_250us_pipeline_close(igh_250us_pipeline_t *pipeline)
{
    pipeline_impl_t *impl;
    if (pipeline == NULL || pipeline->impl == NULL) return;
    impl = pipeline->impl;
    if (impl->cycle_active != 0U || impl->mcan.cycle_active != 0U) {
        (void)igh_mcan_port_cycle_abort_checked(&impl->mcan);
    }
    if (impl->igh_open != 0U) ecan_igh_close(&impl->igh);
    free(impl);
    pipeline->impl = NULL;
}
