/* SPDX-License-Identifier: MIT */
#ifndef IGH_250US_PIPELINE_H
#define IGH_250US_PIPELINE_H

#include <stddef.h>
#include <stdint.h>

#include "ecan_realtime_profile.h"
#include "igh_mcan_port.h"
#include "igh_rt_support.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IGH_250US_PIPELINE_CYCLE_US       ECAN_RT_SM_ECAT_CYCLE_US
#define IGH_250US_PIPELINE_PHASE_COUNT    ECAN_RT_SM_PHASE_COUNT
#define IGH_250US_PIPELINE_CHANNEL_MASK   0x0FU
#define IGH_250US_PIPELINE_CONTROL_US     ECAN_RT_SM_CONTROL_CYCLE_US

typedef enum {
    IGH_250US_PIPELINE_SM = 0,
    /* 兼容旧源码；250 us DC 未进入产品 profile，open() 会拒绝该值。 */
    IGH_250US_PIPELINE_DC = 1
} igh_250us_pipeline_mode_t;

typedef enum {
    IGH_250US_PIPELINE_OPEN_NONE = 0,
    IGH_250US_PIPELINE_OPEN_RUNTIME_TUNING,
    IGH_250US_PIPELINE_OPEN_STACK,
    IGH_250US_PIPELINE_OPEN_CLOCK,
    IGH_250US_PIPELINE_OPEN_WARMUP,
    IGH_250US_PIPELINE_OPEN_NOT_OPERATIONAL,
    IGH_250US_PIPELINE_OPEN_DIAGNOSTICS
} igh_250us_pipeline_open_stage_t;

typedef struct {
    size_t slave_count;
    uint8_t channel_mask;
    /* Channel serviced by each 250 us phase; must be a permutation of 0..3. */
    uint8_t phase_channels[IGH_250US_PIPELINE_PHASE_COUNT];
    uint8_t rxpdo_multiplier;
    uint8_t txpdo_multiplier;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t nominal_sample_point_x10;
    uint16_t data_sample_point_x10;
    igh_250us_pipeline_mode_t mode;
    int32_t dc_sync0_shift_us; /* 已弃用，仅保留结构体源码兼容。 */
    uint32_t wait_op_ms;
    uint32_t warmup_cycles;
    uint32_t spin_us;
    uint8_t lock_memory;
    /* Optional profiling; disabled by default to keep the hot path minimal. */
    uint8_t stage_timing;
    int rt_priority;
    int cpu;
} igh_250us_pipeline_config_t;

typedef struct {
    uint64_t index;
    uint64_t wake_time_ns;
    uint8_t active_channel;
    size_t rx_count;
} igh_250us_pipeline_cycle_t;

typedef struct {
    uint64_t cycles;
    uint64_t rx_frames;
    uint64_t tx_frames;
    uint64_t receive_errors;
    uint64_t send_errors;
    uint64_t no_space_events;
    uint32_t clock_resyncs;
    uint64_t stage_samples;
    uint64_t transport_begin_total_ns;
    uint64_t transport_begin_max_ns;
    uint64_t rx_parse_total_ns;
    uint64_t rx_parse_max_ns;
    uint64_t application_total_ns;
    uint64_t application_max_ns;
    uint64_t publish_total_ns;
    uint64_t publish_max_ns;
    igh_rt_period_stats_t period;
    ecan_igh_sched_stats_t ethercat;
} igh_250us_pipeline_stats_t;

typedef struct {
    igh_250us_pipeline_open_stage_t stage;
    int result;
    uint8_t domain_complete;
    uint8_t all_operational;
    unsigned int working_counter;
    unsigned int expected_working_counter;
    size_t slave_count;
    unsigned int slave_al_state[ECAN_IGH_MAX_SLAVES];
    ecan_igh_sched_stats_t ethercat;
    ecan_igh_diag_event_t last_event;
    uint8_t last_event_valid;
} igh_250us_pipeline_open_diag_t;

/* Opaque: applications do not depend on IgH domain/PDO implementation details. */
typedef struct {
    void *impl;
} igh_250us_pipeline_t;

#define IGH_250US_PIPELINE_INITIALIZER { NULL }

void igh_250us_pipeline_default_config(igh_250us_pipeline_config_t *config);
int igh_250us_pipeline_config_set_phase_order(igh_250us_pipeline_config_t *config,
                                              uint8_t phase0,
                                              uint8_t phase1,
                                              uint8_t phase2,
                                              uint8_t phase3);

/* Last process-start open result; useful before a pipeline context exists. */
igh_250us_pipeline_open_stage_t igh_250us_pipeline_last_open_stage(void);
const char *igh_250us_pipeline_open_stage_name(igh_250us_pipeline_open_stage_t stage);
void igh_250us_pipeline_get_last_open_diag(igh_250us_pipeline_open_diag_t *diag);

int igh_250us_pipeline_open(igh_250us_pipeline_t *pipeline,
                            const igh_250us_pipeline_config_t *config);

/* Wait for the next 250 us boundary, receive PDO data, and select MCAN0..3. */
int igh_250us_pipeline_cycle_begin(igh_250us_pipeline_t *pipeline,
                                   igh_mcan_rx_frame_t *rx_frames,
                                   size_t rx_capacity,
                                   igh_250us_pipeline_cycle_t *cycle);

/* Send on this cycle's active channel. The channel cannot be accidentally mixed. */
int igh_250us_pipeline_send_canfd(igh_250us_pipeline_t *pipeline,
                                  int slave_pos,
                                  uint32_t can_id,
                                  int is_extended,
                                  int brs,
                                  const uint8_t *data,
                                  size_t data_len);

int igh_250us_pipeline_send_can(igh_250us_pipeline_t *pipeline,
                                int slave_pos,
                                uint32_t can_id,
                                int is_extended,
                                const uint8_t *data,
                                size_t data_len);

int igh_250us_pipeline_cycle_end(igh_250us_pipeline_t *pipeline);
/* Discard an active cycle after an application-side error. */
int igh_250us_pipeline_cycle_abort(igh_250us_pipeline_t *pipeline);
int igh_250us_pipeline_get_stats(const igh_250us_pipeline_t *pipeline,
                                 igh_250us_pipeline_stats_t *stats);
void igh_250us_pipeline_close(igh_250us_pipeline_t *pipeline);

#ifdef __cplusplus
}
#endif

#endif /* IGH_250US_PIPELINE_H */
