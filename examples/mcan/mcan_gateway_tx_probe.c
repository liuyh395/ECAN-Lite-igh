#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include "../common/igh_example_args.h"
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "igh_ecan_common.h"
#include "igh_mcan_port.h"
#include "igh_rt_support.h"

#define WARMUP_RX_CAPACITY 128U

typedef struct {
    int slave_pos;
    uint8_t channel;
    uint32_t can_id;
    uint32_t count;
    uint32_t cycle_us;
    uint32_t wait_op_ms;
    uint32_t warmup_cycles;
    uint32_t warmup_stable_cycles;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint16_t nominal_sp_min_x10;
    uint16_t nominal_sp_max_x10;
    uint16_t data_sp_min_x10;
    uint16_t data_sp_max_x10;
    uint16_t nom_brp;
    uint8_t nom_tseg1;
    uint8_t nom_tseg2;
    uint8_t nom_sjw;
    uint8_t data_brp;
    uint8_t data_tseg1;
    uint8_t data_tseg2;
    uint8_t data_sjw;
    uint8_t lowlevel_timing;
    uint8_t pdo_multiplier;
    uint8_t upstream_pdo_multiplier;
    uint8_t data_len;
    uint8_t data_len_set;
    uint8_t canfd_frame;
    uint8_t gateway_canfd;
    uint8_t brs;
    uint8_t is_extended;
    uint8_t scheduled_tx;
    uint32_t schedule_offset_ns;
    uint32_t schedule_offset_step_ns;
    uint8_t dc_sync0;
    uint32_t dc_cycle_us;
    int32_t dc_shift_us;
    uint8_t lock_memory;
    int rt_priority;
    int cpu;
    uint8_t catch_up;
    uint32_t spin_us;
} tx_probe_config_t;

typedef struct {
    uint64_t next_ns;
    uint64_t cycle_ns;
    uint64_t spin_ns;
    uint8_t catch_up;
    uint32_t resyncs;
} periodic_clock_t;

typedef struct {
    uint32_t samples;
    uint64_t min_ns;
    uint64_t max_ns;
    uint64_t sum_ns;
    uint64_t max_abs_err_ns;
    int64_t max_late_ns;
    int64_t max_early_ns;
    uint64_t last_ns;
} period_stats_t;

static uint64_t monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec;
}

static void ns_to_timespec(uint64_t ns, struct timespec *ts)
{
    ts->tv_sec = (time_t)(ns / 1000000000ULL);
    ts->tv_nsec = (long)(ns % 1000000000ULL);
}

static void periodic_clock_start(periodic_clock_t *clock,
                                 uint32_t cycle_us,
                                 uint32_t spin_us,
                                 uint8_t catch_up)
{
    clock->cycle_ns = (uint64_t)cycle_us * 1000ULL;
    clock->spin_ns = (uint64_t)spin_us * 1000ULL;
    if (clock->spin_ns > (clock->cycle_ns / 2ULL)) {
        clock->spin_ns = clock->cycle_ns / 2ULL;
    }
    clock->next_ns = monotonic_ns() + clock->cycle_ns;
    clock->catch_up = catch_up;
    clock->resyncs = 0U;
}

static uint64_t periodic_clock_wait(periodic_clock_t *clock)
{
    struct timespec ts;
    uint64_t deadline_ns = clock->next_ns;
    uint64_t now_ns;
    int rc;

    if (clock->spin_ns != 0U && deadline_ns > clock->spin_ns) {
        uint64_t sleep_until_ns = deadline_ns - clock->spin_ns;

        ns_to_timespec(sleep_until_ns, &ts);
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } while (rc == EINTR);
        while ((now_ns = monotonic_ns()) < deadline_ns) {
            /* short busy wait to reduce scheduler wake-up jitter */
        }
    } else {
        ns_to_timespec(deadline_ns, &ts);
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } while (rc == EINTR);
        now_ns = monotonic_ns();
    }

    if (clock->catch_up == 0U &&
        now_ns > deadline_ns &&
        (now_ns - deadline_ns) > (clock->cycle_ns / 2ULL)) {
        clock->next_ns = now_ns + clock->cycle_ns;
        clock->resyncs++;
    } else {
        clock->next_ns = deadline_ns + clock->cycle_ns;
    }

    return now_ns;
}

static void period_stats_reset(period_stats_t *stats)
{
    memset(stats, 0, sizeof(*stats));
}

static void period_stats_sample(period_stats_t *stats, uint64_t now_ns, uint64_t target_ns)
{
    uint64_t period_ns;
    int64_t err_ns;
    uint64_t abs_err_ns;

    if (stats->last_ns == 0U) {
        stats->last_ns = now_ns;
        return;
    }

    period_ns = now_ns - stats->last_ns;
    stats->last_ns = now_ns;
    err_ns = (int64_t)period_ns - (int64_t)target_ns;
    abs_err_ns = (err_ns < 0) ? (uint64_t)(-err_ns) : (uint64_t)err_ns;

    if (stats->samples == 0U || period_ns < stats->min_ns) {
        stats->min_ns = period_ns;
    }
    if (period_ns > stats->max_ns) {
        stats->max_ns = period_ns;
    }
    if (abs_err_ns > stats->max_abs_err_ns) {
        stats->max_abs_err_ns = abs_err_ns;
    }
    if (err_ns > stats->max_late_ns) {
        stats->max_late_ns = err_ns;
    }
    if (err_ns < 0 && -err_ns > stats->max_early_ns) {
        stats->max_early_ns = -err_ns;
    }
    stats->sum_ns += period_ns;
    stats->samples++;
}

static void print_period_stats(const period_stats_t *stats, uint64_t target_ns)
{
    double avg_us;

    if (stats->samples == 0U) {
        printf("master_period target_us=%.3f samples=0\n", (double)target_ns / 1000.0);
        return;
    }

    avg_us = ((double)stats->sum_ns / (double)stats->samples) / 1000.0;
    printf("master_period target_us=%.3f samples=%" PRIu32
           " min_us=%.3f max_us=%.3f avg_us=%.3f"
           " max_abs_err_us=%.3f max_late_us=%.3f max_early_us=%.3f\n",
           (double)target_ns / 1000.0,
           stats->samples,
           (double)stats->min_ns / 1000.0,
           (double)stats->max_ns / 1000.0,
           avg_us,
           (double)stats->max_abs_err_ns / 1000.0,
           (double)stats->max_late_ns / 1000.0,
           (double)stats->max_early_ns / 1000.0);
}

static void default_config(tx_probe_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->slave_pos = 0;
    cfg->channel = IGH_ECAN_MCAN_CH0;
    cfg->can_id = 0x123U;
    cfg->count = 100U;
    cfg->cycle_us = IGH_ECAN_DEFAULT_CYCLE_US;
    cfg->wait_op_ms = 10000U;
    cfg->warmup_cycles = 5U;
    cfg->warmup_stable_cycles = 4U;
    cfg->nominal_bitrate = IGH_ECAN_DEFAULT_CAN_BITRATE;
    cfg->data_bitrate = IGH_ECAN_DEFAULT_CANFD_BITRATE;
    cfg->nominal_sp_min_x10 = IGH_ECAN_DEFAULT_CAN20_SP_MIN_X10;
    cfg->nominal_sp_max_x10 = IGH_ECAN_DEFAULT_CAN20_SP_MAX_X10;
    cfg->data_sp_min_x10 = IGH_ECAN_DEFAULT_CANFD_SP_MIN_X10;
    cfg->data_sp_max_x10 = IGH_ECAN_DEFAULT_CANFD_SP_MAX_X10;
    cfg->nom_brp = IGH_ECAN_DEFAULT_NOM_BRP;
    cfg->nom_tseg1 = IGH_ECAN_DEFAULT_NOM_TSEG1;
    cfg->nom_tseg2 = IGH_ECAN_DEFAULT_NOM_TSEG2;
    cfg->nom_sjw = IGH_ECAN_DEFAULT_NOM_SJW;
    cfg->data_brp = IGH_ECAN_DEFAULT_DATA_BRP;
    cfg->data_tseg1 = IGH_ECAN_DEFAULT_DATA_TSEG1;
    cfg->data_tseg2 = IGH_ECAN_DEFAULT_DATA_TSEG2;
    cfg->data_sjw = IGH_ECAN_DEFAULT_DATA_SJW;
    cfg->pdo_multiplier = IGH_ECAN_DEFAULT_PDO_MULTIPLIER;
    cfg->upstream_pdo_multiplier = 0U;
    cfg->data_len = 8U;
    cfg->canfd_frame = 0U;
    cfg->gateway_canfd = 0U;
    cfg->brs = 1U;
    cfg->is_extended = 0U;
    cfg->scheduled_tx = 0U;
    cfg->schedule_offset_ns = 100000U;
    cfg->schedule_offset_step_ns = 0U;
    cfg->dc_sync0 = 0U;
    cfg->dc_cycle_us = 0U;
    cfg->dc_shift_us = 0;
    cfg->lock_memory = 1U;
    cfg->rt_priority = 0;
    cfg->cpu = -1;
    cfg->catch_up = 0U;
    cfg->spin_us = 0U;
}

static void print_usage(const char *app)
{
    printf("usage: %s [--slave N] [--channel 0..3] [--id N] [--ext]\n", app);
    printf("          [--classic-can] [--canfd] [--brs] [--no-brs] [--len N]\n");
    printf("          [--gateway-canfd] [--gateway-classic]\n");
    printf("          [--count N] [--cycle-us N] [--wait-op-ms N]\n");
    printf("          [--warmup-cycles N] [--warmup-stable-cycles N]\n");
    printf("          [--bitrate N] [--data-bitrate N] [--pdo-n 5+3-encoding]\n");
    printf("          [--upstream-pdo-n 5+3-encoding]\n");
    printf("          [--nom-sp-min-x10 N] [--nom-sp-max-x10 N]\n");
    printf("          [--data-sp-min-x10 N] [--data-sp-max-x10 N]\n");
    printf("          [--lowlevel-timing]\n");
    printf("          [--nom-brp N] [--nom-tseg1 N] [--nom-tseg2 N] [--nom-sjw N]\n");
    printf("          [--data-brp N] [--data-tseg1 N] [--data-tseg2 N] [--data-sjw N]\n");
    printf("          [--scheduled-tx] [--schedule-offset-ns N] [--schedule-offset-step-ns N]\n");
    printf("          [--dc-sync0] [--dc-cycle-us N] [--dc-shift-us N]\n");
    printf("          [--rt-priority N] [--cpu N] [--spin-us N] [--no-mlock] [--catch-up]\n");
    printf("payload rule: byte[j] = (frame_index + j) & 0xFF\n");
}

static int parse_args(int argc, char **argv, tx_probe_config_t *cfg)
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc) {
            cfg->slave_pos = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--channel") == 0 && i + 1 < argc) {
            cfg->channel = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--id") == 0 && i + 1 < argc) {
            cfg->can_id = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--ext") == 0) {
            cfg->is_extended = 1U;
        } else if (strcmp(argv[i], "--classic-can") == 0) {
            cfg->canfd_frame = 0U;
            if (cfg->data_len_set == 0U) {
                cfg->data_len = 8U;
            }
        } else if (strcmp(argv[i], "--canfd") == 0) {
            cfg->canfd_frame = 1U;
            cfg->gateway_canfd = 1U;
            if (cfg->data_len_set == 0U) {
                cfg->data_len = 64U;
            }
        } else if (strcmp(argv[i], "--gateway-canfd") == 0) {
            cfg->gateway_canfd = 1U;
        } else if (strcmp(argv[i], "--gateway-classic") == 0) {
            cfg->gateway_canfd = 0U;
        } else if (strcmp(argv[i], "--brs") == 0) {
            cfg->brs = 1U;
        } else if (strcmp(argv[i], "--no-brs") == 0) {
            cfg->brs = 0U;
        } else if (strcmp(argv[i], "--len") == 0 && i + 1 < argc) {
            cfg->data_len = igh_example_u8(argv[++i]);
            cfg->data_len_set = 1U;
        } else if (strcmp(argv[i], "--count") == 0 && i + 1 < argc) {
            cfg->count = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--cycle-us") == 0 && i + 1 < argc) {
            cfg->cycle_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--wait-op-ms") == 0 && i + 1 < argc) {
            cfg->wait_op_ms = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--warmup-cycles") == 0 && i + 1 < argc) {
            cfg->warmup_cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--warmup-stable-cycles") == 0 && i + 1 < argc) {
            cfg->warmup_stable_cycles = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc) {
            cfg->nominal_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--data-bitrate") == 0 && i + 1 < argc) {
            cfg->data_bitrate = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--nom-sp-min-x10") == 0 && i + 1 < argc) {
            cfg->nominal_sp_min_x10 = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--nom-sp-max-x10") == 0 && i + 1 < argc) {
            cfg->nominal_sp_max_x10 = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--data-sp-min-x10") == 0 && i + 1 < argc) {
            cfg->data_sp_min_x10 = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--data-sp-max-x10") == 0 && i + 1 < argc) {
            cfg->data_sp_max_x10 = igh_example_u16(argv[++i]);
        } else if (strcmp(argv[i], "--lowlevel-timing") == 0) {
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--nom-brp") == 0 && i + 1 < argc) {
            cfg->nom_brp = igh_example_u16(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--nom-tseg1") == 0 && i + 1 < argc) {
            cfg->nom_tseg1 = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--nom-tseg2") == 0 && i + 1 < argc) {
            cfg->nom_tseg2 = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--nom-sjw") == 0 && i + 1 < argc) {
            cfg->nom_sjw = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--data-brp") == 0 && i + 1 < argc) {
            cfg->data_brp = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--data-tseg1") == 0 && i + 1 < argc) {
            cfg->data_tseg1 = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--data-tseg2") == 0 && i + 1 < argc) {
            cfg->data_tseg2 = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--data-sjw") == 0 && i + 1 < argc) {
            cfg->data_sjw = igh_example_u8(argv[++i]);
            cfg->lowlevel_timing = 1U;
        } else if (strcmp(argv[i], "--pdo-n") == 0 && i + 1 < argc) {
            cfg->pdo_multiplier = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--upstream-pdo-n") == 0 && i + 1 < argc) {
            cfg->upstream_pdo_multiplier = igh_example_u8(argv[++i]);
        } else if (strcmp(argv[i], "--scheduled-tx") == 0) {
            cfg->scheduled_tx = 1U;
        } else if (strcmp(argv[i], "--schedule-offset-ns") == 0 && i + 1 < argc) {
            cfg->schedule_offset_ns = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--schedule-offset-step-ns") == 0 && i + 1 < argc) {
            cfg->schedule_offset_step_ns = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--dc-sync0") == 0) {
            cfg->dc_sync0 = 1U;
        } else if (strcmp(argv[i], "--dc-cycle-us") == 0 && i + 1 < argc) {
            cfg->dc_cycle_us = igh_example_u32(argv[++i]);
            cfg->dc_sync0 = 1U;
        } else if (strcmp(argv[i], "--dc-shift-us") == 0 && i + 1 < argc) {
            cfg->dc_shift_us = igh_example_i32(argv[++i]);
            cfg->dc_sync0 = 1U;
        } else if (strcmp(argv[i], "--rt-priority") == 0 && i + 1 < argc) {
            cfg->rt_priority = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--realtime") == 0) {
            cfg->rt_priority = 80;
        } else if (strcmp(argv[i], "--cpu") == 0 && i + 1 < argc) {
            cfg->cpu = igh_example_int(argv[++i]);
        } else if (strcmp(argv[i], "--spin-us") == 0 && i + 1 < argc) {
            cfg->spin_us = igh_example_u32(argv[++i]);
        } else if (strcmp(argv[i], "--no-mlock") == 0) {
            cfg->lock_memory = 0U;
        } else if (strcmp(argv[i], "--catch-up") == 0) {
            cfg->catch_up = 1U;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            exit(0);
        } else {
            return IGH_ECAN_INVALID_ARG;
        }
    }

    if (cfg->slave_pos < 0 ||
        cfg->channel >= IGH_MCAN_CHANNEL_COUNT ||
        cfg->count == 0U ||
        cfg->cycle_us == 0U ||
        (cfg->warmup_cycles != 0U && cfg->warmup_stable_cycles > cfg->warmup_cycles) ||
        cfg->nominal_bitrate == 0U ||
        cfg->data_bitrate == 0U ||
        cfg->nominal_sp_min_x10 == 0U ||
        cfg->nominal_sp_max_x10 == 0U ||
        cfg->data_sp_min_x10 == 0U ||
        cfg->data_sp_max_x10 == 0U ||
        cfg->nominal_sp_min_x10 > cfg->nominal_sp_max_x10 ||
        cfg->data_sp_min_x10 > cfg->data_sp_max_x10 ||
        cfg->nominal_sp_max_x10 > 1000U ||
        cfg->data_sp_max_x10 > 1000U ||
        cfg->nom_brp == 0U ||
        cfg->nom_tseg1 == 0U ||
        cfg->nom_tseg2 == 0U ||
        cfg->nom_sjw == 0U ||
        cfg->data_brp == 0U ||
        cfg->data_tseg1 == 0U ||
        cfg->data_tseg2 == 0U ||
        cfg->data_sjw == 0U ||
        cfg->rt_priority < 0 ||
        cfg->rt_priority > 99 ||
        !ecan_igh_pdo_size_encoding_is_valid(cfg->pdo_multiplier) ||
        (cfg->upstream_pdo_multiplier != 0U &&
         !ecan_igh_pdo_size_encoding_is_valid(cfg->upstream_pdo_multiplier)) ||
        (cfg->is_extended == 0U && cfg->can_id > IGH_ECAN_CAN_STD_ID_MAX) ||
        (cfg->is_extended != 0U && cfg->can_id > IGH_ECAN_CAN_EXT_ID_MAX) ||
        (cfg->canfd_frame == 0U && cfg->data_len > IGH_ECAN_CLASSIC_CAN_MAX_DLEN) ||
        (cfg->canfd_frame != 0U && cfg->data_len > IGH_ECAN_CANFD_MAX_DLEN)) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (cfg->scheduled_tx != 0U && cfg->dc_sync0 == 0U) {
        cfg->dc_sync0 = 1U;
    }
    if (cfg->canfd_frame != 0U) {
        cfg->gateway_canfd = 1U;
    }
    return IGH_ECAN_OK;
}

static int apply_runtime_tuning(const tx_probe_config_t *probe)
{
    return igh_rt_apply_runtime_tuning_checked(probe->lock_memory,
                                               probe->rt_priority,
                                               probe->cpu);
}

static int should_fail_on_diag_events(void)
{
    const char *value = getenv("ECAN_IGH_FAIL_ON_DIAG_EVENT");

    return (value != NULL && strcmp(value, "0") != 0) ? 1 : 0;
}

static int apply_config(ecan_igh_config_t *igh_config,
                        igh_mcan_port_config_t *mcan_config,
                        const tx_probe_config_t *probe)
{
    size_t slave_count = (size_t)probe->slave_pos + 1U;
    uint8_t channel_bit = IGH_ECAN_BIT_U8(probe->channel);
    uint8_t canfd_mask = (probe->canfd_frame || probe->gateway_canfd) ? channel_bit : 0U;
    uint8_t upstream_multiplier = (probe->upstream_pdo_multiplier != 0U) ?
                                  probe->upstream_pdo_multiplier :
                                  probe->pdo_multiplier;
    int rc;

    if (slave_count > ECAN_IGH_MAX_SLAVES ||
        ecan_igh_config_set_slave_count(igh_config, slave_count) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(igh_config, probe->cycle_us) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(igh_config, upstream_multiplier) !=
            IGH_ECAN_OK ||
        ecan_igh_config_set_slave_pdo_multiplier(igh_config,
                                                 (size_t)probe->slave_pos,
                                                 probe->pdo_multiplier) !=
            IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (probe->dc_sync0 != 0U &&
        ecan_igh_config_enable_dc_sync0(igh_config,
                                        probe->dc_cycle_us,
                                        probe->dc_shift_us) != IGH_ECAN_OK) {
        return IGH_ECAN_INVALID_ARG;
    }
    for (size_t slave = 0U; slave < slave_count; slave++) {
        if (igh_mcan_port_set_channel_mask(mcan_config, slave, channel_bit, canfd_mask) !=
                IGH_ECAN_OK ||
            igh_mcan_port_set_bitrate(mcan_config,
                                      slave,
                                      probe->nominal_bitrate,
                                      probe->data_bitrate) != IGH_ECAN_OK ||
            igh_mcan_port_set_sample_point_range(mcan_config,
                                                 slave,
                                                 probe->nominal_sp_min_x10,
                                                 probe->nominal_sp_max_x10,
                                                 probe->data_sp_min_x10,
                                                 probe->data_sp_max_x10) != IGH_ECAN_OK) {
            return IGH_ECAN_INVALID_ARG;
        }
    }
    rc = igh_mcan_port_apply_config(igh_config, mcan_config);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    if (probe->lowlevel_timing != 0U) {
        uint16_t index = (uint16_t)(IGH_MCAN_OD_CHANNEL_BASE + probe->channel);

        for (size_t slave = 0U; slave < slave_count; slave++) {
            if (ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_TIMING_MODE,
                                         IGH_MCAN_BIT_TIMING_MANUAL) != IGH_ECAN_OK ||
                ecan_igh_config_add_sdo16(igh_config, slave, index,
                                          IGH_MCAN_OD_CHANNEL_NOM_BRP, probe->nom_brp) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_NOM_TSEG1, probe->nom_tseg1) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_NOM_TSEG2, probe->nom_tseg2) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_NOM_SJW, probe->nom_sjw) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_DATA_BRP, probe->data_brp) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_DATA_TSEG1, probe->data_tseg1) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_DATA_TSEG2, probe->data_tseg2) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_DATA_SJW, probe->data_sjw) !=
                    IGH_ECAN_OK ||
                ecan_igh_config_add_sdo8(igh_config, slave, index,
                                         IGH_MCAN_OD_CHANNEL_APPLY,
                                         IGH_ECAN_OD_APPLY_COMMAND) != IGH_ECAN_OK) {
                return IGH_ECAN_INVALID_ARG;
            }
        }
    }

    return IGH_ECAN_OK;
}

static void fill_payload(uint8_t *data, size_t len, uint32_t index)
{
    for (size_t j = 0U; j < len; j++) {
        data[j] = (uint8_t)(index + (uint32_t)j);
    }
}

int main(int argc, char **argv)
{
    tx_probe_config_t probe;
    ecan_igh_config_t igh_config;
    ecan_igh_context_t igh;
    igh_mcan_port_config_t mcan_config;
    igh_mcan_port_t mcan;
    periodic_clock_t clock;
    period_stats_t period_stats;
    uint32_t sent = 0U;
    uint32_t failed = 0U;
    int rc;

    default_config(&probe);
    if (parse_args(argc, argv, &probe) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }
    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (apply_config(&igh_config, &mcan_config, &probe) != IGH_ECAN_OK) {
        print_usage(argv[0]);
        return IGH_ECAN_EXIT_ARG_ERROR;
    }

    printf("MCAN gateway TX probe: slave=%d ch=%u id=0x%08" PRIX32
           " ext=%u frame_canfd=%u gateway_canfd=%u brs=%u len=%u count=%" PRIu32
           " warmup=%" PRIu32 "/%" PRIu32
           " bitrate=%" PRIu32 " data=%" PRIu32 " pdo=%uB upstream_pdo=%uB"
           " sp_nom=%u..%u sp_data=%u..%u"
           " lowlevel=%u nom=%u/%u/%u/%u data_timing=%u/%u/%u/%u"
           " scheduled=%u offset_ns=%" PRIu32 " offset_step_ns=%" PRIu32
           " dc=%u rt_priority=%d cpu=%d spin_us=%" PRIu32 " catch_up=%u\n",
           probe.slave_pos,
           probe.channel,
           probe.can_id,
           probe.is_extended,
           probe.canfd_frame,
           probe.gateway_canfd,
           probe.brs,
           probe.data_len,
           probe.count,
           probe.warmup_cycles,
           probe.warmup_stable_cycles,
           probe.nominal_bitrate,
           probe.data_bitrate,
           (unsigned)ecan_igh_pdo_decode_bytes(probe.pdo_multiplier),
           (unsigned)ecan_igh_pdo_decode_bytes(
               (probe.upstream_pdo_multiplier != 0U) ?
               probe.upstream_pdo_multiplier : probe.pdo_multiplier),
           probe.nominal_sp_min_x10,
           probe.nominal_sp_max_x10,
           probe.data_sp_min_x10,
           probe.data_sp_max_x10,
           probe.lowlevel_timing,
           probe.nom_brp,
           probe.nom_tseg1,
           probe.nom_tseg2,
           probe.nom_sjw,
           probe.data_brp,
           probe.data_tseg1,
           probe.data_tseg2,
           probe.data_sjw,
           probe.scheduled_tx,
           probe.schedule_offset_ns,
           probe.schedule_offset_step_ns,
           probe.dc_sync0,
           probe.rt_priority,
           probe.cpu,
           probe.spin_us,
           probe.catch_up);

    if (apply_runtime_tuning(&probe) != 0) {
        fprintf(stderr, "requested real-time tuning could not be applied\n");
        return IGH_ECAN_EXIT_FAIL;
    }

    rc = ecan_igh_init(&igh, &igh_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "IgH init failed: %d\n", rc);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = igh_mcan_port_init(&mcan, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "MCAN port init failed: %d\n", rc);
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    rc = ecan_igh_wait_op_timeout(&igh, probe.wait_op_ms);
    if (rc != IGH_ECAN_OK) {
        fprintf(stderr, "wait OP failed: %d\n", rc);
        if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
            ecan_igh_print_runtime_diagnostics(&igh, 16U);
        }
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }

    if (probe.warmup_cycles != 0U) {
        igh_mcan_port_warmup_result_t warmup;

        memset(&warmup, 0, sizeof(warmup));
        periodic_clock_start(&clock, probe.cycle_us, probe.spin_us, probe.catch_up);
        for (uint32_t i = 0U; i < probe.warmup_cycles; i++) {
            igh_mcan_rx_frame_t rx[WARMUP_RX_CAPACITY];

            (void)periodic_clock_wait(&clock);
            rc = igh_mcan_port_warmup_cycle_checked(&mcan, rx, WARMUP_RX_CAPACITY, &warmup);
            if (rc != IGH_ECAN_OK) {
                fprintf(stderr, "[warmup] tx-probe failed cycle=%" PRIu32
                        " rc=%d completed=%" PRIu32 " stable=%" PRIu32
                        " last_rx=%zu max_rx=%zu begin_err=%" PRIu32
                        " rx_err=%" PRIu32 " no_space=%" PRIu32
                        " end_err=%" PRIu32 "\n",
                        i,
                        rc,
                        warmup.cycles_completed,
                        warmup.consecutive_stable_cycles,
                        warmup.last_rx_count,
                        warmup.max_rx_count,
                        warmup.begin_errors,
                        warmup.receive_errors,
                        warmup.no_space_events,
                        warmup.end_errors);
                ecan_igh_close(&igh);
                return IGH_ECAN_EXIT_FAIL;
            }
        }
        printf("[warmup] tx-probe cycles=%" PRIu32 " stable=%" PRIu32
               " last_rx=%zu max_rx=%zu begin_err=%" PRIu32
               " rx_err=%" PRIu32 " no_space=%" PRIu32
               " end_err=%" PRIu32 "\n",
               warmup.cycles_completed,
               warmup.consecutive_stable_cycles,
               warmup.last_rx_count,
               warmup.max_rx_count,
               warmup.begin_errors,
               warmup.receive_errors,
               warmup.no_space_events,
               warmup.end_errors);
        if (probe.warmup_stable_cycles != 0U &&
            warmup.consecutive_stable_cycles < probe.warmup_stable_cycles) {
            fprintf(stderr, "[warmup] tx-probe unstable: stable=%" PRIu32
                    " required=%" PRIu32 " last_rx=%zu max_rx=%zu\n",
                    warmup.consecutive_stable_cycles,
                    probe.warmup_stable_cycles,
                    warmup.last_rx_count,
                    warmup.max_rx_count);
            ecan_igh_close(&igh);
            return IGH_ECAN_EXIT_FAIL;
        }
    }

    periodic_clock_start(&clock, probe.cycle_us, probe.spin_us, probe.catch_up);
    if (ecan_igh_reset_runtime_diagnostics(&igh) != IGH_ECAN_OK) {
        fprintf(stderr, "failed to reset adapter runtime diagnostics\n");
        ecan_igh_close(&igh);
        return IGH_ECAN_EXIT_FAIL;
    }
    period_stats_reset(&period_stats);
    for (uint32_t i = 0U; i < probe.count; i++) {
        uint8_t data[64];
        uint64_t offset_calc = (uint64_t)probe.schedule_offset_ns +
                               ((uint64_t)i * (uint64_t)probe.schedule_offset_step_ns);
        uint32_t offset_ns = (offset_calc > UINT32_MAX) ? UINT32_MAX : (uint32_t)offset_calc;
        uint64_t cycle_start_ns;

        cycle_start_ns = periodic_clock_wait(&clock);
        period_stats_sample(&period_stats, cycle_start_ns, clock.cycle_ns);
        fill_payload(data, probe.data_len, i);
        rc = igh_mcan_port_cycle_begin_checked(&mcan);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "cycle begin failed: cycle=%" PRIu32 " rc=%d\n", i, rc);
            failed++;
            continue;
        }
        if (probe.canfd_frame) {
            if (probe.scheduled_tx != 0U) {
                rc = igh_mcan_port_schedule_canfd(&mcan,
                                                  probe.slave_pos,
                                                  probe.channel,
                                                  probe.can_id,
                                                  probe.is_extended,
                                                  probe.brs,
                                                  data,
                                                  probe.data_len,
                                                  offset_ns,
                                                  (uint16_t)(i + 1U));
            } else {
                rc = igh_mcan_port_send_canfd(&mcan,
                                              probe.slave_pos,
                                              probe.channel,
                                              probe.can_id,
                                              probe.is_extended,
                                              probe.brs,
                                              data,
                                              probe.data_len);
            }
        } else {
            if (probe.scheduled_tx != 0U) {
                rc = igh_mcan_port_schedule_can(&mcan,
                                                probe.slave_pos,
                                                probe.channel,
                                                probe.can_id,
                                                probe.is_extended,
                                                data,
                                                probe.data_len,
                                                offset_ns,
                                                (uint16_t)(i + 1U));
            } else {
                rc = igh_mcan_port_send_can(&mcan,
                                            probe.slave_pos,
                                            probe.channel,
                                            probe.can_id,
                                            probe.is_extended,
                                            data,
                                            probe.data_len);
            }
        }
        if (rc == IGH_ECAN_OK) {
            sent++;
        } else {
            failed++;
        }
        {
            int end_rc = igh_mcan_port_cycle_end_checked(&mcan);
            if (end_rc != IGH_ECAN_OK) {
                fprintf(stderr, "cycle end failed: cycle=%" PRIu32 " rc=%d\n", i, end_rc);
                failed++;
            }
        }
    }

    printf("mcan_tx sent=%" PRIu32 " failed=%" PRIu32 "\n", sent, failed);
    print_period_stats(&period_stats, clock.cycle_ns);
    printf("master_period_resyncs=%" PRIu32 "\n", clock.resyncs);
    if (should_fail_on_diag_events()) {
        ecan_igh_sched_stats_t stats;
        int stats_rc = ecan_igh_get_sched_stats(&igh, &stats);

        if (stats_rc != IGH_ECAN_OK) {
            fprintf(stderr, "adapter runtime diagnostics snapshot failed: %d\n",
                    stats_rc);
            failed++;
        } else if (stats.cycle_overrun_count != 0U ||
                   stats.wkc_mismatch_count != 0U) {
            fprintf(stderr,
                    "adapter runtime diagnostics detected: overruns=%" PRIu64
                    " wkc_mismatches=%" PRIu64 "\n",
                    stats.cycle_overrun_count,
                    stats.wkc_mismatch_count);
            failed++;
        }
    }
    if (getenv("ECAN_IGH_PRINT_RUNTIME_DIAG") != NULL) {
        ecan_igh_print_runtime_diagnostics(&igh, 16U);
    }
    ecan_igh_close(&igh);
    return (failed == 0U) ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_FAIL;
}
