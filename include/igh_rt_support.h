#ifndef IGH_RT_SUPPORT_H
#define IGH_RT_SUPPORT_H

#include <errno.h>
#include <inttypes.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>

typedef struct {
    uint64_t next_ns;
    uint64_t cycle_ns;
    uint64_t spin_ns;
    uint8_t catch_up;
    uint32_t resyncs;
    int last_error;
} igh_rt_periodic_clock_t;

typedef struct {
    uint32_t samples;
    uint32_t invalid_samples;
    uint64_t min_ns;
    uint64_t max_ns;
    uint64_t sum_ns;
    uint64_t max_abs_err_ns;
    int64_t max_late_ns;
    int64_t max_early_ns;
    uint64_t last_ns;
} igh_rt_period_stats_t;

static inline uint64_t igh_rt_monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec;
}

static inline void igh_rt_ns_to_timespec(uint64_t ns, struct timespec *ts)
{
    ts->tv_sec = (time_t)(ns / 1000000000ULL);
    ts->tv_nsec = (long)(ns % 1000000000ULL);
}

static inline int igh_rt_periodic_clock_start_checked(igh_rt_periodic_clock_t *clock,
                                                      uint32_t cycle_us,
                                                      uint32_t spin_us,
                                                      uint8_t catch_up)
{
    uint64_t now_ns;

    if (clock == NULL || cycle_us == 0U) {
        return -1;
    }
    memset(clock, 0, sizeof(*clock));
    clock->cycle_ns = (uint64_t)cycle_us * 1000ULL;
    clock->spin_ns = (uint64_t)spin_us * 1000ULL;
    if (clock->spin_ns > (clock->cycle_ns / 2ULL)) {
        clock->spin_ns = clock->cycle_ns / 2ULL;
    }
    now_ns = igh_rt_monotonic_ns();
    if (now_ns == 0U) {
        clock->last_error = (errno != 0) ? errno : EIO;
        return -1;
    }
    /*
     * Keep the application cycle on an absolute period boundary.  DC Sync0
     * shifts are evaluated against the DC time modulo the cycle; starting at
     * "now + period" would therefore give every process launch a random
     * application-to-Sync0 phase.
     */
    clock->next_ns = (clock->cycle_ns != 0U) ?
                     (now_ns - (now_ns % clock->cycle_ns) + clock->cycle_ns) :
                     now_ns;
    clock->catch_up = catch_up;
    return 0;
}

static inline void igh_rt_periodic_clock_start(igh_rt_periodic_clock_t *clock,
                                               uint32_t cycle_us,
                                               uint32_t spin_us,
                                               uint8_t catch_up)
{
    (void)igh_rt_periodic_clock_start_checked(clock, cycle_us, spin_us, catch_up);
}

static inline uint64_t igh_rt_periodic_clock_wait(igh_rt_periodic_clock_t *clock)
{
    struct timespec ts;
    uint64_t deadline_ns;
    uint64_t now_ns;
    int rc;

    if (clock == NULL || clock->cycle_ns == 0U || clock->next_ns == 0U) {
        if (clock != NULL) {
            clock->last_error = EINVAL;
        }
        return 0U;
    }
    deadline_ns = clock->next_ns;
    if (clock->spin_ns != 0U && deadline_ns > clock->spin_ns) {
        igh_rt_ns_to_timespec(deadline_ns - clock->spin_ns, &ts);
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } while (rc == EINTR);
        if (rc != 0) {
            clock->last_error = rc;
            return 0U;
        }
        while ((now_ns = igh_rt_monotonic_ns()) != 0U && now_ns < deadline_ns) {
            /* short busy wait to reduce scheduler wake-up jitter */
        }
    } else {
        igh_rt_ns_to_timespec(deadline_ns, &ts);
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } while (rc == EINTR);
        if (rc != 0) {
            clock->last_error = rc;
            return 0U;
        }
        now_ns = igh_rt_monotonic_ns();
    }

    if (now_ns == 0U) {
        clock->last_error = (errno != 0) ? errno : EIO;
        return 0U;
    }

    if (clock->catch_up == 0U &&
        now_ns > deadline_ns &&
        (now_ns - deadline_ns) > (clock->cycle_ns / 2ULL)) {
        clock->next_ns = (clock->cycle_ns != 0U) ?
                         (now_ns - (now_ns % clock->cycle_ns) + clock->cycle_ns) :
                         now_ns;
        clock->resyncs++;
    } else {
        if (UINT64_MAX - deadline_ns < clock->cycle_ns) {
            clock->last_error = EOVERFLOW;
            return 0U;
        }
        clock->next_ns = deadline_ns + clock->cycle_ns;
    }
    return now_ns;
}

static inline void igh_rt_period_stats_reset(igh_rt_period_stats_t *stats)
{
    if (stats != NULL) {
        memset(stats, 0, sizeof(*stats));
    }
}

static inline void igh_rt_period_stats_sample(igh_rt_period_stats_t *stats,
                                              uint64_t now_ns,
                                              uint64_t target_ns)
{
    uint64_t period_ns;
    int64_t err_ns;
    uint64_t abs_err_ns;

    if (stats == NULL || now_ns == 0U || target_ns == 0U) {
        if (stats != NULL) {
            stats->invalid_samples++;
        }
        return;
    }
    if (stats->last_ns == 0U) {
        stats->last_ns = now_ns;
        return;
    }

    if (now_ns <= stats->last_ns ||
        now_ns - stats->last_ns > (uint64_t)INT64_MAX ||
        target_ns > (uint64_t)INT64_MAX) {
        stats->invalid_samples++;
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

static inline void igh_rt_print_period_stats(const char *name,
                                             const igh_rt_period_stats_t *stats,
                                             uint64_t target_ns)
{
    double avg_us;

    if (name == NULL || stats == NULL) {
        return;
    }
    if (stats->samples == 0U) {
        printf("%s target_us=%.3f samples=0 invalid=%" PRIu32 "\n",
               name, (double)target_ns / 1000.0, stats->invalid_samples);
        return;
    }

    avg_us = ((double)stats->sum_ns / (double)stats->samples) / 1000.0;
    printf("%s target_us=%.3f samples=%" PRIu32
           " min_us=%.3f max_us=%.3f avg_us=%.3f"
           " invalid=%" PRIu32
           " max_abs_err_us=%.3f max_late_us=%.3f max_early_us=%.3f\n",
           name,
           (double)target_ns / 1000.0,
           stats->samples,
           (double)stats->min_ns / 1000.0,
           (double)stats->max_ns / 1000.0,
           avg_us,
           stats->invalid_samples,
           (double)stats->max_abs_err_ns / 1000.0,
           (double)stats->max_late_ns / 1000.0,
           (double)stats->max_early_ns / 1000.0);
}

static inline int igh_rt_apply_runtime_tuning_checked(uint8_t lock_memory,
                                                      int rt_priority,
                                                      int cpu)
{
    int failed = 0;

    if (lock_memory != 0U && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        fprintf(stderr, "[warn] mlockall failed: %s\n", strerror(errno));
        failed = 1;
    }

    if (cpu >= 0) {
        cpu_set_t set;

        if (cpu >= CPU_SETSIZE) {
            fprintf(stderr, "[warn] cpu=%d is outside CPU_SETSIZE=%d\n",
                    cpu, CPU_SETSIZE);
            failed = 1;
        } else {
            CPU_ZERO(&set);
            CPU_SET(cpu, &set);
        }
        if (cpu < CPU_SETSIZE && sched_setaffinity(0, sizeof(set), &set) != 0) {
            fprintf(stderr, "[warn] sched_setaffinity cpu=%d failed: %s\n",
                    cpu, strerror(errno));
            failed = 1;
        }
    }

    if (rt_priority > 0) {
        struct sched_param param;
        int max_priority = sched_get_priority_max(SCHED_FIFO);

        if (max_priority < 0 || rt_priority > max_priority) {
            fprintf(stderr, "[warn] fifo priority=%d is invalid\n", rt_priority);
            failed = 1;
        } else {
            memset(&param, 0, sizeof(param));
            param.sched_priority = rt_priority;
        }
        if (rt_priority <= max_priority &&
            sched_setscheduler(0, SCHED_FIFO, &param) != 0) {
            fprintf(stderr, "[warn] sched_setscheduler fifo=%d failed: %s\n",
                    rt_priority, strerror(errno));
            failed = 1;
        }
    }
    return failed == 0 ? 0 : -1;
}

static inline void igh_rt_apply_runtime_tuning(uint8_t lock_memory,
                                               int rt_priority,
                                               int cpu)
{
    (void)igh_rt_apply_runtime_tuning_checked(lock_memory, rt_priority, cpu);
}

#endif /* IGH_RT_SUPPORT_H */
