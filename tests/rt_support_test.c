#define _GNU_SOURCE

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "igh_rt_support.h"

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__, __LINE__, #expr); \
        return 1; \
    } \
} while (0)

int main(void)
{
    igh_rt_periodic_clock_t clock;
    igh_rt_period_stats_t stats;

    memset(&clock, 0xA5, sizeof(clock));
    CHECK(igh_rt_periodic_clock_start_checked(&clock, 0U, 0U, 0U) != 0);

    igh_rt_period_stats_reset(&stats);
    igh_rt_period_stats_sample(&stats, 0U, 1000000U);
    CHECK(stats.invalid_samples == 1U && stats.samples == 0U);
    igh_rt_period_stats_sample(&stats, 1000000U, 1000000U);
    igh_rt_period_stats_sample(&stats, 2000000U, 1000000U);
    CHECK(stats.samples == 1U);
    CHECK(stats.min_ns == 1000000U && stats.max_ns == 1000000U);
    igh_rt_period_stats_sample(&stats, 1500000U, 1000000U);
    CHECK(stats.invalid_samples == 2U && stats.samples == 1U);

    puts("rt_support_test: PASS");
    return 0;
}
