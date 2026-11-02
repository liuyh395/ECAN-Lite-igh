#define _DEFAULT_SOURCE

#include "igh_adapter_internal.h"

#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

/* Return a monotonic timestamp in nanoseconds for DC application time. */
static uint64_t monotonic_ns(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * 1000000000ULL) + (uint64_t)ts.tv_nsec;
}

/* Sleep most of an absolute phase and busy-wait only the final short tail. */
static int wait_until_ns_tail(uint64_t target_ns, uint64_t spin_tail_ns)
{
    uint64_t now_ns = monotonic_ns();

    if (target_ns == 0U || now_ns == 0U) {
        return IGH_ECAN_FAIL;
    }

    if (now_ns + spin_tail_ns < target_ns) {
        struct timespec ts;
        int rc;

        ts.tv_sec = (time_t)((target_ns - spin_tail_ns) / 1000000000ULL);
        ts.tv_nsec = (long)((target_ns - spin_tail_ns) % 1000000000ULL);
        do {
            rc = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } while (rc == EINTR);
        if (rc != 0) {
            return IGH_ECAN_FAIL;
        }
    }
    for (;;) {
        now_ns = monotonic_ns();
        if (now_ns == 0U) {
            return IGH_ECAN_FAIL;
        }
        if (now_ns >= target_ns) {
            break;
        }
        __asm__ __volatile__("" ::: "memory");
    }
    return IGH_ECAN_OK;
}

static int wait_until_ns(uint64_t target_ns)
{
    return wait_until_ns_tail(target_ns, 30000ULL);
}

/* Early preparation must not immediately sleep again just before TX. Keep
 * this output-only spin bounded to 100 us; other waits retain a 30 us tail. */
static int wait_for_dc_output(ecan_igh_context_t *ctx)
{
    uint64_t tail = (uint64_t)ctx->dc_prepare_ahead_us * 1000ULL;
    if (tail < 30000ULL) tail = 30000ULL;
    if (tail > 100000ULL) tail = 100000ULL;
    return wait_until_ns_tail(ctx->cycle_start_ns +
        (uint64_t)ctx->rxpdo_phase_us * 1000ULL, tail);
}

static uint64_t elapsed_ns_since(uint64_t begin_ns)
{
    uint64_t end_ns = monotonic_ns();

    if (begin_ns == 0U || end_ns < begin_ns) {
        return 0U;
    }
    return end_ns - begin_ns;
}

/* Return the first absolute cycle phase strictly after now_ns. */
static uint64_t next_cycle_phase_ns(uint64_t now_ns,
                                    uint64_t cycle_ns,
                                    uint64_t phase_ns)
{
    uint64_t target_ns;

    if (cycle_ns == 0U) {
        return now_ns;
    }
    phase_ns %= cycle_ns;
    target_ns = now_ns - (now_ns % cycle_ns) + phase_ns;
    if (target_ns <= now_ns) {
        target_ns += cycle_ns;
    }
    return target_ns;
}

/* Advance an absolute deadline without replaying missed periods. */
static uint64_t advance_cycle_deadline_ns(uint64_t deadline_ns,
                                          uint64_t cycle_ns,
                                          uint64_t now_ns)
{
    uint64_t periods;

    if (cycle_ns == 0U) {
        return now_ns;
    }
    deadline_ns += cycle_ns;
    if (deadline_ns > now_ns) {
        return deadline_ns;
    }
    periods = ((now_ns - deadline_ns) / cycle_ns) + 1U;
    return deadline_ns + periods * cycle_ns;
}

uint64_t ecan_igh_adapter_monotonic_ns(void)
{
    return monotonic_ns();
}

void ecan_igh_adapter_update_max_u64(uint64_t *slot, uint64_t value)
{
    if ((slot != NULL) && value > *slot) {
        *slot = value;
    }
}

/* Return a monotonic timestamp in milliseconds for timeout loops. */
static uint64_t monotonic_ms(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) {
        return 0U;
    }
    return ((uint64_t)ts.tv_sec * 1000ULL) + ((uint64_t)ts.tv_nsec / 1000000ULL);
}

/* Publish the current application time for every active master.
 *
 * IgH also uses app_time while initializing the DC-capable reference slave
 * when the application selected free-run/SM mode.  Suppressing this call when
 * Sync0 is disabled leaves dc_ref_time unset and races master activation with
 * the FSM's system-time pass, producing the misleading
 * "No application time received" warning.  Supplying a monotonic value is
 * harmless for non-DC operation and keeps all activation paths deterministic.
 */
static void update_dc_application_time(ecan_igh_context_t *ctx)
{
    uint64_t app_time_ns;

    if ((ctx == NULL) || (ctx->master == NULL)) {
        return;
    }
    /* app_time是当前时间，不是命令所属周期。启动1kHz轮询若反复提交
     * 20/50/100ms向下取整网格，会把量化误差带入ESC系统时间初始化。
     * 首次相位原点在activate_master_domain中单独设置，随后不再量化。 */
    app_time_ns = monotonic_ns();
    if (app_time_ns == 0U) {
        return;
    }
    ecrt_master_application_time(ctx->master, app_time_ns);
}

static void process_runtime_domains(ecan_igh_context_t *ctx);

/* Retire the final cyclic datagrams before returning the master to IgH.
 *
 * The last application cycle ends with ecrt_master_send().  Closing the
 * master immediately afterwards can switch to the idle thread before the
 * generic NIC has delivered the process-data replies.  Shutdown is
 * non-realtime, so wait for at most two milliseconds and perform one final
 * receive/process pass.  This is not an OP-to-PREOP handshake and does not
 * resolve a master-internal operation-to-idle FSM race.  An application-side
 * delay cannot guarantee that handoff; see docs/MASTER_COMPATIBILITY.md.
 */
static void drain_master_before_close(ecan_igh_context_t *ctx)
{
    struct timespec delay = {0, 2000000L};

    if ((ctx == NULL) || (ctx->master == NULL) || (ctx->domain_pd == NULL)) {
        return;
    }

    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {
    }
    update_dc_application_time(ctx);
    ecrt_master_receive(ctx->master);
    process_runtime_domains(ctx);
}

static const char *diag_event_name(uint32_t event_type)
{
    switch (event_type) {
    case ECAN_IGH_DIAG_EVENT_WKC_MISMATCH:
        return "wkc-mismatch";
    case ECAN_IGH_DIAG_EVENT_CYCLE_OVERRUN:
        return "cycle-overrun";
    case ECAN_IGH_DIAG_EVENT_DC_RECEIVE_TIMEOUT:
        return "dc-receive-timeout";
    default:
        return "unknown";
    }
}

static void append_diag_domain_sample(ecan_igh_diag_event_t *event,
                                      uint16_t slave_index,
                                      uint8_t kind,
                                      ec_domain_t *domain)
{
    ec_domain_state_t state;
    uint32_t index;

    if (event == NULL || domain == NULL ||
        event->domain_count >= ECAN_IGH_DIAG_DOMAIN_MAX) {
        return;
    }
    memset(&state, 0, sizeof(state));
    ecrt_domain_state(domain, &state);
    index = event->domain_count++;
    event->domains[index].slave_index = slave_index;
    event->domains[index].kind = kind;
    event->domains[index].wc_state = (uint8_t)state.wc_state;
    event->domains[index].working_counter = state.working_counter;
    if (event->first_bad_domain < 0 && state.wc_state != EC_WC_COMPLETE) {
        event->first_bad_domain = (int32_t)index;
    }
}

static void snapshot_diag_domains(ecan_igh_context_t *ctx,
                                  ecan_igh_diag_event_t *event)
{
    event->first_bad_domain = -1;
    if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            append_diag_domain_sample(event, (uint16_t)slave,
                                      ECAN_IGH_DIAG_DOMAIN_RXPDO,
                                      ctx->rx_domains[slave]);
            append_diag_domain_sample(event, (uint16_t)slave,
                                      ECAN_IGH_DIAG_DOMAIN_TXPDO,
                                      ctx->tx_domains[slave]);
        }
        return;
    }
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        append_diag_domain_sample(event, (uint16_t)slave,
                                  ECAN_IGH_DIAG_DOMAIN_COMBINED,
                                  ctx->domains[slave]);
    }
}

static void record_diag_event(ecan_igh_context_t *ctx,
                              uint32_t event_type,
                              uint64_t late_ns,
                              const ec_domain_state_t *domain_state)
{
    ecan_igh_diag_event_t *event;
    uint64_t sequence;

    if (ctx == NULL) {
        return;
    }

    sequence = ctx->diag_event_head + 1U;
    event = &ctx->diag_events[ctx->diag_event_head % ECAN_IGH_DIAG_EVENT_RING_SIZE];
    memset(event, 0, sizeof(*event));
    event->sequence = sequence;
    event->ethercat_cycle = ctx->sched_stats.ethercat_cycle;
    event->record_time_ns = monotonic_ns();
    event->event_type = event_type;
    event->wkc_expected = ctx->domain_expected_working_counter;
    event->late_ns = late_ns;
    event->receive_process_ns = ctx->last_receive_process_ns;
    event->txpdo_copy_ns = ctx->last_txpdo_copy_ns;
    event->rxpdo_copy_ns = ctx->last_rxpdo_copy_ns;
    event->queue_ns = ctx->last_queue_ns;
    event->master_send_ns = ctx->last_master_send_ns;

    if (ctx->cycle_start_ns != 0U) {
        event->app_time_ns = ctx->cycle_start_ns;
    }
    if (ctx->cycle_begin_ns != 0U && event->record_time_ns >= ctx->cycle_begin_ns) {
        event->cycle_exec_ns = event->record_time_ns - ctx->cycle_begin_ns;
    }

    if (domain_state != NULL) {
        event->wkc_current = domain_state->working_counter;
        snapshot_diag_domains(ctx, event);
    } else {
        event->wkc_current = ctx->domain_working_counter;
        event->first_bad_domain = -1;
    }

    ctx->diag_event_head = sequence;
    if (sequence > ECAN_IGH_DIAG_EVENT_RING_SIZE) {
        ctx->diag_event_dropped = sequence - ECAN_IGH_DIAG_EVENT_RING_SIZE;
    }
}

static void maybe_record_wkc_event(ecan_igh_context_t *ctx,
                                   const ec_domain_state_t *domain_state)
{
    if ((ctx == NULL) || (domain_state == NULL)) {
        return;
    }
    if (domain_state->wc_state == EC_WC_COMPLETE &&
        ctx->domain_expected_working_counter == 0U) {
        ctx->domain_expected_working_counter = domain_state->working_counter;
    }
    if (domain_state->wc_state != EC_WC_COMPLETE ||
        (ctx->domain_expected_working_counter != 0U &&
         domain_state->working_counter != ctx->domain_expected_working_counter)) {
        ctx->sched_stats.wkc_mismatch_count++;
        record_diag_event(ctx, ECAN_IGH_DIAG_EVENT_WKC_MISMATCH, 0U, domain_state);
    }
}

/* Queue reference/slave clock synchronization for the next master send. */
static void queue_dc_clock_sync(ecan_igh_context_t *ctx)
{
    if ((ctx == NULL) || (ctx->master == NULL) || ctx->dc_assign_activate == 0U) {
        return;
    }

    /* 相位由首次dc_ref_time和Sync配置确定；当前时间与业务网格分离。
     * 仍由原收发调度控制报文时刻，不改变所有权或Sync0前准入。 */
    update_dc_application_time(ctx);
    /* Application time fixes the cycle phase; drift compensation needs the
     * current clock, not the earlier planned deadline. Reusing app_time_ns
     * here converts queue/scheduler lateness into a false DC clock error. */
    uint64_t reference_time_ns = monotonic_ns();
    if (reference_time_ns == 0U) return;
    ecrt_master_sync_reference_clock_to(ctx->master, reference_time_ns);
    ecrt_master_sync_slave_clocks(ctx->master);
}

/* Sample every slave AL state and cache OP status in the runtime context. */
static void update_slave_states(ecan_igh_context_t *ctx)
{
    int all_op = 1;

    if (ctx == NULL) {
        return;
    }

    for (size_t i = 0U; i < ctx->slave_count; i++) {
        ec_slave_config_state_t state;

        memset(&state, 0, sizeof(state));
        ecrt_slave_config_state(ctx->slaves[i].sc, &state);
        ctx->slave_al_state[i] = state.al_state;
        /* AL=OP alone can belong to the previous application configuration.
         * IgH keeps force_config set until this slave_config has actually
         * been applied; state.operational includes that ownership check. */
        ctx->slave_operational[i] = state.operational ? 1U : 0U;
        if (ctx->slave_operational[i] == 0U) {
            all_op = 0;
        }
    }
    ctx->slaves_operational = all_op ? 1U : 0U;
}

/* Slave AL state changes much more slowly than process data. Poll every cycle
 * while entering OP or after a WKC problem, then decimate the healthy hot path.
 */
static void maybe_update_slave_states(ecan_igh_context_t *ctx, int force)
{
    if (ctx == NULL) {
        return;
    }
    if (force != 0 || ctx->slaves_operational == 0U ||
        (ctx->sched_stats.ethercat_cycle % ECAN_IGH_SLAVE_STATE_POLL_INTERVAL) == 0U) {
        update_slave_states(ctx);
    }
}

/* Process every active domain without consuming a domain generation twice. */
static void process_runtime_domains(ecan_igh_context_t *ctx)
{
    if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            ecrt_domain_process(ctx->rx_domains[slave]);
            ecrt_domain_process(ctx->tx_domains[slave]);
        }
        return;
    }
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        if (ctx->domains[slave] != NULL) {
            ecrt_domain_process(ctx->domains[slave]);
        }
    }
}

/* Aggregate WKC across either combined or direction-split domains. */
static void sample_runtime_domain_state(ecan_igh_context_t *ctx,
                                        ec_domain_state_t *aggregate_state)
{
    unsigned int working_counter = 0U;
    int complete = 1;

    memset(aggregate_state, 0, sizeof(*aggregate_state));
    if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            ec_domain_state_t rx_state;
            ec_domain_state_t tx_state;

            memset(&rx_state, 0, sizeof(rx_state));
            memset(&tx_state, 0, sizeof(tx_state));
            ecrt_domain_state(ctx->rx_domains[slave], &rx_state);
            ecrt_domain_state(ctx->tx_domains[slave], &tx_state);
            working_counter += rx_state.working_counter + tx_state.working_counter;
            if (rx_state.wc_state != EC_WC_COMPLETE ||
                tx_state.wc_state != EC_WC_COMPLETE) {
                complete = 0;
            }
        }
        aggregate_state->working_counter = working_counter;
        aggregate_state->wc_state = complete ? EC_WC_COMPLETE : EC_WC_INCOMPLETE;
        return;
    }
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        ec_domain_state_t state;

        memset(&state, 0, sizeof(state));
        ecrt_domain_state(ctx->domains[slave], &state);
        working_counter += state.working_counter;
        if (state.wc_state != EC_WC_COMPLETE) {
            complete = 0;
        }
    }
    aggregate_state->working_counter = working_counter;
    aggregate_state->wc_state = complete ? EC_WC_COMPLETE : EC_WC_INCOMPLETE;
}

/* Sample one half of the direction-split DC exchange.  RxPDO writes are
 * retired before TxPDO reads are queued, so their WKC must be saved at that
 * point instead of re-reading both domain generations at final collection. */
static void sample_split_domain_state(ec_domain_t *const *domains,
                                      size_t domain_count,
                                      ec_domain_state_t *aggregate_state)
{
    unsigned int working_counter = 0U;
    int complete = 1;

    memset(aggregate_state, 0, sizeof(*aggregate_state));
    for (size_t domain = 0U; domain < domain_count; domain++) {
        ec_domain_state_t state;

        memset(&state, 0, sizeof(state));
        ecrt_domain_state(domains[domain], &state);
        working_counter += state.working_counter;
        if (state.wc_state != EC_WC_COMPLETE) {
            complete = 0;
        }
    }
    aggregate_state->working_counter = working_counter;
    aggregate_state->wc_state = complete ? EC_WC_COMPLETE : EC_WC_INCOMPLETE;
}

/* IgH's public domain state cannot distinguish an outstanding datagram from
 * a completed zero-WKC reply, or a partial multi-datagram domain. Only a full
 * WKC retires ownership: otherwise bounded receive-only polling, then fail-stop.
 * Never queue the same outstanding domain.
 * Repeated process calls do not consume the application shadow or RX codec. */
static void retire_sm_domains(ecan_igh_context_t *ctx)
{
    for (size_t i = 0U; i < ctx->slave_count; i++) {
        if ((ctx->sm_pending_mask & (1U << i)) != 0U) {
            /* An ioctl failure must not turn an uninitialized state into a
             * false completion. This also supports older void-return APIs. */
            ec_domain_state_t state = {0};
            if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
                ec_domain_state_t input = {0};
                ecrt_domain_state(ctx->rx_domains[i], &state);
                ecrt_domain_state(ctx->tx_domains[i], &input);
                if (input.wc_state != EC_WC_COMPLETE) continue;
            } else {
                ecrt_domain_state(ctx->domains[i], &state);
            }
            if (state.wc_state == EC_WC_COMPLETE)
                ctx->sm_pending_mask &= ~(1U << i);
        }
    }
}

/* 仅在本轮首次完整process之后的接收重试中使用。已完成的域没有再次queue，
 * 其代次和WKC保持不变；不必为等待最后一个报文重复进入所有域的process ioctl。
 * 未完成域仍每轮process并按完整WKC确认，最终统计仍覆盖全部域。
 * DC分离模式的两个方向必须作为同一从站的未完成事务一起处理。 */
static void process_pending_sm_domains(ecan_igh_context_t *ctx)
{
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        if ((ctx->sm_pending_mask & (1U << slave)) == 0U) continue;
        if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
            ecrt_domain_process(ctx->rx_domains[slave]);
            ecrt_domain_process(ctx->tx_domains[slave]);
        } else if (ctx->domains[slave] != NULL) {
            ecrt_domain_process(ctx->domains[slave]);
        }
    }
}

/* 仅失败时保存实际收集范围，不将三个LWR的WKC与六个双向域总值比较。
 * 超期与不完整WKC分开计数；不重新收包、排队、复制PDO或解除故障锁存。 */
static void record_dc_receive_timeout(ecan_igh_context_t *ctx,
                                      ec_domain_t *const *domains,
                                      const ec_domain_state_t *state,
                                      uint64_t start, uint64_t deadline,
                                      uint64_t observed, unsigned int attempts)
{
    uint32_t scope = domains == ctx->rx_domains ? ECAN_IGH_DIAG_SCOPE_RXPDO :
                     domains == ctx->tx_domains ? ECAN_IGH_DIAG_SCOPE_TXPDO :
                     ECAN_IGH_DIAG_SCOPE_ALL;
    record_diag_event(ctx, ECAN_IGH_DIAG_EVENT_DC_RECEIVE_TIMEOUT,
                      observed >= deadline ? observed - deadline : 0U, NULL);
    ecan_igh_diag_event_t *event =
        &ctx->diag_events[(ctx->diag_event_head - 1U) % ECAN_IGH_DIAG_EVENT_RING_SIZE];
    event->flags = scope;
    event->wkc_current = state->working_counter;
    event->receive_start_ns = start;
    event->receive_deadline_ns = deadline;
    event->receive_observed_ns = observed;
    event->receive_attempts = attempts;
    event->receive_wc_state = state->wc_state;
    event->receive_stop_reason = !observed ? ECAN_IGH_DIAG_RECEIVE_CLOCK :
        observed >= deadline ? ECAN_IGH_DIAG_RECEIVE_DEADLINE : ECAN_IGH_DIAG_RECEIVE_ATTEMPTS;
    if (observed >= start) event->receive_process_ns = observed - start;
    if (scope == ECAN_IGH_DIAG_SCOPE_ALL) {
        snapshot_diag_domains(ctx, event);
    } else {
        /* Rx方向使用上一完整收集值；首次完整结果可直接采用IgH确认值。
         * 未确认时expected=0表示未知，不按从站数量硬编码。 */
        event->wkc_expected = scope == ECAN_IGH_DIAG_SCOPE_RXPDO ?
            ctx->split_rx_working_counter :
            (ctx->domain_expected_working_counter >= ctx->split_rx_working_counter ?
             ctx->domain_expected_working_counter - ctx->split_rx_working_counter : 0U);
        if (!event->wkc_expected && state->wc_state == EC_WC_COMPLETE)
            event->wkc_expected = state->working_counter;
        for (size_t n = 0U; n < ctx->slave_count; n++)
            append_diag_domain_sample(event, (uint16_t)n,
                scope == ECAN_IGH_DIAG_SCOPE_RXPDO ? ECAN_IGH_DIAG_DOMAIN_RXPDO :
                                                    ECAN_IGH_DIAG_DOMAIN_TXPDO,
                domains[n]);
    }
    /* 尚未执行receive/process时没有当前代次WKC证据，不产生额外WKC计数。 */
    if (attempts && state->wc_state != EC_WC_COMPLETE)
        ctx->sched_stats.wkc_mismatch_count++;
}

/* 仅补充本次已记录的WKC失败事件，不再收包/采样域，不改变故障或所有权。
 * SM收包可能包含多个以太网帧，必须记录真实截止与尝试次数，不能从WKC=0
 * 单独推断从站拒绝。成功路径不调用本函数，也不增加时钟读取。 */
static void annotate_sm_receive_timeout(ecan_igh_context_t *ctx, uint64_t start,
                                        uint64_t deadline, uint64_t observed,
                                        unsigned int attempts)
{
    if (!ctx->diag_event_head) return;
    ecan_igh_diag_event_t *event =
        &ctx->diag_events[(ctx->diag_event_head - 1U) % ECAN_IGH_DIAG_EVENT_RING_SIZE];
    if (event->event_type != ECAN_IGH_DIAG_EVENT_WKC_MISMATCH) return;
    event->receive_start_ns = start;
    event->receive_deadline_ns = deadline;
    event->receive_observed_ns = observed;
    event->receive_attempts = attempts;
    event->receive_wc_state = ctx->domain_wc_state;
    event->receive_stop_reason = !observed ? ECAN_IGH_DIAG_RECEIVE_CLOCK :
        observed >= deadline ? ECAN_IGH_DIAG_RECEIVE_DEADLINE : ECAN_IGH_DIAG_RECEIVE_ATTEMPTS;
}

/* Receive/process the domain and optionally update working-counter diagnostics. */
static void receive_domain_state(ecan_igh_context_t *ctx, int sample_domain)
{
    ec_domain_state_t aggregate_state;
    uint64_t t0;
    uint64_t elapsed_ns;
    uint64_t failed_deadline = 0U, failed_observed = 0U;
    uint64_t polls_before = ctx->sched_stats.sm_receive_poll_calls;

    /* The outer scheduler can enter immediately after a paced send. Avoid
     * repeatedly entering the kernel before the reply has had a quarter cycle
     * to arrive. This is a host receive delay, not another EtherCAT read and
     * not a change to the slave's publication point. Late callers do not wait.
     * Keep this wait outside receive/process execution timing. */
    if (ctx->dc_assign_activate == 0U && ctx->sm_pending_mask != 0U &&
        ctx->sm_last_send_ns != 0U && ctx->sm_receive_fault == 0U) {
        uint64_t now = monotonic_ns();
        /* Retire early enough to leave command preparation margin. Waiting
         * half a 250 us cycle plus a slow receive ioctl exhausted that margin.
         * Early arrival still requires full WKC in the bounded poll below. */
        uint64_t earliest = ctx->sm_last_send_ns + (uint64_t)ctx->cycle_us * 250ULL;
        if (now < earliest) {
            ctx->sched_stats.sm_receive_waits++;
            ecan_igh_adapter_update_max_u64(&ctx->sched_stats.sm_receive_wait_max_ns,
                                            earliest - now);
            /* Keep the final 100 us runnable on the RT CPU. A 30 us tail
             * repeatedly slept through the short receive point on the
             * target PREEMPT_RT kernel. This remains a bounded wait, not
             * an unbounded poll or a relaxation of receive ownership. */
            if (wait_until_ns_tail(earliest, 100000ULL) != IGH_ECAN_OK) {
                ctx->sm_receive_fault = 1U;
                return;
            }
        }
    }
    update_dc_application_time(ctx);
    t0 = monotonic_ns();
    if (ctx->dc_assign_activate == 0U) ctx->sched_stats.sm_receive_poll_calls++;
    ecrt_master_receive(ctx->master);
    process_runtime_domains(ctx);
    if (ctx->sm_pending_mask != 0U && ctx->sm_receive_fault == 0U) {
        retire_sm_domains(ctx);
        if (ctx->sm_pending_mask != 0U) {
            /* SM统一按已建立的3/4周期交付点收集。三台双向256B也会跨
             * 以太网帧，250us周期不能在固定62.5+100us处先于交付点报错。
             * 不改变下一发送期限，最后1/4周期仍留给命令准备；未确认的域
             * 不释放、不重排、不复用。尚无发送期限或DC预启动仍用原预算。
             * 时间与尝试次数同时有界，避免时钟异常形成无界轮询。 */
            uint64_t budget = (uint64_t)ctx->cycle_us * 500ULL;
            if (budget > 100000ULL) budget = 100000ULL;
            uint64_t start = monotonic_ns();
            uint64_t deadline = start + budget;
            unsigned int attempt_limit = 128U;
            if (ctx->dc_assign_activate == 0U && ctx->cycle_us != 0U &&
                ctx->sm_next_send_ns > (uint64_t)ctx->cycle_us * 250ULL) {
                deadline = ctx->sm_next_send_ns - (uint64_t)ctx->cycle_us * 250ULL;
                attempt_limit = 512U;
            }
            for (unsigned int attempt = 0U; attempt < attempt_limit &&
                 ctx->sm_pending_mask != 0U &&
                 monotonic_ns() < deadline; attempt++) {
                ctx->sched_stats.sm_receive_poll_calls++;
                ecrt_master_receive(ctx->master);
                process_pending_sm_domains(ctx);
                retire_sm_domains(ctx);
            }
            if (ctx->sm_pending_mask != 0U) {
                ctx->sm_receive_fault = 1U;
                ctx->sched_stats.sm_receive_timeouts++;
                failed_deadline = deadline;
                failed_observed = monotonic_ns();
            } else {
                ctx->sched_stats.sm_late_receive_recovered++;
            }
        }
    }
    elapsed_ns = elapsed_ns_since(t0);
    ctx->last_receive_process_ns = elapsed_ns;
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.receive_process_max_ns, elapsed_ns);
    if (sample_domain != 0) {
        sample_runtime_domain_state(ctx, &aggregate_state);
        ctx->domain_working_counter = aggregate_state.working_counter;
        ctx->domain_wc_state = aggregate_state.wc_state;
        ctx->domain_complete = (aggregate_state.wc_state == EC_WC_COMPLETE) ? 1U : 0U;
        maybe_record_wkc_event(ctx, &aggregate_state);
        if (failed_deadline != 0U)
            annotate_sm_receive_timeout(ctx, t0, failed_deadline, failed_observed,
                (unsigned int)(ctx->sched_stats.sm_receive_poll_calls - polls_before));
        maybe_update_slave_states(ctx, aggregate_state.wc_state != EC_WC_COMPLETE);
    } else {
        maybe_update_slave_states(ctx, 0);
    }
}

/* Reject known-late output before queueing any PDO. This cannot bound a later
 * kernel/NIC stall: firmware generation, expiry and ownership guards remain
 * authoritative. A rejected command is an explicit session failure, not a
 * silently skipped success or a command retried in a newer motion cycle. */
static int check_dc_output_deadline(ecan_igh_context_t *ctx)
{
    if (ctx->dc_assign_activate == 0U || !ecan_igh_collects_at_cycle_end(ctx) ||
        !ctx->slaves_operational || !ctx->cycle_active)
        return IGH_ECAN_OK;
    uint64_t now = monotonic_ns();
    if (now == 0U) return IGH_ECAN_FAIL;
    int64_t slack = (int64_t)ctx->cycle_start_ns + ctx->dc_sync0_shift_ns - (int64_t)now;
    if (ctx->sched_stats.dc_prepare_samples++ == 0U ||
        slack < ctx->sched_stats.dc_prepare_slack_min_ns)
        ctx->sched_stats.dc_prepare_slack_min_ns = slack;
    if (ctx->dc_min_prepare_us != 0U && slack < (int64_t)ctx->dc_min_prepare_us * 1000LL) {
        ctx->sched_stats.dc_send_deadline_misses++;
        ctx->dc_receive_fault = 4U;
        ctx->domain_complete = 0U;
        return IGH_ECAN_TIMEOUT;
    }
    return IGH_ECAN_OK;
}

/* Queue the domain and send the master datagrams for this cycle. */
static int send_domain_state(ecan_igh_context_t *ctx)
{
    uint64_t t0;
    uint64_t elapsed_ns;

    if (ctx->pdo_queue_uncommitted != 0U || ctx->sm_receive_fault != 0U || ctx->dc_receive_fault != 0U ||
        ctx->sm_pending_mask != 0U)
        return IGH_ECAN_TIMEOUT;
    if (ctx->dc_assign_activate == 0U && ctx->sm_last_send_ns != 0U) {
        uint64_t now = monotonic_ns();
        uint64_t period = (uint64_t)ctx->cycle_us * 1000ULL;
        uint64_t earliest = ctx->sm_next_send_ns;
        /* Ordinary send ioctl cost must not accumulate in the period. Keep
         * an absolute deadline, but never catch up after a delayed ioctl or
         * a phase error larger than one eighth of a cycle. In that case wait
         * a full cycle after completion, and report the rebase. This is a
         * scheduling guard, not a relaxation of WKC or test acceptance. */
        if (earliest == 0U || earliest < ctx->sm_last_send_ns + period - period / 8U) {
            earliest = ctx->sm_last_send_ns + period;
            ctx->sm_next_send_ns = earliest;
            ctx->sched_stats.sm_pacing_rebases++;
        }
        /* 应用本身耗尽本次发送窗口时，不把已生成命令补发到下一周期。
         * 已进入内核后的迟到仍按下方完成时间退让；二者不能混为一谈。 */
        if (ctx->cycle_active != 0U && ctx->split_cycle_collected != 0U &&
            now > earliest + period / 8U) {
            ctx->sm_receive_fault = 2U; /* output deadline, cleared only by reinit */
            return IGH_ECAN_TIMEOUT;
        }
        if (now < earliest) {
            ctx->sched_stats.sm_pacing_waits++;
            ecan_igh_adapter_update_max_u64(&ctx->sched_stats.sm_pacing_wait_max_ns,
                                            earliest - now);
            if (wait_until_ns_tail(earliest, 100000ULL) != IGH_ECAN_OK) return IGH_ECAN_FAIL;
        }
    }
    int deadline_rc = check_dc_output_deadline(ctx);
    if (deadline_rc != IGH_ECAN_OK) return deadline_rc;
    queue_dc_clock_sync(ctx);
    t0 = monotonic_ns();
    ctx->pdo_queue_uncommitted = 1U;
    if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            if (ecrt_domain_queue(ctx->rx_domains[slave]) ||
                ecrt_domain_queue(ctx->tx_domains[slave])) return IGH_ECAN_FAIL;
        }
    } else {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            if (ecrt_domain_queue(ctx->domains[slave])) return IGH_ECAN_FAIL;
        }
    }
    elapsed_ns = elapsed_ns_since(t0);
    ctx->last_queue_ns = elapsed_ns;
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.queue_max_ns, elapsed_ns);
    t0 = monotonic_ns();
    if (ecrt_master_send(ctx->master) < 0) return IGH_ECAN_FAIL;
    ctx->pdo_queue_uncommitted = 0U;
    uint64_t send_completed_ns = monotonic_ns();
    if (ecan_igh_collects_at_cycle_end(ctx) &&
        ctx->rxpdo_phase_us == ctx->txpdo_phase_us &&
        ctx->slaves_operational != 0U) {
        ctx->sm_pending_mask = (1U << ctx->slave_count) - 1U;
    }
    if (ctx->dc_assign_activate == 0U && ctx->slaves_operational != 0U) {
        uint64_t period = (uint64_t)ctx->cycle_us * 1000ULL;
        uint64_t deadline = ctx->sm_next_send_ns;
        if (send_completed_ns < t0 || send_completed_ns - t0 > period / 8U) {
            /* The enqueue instant is unknown during a stalled ioctl. Do not
             * compensate that delay on the next command. */
            ctx->sm_next_send_ns = send_completed_ns + period;
            ctx->sched_stats.sm_pacing_rebases++;
        } else if (deadline == 0U || t0 > deadline + period / 8U) {
            ctx->sm_next_send_ns = t0 + period;
            if (deadline != 0U) { ctx->sched_stats.sm_pacing_rebases++; }
        } else {
            ctx->sm_next_send_ns = deadline + period;
        }
        ctx->sm_last_send_ns = send_completed_ns;
        ctx->sm_pending_mask = (1U << ctx->slave_count) - 1U;
    }
    elapsed_ns = send_completed_ns >= t0 ? send_completed_ns - t0 : 0U;
    ctx->last_master_send_ns = elapsed_ns;
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.master_send_max_ns, elapsed_ns);
    return IGH_ECAN_OK;
}

/* Queue all master-output domains in one send at the configured RxPDO phase. */
static int send_split_outputs(ecan_igh_context_t *ctx)
{
    uint64_t queue_start_ns;
    uint64_t send_start_ns;

    if (ctx->dc_receive_fault != 0U || ctx->pdo_queue_uncommitted != 0U) return IGH_ECAN_TIMEOUT;

    /* 准入必须先于queue：标准IgH没有取消已排队PDO的公开接口。提前queue后
     * 若等待/截止失败，故障退出的时钟send可能顺带发出旧命令。 */
    if (wait_for_dc_output(ctx) != IGH_ECAN_OK) return IGH_ECAN_FAIL;
    int deadline_rc = check_dc_output_deadline(ctx);
    if (deadline_rc != IGH_ECAN_OK) return deadline_rc;
    ecan_igh_adapter_copy_shadow_rxpdo_to_domain(ctx);
    queue_start_ns = monotonic_ns();
    ctx->pdo_queue_uncommitted = 1U;
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        if (ecrt_domain_queue(ctx->rx_domains[slave])) return IGH_ECAN_FAIL;
    }
    ctx->last_queue_ns = elapsed_ns_since(queue_start_ns);
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.queue_max_ns,
                                    ctx->last_queue_ns);
    queue_dc_clock_sync(ctx);

    send_start_ns = monotonic_ns();
    if (ecrt_master_send(ctx->master) < 0) return IGH_ECAN_FAIL;
    ctx->pdo_queue_uncommitted = 0U;
    ctx->last_master_send_ns = elapsed_ns_since(send_start_ns);
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.master_send_max_ns,
                                    ctx->last_master_send_ns);
    return IGH_ECAN_OK;
}

/* Retire the RxPDO write before queuing the delayed TxPDO read.  With a small
 * process image both datagrams can appear to work when receive() is deferred
 * until the end of the cycle.  Once each direction approaches an Ethernet
 * frame, however, the still-SENT LWR occupies the master datagram slot: the
 * following LRD is skipped and its eventual reply is reported unmatched.
 * Processing only the output domains here preserves the input-domain
 * generation for the final collection phase. */
/* Receive only: never queue or copy application shadows during compensation.
 * IgH recomputes domain WKC from datagram states on process(); a later receive
 * can complete that same generation provided it has not been re-queued. */
static int collect_split_domains(ecan_igh_context_t *ctx,
                                 ec_domain_t *const *domains,
                                 uint64_t deadline_ns,
                                 ec_domain_state_t *state)
{
    uint64_t start = monotonic_ns();
    uint64_t observed = start;
    unsigned int attempts = 0U;
    *state = (ec_domain_state_t){0};
    if (start == 0U || start >= deadline_ns) {
        ctx->sched_stats.dc_receive_timeouts++;
        record_dc_receive_timeout(ctx, domains, state, start, deadline_ns, start, 0U);
        return IGH_ECAN_TIMEOUT;
    }
    if (deadline_ns > start + 100000ULL) deadline_ns = start + 100000ULL;
    for (unsigned int attempt = 0U; attempt < 128U; attempt++) {
        attempts = attempt + 1U;
        ecrt_master_receive(ctx->master);
        if (domains != NULL) {
            for (size_t slave = 0U; slave < ctx->slave_count; slave++)
                ecrt_domain_process(domains[slave]);
            sample_split_domain_state(domains, ctx->slave_count, state);
        } else {
            /* Co-queued LWR/LRD: retire BOTH directions of this generation.
             * One complete direction must never publish partial input state. */
            process_runtime_domains(ctx);
            sample_runtime_domain_state(ctx, state);
        }
        observed = monotonic_ns();
        if (observed == 0U || observed >= deadline_ns) break;
        if (state->wc_state == EC_WC_COMPLETE) {
            if (attempt != 0U) ctx->sched_stats.dc_late_receive_recovered++;
            return IGH_ECAN_OK;
        }
    }
    ctx->sched_stats.dc_receive_timeouts++;
    record_dc_receive_timeout(ctx, domains, state, start, deadline_ns, observed, attempts);
    return IGH_ECAN_TIMEOUT;
}

static int receive_split_outputs(ecan_igh_context_t *ctx)
{
    ec_domain_state_t rx_state;
    uint64_t t0 = monotonic_ns();
    uint64_t deadline = ctx->cycle_start_ns +
        ((uint64_t)ctx->txpdo_phase_us +
         (ctx->txpdo_collect_phase_us - ctx->txpdo_phase_us) / 2U) * 1000ULL;
    int rc = collect_split_domains(ctx, ctx->rx_domains, deadline, &rx_state);
    ctx->split_rx_working_counter = rx_state.working_counter;
    ctx->split_rx_wc_state = rx_state.wc_state;
    ctx->last_receive_process_ns = elapsed_ns_since(t0);
    if (rc != IGH_ECAN_OK) {
        /* 输出确认失败会直接退出本周期，不再经过输入回收的峰值汇总。
         * 保留这次实际耗时，避免超窗越长、汇总却只显示成功周期的短耗时。 */
        ecan_igh_adapter_update_max_u64(&ctx->sched_stats.receive_process_max_ns,
                                        ctx->last_receive_process_ns);
        ctx->dc_receive_fault = 1U;
        ctx->domain_complete = 0U;
        ctx->domain_wc_state = EC_WC_INCOMPLETE;
        ctx->domain_working_counter = rx_state.working_counter;
        /* collect_split_domains已记录当前输出范围及截止原因。 */
    }
    return rc;
}

/* Group all delayed TxPDO reads into one Ethernet frame. */
static int send_split_inputs(ecan_igh_context_t *ctx)
{
    uint64_t t0;

    if (wait_until_ns(ctx->cycle_start_ns +
                      (uint64_t)ctx->txpdo_phase_us * 1000ULL) != IGH_ECAN_OK) {
        return IGH_ECAN_FAIL;
    }
    int rc = receive_split_outputs(ctx);
    if (rc != IGH_ECAN_OK) return rc;
    if (monotonic_ns() >= ctx->cycle_start_ns +
            (uint64_t)ctx->txpdo_collect_phase_us * 1000ULL) {
        ctx->dc_receive_fault = 1U;
        ctx->sched_stats.dc_receive_timeouts++;
        ctx->domain_complete = 0U;
        return IGH_ECAN_TIMEOUT;
    }
    t0 = monotonic_ns();
    ctx->pdo_queue_uncommitted = 1U;
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        if (ecrt_domain_queue(ctx->tx_domains[slave])) return IGH_ECAN_FAIL;
    }
    ctx->last_queue_ns += elapsed_ns_since(t0);
    t0 = monotonic_ns();
    if (ecrt_master_send(ctx->master) < 0) return IGH_ECAN_FAIL;
    ctx->pdo_queue_uncommitted = 0U;
    ctx->last_master_send_ns += elapsed_ns_since(t0);
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.queue_max_ns,
                                    ctx->last_queue_ns);
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.master_send_max_ns,
                                    ctx->last_master_send_ns);
    return IGH_ECAN_OK;
}

/* Complete the measured 1ms output -> device -> input exchange in one cycle. */
static int finish_dc_single_cycle(ecan_igh_context_t *ctx)
{
    uint32_t early_poll_window_us = 40U;
    uint32_t phase_gap_us;
    ec_domain_state_t aggregate_state;
    ec_domain_state_t tx_state;
    uint64_t receive_start_ns;
    uint64_t first_collect_ns;

    int rc = send_split_outputs(ctx);
    if (rc != IGH_ECAN_OK) goto failed;
    rc = send_split_inputs(ctx);
    if (rc != IGH_ECAN_OK) goto failed;
    phase_gap_us = ctx->txpdo_collect_phase_us - ctx->txpdo_phase_us;
    if (early_poll_window_us >= phase_gap_us) {
        early_poll_window_us = phase_gap_us / 2U;
    }
    first_collect_ns = ctx->cycle_start_ns +
        (uint64_t)(ctx->txpdo_collect_phase_us - early_poll_window_us) * 1000ULL;
    if (wait_until_ns(first_collect_ns) != IGH_ECAN_OK) {
        rc = IGH_ECAN_FAIL;
        goto failed;
    }
    update_dc_application_time(ctx);
    receive_start_ns = monotonic_ns();
    ecrt_master_receive(ctx->master);

    /* Poll early, then collect the same generation at the configured phase.
     * Late compensation never issues another EtherCAT read or repeats a PDO
     * decode; it only drains replies to the already queued read. */
    if (wait_until_ns(ctx->cycle_start_ns +
                      (uint64_t)ctx->txpdo_collect_phase_us * 1000ULL) !=
        IGH_ECAN_OK) {
        rc = IGH_ECAN_FAIL;
        goto failed;
    }
    rc = collect_split_domains(ctx, ctx->tx_domains,
        ctx->cycle_start_ns + (uint64_t)ctx->cycle_us * 1000ULL, &tx_state);

    aggregate_state.working_counter = ctx->split_rx_working_counter +
                                      tx_state.working_counter;
    aggregate_state.wc_state =
        (ctx->split_rx_wc_state == EC_WC_COMPLETE &&
         tx_state.wc_state == EC_WC_COMPLETE) ?
        EC_WC_COMPLETE : EC_WC_INCOMPLETE;
    ctx->last_receive_process_ns += elapsed_ns_since(receive_start_ns);
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.receive_process_max_ns,
                                    ctx->last_receive_process_ns);
    ctx->domain_working_counter = aggregate_state.working_counter;
    ctx->domain_wc_state = aggregate_state.wc_state;
    ctx->domain_complete =
        (aggregate_state.wc_state == EC_WC_COMPLETE) ? 1U : 0U;
    if (rc == IGH_ECAN_OK) maybe_record_wkc_event(ctx, &aggregate_state);
    maybe_update_slave_states(ctx,
                              aggregate_state.wc_state != EC_WC_COMPLETE);
    if (rc != IGH_ECAN_OK) {
        ctx->dc_receive_fault = 2U;
        return rc;
    }
    ecan_igh_adapter_copy_domain_txpdo_to_shadow(ctx);
    ctx->split_cycle_collected = 1U;
    return IGH_ECAN_OK;
failed:
    /* A clock/wait failure can occur after sending a direction. Even if the
     * application retries, do not re-queue an uncollected generation. Preserve
     * the more specific direction fault when one was already recorded. */
    if (ctx->dc_receive_fault == 0U) ctx->dc_receive_fault = 3U;
    return rc;
}

/* Combined DC: retire LRW or co-queued LWR/LRD before returning fresh inputs.
 * Completion is bounded and fail-stop. No requeue, resend or shadow mutation
 * is permitted while the previous generation still owns a domain. */
static int finish_dc_combined_cycle(ecan_igh_context_t *ctx, int copy_outputs)
{
    ec_domain_state_t state = {0};
    int rc = IGH_ECAN_TIMEOUT;
    uint64_t t0;
    if (ctx->dc_receive_fault != 0U || ctx->sm_receive_fault != 0U ||
        ctx->sm_pending_mask != 0U) goto failed;
    if (copy_outputs) ecan_igh_adapter_copy_shadow_rxpdo_to_domain(ctx);
    if (wait_for_dc_output(ctx) != IGH_ECAN_OK) goto failed;
    rc = send_domain_state(ctx);
    if (rc != IGH_ECAN_OK) goto failed;
    if (wait_until_ns(ctx->cycle_start_ns + (uint64_t)ctx->txpdo_collect_phase_us * 1000ULL)
        != IGH_ECAN_OK) goto failed;
    t0 = monotonic_ns();
    rc = collect_split_domains(ctx,
        ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE ? NULL : ctx->domains,
        ctx->cycle_start_ns + (uint64_t)ctx->cycle_us * 1000ULL, &state);
    ctx->last_receive_process_ns = elapsed_ns_since(t0);
    ecan_igh_adapter_update_max_u64(&ctx->sched_stats.receive_process_max_ns,
                                    ctx->last_receive_process_ns);
    ctx->domain_working_counter = state.working_counter;
    ctx->domain_wc_state = state.wc_state;
    ctx->domain_complete = state.wc_state == EC_WC_COMPLETE;
    if (rc == IGH_ECAN_OK) maybe_record_wkc_event(ctx, &state);
    maybe_update_slave_states(ctx, !ctx->domain_complete);
    if (rc != IGH_ECAN_OK) goto failed;
    ctx->sm_pending_mask = 0U;
    ecan_igh_adapter_copy_domain_txpdo_to_shadow(ctx);
    ctx->split_cycle_collected = 1U;
    return IGH_ECAN_OK;
failed:
    if (ctx->dc_receive_fault == 0U) ctx->dc_receive_fault = 3U;
    ctx->domain_complete = 0U;
    return rc == IGH_ECAN_OK ? IGH_ECAN_FAIL : rc;
}

/* Copy static config into the runtime context and initialize the lock. */
static int init_runtime_config(ecan_igh_context_t *ctx, const ecan_igh_config_t *config)
{
    pthread_mutexattr_t attr;

    memset(ctx, 0, sizeof(*ctx));
    if (pthread_mutexattr_init(&attr) != 0) {
        return IGH_ECAN_FAIL;
    }
    if (pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_ERRORCHECK) != 0 ||
        pthread_mutex_init(&ctx->lock, &attr) != 0) {
        (void)pthread_mutexattr_destroy(&attr);
        return IGH_ECAN_FAIL;
    }
    (void)pthread_mutexattr_destroy(&attr);
    ctx->lock_initialized = 1U;
    ctx->slave_count = config->slave_count;
    ctx->pdo_multiplier = config->pdo_multiplier;
    memcpy(ctx->slave_pdo_multiplier,
           config->slave_pdo_multiplier,
           sizeof(ctx->slave_pdo_multiplier));
    ctx->rxpdo_multiplier = config->rxpdo_multiplier;
    ctx->txpdo_multiplier = config->txpdo_multiplier;
    memcpy(ctx->slave_rxpdo_multiplier,
           config->slave_rxpdo_multiplier,
           sizeof(ctx->slave_rxpdo_multiplier));
    memcpy(ctx->slave_txpdo_multiplier,
           config->slave_txpdo_multiplier,
           sizeof(ctx->slave_txpdo_multiplier));
    ctx->init_flags = config->init_flags;
    ctx->cycle_us = config->cycle_us;
    ctx->sched_stats.cycle_us = config->cycle_us;
    ctx->dc_assign_activate = config->dc_assign_activate;
    ctx->dc_sync0_cycle_ns = (config->dc_sync0_cycle_ns != 0U) ?
                             config->dc_sync0_cycle_ns :
                             cycle_us_to_ns(config->cycle_us);
    ctx->dc_sync0_shift_ns = config->dc_sync0_shift_ns;
    ctx->dc_sync1_offset_ns = config->dc_sync1_offset_ns;
    ctx->cycle_mode = config->cycle_mode;
    ctx->dc_prepare_ahead_us = config->dc_prepare_ahead_us;
    ctx->dc_min_prepare_us = config->dc_min_prepare_us;
    ctx->rxpdo_phase_us = config->rxpdo_phase_us;
    ctx->txpdo_phase_us = config->txpdo_phase_us;
    ctx->txpdo_collect_phase_us = config->txpdo_collect_phase_us;
    return IGH_ECAN_OK;
}

/* Request the IgH master and create the process-data domain. */
static int open_master_domain(ecan_igh_context_t *ctx, const ecan_igh_config_t *config)
{
    ctx->master = ecrt_request_master(config->master_index);
    if (ctx->master == NULL) {
        perror("ecrt_request_master");
        return IGH_ECAN_NOT_READY;
    }

    (void)ecrt_master_set_send_interval(ctx->master, ctx->cycle_us);

    if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        for (size_t slave = 0U; slave < config->slave_count; slave++) {
            ctx->rx_domains[slave] = ecrt_master_create_domain(ctx->master);
            ctx->tx_domains[slave] = ecrt_master_create_domain(ctx->master);
            if (ctx->rx_domains[slave] == NULL || ctx->tx_domains[slave] == NULL) {
                perror("ecrt_master_create_domain");
                return IGH_ECAN_NOT_READY;
            }
        }
    } else {
        for (size_t slave = 0U; slave < config->slave_count; slave++) {
            ctx->domains[slave] = ecrt_master_create_domain(ctx->master);
            if (ctx->domains[slave] == NULL) {
                perror("ecrt_master_create_domain");
                return IGH_ECAN_NOT_READY;
            }
        }
    }
    ctx->domain = (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) ?
                  ctx->rx_domains[0] : ctx->domains[0];
    return IGH_ECAN_OK;
}

/* Close partial runtime state and propagate the original init error. */
static int init_fail(ecan_igh_context_t *ctx, int rc)
{
    ecan_igh_close(ctx);
    return rc;
}

/* Configure identity, PDO mapping and optional DC settings for one slave. */
static int configure_one_slave(ecan_igh_context_t *ctx,
                               const ecan_igh_config_t *config,
                               size_t slave)
{
    ecan_igh_slave_t *slave_ctx = &ctx->slaves[slave];
    uint8_t rx_multiplier = active_slave_rxpdo_multiplier(ctx, slave);
    uint8_t tx_multiplier = active_slave_txpdo_multiplier(ctx, slave);

    if (!ECAN_IGH_PDO_MULTIPLIER_IS_VALID(rx_multiplier) ||
        !ECAN_IGH_PDO_MULTIPLIER_IS_VALID(tx_multiplier)) {
        return IGH_ECAN_INVALID_ARG;
    }

    slave_ctx->config = config->slaves[slave];
    slave_ctx->sc = ecrt_master_slave_config(ctx->master,
                                             (uint16_t)slave_ctx->config.alias,
                                             (uint16_t)slave_ctx->config.position,
                                             slave_ctx->config.vendor_id,
                                             slave_ctx->config.product_id);
    if (slave_ctx->sc == NULL) {
        fprintf(stderr, "slave config failed: position=%d\n", slave_ctx->config.position);
        return IGH_ECAN_NOT_READY;
    }

    ecan_igh_adapter_build_pdo_infos(slave_ctx, rx_multiplier, tx_multiplier);
    if (ecrt_slave_config_pdos(slave_ctx->sc, EC_END, slave_ctx->syncs) != 0) {
        fprintf(stderr, "PDO local mapping failed: position=%d\n", slave_ctx->config.position);
        return IGH_ECAN_FAIL;
    }

    if (ctx->dc_assign_activate != 0U) {
        ecrt_slave_config_dc(slave_ctx->sc,
                             ctx->dc_assign_activate,
                             ctx->dc_sync0_cycle_ns,
                             ctx->dc_sync0_shift_ns,
                             ctx->dc_sync1_offset_ns,
                             0);
        if (ctx->dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC01 &&
            (ecrt_slave_config_sdo16(slave_ctx->sc, 0x1C32U, 1U, 2U) != 0 ||
             ecrt_slave_config_sdo16(slave_ctx->sc, 0x1C33U, 1U, 3U) != 0)) {
            fprintf(stderr, "DC Sync0/Sync1 SDO queue failed: position=%d\n",
                    slave_ctx->config.position);
            return IGH_ECAN_FAIL;
        }
        if (ctx->dc_assign_activate == ECAN_IGH_DC_ASSIGN_SYNC01) {
            ctx->dc_phase_requests[slave] =
                ecrt_slave_config_create_reg_request(slave_ctx->sc, 24U);
            if (ctx->dc_phase_requests[slave] == NULL) return IGH_ECAN_FAIL;
        }
    }

    if ((ctx->init_flags & ECAN_IGH_INIT_SKIP_STARTUP_SDO) == 0U &&
        ecan_igh_adapter_configure_pdo_assignment(slave_ctx->sc,
                                                  rx_multiplier,
                                                  tx_multiplier) != IGH_ECAN_OK) {
        fprintf(stderr, "PDO assignment SDO queue failed: position=%d\n", slave_ctx->config.position);
        return IGH_ECAN_FAIL;
    }
    return IGH_ECAN_OK;
}

/* Configure every active slave in the runtime context. */
static int configure_slaves(ecan_igh_context_t *ctx, const ecan_igh_config_t *config)
{
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        int rc = configure_one_slave(ctx, config, slave);
        if (rc != IGH_ECAN_OK) {
            return rc;
        }
    }
    return IGH_ECAN_OK;
}

/* Register all dynamic PDO entries with the IgH process-data domain. */
static int register_pdo_entries(ecan_igh_context_t *ctx)
{
    for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
        if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
            if (ecan_igh_adapter_build_slave_direction_domain_regs(
                    ctx, slave, ECAN_IGH_PDO_RX) != IGH_ECAN_OK ||
                ecan_igh_adapter_build_slave_direction_domain_regs(
                    ctx, slave, ECAN_IGH_PDO_TX) != IGH_ECAN_OK ||
                ecrt_domain_reg_pdo_entry_list(ctx->rx_domains[slave],
                                               ctx->rx_slave_domain_regs[slave]) != 0 ||
                ecrt_domain_reg_pdo_entry_list(ctx->tx_domains[slave],
                                               ctx->tx_slave_domain_regs[slave]) != 0) {
                fprintf(stderr, "direction PDO registration failed: slave=%zu\n", slave);
                return IGH_ECAN_FAIL;
            }
        } else {
            if (ecan_igh_adapter_build_slave_domain_regs(ctx, slave) != IGH_ECAN_OK ||
                ecrt_domain_reg_pdo_entry_list(ctx->domains[slave],
                                               ctx->slave_domain_regs[slave]) != 0) {
                fprintf(stderr, "PDO entry registration failed: slave=%zu\n", slave);
                return IGH_ECAN_FAIL;
            }
        }
    }
    return IGH_ECAN_OK;
}

/* Activate the master, obtain the process image and cache PDO views. */
static int activate_master_domain(ecan_igh_context_t *ctx)
{
    if (ctx->dc_assign_activate != 0U) {
        uint64_t cycle_ns = cycle_us_to_ns(ctx->cycle_us);

        if (cycle_ns != 0U) {
            uint64_t now_ns = monotonic_ns();

            if (now_ns == 0U) {
                return IGH_ECAN_FAIL;
            }

            /* IgH derives the first Sync0 start time while activating the
             * master. Give every DC mode the same absolute period grid as its
             * runtime loop instead of a random launch phase. */
            ctx->cycle_start_ns = now_ns - (now_ns % cycle_ns);
        }
    }
    /* IgH仅首次application_time锁存dc_ref_time作为相位原点。保留网格
     * 原点，但在激活前立即补交当前时间，避免以旧网格初始化ESC时钟。 */
    uint64_t seed_ns = ctx->cycle_start_ns ? ctx->cycle_start_ns : monotonic_ns();
    if (seed_ns == 0U) return IGH_ECAN_FAIL;
    ecrt_master_application_time(ctx->master, seed_ns);
    update_dc_application_time(ctx);

    if (ecrt_master_activate(ctx->master) != 0) {
        fprintf(stderr, "master activate failed\n");
        return IGH_ECAN_FAIL;
    }

    if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            ctx->rx_domain_pds[slave] = ecrt_domain_data(ctx->rx_domains[slave]);
            ctx->tx_domain_pds[slave] = ecrt_domain_data(ctx->tx_domains[slave]);
            if (ctx->rx_domain_pds[slave] == NULL || ctx->tx_domain_pds[slave] == NULL) {
                return IGH_ECAN_NOT_READY;
            }
            ctx->slaves[slave].rx_domain_pd = ctx->rx_domain_pds[slave];
            ctx->slaves[slave].tx_domain_pd = ctx->tx_domain_pds[slave];
        }
    } else {
        for (size_t slave = 0U; slave < ctx->slave_count; slave++) {
            ctx->domain_pds[slave] = ecrt_domain_data(ctx->domains[slave]);
            if (ctx->domain_pds[slave] == NULL) {
                return IGH_ECAN_NOT_READY;
            }
            ctx->slaves[slave].domain_pd = ctx->domain_pds[slave];
            ctx->slaves[slave].rx_domain_pd = ctx->domain_pds[slave];
            ctx->slaves[slave].tx_domain_pd = ctx->domain_pds[slave];
        }
    }
    ctx->domain_pd = (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) ?
                     ctx->rx_domain_pds[0] : ctx->domain_pds[0];
    ecan_igh_adapter_cache_slave_domain_views(ctx);
    update_dc_application_time(ctx);
    return IGH_ECAN_OK;
}

/* Pure grid calculation: early preparation never shifts DC clock phases. */
static uint64_t dc_period_start_ns(uint64_t now, uint64_t period, uint32_t ahead_us)
{
    if (period == 0U) return now;
    uint64_t start = now - now % period;
    uint64_t ahead = (uint64_t)ahead_us * 1000ULL;
    if (ahead != 0U && ahead < period && now - start >= period - ahead)
        start += period;
    return start;
}

static int begin_cycle_stats(ecan_igh_context_t *ctx)
{
    uint64_t now_ns;
    uint64_t cycle_ns;
    uint64_t period_start_ns;
    uint64_t previous_begin_ns;

    if (ctx == NULL) {
        return IGH_ECAN_INVALID_ARG;
    }
    if (ctx->sm_receive_fault != 0U || ctx->dc_receive_fault != 0U)
        return IGH_ECAN_TIMEOUT;

    now_ns = monotonic_ns();
    if (now_ns == 0U) {
        return IGH_ECAN_FAIL;
    }
    cycle_ns = cycle_us_to_ns(ctx->cycle_us);
    period_start_ns = now_ns;
    if (ctx->dc_assign_activate != 0U && cycle_ns != 0U) {
        /* All DC exchanges must stay tied to one absolute period grid.
         * Basing application time on the thread's actual wake-up moves the
         * EtherCAT frame relative to the free-running Sync0 whenever Linux
         * wakes late, including the ordinary cross-cycle mode. */
        period_start_ns = dc_period_start_ns(now_ns, cycle_ns, ctx->dc_prepare_ahead_us);
        /* An explicitly configured early caller prepares the upcoming grid
         * period, not a second command for the period still finishing. The
         * send path still waits for that period's configured wire phase. */
        /*
         * The OP-wait loop also exchanges frames on this grid.  An
         * application may enter its first DC cycle immediately after OP, but
         * it must not reuse the period that emitted the final wait frame.
         */
        if (ctx->slaves_operational != 0U && ctx->cycle_start_ns != 0U &&
            period_start_ns <= ctx->cycle_start_ns) {
            period_start_ns = ctx->cycle_start_ns + cycle_ns;
        }
    }
    previous_begin_ns = ctx->cycle_begin_ns;
    ctx->cycle_begin_ns = now_ns;
    ctx->cycle_start_ns = period_start_ns;
    ctx->sched_stats.ethercat_cycle++;
    if (previous_begin_ns != 0U && cycle_ns != 0U) {
        uint64_t expected_ns = previous_begin_ns + cycle_ns;
        if (now_ns > expected_ns) {
            uint64_t late_ns = now_ns - expected_ns;
            ecan_igh_adapter_update_max_u64(&ctx->sched_stats.cycle_late_max_ns, late_ns);
            if (late_ns > (cycle_ns / 10U)) {
                ctx->sched_stats.cycle_overrun_count++;
                record_diag_event(ctx, ECAN_IGH_DIAG_EVENT_CYCLE_OVERRUN, late_ns, NULL);
            }
        }
    }
    return IGH_ECAN_OK;
}

static void finish_cycle_stats(ecan_igh_context_t *ctx)
{
    uint64_t now_ns;

    if ((ctx == NULL) || ctx->cycle_begin_ns == 0U) {
        return;
    }

    now_ns = monotonic_ns();
    if (now_ns >= ctx->cycle_begin_ns) {
        ecan_igh_adapter_update_max_u64(&ctx->sched_stats.cycle_exec_max_ns,
                                        now_ns - ctx->cycle_begin_ns);
    }
}

/* Public runtime API                                                          */
/* -------------------------------------------------------------------------- */

/* Initialize the IgH master, slaves, startup SDOs and process-data domain. */
int ecan_igh_init(ecan_igh_context_t *ctx, const ecan_igh_config_t *config)
{
    int rc;

    if ((ctx == NULL) || !ecan_igh_adapter_config_is_valid(config)) {
        return IGH_ECAN_INVALID_ARG;
    }

    rc = init_runtime_config(ctx, config);
    if (rc != IGH_ECAN_OK) {
        return rc;
    }

    rc = open_master_domain(ctx, config);
    if (rc != IGH_ECAN_OK) {
        return init_fail(ctx, rc);
    }

    rc = configure_slaves(ctx, config);
    if (rc != IGH_ECAN_OK) {
        return init_fail(ctx, rc);
    }

    if ((ctx->init_flags & ECAN_IGH_INIT_SKIP_STARTUP_SDO) == 0U) {
        rc = ecan_igh_adapter_apply_startup_sdos(ctx, config);
        if (rc != IGH_ECAN_OK) {
            return init_fail(ctx, rc);
        }
    }

    rc = register_pdo_entries(ctx);
    if (rc != IGH_ECAN_OK) {
        return init_fail(ctx, rc);
    }

    rc = activate_master_domain(ctx);
    if (rc != IGH_ECAN_OK) {
        return init_fail(ctx, rc);
    }

    if ((ctx->init_flags & ECAN_IGH_INIT_PRINT_PDO_OFFSETS) != 0U ||
        getenv("ECAN_IGH_PRINT_PDO_OFFSETS") != NULL) {
        ecan_igh_print_pdo_offsets(ctx);
    }
    return IGH_ECAN_OK;
}

int ecan_igh_adapter_dc_phase_matches(const uint8_t *data,
                                      uint32_t cycle_ns, uint32_t phase0_ns,
                                      uint32_t offset_ns)
{
    return data != NULL && cycle_ns != 0U && offset_ns != 0U &&
        phase0_ns < cycle_ns && offset_ns < cycle_ns &&
        EC_READ_U32(data + 16U) == cycle_ns && EC_READ_U32(data + 20U) == offset_ns &&
        EC_READ_U64(data) % cycle_ns == phase0_ns &&
        EC_READ_U64(data + 8U) % cycle_ns == ((uint64_t)phase0_ns + offset_ns) % cycle_ns;
}

/* Read back the ESC start grid before releasing an application to send CAN.
 * Some IgH versions align startup to Sync0 cycle + Sync1 offset. The interval
 * registers can be correct while the absolute phase is wrong. Never silently
 * accept that configuration as a verified Sync01 startup. */
static int verify_dc_start_phase(ecan_igh_context_t *ctx)
{
    if (ctx->dc_assign_activate != ECAN_IGH_DC_ASSIGN_SYNC01 ||
        ctx->dc_phase_check_complete != 0U) return IGH_ECAN_OK;
    if (ctx->dc_phase_check_started == 0U) {
        for (size_t i = 0U; i < ctx->slave_count; i++)
            if (ecrt_reg_request_state(ctx->dc_phase_requests[i]) == EC_REQUEST_BUSY)
                return IGH_ECAN_NOT_READY;
        for (size_t i = 0U; i < ctx->slave_count; i++)
            ecrt_reg_request_read(ctx->dc_phase_requests[i], 0x0990U, 24U);
        ctx->dc_phase_check_started = 1U;
        return IGH_ECAN_NOT_READY;
    }
    for (size_t i = 0U; i < ctx->slave_count; i++) {
        ec_request_state_t state = ecrt_reg_request_state(ctx->dc_phase_requests[i]);
        if (state == EC_REQUEST_BUSY || state == EC_REQUEST_UNUSED) return IGH_ECAN_NOT_READY;
        if (state != EC_REQUEST_SUCCESS) return IGH_ECAN_FAIL;
    }
    for (size_t i = 0U; i < ctx->slave_count; i++) {
        const uint8_t *data = ecrt_reg_request_data(ctx->dc_phase_requests[i]);
        uint32_t cycle = EC_READ_U32(data + 16U);
        uint32_t offset = EC_READ_U32(data + 20U);
        uint32_t phase0 = cycle ? (uint32_t)(EC_READ_U64(data) % cycle) : 0U;
        uint32_t phase1 = cycle ? (uint32_t)(EC_READ_U64(data + 8U) % cycle) : 0U;
        int valid = ecan_igh_adapter_dc_phase_matches(data, ctx->dc_sync0_cycle_ns,
                         (uint32_t)ctx->dc_sync0_shift_ns, ctx->dc_sync1_offset_ns);
        fprintf(stderr, "[DC-PHASE] slave=%d sync0=%u sync1=%u cycle=%u offset=%u ns %s\n",
                ctx->slaves[i].config.position, phase0, phase1, cycle, offset,
                valid ? "verified" : "MISMATCH: check IgH DC start-grid alignment");
        if (!valid) return IGH_ECAN_FAIL;
    }
    ctx->dc_phase_check_complete = 1U;
    return IGH_ECAN_OK;
}

/* OP and WKC alone do not establish the requested DC phase. Sync01 startup
 * additionally verifies the ESC start grid before returning ready. */
int ecan_igh_poll_op(ecan_igh_context_t *ctx)
{
    int all_op;
    int rc;

    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->domain_pd != NULL &&
                       ctx->lock_initialized != 0U);
    rc = pthread_mutex_trylock(&ctx->lock);
    if (rc != 0) {
        return (rc == EBUSY) ? IGH_ECAN_NOT_READY : IGH_ECAN_FAIL;
    }
    if (ctx->stopping || ctx->pdo_queue_uncommitted) {
        (void)pthread_mutex_unlock(&ctx->lock);
        return IGH_ECAN_FAIL;
    }
    rc = begin_cycle_stats(ctx);
    if (rc != IGH_ECAN_OK) {
        (void)pthread_mutex_unlock(&ctx->lock);
        return rc;
    }
    receive_domain_state(ctx, 1);
    all_op = (ctx->slaves_operational != 0U && ctx->domain_complete != 0U);
    ctx->startup_poll_started = 1U;
    int send_rc = send_domain_state(ctx);
    if (ctx->slaves_operational == 0U) {
        ctx->dc_phase_check_started = 0U;
        ctx->dc_phase_check_complete = 0U;
    }
    rc = send_rc != IGH_ECAN_OK ? send_rc :
         (all_op ? verify_dc_start_phase(ctx) : IGH_ECAN_NOT_READY);
    finish_cycle_stats(ctx);
    if (pthread_mutex_unlock(&ctx->lock) != 0) {
        return IGH_ECAN_FAIL;
    }
    return rc;
}

/* 启动诊断每次最多读取8个从站，避免大拓扑的一次遍历阻塞配置报文。
 * 别名按IgH规则取首次出现的位置，再加相对位置；不把别名偏移当环位置。
 * 状态只在一次wait调用内使用，扫描/链路变化或读取失败后重新解析。 */
typedef struct {
    unsigned int cursor;
    unsigned int slave_count;
    uint32_t alias_base[ECAN_IGH_MAX_SLAVES];
} ecan_igh_startup_probe_t;

static int startup_configuration_error(ecan_igh_context_t *ctx,
                                       ecan_igh_startup_probe_t *probe)
{
    ec_master_info_t master_info;

    if (ecrt_master(ctx->master, &master_info) != 0 ||
        master_info.scan_busy || !master_info.link_up || master_info.slave_count == 0U) {
        probe->cursor = 0U;
        return IGH_ECAN_NOT_READY;
    }
    if (probe->slave_count != master_info.slave_count) {
        probe->slave_count = master_info.slave_count;
        probe->cursor = 0U;
    }
    for (unsigned int budget = 0U; budget < 8U; budget++) {
        ec_slave_info_t info;
        if (probe->cursor == 0U) {
            for (size_t i = 0U; i < ctx->slave_count; i++) probe->alias_base[i] = UINT32_MAX;
        }
        if (probe->cursor > UINT16_MAX ||
            ecrt_master_get_slave(ctx->master, (uint16_t)probe->cursor, &info) != 0) {
            probe->cursor = 0U;
            return IGH_ECAN_NOT_READY; /* 查询失败不是从站配置失败，不使用未初始化结果。 */
        }
        for (size_t i = 0U; i < ctx->slave_count; i++) {
            const ecan_igh_slave_config_t *config = &ctx->slaves[i].config;
            uint32_t base = 0U;
            if (config->alias != 0) {
                if (probe->alias_base[i] == UINT32_MAX && info.alias == config->alias)
                    probe->alias_base[i] = probe->cursor;
                base = probe->alias_base[i];
                if (base == UINT32_MAX) continue;
            }
            if (base + (uint32_t)config->position != probe->cursor || !info.error_flag ||
                info.vendor_id != config->vendor_id || info.product_code != config->product_id)
                continue;
            ec_slave_config_state_t state = {0};
            ecrt_slave_config_state(ctx->slaves[i].sc, &state);
            /* 未挂接的配置、扫描中的拓扑不能借用相邻同型号从站的错误。 */
            ec_master_info_t confirm;
            if (!state.online || ecrt_master(ctx->master, &confirm) != 0 ||
                confirm.scan_busy || !confirm.link_up || confirm.slave_count != probe->slave_count) {
                probe->cursor = 0U;
                return IGH_ECAN_NOT_READY;
            }
            fprintf(stderr, "EtherCAT startup configuration failed: ring=%u alias=%d position=%d al=0x%02x\n",
                    probe->cursor, config->alias, config->position, info.al_state);
            return IGH_ECAN_FAIL;
        }
        if (++probe->cursor >= probe->slave_count) {
            probe->cursor = 0U;
            break;
        }
    }
    return IGH_ECAN_NOT_READY;
}

/* Wait until every slave reaches OP, a confirmed configuration error, or timeout. */
int ecan_igh_wait_op_timeout(ecan_igh_context_t *ctx, uint32_t timeout_ms)
{
    const unsigned int required_ready_cycles = 3U;
    uint64_t start_ms;
    uint64_t cycle_ns;
    uint64_t phase_ns;
    uint64_t next_poll_ns;
    unsigned int ready_cycles = 0U;
    uint64_t next_error_check_ns = 0U;
    ecan_igh_startup_probe_t startup_probe = {0};

    ECAN_IGH_CHECK_ARG(ctx != NULL);

    start_ms = monotonic_ms();
    if (start_ms == 0U) {
        return IGH_ECAN_FAIL;
    }
    /* 启动时以独立1kHz推进配置FSM、邮箱及DC校时。沿用10/20ms业务周期会
     * 同时拖慢配置报文及校时，导致等待OP超时或5秒同步等待告警。
     * 不提高到250us，避免大domain启动探测重用在途报文；每次仍先收后发。
     * 首次完整PDO或到达OP后恢复不快于业务周期的检查节拍：单缓冲TxPDO可能每个DC周期
     * 只发布一次，不能在两次发布之间反复排队并把未完整WKC当作成功。
     * 不改ctx中的业务周期、Sync0/Sync1和收发相位。此接口只用于业务开始前，
     * 应用仍须通过空命令预热建立会话，不能拿它代替正式周期收发。
     * Service startup independently of the application period, at a bounded 1kHz. */
    cycle_ns = 1000000ULL;
    /* 分离DC的启动探测同样包含输入读取。固定在相位0会绕过用户为
     * Sync0/Sync1发布预留的读取窗口，尤其跨周期Sync1或仅Sync0时可与
     * 单缓冲发布争用。沿用配置的输入读取相位，不改变配置FSM的1kHz
     * 节拍、不放宽连续完整WKC及DC相位校验。 */
    phase_ns = ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE &&
               ctx->rxpdo_phase_us < ctx->txpdo_phase_us ?
               (uint64_t)ctx->txpdo_phase_us * 1000ULL : 0U;
    next_poll_ns = next_cycle_phase_ns(monotonic_ns(), cycle_ns, phase_ns);
    if (next_poll_ns == 0U) {
        return IGH_ECAN_FAIL;
    }
    for (;;) {
        int rc;

        /*
         * A relative usleep(cycle_us) adds receive/process/send time to each
         * interval. Use an absolute startup grid and skip missed ticks.
         */
        if (wait_until_ns(next_poll_ns) != IGH_ECAN_OK) {
            return IGH_ECAN_FAIL;
        }
        rc = ecan_igh_poll_op(ctx);
        if (rc == IGH_ECAN_OK) {
            ready_cycles++;
            if (ready_cycles >= required_ready_cycles) {
                return IGH_ECAN_OK;
            }
        }
        else if (rc == IGH_ECAN_NOT_READY) {
            ready_cycles = 0U;
            uint64_t now_ns = monotonic_ns();
            if (now_ns == 0U) return IGH_ECAN_FAIL;
            if (now_ns >= next_error_check_ns) {
                if (startup_configuration_error(ctx, &startup_probe) == IGH_ECAN_FAIL)
                    return IGH_ECAN_FAIL;
                next_error_check_ns = now_ns + 10000000ULL;
            }
        }
        else {
            return rc;
        }
        if (timeout_ms != 0U) {
            uint64_t now_ms = monotonic_ms();

            if (now_ms == 0U) {
                return IGH_ECAN_FAIL;
            }
            if (now_ms - start_ms >= timeout_ms) {
                return IGH_ECAN_TIMEOUT;
            }
        }
        {
            uint64_t now_ns = monotonic_ns();

            if (now_ns == 0U) {
                return IGH_ECAN_FAIL;
            }
            /* SAFEOP也能返回完整输入，不能等状态缓存更新到OP才切换。
             * 一旦切换不再因短暂不完整WKC加快重排；原接收保护仍决定失败。 */
            if ((ctx->domain_complete != 0U || ctx->slaves_operational != 0U) &&
                ctx->cycle_us > 1000U && cycle_ns == 1000000ULL) {
                cycle_ns = (uint64_t)ctx->cycle_us * 1000ULL;
                phase_ns = (uint64_t)(ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE &&
                    ctx->rxpdo_phase_us < ctx->txpdo_phase_us ?
                    ctx->txpdo_phase_us : ctx->rxpdo_phase_us) * 1000ULL;
                /* 等待至少一个完整发布周期，再对齐正式发包相位。只拉长间隔而
                 * 保留任意启动相位，末次探测可能提前取走首个业务周期需要的输入。 */
                next_poll_ns = next_cycle_phase_ns(now_ns + cycle_ns, cycle_ns, phase_ns);
                continue;
            }
            next_poll_ns = advance_cycle_deadline_ns(next_poll_ns,
                                                     cycle_ns,
                                                     now_ns);
        }
    }
}

/* Wait indefinitely until every configured slave reaches OP. */
int ecan_igh_wait_op(ecan_igh_context_t *ctx)
{
    return ecan_igh_wait_op_timeout(ctx, 0U);
}

/* 迟到发生在 begin 之前时，尚未生成新命令，可选择新的安全发送期限。
 * 迟到发生在 begin/end 之间时则由 send 拒绝；禁止把已生成的命令跨周期保留。
 * 这里只记录重定位；原 begin 间隔和超周期统计仍照常采样。 */
static void prepare_sm_begin(ecan_igh_context_t *ctx)
{
    if (ctx->dc_assign_activate != 0U || !ctx->split_cycle_collected ||
        ctx->sm_receive_fault || ctx->sm_pending_mask) return;
    uint64_t now = monotonic_ns();
    uint64_t period = (uint64_t)ctx->cycle_us * 1000ULL;
    if (now > ctx->sm_next_send_ns + period / 8U) {
        uint64_t next = now + period / 4U;
        uint64_t safe = ctx->sm_last_send_ns + period;
        ctx->sm_next_send_ns = next > safe ? next : safe;
        ctx->sched_stats.sm_pacing_rebases++;
    }
}

/* 两种锁获取接口共用同一入口事务，避免故障路径与输入代次处理分叉。
 * already-collected 仅在完整 WKC 和 shadow 提交之后设置；不以调用次数代替确认。 */
static int begin_cycle_locked(ecan_igh_context_t *ctx)
{
    int rc;
    if (ctx->cycle_active != 0U || ctx->stopping != 0U || ctx->pdo_queue_uncommitted != 0U) {
        (void)pthread_mutex_unlock(&ctx->lock);
        return IGH_ECAN_NOT_READY;
    }
    prepare_sm_begin(ctx);
    ctx->business_started = 1U;
    ctx->cycle_active = 1U;
    rc = begin_cycle_stats(ctx);
    if (rc != IGH_ECAN_OK) {
        ctx->cycle_active = 0U;
        (void)pthread_mutex_unlock(&ctx->lock);
        return rc;
    }
    if (ecan_igh_collects_at_cycle_end(ctx) &&
        ctx->split_cycle_collected != 0U) {
        /*
         * The previous cycle already received and processed every queued
         * datagram at its collect phase.  A second receive here can consume a
         * late direction-domain response without processing its generation,
         * which shows up as alternating incomplete WKC samples.
         */
        /* Clock synchronization and AL-state polling already occur in the
         * end phase. Avoid repeating their ioctls before new commands. */
    } else {
        receive_domain_state(ctx, 1);
        if (ctx->sm_receive_fault != 0U || ctx->dc_receive_fault != 0U) {
            finish_cycle_stats(ctx);
            ctx->cycle_active = 0U;
            (void)pthread_mutex_unlock(&ctx->lock);
            return IGH_ECAN_TIMEOUT;
        }
        ecan_igh_adapter_copy_domain_txpdo_to_shadow(ctx);
    }
    return IGH_ECAN_OK;
}

/* Begin a blocking application cycle and refresh TxPDO input shadows. */
int ecan_igh_cycle_begin_checked(ecan_igh_context_t *ctx)
{
    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->domain_pd != NULL &&
                       ctx->lock_initialized != 0U);
    if (pthread_mutex_lock(&ctx->lock) != 0) return IGH_ECAN_FAIL;
    return begin_cycle_locked(ctx);
}

/* Begin a cycle only if the runtime lock can be acquired immediately. */
int ecan_igh_cycle_try_begin(ecan_igh_context_t *ctx)
{
    int lock_rc;

    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->domain_pd != NULL &&
                       ctx->lock_initialized != 0U);
    lock_rc = pthread_mutex_trylock(&ctx->lock);
    if (lock_rc != 0) {
        return (lock_rc == EBUSY) ? IGH_ECAN_NOT_READY : IGH_ECAN_FAIL;
    }
    return begin_cycle_locked(ctx);
}

/* SM 同周期回收：不发送第二个读报文，不重排未确认的域。
 * 超时保留旧 shadow 并锁存停发；完整返回才发布本代输入。
 * begin 仍可读取上次 end 的 shadow，兼容已有先收后发的应用调用顺序。 */
static int finish_sm_cycle(ecan_igh_context_t *ctx)
{
    int rc = send_domain_state(ctx);
    ctx->split_cycle_collected = 0U;
    if (rc != IGH_ECAN_OK) return rc;
    receive_domain_state(ctx, 1);
    if (ctx->sm_receive_fault != 0U) return IGH_ECAN_TIMEOUT;
    /* 完整回包可能提前到达；固定本次输入交付点，避免回包抖动直接成为
     * 下一次应用 begin 的抖动。保留末四分之一周期给下一批命令生成。
     * 迟到不追回、不清统计；单次内核调用超出边界仍会暴露为超周期。 */
    uint64_t handoff = ctx->sm_next_send_ns - (uint64_t)ctx->cycle_us * 250ULL;
    if (monotonic_ns() < handoff &&
        wait_until_ns_tail(handoff, 100000ULL) != IGH_ECAN_OK) {
        ctx->sm_receive_fault = 1U;
        return IGH_ECAN_FAIL;
    }
    ecan_igh_adapter_copy_domain_txpdo_to_shadow(ctx);
    ctx->split_cycle_collected = 1U;
    return IGH_ECAN_OK;
}

/* Publish RxPDO outputs, send the domain and release the cycle lock. */
int ecan_igh_cycle_end_checked(ecan_igh_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->domain_pd != NULL);
    ECAN_IGH_RETURN_IF(ctx->cycle_active == 0U, IGH_ECAN_NOT_READY);
    if (ctx->dc_assign_activate == 0U) {
        ecan_igh_adapter_copy_shadow_rxpdo_to_domain(ctx);
        rc = finish_sm_cycle(ctx);
    } else if (ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE &&
        ctx->rxpdo_phase_us < ctx->txpdo_phase_us) {
        rc = finish_dc_single_cycle(ctx);
    } else if (ecan_igh_collects_at_cycle_end(ctx)) {
        rc = finish_dc_combined_cycle(ctx, 1);
    } else {
        ecan_igh_adapter_copy_shadow_rxpdo_to_domain(ctx);
        rc = send_domain_state(ctx);
    }
    finish_cycle_stats(ctx);
    ctx->cycle_active = 0U;
    if (pthread_mutex_unlock(&ctx->lock) != 0) {
        return IGH_ECAN_FAIL;
    }
    return rc;
}

/* Release an active cycle without publishing new RxPDO shadow contents. */
int ecan_igh_cycle_abort_checked(ecan_igh_context_t *ctx)
{
    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->lock_initialized != 0U);
    ECAN_IGH_RETURN_IF(ctx->cycle_active == 0U, IGH_ECAN_NOT_READY);
    ctx->cycle_active = 0U;
    return (pthread_mutex_unlock(&ctx->lock) == 0) ?
           IGH_ECAN_OK : IGH_ECAN_FAIL;
}

/* Resend the current process image without changing application PDO buffers. */
int ecan_igh_cycle_repeat_output_checked(ecan_igh_context_t *ctx)
{
    int rc = IGH_ECAN_OK;

    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->domain_pd != NULL &&
                       ctx->lock_initialized != 0U);
    if (pthread_mutex_lock(&ctx->lock) != 0) {
        return IGH_ECAN_FAIL;
    }
    if (ctx->cycle_active != 0U || ctx->stopping != 0U || ctx->pdo_queue_uncommitted != 0U) {
        (void)pthread_mutex_unlock(&ctx->lock);
        return IGH_ECAN_NOT_READY;
    }
    ctx->business_started = 1U;
    ctx->cycle_active = 1U;
    rc = begin_cycle_stats(ctx);
    if (rc != IGH_ECAN_OK) {
        ctx->cycle_active = 0U;
        (void)pthread_mutex_unlock(&ctx->lock);
        return rc;
    }
    if (ctx->dc_assign_activate == 0U) {
        /* Explicit repeat API only; a scheduler must never call this to catch up.
         * Drain any startup ownership before using the existing process image. */
        if (!ctx->split_cycle_collected) receive_domain_state(ctx, 0);
        rc = finish_sm_cycle(ctx);
    } else if (ecan_igh_collects_at_cycle_end(ctx)) {
        if (ctx->split_cycle_collected != 0U) {
            /* Previous generation was retired in the end phase. */
        } else {
            receive_domain_state(ctx, 0);
        }
        rc = ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE &&
            ctx->rxpdo_phase_us < ctx->txpdo_phase_us ?
            finish_dc_single_cycle(ctx) : finish_dc_combined_cycle(ctx, 0);
    } else {
        receive_domain_state(ctx, 0);
        rc = send_domain_state(ctx);
    }
    finish_cycle_stats(ctx);
    ctx->cycle_active = 0U;
    if (pthread_mutex_unlock(&ctx->lock) != 0) {
        return IGH_ECAN_FAIL;
    }
    return rc;
}

void ecan_igh_cycle_begin(ecan_igh_context_t *ctx)
{
    (void)ecan_igh_cycle_begin_checked(ctx);
}

void ecan_igh_cycle_end(ecan_igh_context_t *ctx)
{
    (void)ecan_igh_cycle_end_checked(ctx);
}

void ecan_igh_cycle_abort(ecan_igh_context_t *ctx)
{
    (void)ecan_igh_cycle_abort_checked(ctx);
}

void ecan_igh_cycle_repeat_output(ecan_igh_context_t *ctx)
{
    (void)ecan_igh_cycle_repeat_output_checked(ctx);
}

int ecan_igh_is_dc_single_cycle(const ecan_igh_context_t *ctx)
{
    return (ctx != NULL &&
            ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) ? 1 : 0;
}

int ecan_igh_collects_at_cycle_end(const ecan_igh_context_t *ctx)
{
    return ctx != NULL && (ctx->dc_assign_activate == 0U ||
                            ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE ||
                            ctx->cycle_mode == ECAN_IGH_CYCLE_MODE_DC_COMBINED);
}

/* Return whether the last sampled domain working counter was complete. */
int ecan_igh_domain_is_complete(const ecan_igh_context_t *ctx)
{
    return (ctx != NULL && ctx->domain_complete != 0U) ? 1 : 0;
}

/* Return the last sampled IgH domain working counter. */
unsigned int ecan_igh_domain_working_counter(const ecan_igh_context_t *ctx)
{
    return (ctx != NULL) ? ctx->domain_working_counter : 0U;
}

/* Return the last sampled IgH domain working-counter state. */
ec_wc_state_t ecan_igh_domain_wc_state(const ecan_igh_context_t *ctx)
{
    return (ctx != NULL) ? ctx->domain_wc_state : EC_WC_ZERO;
}

/* Return whether every configured slave was OP in the last sample. */
int ecan_igh_slaves_are_operational(const ecan_igh_context_t *ctx)
{
    return (ctx != NULL && ctx->slaves_operational != 0U) ? 1 : 0;
}

/* Return whether one configured slave was OP in the last sample. */
int ecan_igh_slave_is_operational(const ecan_igh_context_t *ctx, size_t slave_pos)
{
    if ((ctx == NULL) || slave_pos >= ctx->slave_count) {
        return 0;
    }
    return (ctx->slave_operational[slave_pos] != 0U) ? 1 : 0;
}

/* Return one slave's last sampled EtherCAT AL state. */
unsigned int ecan_igh_slave_al_state(const ecan_igh_context_t *ctx, size_t slave_pos)
{
    if ((ctx == NULL) || slave_pos >= ctx->slave_count) {
        return 0U;
    }
    return ctx->slave_al_state[slave_pos];
}

int ecan_igh_get_sched_stats(const ecan_igh_context_t *ctx,
                             ecan_igh_sched_stats_t *stats)
{
    ecan_igh_context_t *mutable_ctx;
    int lock_rc;

    ECAN_IGH_CHECK_ARG(ctx != NULL && stats != NULL &&
                       ctx->lock_initialized != 0U);
    mutable_ctx = (ecan_igh_context_t *)(uintptr_t)ctx;
    lock_rc = pthread_mutex_trylock(&mutable_ctx->lock);
    if (lock_rc != 0) {
        return (lock_rc == EBUSY) ? IGH_ECAN_NOT_READY : IGH_ECAN_FAIL;
    }
    *stats = ctx->sched_stats;
    return (pthread_mutex_unlock(&mutable_ctx->lock) == 0) ?
           IGH_ECAN_OK : IGH_ECAN_FAIL;
}

int ecan_igh_reset_runtime_diagnostics(ecan_igh_context_t *ctx)
{
    uint32_t cycle_us;
    int lock_rc;

    ECAN_IGH_CHECK_ARG(ctx != NULL && ctx->lock_initialized != 0U);
    lock_rc = pthread_mutex_trylock(&ctx->lock);
    if (lock_rc != 0) {
        return (lock_rc == EBUSY) ? IGH_ECAN_NOT_READY : IGH_ECAN_FAIL;
    }
    if (ctx->cycle_active != 0U) {
        (void)pthread_mutex_unlock(&ctx->lock);
        return IGH_ECAN_NOT_READY;
    }
    cycle_us = ctx->cycle_us;
    memset(&ctx->sched_stats, 0, sizeof(ctx->sched_stats));
    ctx->sched_stats.cycle_us = cycle_us;
    memset(ctx->diag_events, 0, sizeof(ctx->diag_events));
    ctx->diag_event_head = 0U;
    ctx->diag_event_dropped = 0U;
    ctx->domain_expected_working_counter = 0U;
    ctx->cycle_start_ns = 0U;
    ctx->cycle_begin_ns = 0U;
    ctx->last_receive_process_ns = 0U;
    ctx->last_txpdo_copy_ns = 0U;
    ctx->last_rxpdo_copy_ns = 0U;
    ctx->last_queue_ns = 0U;
    ctx->last_master_send_ns = 0U;
    return (pthread_mutex_unlock(&ctx->lock) == 0) ?
           IGH_ECAN_OK : IGH_ECAN_FAIL;
}

static void get_diag_events_unlocked(const ecan_igh_context_t *ctx,
                                     uint64_t cursor,
                                     uint32_t max_events,
                                     ecan_igh_diag_event_snapshot_t *snapshot)
{
    uint64_t first_sequence;
    uint64_t start_sequence;
    uint32_t limit;

    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->head_sequence = ctx->diag_event_head;
    snapshot->dropped = ctx->diag_event_dropped;

    if (ctx->diag_event_head == 0U || max_events == 0U) {
        snapshot->cursor = cursor;
        snapshot->max_events = max_events;
        return;
    }

    limit = max_events;
    if (limit > ECAN_IGH_DIAG_EVENT_SNAPSHOT_MAX) {
        limit = ECAN_IGH_DIAG_EVENT_SNAPSHOT_MAX;
    }
    snapshot->max_events = limit;

    first_sequence = ctx->diag_event_dropped + 1U;
    if (cursor == 0U) {
        start_sequence = (ctx->diag_event_head >= limit) ?
                         (ctx->diag_event_head - limit + 1U) :
                         first_sequence;
    } else {
        start_sequence = cursor + 1U;
    }
    if (start_sequence < first_sequence) {
        start_sequence = first_sequence;
    }

    for (uint64_t sequence = start_sequence;
         sequence <= ctx->diag_event_head && snapshot->count < limit;
         sequence++) {
        snapshot->events[snapshot->count++] =
            ctx->diag_events[(sequence - 1U) % ECAN_IGH_DIAG_EVENT_RING_SIZE];
        snapshot->cursor = sequence;
    }
    if (snapshot->count == 0U) {
        snapshot->cursor = cursor;
    }
}

int ecan_igh_get_diag_events(const ecan_igh_context_t *ctx,
                             uint64_t cursor,
                             uint32_t max_events,
                             ecan_igh_diag_event_snapshot_t *snapshot)
{
    ecan_igh_context_t *mutable_ctx;
    int lock_rc;

    ECAN_IGH_CHECK_ARG(ctx != NULL && snapshot != NULL &&
                       ctx->lock_initialized != 0U);
    mutable_ctx = (ecan_igh_context_t *)(uintptr_t)ctx;
    lock_rc = pthread_mutex_trylock(&mutable_ctx->lock);
    if (lock_rc != 0) {
        return (lock_rc == EBUSY) ? IGH_ECAN_NOT_READY : IGH_ECAN_FAIL;
    }
    get_diag_events_unlocked(ctx, cursor, max_events, snapshot);
    return (pthread_mutex_unlock(&mutable_ctx->lock) == 0) ?
           IGH_ECAN_OK : IGH_ECAN_FAIL;
}

void ecan_igh_print_runtime_diagnostics(const ecan_igh_context_t *ctx,
                                        uint32_t max_events)
{
    ecan_igh_diag_event_snapshot_t snapshot;
    ecan_igh_sched_stats_t stats;
    ecan_igh_context_t *mutable_ctx;
    unsigned int wkc;
    unsigned int expected_wkc;
    ec_wc_state_t wc_state;
    uint8_t all_op;
    uint64_t event_head;
    uint64_t event_dropped;
    int lock_rc;

    if (ctx == NULL || ctx->lock_initialized == 0U) {
        return;
    }

    mutable_ctx = (ecan_igh_context_t *)(uintptr_t)ctx;
    lock_rc = pthread_mutex_trylock(&mutable_ctx->lock);
    if (lock_rc != 0) {
        printf("[diag] igh-runtime snapshot=%s\n",
               (lock_rc == EBUSY) ? "busy" : "lock-error");
        fflush(stdout);
        return;
    }
    stats = ctx->sched_stats;
    wkc = ctx->domain_working_counter;
    expected_wkc = ctx->domain_expected_working_counter;
    wc_state = ctx->domain_wc_state;
    all_op = ctx->slaves_operational;
    event_head = ctx->diag_event_head;
    event_dropped = ctx->diag_event_dropped;
    get_diag_events_unlocked(ctx, 0U, max_events, &snapshot);
    if (pthread_mutex_unlock(&mutable_ctx->lock) != 0) {
        printf("[diag] igh-runtime snapshot=unlock-error\n");
        fflush(stdout);
        return;
    }

    printf("[diag] igh-runtime cycles=%" PRIu64 " cycle_us=%u"
           " cycle_exec_max_ns=%" PRIu64 " cycle_late_max_ns=%" PRIu64
           " overruns=%" PRIu64 " receive_process_max_ns=%" PRIu64
           " txpdo_copy_max_ns=%" PRIu64 " rxpdo_copy_max_ns=%" PRIu64
           " queue_max_ns=%" PRIu64 " master_send_max_ns=%" PRIu64
           " wkc_mismatches=%" PRIu64
           " wkc=%u expected=%u wc_state=%d all_op=%u events=%" PRIu64
           " dropped=%" PRIu64 "\n",
           stats.ethercat_cycle,
           stats.cycle_us,
           stats.cycle_exec_max_ns,
           stats.cycle_late_max_ns,
           stats.cycle_overrun_count,
           stats.receive_process_max_ns,
           stats.txpdo_copy_max_ns,
           stats.rxpdo_copy_max_ns,
           stats.queue_max_ns,
           stats.master_send_max_ns,
           stats.wkc_mismatch_count,
           wkc,
           expected_wkc,
           (int)wc_state,
           (unsigned)all_op,
           event_head,
           event_dropped);
    printf("[diag] igh-dc-admission samples=%" PRIu64 " host_slack_min_ns=%" PRId64
           " deadline_misses=%" PRIu64 "\n", stats.dc_prepare_samples,
           stats.dc_prepare_slack_min_ns, stats.dc_send_deadline_misses);
    printf("[diag] igh-sm late_receive_recovered=%" PRIu64
           " receive_timeouts=%" PRIu64 " pacing_waits=%" PRIu64
           " pacing_wait_max_ns=%" PRIu64 " pacing_rebases=%" PRIu64 "\n",
           stats.sm_late_receive_recovered, stats.sm_receive_timeouts,
           stats.sm_pacing_waits, stats.sm_pacing_wait_max_ns, stats.sm_pacing_rebases);
    printf("[diag] igh-dc late_receive_recovered=%" PRIu64
           " receive_timeouts=%" PRIu64 "\n",
           stats.dc_late_receive_recovered, stats.dc_receive_timeouts);
    printf("[diag] igh-sm-rx poll_calls=%" PRIu64 " waits=%" PRIu64
           " wait_max_ns=%" PRIu64 "\n", stats.sm_receive_poll_calls,
           stats.sm_receive_waits, stats.sm_receive_wait_max_ns);
    for (uint32_t i = 0U; i < snapshot.count; i++) {
        const ecan_igh_diag_event_t *event = &snapshot.events[i];
        const ecan_igh_diag_domain_sample_t *bad = NULL;

        if (event->first_bad_domain >= 0 &&
            (uint32_t)event->first_bad_domain < event->domain_count) {
            bad = &event->domains[event->first_bad_domain];
        }
        printf("[diag] igh-event seq=%" PRIu64 " cycle=%" PRIu64
               " type=%s wkc=%u expected=%u bad_domain=%d bad_slave=%u"
               " bad_kind=%u bad_wkc=%u bad_state=%u late_ns=%" PRIu64
               " recv_ns=%" PRIu64 " txcopy_ns=%" PRIu64
               " rxcopy_ns=%" PRIu64 " queue_ns=%" PRIu64
               " send_ns=%" PRIu64 " exec_ns=%" PRIu64
               " scope=%u receive_start_ns=%" PRIu64 " receive_deadline_ns=%" PRIu64
               " receive_observed_ns=%" PRIu64 " receive_attempts=%u receive_wc_state=%u"
               " receive_stop_reason=%u\n",
               event->sequence,
               event->ethercat_cycle,
               diag_event_name(event->event_type),
               event->wkc_current,
               event->wkc_expected,
               event->first_bad_domain,
               bad != NULL ? (unsigned)bad->slave_index : 0U,
               bad != NULL ? (unsigned)bad->kind : 0U,
               bad != NULL ? bad->working_counter : 0U,
               bad != NULL ? (unsigned)bad->wc_state : 0U,
               event->late_ns,
               event->receive_process_ns,
               event->txpdo_copy_ns,
               event->rxpdo_copy_ns,
               event->queue_ns,
               event->master_send_ns,
               event->cycle_exec_ns,
               event->flags, event->receive_start_ns, event->receive_deadline_ns,
               event->receive_observed_ns, event->receive_attempts,
               event->receive_wc_state, event->receive_stop_reason);
    }
    fflush(stdout);
}

/* Release the master, destroy runtime synchronization and clear the context. */
void ecan_igh_close(ecan_igh_context_t *ctx)
{
    int lock_rc;

    if (ctx == NULL) {
        return;
    }
    if (ctx->lock_initialized == 0U) {
        memset(ctx, 0, sizeof(*ctx));
        return;
    }
    lock_rc = pthread_mutex_trylock(&ctx->lock);
    if (lock_rc != 0) {
        fprintf(stderr, "ecan_igh_close: runtime is %s; close deferred\n",
                (lock_rc == EBUSY) ? "busy" : "not lockable");
        return;
    }
    if (ctx->master != NULL) {
        if (ctx->domain_pd != NULL) {
            int deactivate_rc;

            drain_master_before_close(ctx);
            deactivate_rc = ecrt_master_deactivate(ctx->master);
            if (deactivate_rc != 0) {
                fprintf(stderr, "ecan_igh_close: master deactivate failed: %d\n",
                        deactivate_rc);
            }
        }
        ecrt_release_master(ctx->master);
        ctx->master = NULL;
    }
    if (pthread_mutex_unlock(&ctx->lock) != 0 ||
        pthread_mutex_destroy(&ctx->lock) != 0) {
        fprintf(stderr, "ecan_igh_close: mutex cleanup failed\n");
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
}
