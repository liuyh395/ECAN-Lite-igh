#ifndef IGH_EXAMPLE_STOP_H
#define IGH_EXAMPLE_STOP_H

/* 示例层的标准IgH CLI接入；主站库本身不依赖外部进程。
 * 正常停机调用前：业务已经结束、应答按业务协议收齐、最后一个空PDO已完整回收。
 * 启动撤销、运行故障退出分别使用下方专用入口及其前提，不能冒充正常停机。
 * 不负责电机安全停止、预约取消或故障后的自动恢复。
 * 一个周期线程调用；不得与其他周期/close并发。CLI路径必须由可信部署提供。 */
#include "igh_mcan_stop.h"
#include "igh_rt_support.h"
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>

extern char **environ;

typedef struct {
    igh_mcan_stop_t transaction;
    uint64_t elapsed_us;
    int cli_status;
    int cli_error;
    pid_t unreaped_child; /* 非零：已请求终止但尚未回收；调用方不能报告正常退出。 */
} igh_example_stop_result_t;

static inline int igh_example_stop_impl(igh_mcan_port_t *port, int master_index,
                                         const char *cli, uint32_t timeout_ms,
                                         igh_example_stop_result_t *result, int kind)
{
    posix_spawnattr_t attr;
    struct sched_param normal = {0};
    sigset_t defaults, mask;
    char positions[ECAN_IGH_MAX_SLAVES * 7U + 1U] = {0};
    char master[20];
    size_t used = 0;
    pid_t child = -1;
    int rc, request = 0, reaped = 0;
    uint64_t start;
    igh_rt_periodic_clock_t clock;

    if (!result || !port || !port->igh || master_index < 0 || !cli || !*cli || !timeout_ms)
        return IGH_ECAN_INVALID_ARG;
    memset(result, 0, sizeof(*result));
    start = igh_rt_monotonic_ns();
    rc = kind==2 ? igh_mcan_fault_stop_prepare(&result->transaction, port, timeout_ms) :
         kind==1 ? igh_mcan_startup_abort_prepare(&result->transaction, port, timeout_ms) :
                   igh_mcan_stop_prepare(&result->transaction, port, timeout_ms);
    if (rc != IGH_ECAN_OK) goto finished;

    for (size_t n = 0; n < port->igh->slave_count; ++n) {
        int count = snprintf(positions + used, sizeof(positions) - used,
                             "%s%u", n ? "," : "", result->transaction.positions[n]);
        if (count < 0 || (size_t)count >= sizeof(positions) - used) {
            result->cli_error = EOVERFLOW;
            goto request_failed;
        }
        used += (size_t)count;
    }
    (void)snprintf(master, sizeof(master), "%d", master_index);
    char *args[] = {(char *)cli, "states", "-m", master, "-p", positions, "PREOP", NULL};
    rc = posix_spawnattr_init(&attr);
    if (rc) {result->cli_error = rc; goto request_failed;}
    /* CLI不继承实时优先级或应用的信号屏蔽；只请求已核对身份的目标。 */
    sigemptyset(&defaults); sigaddset(&defaults, SIGINT); sigaddset(&defaults, SIGTERM);
    sigemptyset(&mask);
    rc = posix_spawnattr_setschedpolicy(&attr, SCHED_OTHER);
    if (!rc) rc = posix_spawnattr_setschedparam(&attr, &normal);
    if (!rc) rc = posix_spawnattr_setsigdefault(&attr, &defaults);
    if (!rc) rc = posix_spawnattr_setsigmask(&attr, &mask);
    if (!rc) rc = posix_spawnattr_setflags(&attr, POSIX_SPAWN_SETSCHEDULER |
        POSIX_SPAWN_SETSCHEDPARAM | POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK);
    if (!rc) rc = posix_spawnp(&child, cli, NULL, &attr, args, environ);
    (void)posix_spawnattr_destroy(&attr);
    if (rc) {result->cli_error = rc; goto request_failed;}

    /* 管理轮询最多10kHz，不随短业务周期继续加速。每次poll包含多个主站/
     * 从站查询；250us/8的31us轮询可耗尽同核FIFO线程的空闲时间，使普通
     * 优先级CLI在exec阶段饥饿。空PDO仍由stop_poll按原周期且确认所有权后
     * 排队，不因查询节拍放宽WKC、重发旧命令或延长整个停止事务的期限。 */
    igh_rt_periodic_clock_start(&clock, 100U, 0, 0);
    do {
        (void)igh_rt_periodic_clock_wait(&clock);
        if (!reaped) {
            pid_t got = waitpid(child, &result->cli_status, WNOHANG);
            if (got == child) {
                reaped = 1;
                request = WIFEXITED(result->cli_status) && WEXITSTATUS(result->cli_status) == 0 ? 1 : -1;
            } else if (got < 0 && errno != EINTR) {
                result->cli_error = errno;
                request = -1;
                /* ECHILD意味着已没有本进程可回收的子进程；不能再kill可能复用的PID。 */
                if (errno == ECHILD) reaped = 1;
            }
        }
        rc = igh_mcan_stop_poll(&result->transaction, request);
    } while (rc == IGH_ECAN_NOT_READY);

    if (!reaped) {
        (void)kill(child, SIGKILL);
        uint64_t cleanup_end = igh_rt_monotonic_ns() + 20000000ULL;
        do {
            pid_t got = waitpid(child, &result->cli_status, WNOHANG);
            if (got == child || (got < 0 && errno == ECHILD)) {reaped = 1; break;}
            (void)igh_rt_periodic_clock_wait(&clock);
        } while (igh_rt_monotonic_ns() < cleanup_end);
        if (!reaped) result->unreaped_child = child;
        /* 即使状态随后到达，超时/管理进程故障仍然是失败。 */
        if (rc == IGH_ECAN_OK) rc = IGH_ECAN_FAIL;
    }
    goto finished;

request_failed:
    /* prepare之后的管理错误必须送入事务，锁存失败，不解除普通周期保护。 */
    rc = igh_mcan_stop_poll(&result->transaction, -1);
    if (rc == IGH_ECAN_NOT_READY || rc == IGH_ECAN_OK) rc = IGH_ECAN_FAIL;
finished:
    result->elapsed_us = (igh_rt_monotonic_ns() - start) / 1000ULL;
    return rc;
}

static inline int igh_example_stop_preop(igh_mcan_port_t *port, int master_index,
                                         const char *cli, uint32_t timeout_ms,
                                         igh_example_stop_result_t *result)
{
    return igh_example_stop_impl(port,master_index,cli,timeout_ms,result,0);
}

/* 启动等待失败后的有界撤销；仅用于未运行过任何业务周期的全零启动输出。
 * 撤销成功只表示PREOP已确认，调用者必须继续报告原始启动失败。 */
static inline int igh_example_startup_abort_preop(igh_mcan_port_t *port, int master_index,
                                                 const char *cli, uint32_t timeout_ms,
                                                 igh_example_stop_result_t *result)
{
    return igh_example_stop_impl(port,master_index,cli,timeout_ms,result,1);
}

/* 运行期错误的退出；成功后仍由调用者返回原始业务失败，不恢复业务。 */
static inline int igh_example_fault_stop_preop(igh_mcan_port_t *port, int master_index,
                                               const char *cli, uint32_t timeout_ms,
                                               igh_example_stop_result_t *result)
{
    return igh_example_stop_impl(port,master_index,cli,timeout_ms,result,2);
}

#endif
