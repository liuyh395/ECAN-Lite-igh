#ifndef IGH_MCAN_STOP_H
#define IGH_MCAN_STOP_H
#include "igh_mcan_port.h"
#ifdef __cplusplus
extern "C" {
#endif

/* 正常停机，不是电机安全停止。调用者先停止业务并收齐应答，以已完整回收的
 * 空PDO周期结束；prepare成功后不得再调用普通周期API。
 * 分离域各自等待精确WKC确认；未确认输入不阻断已确认的空输出，
 * 未确认输出不得改写或重排。合并LRW仍按整个域确认。
 * 管理侧异步向positions[]提交PREOP请求，周期线程持续调用poll，建议间隔
 * 不大于原周期的1/4；实际空PDO发送间隔不小于原周期，不追赶迟到周期。
 * request_result: 0=请求仍在进行，1=请求提交成功，负数=请求失败。
 * poll仅在请求成功且所有实际AL状态均为PREOP时返回OK。
 * NOT_READY表示继续服务；其他错误为终态，不自动close、确认错误或恢复业务。
 * 故障停止允许管理侧已显式处理错误后、实际处于无错误PREOP的从站加入；
 * 仍核对身份与拓扑，确认PREOP后退役其在途域，不改写或重发旧业务PDO。
 * timeout_ms必须非零。信息/域查询返回后重新核对期限，再判定完成或发送；
 * 此期限不保证内核阻塞调用的硬实时上限，不会中断仍在执行的内核调用。
 * session须零初始化；仅一个线程拥有port和session，不可与close并发。
 */
typedef struct {
    igh_mcan_port_t *port;
    uint16_t positions[ECAN_IGH_MAX_SLAVES];
    unsigned int al[ECAN_IGH_MAX_SLAVES];
    ec_domain_t *domains[ECAN_IGH_DIAG_DOMAIN_MAX];
    unsigned int expected[ECAN_IGH_DIAG_DOMAIN_MAX];
    uint8_t owner[ECAN_IGH_DIAG_DOMAIN_MAX];
    uint8_t pending[ECAN_IGH_DIAG_DOMAIN_MAX];
    uint8_t output[ECAN_IGH_DIAG_DOMAIN_MAX]; /* 该域包含主站输出；写前必须确认自身所有权。 */
    size_t domain_count;
    unsigned int topology_count;
    uint64_t deadline_ns, next_send_ns;
    uint64_t queued, retired_after_op, incomplete_polls;
    int result;
    uint8_t active, request_accepted, startup_abort, fault_stop;
    /* 仅故障停机接管后观察到的SAFEOP+ERROR；保留历史，不表示已清错。
     * polls为逐从站观测次数，不是独立故障次数。字段新增后调用方须重新编译。 */
    uint64_t al_error_polls;
    uint8_t al_error_seen[ECAN_IGH_MAX_SLAVES];
} igh_mcan_stop_t;

int igh_mcan_stop_prepare(igh_mcan_stop_t *stop, igh_mcan_port_t *port,
                          uint32_t timeout_ms);
/* 仅撤销尚未进入业务周期的启动交换。允许各目标处于INIT/PREOP/SAFEOP/OP，
 * 但身份必须匹配且无错误标志；主站输出必须仍为全零，不接受已编码命令。
 * 将所有启动域作为待确认接管，不假定当前WKC完整，不清除原始故障/统计。
 * 后续仍用stop_poll并异步请求PREOP；启动撤销期间SAFEOP/OP均维持原全零
 * 启动输出，各域独立确认；未回收的域不重排，不阻断已确认的另一域。
 * 退到INIT/PREOP才可按退出退役。正常停机接管前仍要求最后空周期双向完整回收。
 * 接管后禁止普通周期，最终启动结果仍失败。
 * 不用于运行故障、电机停止、任意原始域写入或init尚未激活的清理。
 */
int igh_mcan_startup_abort_prepare(igh_mcan_stop_t *stop, igh_mcan_port_t *port,
                                  uint32_t timeout_ms);
/* 运行期失败的有界退出，不是业务恢复或电机安全停止。允许业务数据仍在writer中；
 * prepare不改任何在途字节，poll仅在输出域完整回收后用新序号空PDO替换。
 * 分离域独立确认，未确认的输入域不阻断已确认输出；未确认输出绝不重排。
 * 不发送/重试旧命令，不清除原故障或完整性统计；可能仍有已发CAN命令在设备中。
 * 必须先结束/abort当前应用周期；仅允许身份一致且无AL错误的OP/SAFEOP目标。
 * queue/send中途错误、拓扑/链路异常等无法确认安全边界时返回失败，不自动清错。
 * 接管后才出现SAFEOP+ERROR时，暂停该从站PDO排队并保留在途所有权，记录
 * al_error_seen/al_error_polls；等待管理侧标准PREOP请求完成，不直接写AL ACK。
 * 仍须在总期限内确认无错误PREOP；其他状态错误、接管前AL错误继续拒绝。
 * 成功仅指已确认PREOP，调用者的原始业务失败必须保留。 */
int igh_mcan_fault_stop_prepare(igh_mcan_stop_t *stop, igh_mcan_port_t *port,
                               uint32_t timeout_ms);
int igh_mcan_stop_poll(igh_mcan_stop_t *stop, int request_result);
#ifdef __cplusplus
}
#endif
#endif
