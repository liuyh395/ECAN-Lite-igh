#define _POSIX_C_SOURCE 200809L
#include "igh_mcan_stop.h"
#include "igh_adapter_internal.h"
#include "igh_mcan_pdo_internal.h"
#include <errno.h>
#include <string.h>
#include <time.h>

static uint64_t stop_now(void)
{
    struct timespec t;
    if (clock_gettime(CLOCK_MONOTONIC,&t)) return 0;
    return (uint64_t)t.tv_sec*1000000000ULL+(uint64_t)t.tv_nsec;
}

/* 配置寻址转换为绝对环位置。非零alias以首个匹配为基址，随后加相对位置。
 * 只读标准主站信息；不通过写ESC AL Control绕过IgH状态机。 */
static int stop_position(igh_mcan_stop_t *s, size_t n)
{
    ecan_igh_context_t *c=s->port->igh;
    const ecan_igh_slave_config_t *cfg=&c->slaves[n].config;
    ec_slave_info_t info;
    unsigned int base=0;
    if (cfg->position<0 || cfg->alias<0 || cfg->alias>65535) return IGH_ECAN_INVALID_ARG;
    if (cfg->alias) {
        for (;base<s->topology_count;base++) {
            if (stop_now()>=s->deadline_ns) return IGH_ECAN_TIMEOUT;
            if (ecrt_master_get_slave(c->master,base,&info)) return IGH_ECAN_FAIL;
            if (info.alias==cfg->alias) break;
        }
    }
    uint64_t pos=(uint64_t)base+(unsigned int)cfg->position;
    if (pos>=s->topology_count || pos>65535) return IGH_ECAN_NOT_READY;
    if (ecrt_master_get_slave(c->master,(uint16_t)pos,&info)) return IGH_ECAN_FAIL;
    if (info.vendor_id!=cfg->vendor_id || info.product_code!=cfg->product_id || info.error_flag)
        return IGH_ECAN_NOT_READY;
    if (s->startup_abort) {
        if (info.al_state!=1 && info.al_state!=2 && info.al_state!=4 && info.al_state!=8)
            return IGH_ECAN_NOT_READY;
    } else if (info.al_state!=8 && !(s->fault_stop && (info.al_state==4 || info.al_state==2))) return IGH_ECAN_NOT_READY;
    /* 故障已由管理侧显式处理并进入PREOP时，仍完成身份/拓扑/所有权检查。
     * 不自动确认error_flag；PREOP域只在poll确认输出关闭后退役，不再发送旧PDO。 */
    for (size_t i=0;i<n;i++) if (s->positions[i]==pos) return IGH_ECAN_INVALID_ARG;
    s->positions[n]=(uint16_t)pos;
    return IGH_ECAN_OK;
}

static int stop_add_domain(igh_mcan_stop_t *s, ec_domain_t *domain, size_t owner, int output)
{
    ec_domain_state_t ds={0};
    if (!domain || s->domain_count>=ECAN_IGH_DIAG_DOMAIN_MAX) return IGH_ECAN_INVALID_ARG;
    for (size_t i=0;i<s->domain_count;i++) if (s->domains[i]==domain) return IGH_ECAN_INVALID_ARG;
    if (ecrt_domain_state(domain,&ds)) return IGH_ECAN_FAIL;
    if (!s->startup_abort && !s->fault_stop && (ds.wc_state!=EC_WC_COMPLETE || !ds.working_counter))
        return IGH_ECAN_NOT_READY;
    size_t i=s->domain_count++;
    s->domains[i]=domain;s->owner[i]=(uint8_t)owner;s->output[i]=(uint8_t)output;
    /* 启动或故障末次send之后可能尚未receive/process，缓存WKC不能证明新代次已回收。
     * 每个域都先标记pending；首次实际完整回收后才学习expected，不硬编码3/1。 */
    s->expected[i]=(s->startup_abort || s->fault_stop)?0U:ds.working_counter;
    s->pending[i]=(s->startup_abort || s->fault_stop);
    return IGH_ECAN_OK;
}

static int stop_prepare(igh_mcan_stop_t *s, igh_mcan_port_t *p, uint32_t timeout_ms,
                         int mode)
{
    int startup_abort=mode==1, fault_stop=mode==2;
    if (!s || !p || !p->igh || !timeout_ms || s->active) return IGH_ECAN_INVALID_ARG;
    ecan_igh_context_t *c=p->igh;
    if (!c->master || !c->domain_pd || !c->lock_initialized ||
        !c->slave_count || c->slave_count>ECAN_IGH_MAX_SLAVES) return IGH_ECAN_NOT_READY;
    int lock=pthread_mutex_trylock(&c->lock);
    if (lock) return lock==EBUSY?IGH_ECAN_NOT_READY:IGH_ECAN_FAIL;
    int rc=IGH_ECAN_NOT_READY;
    if (c->stopping || c->cycle_active || p->cycle_active || c->pdo_queue_uncommitted) goto done;
    if (startup_abort) {
        if (!c->startup_poll_started || c->business_started) goto done;
    } else if (fault_stop) {
        if (!c->business_started) goto done;
    } else if (c->sm_pending_mask || c->sm_receive_fault || c->dc_receive_fault ||
               !c->split_cycle_collected) goto done;
    memset(s,0,sizeof(*s));s->port=p;s->startup_abort=(uint8_t)startup_abort;
    s->fault_stop=(uint8_t)fault_stop;
    uint64_t now=stop_now();if (!now) {rc=IGH_ECAN_FAIL;goto done;}
    s->deadline_ns=now+(uint64_t)timeout_ms*1000000ULL;
    ec_master_info_t mi={0};
    if (ecrt_master(c->master,&mi) || !mi.link_up || mi.scan_busy) goto done;
    s->topology_count=mi.slave_count;
    for (size_t n=0;n<c->slave_count;n++) {
        rc=IGH_ECAN_NOT_READY;
        igh_mcan_pdo_writer_t *w=&p->writers[n];
        if (w->data!=c->slaves[n].rxpdo_view || !w->data || (!fault_stop && (w->frame_count || w->frame_bytes)) ||
            !igh_mcan_pdo_size_is_valid(w->size) || !c->slaves[n].rx_domain_pd) goto done;
        if (startup_abort) {
            /* 只读核对，不清零、不重建PDO。包括非连续映射的真实输出区域，
             * 防止shadow为零但域中仍有旧命令。代次未确认时绝不改写这些字节。 */
            for (size_t b=0;b<w->size;b++) {
                size_t at=c->slaves[n].rx_domain_contiguous?b:
                    c->slaves[n].rx_offsets[b/ECAN_IGH_UNIT_BYTES]+b%ECAN_IGH_UNIT_BYTES;
                if (w->data[b] || c->slaves[n].rx_domain_pd[at]) goto done;
            }
        }
        rc=stop_position(s,n);if (rc) goto done;
        if (c->cycle_mode==ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE) {
            rc=stop_add_domain(s,c->rx_domains[n],n,1);if (rc) goto done;
            rc=stop_add_domain(s,c->tx_domains[n],n,0);if (rc) goto done;
        } else {rc=stop_add_domain(s,c->domains[n],n,1);if (rc) goto done;}
    }
    if (stop_now()>=s->deadline_ns) {rc=IGH_ECAN_TIMEOUT;goto done;}
    /* 排空与WKC前提全部通过后才接管；不清除原业务诊断和故障锁存。 */
    c->stopping=1;s->active=1;s->result=IGH_ECAN_NOT_READY;
    s->next_send_ns=now+(uint64_t)c->cycle_us*1000ULL;rc=IGH_ECAN_OK;
done:
    pthread_mutex_unlock(&c->lock);
    return rc;
}

int igh_mcan_stop_prepare(igh_mcan_stop_t *s, igh_mcan_port_t *p, uint32_t timeout_ms)
{
    return stop_prepare(s,p,timeout_ms,0);
}

int igh_mcan_startup_abort_prepare(igh_mcan_stop_t *s, igh_mcan_port_t *p, uint32_t timeout_ms)
{
    return stop_prepare(s,p,timeout_ms,1);
}

int igh_mcan_fault_stop_prepare(igh_mcan_stop_t *s, igh_mcan_port_t *p, uint32_t timeout_ms)
{
    return stop_prepare(s,p,timeout_ms,2);
}

/* 启动撤销可能正处在SAFEOP->OP的在途AL请求中。仍需维持原全零启动
 * 映像，直至该请求结束并执行PREOP。正常业务停机仍只服务OP。
 * 同一个判断同时用于pending退役和重排资格，避免把SAFEOP在途域误退役。 */
static int stop_needs_process_data(const igh_mcan_stop_t *s,unsigned int al)
{
    return al==8 || ((s->startup_abort || s->fault_stop) && al==4);
}

static int stop_service(igh_mcan_stop_t *s,int request)
{
    ecan_igh_context_t *c=s->port->igh;
    uint64_t now=stop_now();
    if (!now) return IGH_ECAN_FAIL;
    if (now>=s->deadline_ns) return IGH_ECAN_TIMEOUT;
    if (request<0) return IGH_ECAN_FAIL;
    if (request>0) s->request_accepted=1;
    ec_master_info_t mi={0};
    if (ecrt_master(c->master,&mi)) return IGH_ECAN_FAIL;
    if (!mi.link_up || mi.scan_busy || mi.slave_count!=s->topology_count) return IGH_ECAN_FAIL;
    if (ecrt_master_receive(c->master)) return IGH_ECAN_FAIL;
    unsigned int complete=0, all_op=1;
    c->slaves_operational=0;
    for (size_t n=0;n<c->slave_count;n++) {
        ec_slave_info_t info={0};
        if (ecrt_master_get_slave(c->master,s->positions[n],&info)) return IGH_ECAN_FAIL;
        unsigned int al=info.al_state & 15U;
        unsigned int error=info.error_flag || (info.al_state & 16U);
        if (info.vendor_id!=c->slaves[n].config.vendor_id ||
            info.product_code!=c->slaves[n].config.product_id || (info.al_state & ~31U) ||
            (al!=1 && al!=2 && al!=4 && al!=8))
            return IGH_ECAN_FAIL;
        /* 故障停机接管后，Sync看门狗可能先于PREOP请求进入SAFEOP+ERROR。
         * 只在该过渡中继续标准主站状态服务，不直接ACK/清错，不退役在途域，
         * 不排PDO；最终仍须无错误PREOP和管理请求成功。原业务故障不清除。 */
        if (error && !(s->fault_stop && al==4U)) return IGH_ECAN_FAIL;
        if (error) {s->al_error_seen[n]=1U;s->al_error_polls++;}
        s->al[n]=al | (error?16U:0U);complete+=al==2U && !error;
        c->slave_al_state[n]=s->al[n];
        c->slave_operational[n]=(uint8_t)(al==8U && !error);
        all_op&=c->slave_operational[n];
    }
    c->slaves_operational=(uint8_t)all_op;
    for (size_t i=0;i<s->domain_count;i++) {
        ec_domain_state_t ds={0};
        if (ecrt_domain_process(s->domains[i]) || ecrt_domain_state(s->domains[i],&ds)) return IGH_ECAN_FAIL;
        if (!s->pending[i]) continue;
        if (s->al[s->owner[i]] & 16U) {s->incomplete_polls++;continue;}
        if (ds.wc_state==EC_WC_COMPLETE && ds.working_counter &&
            (ds.working_counter==s->expected[i] || ((s->startup_abort || s->fault_stop) && !s->expected[i]))) {
            s->expected[i]=ds.working_counter;s->pending[i]=0;
        }
        else if (!stop_needs_process_data(s,s->al[s->owner[i]])) {s->pending[i]=0;s->retired_after_op++;}
        else s->incomplete_polls++;
    }
    /* 标准IgH信息/域查询可能等待内核锁或被调度打断。入口时间不能用于
     * 查询后的完成/发送判定：跨过发送时刻应在本次服务，跨过总截止则失败。
     * 重新取时不放宽WKC、pending或总超时，也不补发错过的历史周期。 */
    now=stop_now();
    if (!now) return IGH_ECAN_FAIL;
    if (now>=s->deadline_ns) return IGH_ECAN_TIMEOUT;
    if (complete==c->slave_count && s->request_accepted) return IGH_ECAN_OK;
    if (now<s->next_send_ns) return IGH_ECAN_NOT_READY;
    /* 迟到不追赶，不压缩相邻发送间隔；此路径只发送空PDO及状态服务数据。
     * 正常停机已收齐业务并确认最后空包，与启动撤销/故障停止统一按域确认。
     * 输入域未就绪不能阻断已确认的空输出；每个未确认域仍保持pending，绝不重排。
     * 合并LRW只有一个域，保护不变；分离LWR/LRD仅解除二者的等待耦合。
     * 启动映像始终不改写，故障输出确认后才换为空包。 */
    for (size_t n=0;n<c->slave_count;n++) {
        unsigned int output_pending=0;
        for (size_t i=0;i<s->domain_count;i++) if (s->owner[i]==n) {
            if (s->output[i]) output_pending|=s->pending[i];
        }
        if ((s->al[n] & 16U) || !stop_needs_process_data(s,s->al[n])) continue;
        /* 故障接管绝不先清writer：它可能仍是输出域正在使用的同一块内存。
         * 仅输出域完整确认后创建新序号空PDO；旧正文不再排队。 */
        if (!s->startup_abort && !output_pending) {
          /* 故障可能发生在append之后、finish之前，last_clear_bytes尚未覆盖
           * 当前正文。确认输出所有权后全清，仅此故障退出路径承担全长清零。 */
          if (s->fault_stop) memset(s->port->writers[n].data,0,s->port->writers[n].size);
          igh_mcan_pdo_writer_begin(&s->port->writers[n]);
          igh_mcan_pdo_writer_finish(&s->port->writers[n]);
        }
        ecan_igh_slave_t *slave=&c->slaves[n];
        if (!s->startup_abort && !output_pending && !slave->rx_domain_contiguous) {
            size_t units=s->port->writers[n].size/ECAN_IGH_UNIT_BYTES;
            for (size_t u=0;u<units;u++)
                memcpy(slave->rx_domain_pd+slave->rx_offsets[u],
                       s->port->writers[n].data+u*ECAN_IGH_UNIT_BYTES,ECAN_IGH_UNIT_BYTES);
        }
        for (size_t i=0;i<s->domain_count;i++) if (s->owner[i]==n) {
            if (s->pending[i]) continue;
            c->pdo_queue_uncommitted=1U;
            if (ecrt_domain_queue(s->domains[i])) return IGH_ECAN_FAIL;
            s->pending[i]=1;s->queued++;
        }
    }
    if (ecrt_master_application_time(c->master,stop_now())) return IGH_ECAN_FAIL;
    if (c->dc_assign_activate) {
        if (ecrt_master_sync_reference_clock(c->master) || ecrt_master_sync_slave_clocks(c->master)) return IGH_ECAN_FAIL;
    }
    int sent=ecrt_master_send(c->master);
    if (sent>=0) c->pdo_queue_uncommitted=0U;
    s->next_send_ns=stop_now()+(uint64_t)c->cycle_us*1000ULL;
    return sent<0?IGH_ECAN_FAIL:IGH_ECAN_NOT_READY;
}

int igh_mcan_stop_poll(igh_mcan_stop_t *s,int request_result)
{
    if (!s || !s->active || !s->port || !s->port->igh) return IGH_ECAN_INVALID_ARG;
    if (s->result!=IGH_ECAN_NOT_READY) return s->result;
    if (stop_now()>=s->deadline_ns) {s->result=IGH_ECAN_TIMEOUT;return s->result;}
    ecan_igh_context_t *c=s->port->igh;
    if (!c->master || !c->lock_initialized || !c->stopping) return IGH_ECAN_NOT_READY;
    int lock=pthread_mutex_trylock(&c->lock);
    if (lock) return lock==EBUSY?IGH_ECAN_NOT_READY:IGH_ECAN_FAIL;
    int rc=stop_service(s,request_result);
    if (rc!=IGH_ECAN_NOT_READY) s->result=rc;
    pthread_mutex_unlock(&c->lock);
    return rc;
}
