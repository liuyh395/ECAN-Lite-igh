/* Compile the actual stop implementation; EtherCAT and clock are controlled.
 * No synthetic reimplementation of ownership or completion decisions. */
#define _POSIX_C_SOURCE 200809L
#include "igh_mcan_stop.h"
#include <time.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static uint64_t now=1000000000ULL;
static ec_master_info_t mi;
static ec_slave_info_t info[4];
static ec_domain_state_t ds[4];
static unsigned queued,send_calls,ack,assertions;
static int receive_failure;
static uint64_t info_delay_ns;
static int fake_clock(clockid_t id,struct timespec *t)
{(void)id;t->tv_sec=now/1000000000ULL;t->tv_nsec=now%1000000000ULL;return 0;}
int ecrt_master(ec_master_t *m,ec_master_info_t *v){(void)m;*v=mi;return 0;}
int ecrt_master_get_slave(ec_master_t *m,uint16_t n,ec_slave_info_t *v)
{(void)m;if(n>=4)return -1;now+=info_delay_ns;*v=info[n];return 0;}
int ecrt_master_receive(ec_master_t *m)
{(void)m;if(receive_failure)return -1;if(ack)for(unsigned n=0;n<4;n++){ds[n].wc_state=EC_WC_COMPLETE;ds[n].working_counter=3;}return 0;}
int ecrt_domain_process(ec_domain_t *d){(void)d;return 0;}
int ecrt_domain_state(const ec_domain_t *d,ec_domain_state_t *v){*v=ds[(uintptr_t)d-1];return 0;}
int ecrt_domain_queue(ec_domain_t *d)
{unsigned n=(uintptr_t)d-1;queued++;ds[n].wc_state=EC_WC_ZERO;ds[n].working_counter=0;return 0;}
int ecrt_master_send(ec_master_t *m){(void)m;send_calls++;return 100;}
int ecrt_master_application_time(ec_master_t *m,uint64_t t){(void)m;(void)t;return 0;}
int ecrt_master_sync_reference_clock(ec_master_t *m){(void)m;return 0;}
int ecrt_master_sync_slave_clocks(ec_master_t *m){(void)m;return 0;}
#define clock_gettime fake_clock
#ifndef ECAN_STOP_TEST_SOURCE
#define ECAN_STOP_TEST_SOURCE "igh_mcan_stop.c"
#endif
#include ECAN_STOP_TEST_SOURCE
#undef clock_gettime
#define T(x) do{assertions++;if(!(x)){fprintf(stderr,"FAIL %u line %d: %s\n",assertions,__LINE__,#x);exit(1);}}while(0)
static ecan_igh_context_t c;
static igh_mcan_port_t p;
static igh_mcan_stop_t s;
static uint8_t bytes[2][128],shadow[2][128];
static void setup(int split)
{
 if(c.lock_initialized)pthread_mutex_destroy(&c.lock);
 memset(&c,0,sizeof(c));memset(&p,0,sizeof(p));memset(&s,0,sizeof(s));memset(&mi,0,sizeof(mi));
 memset(info,0,sizeof(info));queued=send_calls=receive_failure=0;ack=1;now=1000000000ULL;info_delay_ns=0;
 mi.link_up=1;mi.slave_count=2;c.master=(ec_master_t*)1;c.domain_pd=bytes[0];
 c.slaves_operational=1;
 c.slave_count=2;c.cycle_us=1000;c.split_cycle_collected=1;c.lock_initialized=1;
 pthread_mutex_init(&c.lock,NULL);p.igh=&c;
 c.cycle_mode=split?ECAN_IGH_CYCLE_MODE_DC_SINGLE_CYCLE:ECAN_IGH_CYCLE_MODE_STANDARD;
 for(unsigned n=0;n<4;n++){ds[n].wc_state=EC_WC_COMPLETE;ds[n].working_counter=3;}
 for(unsigned n=0;n<2;n++){
  info[n].al_state=8;info[n].vendor_id=123;info[n].product_code=456;
  c.slave_al_state[n]=8;c.slave_operational[n]=1;
  c.slaves[n].config=(ecan_igh_slave_config_t){0,(int)n,123,456};
  c.slaves[n].rxpdo_view=bytes[n];c.slaves[n].rx_domain_pd=bytes[n];c.slaves[n].rx_domain_contiguous=1;
  c.domains[n]=(ec_domain_t*)(uintptr_t)(n+1);
  c.rx_domains[n]=(ec_domain_t*)(uintptr_t)(n*2+1);c.tx_domains[n]=(ec_domain_t*)(uintptr_t)(n*2+2);
  igh_mcan_pdo_writer_init(&p.writers[n],bytes[n],128,15,0,36);
  igh_mcan_pdo_writer_begin(&p.writers[n]);igh_mcan_pdo_writer_finish(&p.writers[n]);
 }
}
static void startup_setup(int split)
{
 setup(split);memset(bytes,0,sizeof(bytes));memset(shadow,0,sizeof(shadow));
 c.startup_poll_started=1;c.split_cycle_collected=0;ack=0;info[1].al_state=2;
 for(unsigned n=0;n<4;n++){ds[n].wc_state=EC_WC_ZERO;ds[n].working_counter=0;}
}
int main(void)
{
 for(int split=0;split<2;split++) {
  startup_setup(split);info[0].al_state=4;
  T(igh_mcan_startup_abort_prepare(&s,&p,20)==0);
  now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(!queued);
#ifdef ECAN_STOP_EXPECT_OLD
  T(!s.pending[0]); /* 旧版把SAFEOP未确认域错误退役。 */
  ack=1;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(!queued); /* 完整WKC之后仍不服务SAFEOP。 */
#else
  T(s.pending[0] && s.incomplete_polls==(split?2U:1U));
  /* 全零启动撤销可继续已确认的输出域，不重排未确认的输入域。 */
  ds[0].working_counter=3;ds[0].wc_state=EC_WC_COMPLETE;
  if(split){now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==1);T(s.pending[1] && !s.expected[1]);}
  ack=1;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  unsigned previous=queued;T(previous==(split?3U:1U));
  for(unsigned b=0;b<128;b++)T(bytes[0][b]==0);
  ack=0;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(queued==previous && s.pending[0]);
  info[0].al_state=8;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(queued==previous && s.pending[0]); /* SAFEOP->OP不解除旧代次。 */
  ack=1;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(queued==previous+(split?2U:1U));
  ack=0;info[0].al_state=2;T(igh_mcan_stop_poll(&s,1)==0);
  T(!s.pending[0]);
#endif
 }
#ifdef ECAN_STOP_EXPECT_OLD
 printf("old actual stop: SAFEOP starvation and premature retirement reproduced (%u assertions)\n",assertions);return 0;
#endif
 /* 查询跨越发送时刻不能再等待下一poll；跨越总截止则不得发送或成功返回。
  * 覆盖合并与分离；只注入信息API等待，不修改实际实现的决策。 */
 for(int split=0;split<2;split++){
  setup(split);T(igh_mcan_stop_prepare(&s,&p,20)==0);
  now+=900000;info_delay_ns=100000;
  T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(send_calls==1 && queued==s.domain_count); /* 两次查询后1.1ms，立即服务一次。 */
  T(s.next_send_ns==now+1000000);
  info_delay_ns=0;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(send_calls==1);
  setup(split);T(igh_mcan_stop_prepare(&s,&p,20)==0);
  uint8_t original[sizeof(bytes)];memcpy(original,bytes,sizeof(bytes));
  now+=19000000;info_delay_ns=1000000;
  T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);
  T(!queued && !send_calls && !memcmp(original,bytes,sizeof(bytes)));
  info_delay_ns=0;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);
  setup(split);T(igh_mcan_stop_prepare(&s,&p,20)==0);
  info[0].al_state=info[1].al_state=2;now+=19000000;info_delay_ns=1000000;
  T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);T(!queued && !send_calls);
 }
 /* 正常停机已排空业务；分离输入域不得饿死确认完毕的空输出。
  * 但输入/输出自身的pending和精确WKC仍各自有效。 */
 setup(1);T(igh_mcan_stop_prepare(&s,&p,20)==0);
 now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==4);
 ack=0;
 for(unsigned n=0;n<5;n++){
  ds[0].wc_state=EC_WC_COMPLETE;ds[0].working_counter=3;
  unsigned before=queued;now+=1000000;
  T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==before+1);
  T(s.pending[0] && s.pending[1] && s.pending[2] && s.pending[3]);
  T(!p.writers[0].frame_count && !p.writers[0].frame_bytes);
 }
 uint8_t pending_bytes[128];memcpy(pending_bytes,bytes[0],128);
 unsigned before=queued;
 ds[0].wc_state=EC_WC_COMPLETE;ds[0].working_counter=2; /* 错WKC不是所有权归还。 */
 now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
 T(queued==before && s.pending[0] && s.expected[0]==3);
 T(!memcmp(pending_bytes,bytes[0],128));
 ds[1].wc_state=EC_WC_COMPLETE;ds[1].working_counter=3;
 now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
 T(queued==before+1 && s.pending[0] && s.pending[1]);
 T(!memcmp(pending_bytes,bytes[0],128)); /* 只补输入读取，不能改动待确认输出。 */
 for(int split=0;split<2;split++){
  setup(split);T(igh_mcan_stop_prepare(&s,&p,2000)==0);T(c.stopping==1);
  T(s.domain_count==(split?4U:2U));T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(!queued);
  now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==s.domain_count);
  ack=0;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==s.domain_count);
  T(s.incomplete_polls==s.domain_count);
  info[0].al_state=1;info[1].al_state=2;now+=1000000;
  T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(s.retired_after_op==s.domain_count);
  T(queued==s.domain_count);info[0].al_state=2;now+=1000000;
  T(igh_mcan_stop_poll(&s,1)==0);T(igh_mcan_stop_poll(&s,1)==0);T(c.stopping==1);
  T(!c.slaves_operational);T(c.slave_al_state[0]==2 && c.slave_al_state[1]==2);
 }
 setup(0);p.writers[1].frame_count=1;T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping);
 setup(0);c.sm_pending_mask=1;T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 setup(0);c.split_cycle_collected=0;T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 setup(0);ds[1].working_counter=0;T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 setup(0);c.slaves[1].config.position=0;T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_INVALID_ARG);
 setup(0);c.slaves[0].config.alias=7;info[0].alias=7;T(igh_mcan_stop_prepare(&s,&p,20)==0);T(s.positions[0]==0);
 setup(0);c.slaves[0].config.alias=99;T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);info[0].al_state=0x14;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_FAIL);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);mi.link_up=0;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_FAIL);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);mi.slave_count=3;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_FAIL);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);T(igh_mcan_stop_poll(&s,-1)==IGH_ECAN_FAIL);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);info[0].al_state=info[1].al_state=2;
 T(igh_mcan_stop_poll(&s,0)==IGH_ECAN_NOT_READY);T(igh_mcan_stop_poll(&s,1)==0);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);pthread_mutex_lock(&c.lock);
 T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);now+=20000000;
 T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);pthread_mutex_unlock(&c.lock);T(!queued);
 setup(0);T(igh_mcan_stop_prepare(&s,&p,20)==0);receive_failure=1;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_FAIL);
 setup(0);c.slaves[0].rx_domain_contiguous=0;c.slaves[0].rxpdo_view=shadow[0];
 p.writers[0].data=shadow[0];for(unsigned u=0;u<8;u++)c.slaves[0].rx_offsets[u]=(7-u)*16;
 T(igh_mcan_stop_prepare(&s,&p,20)==0);now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
 for(unsigned u=0;u<8;u++)T(!memcmp(bytes[0]+(7-u)*16,shadow[0]+u*16,16));
 for(int split=0;split<2;split++) {
  startup_setup(split);c.sm_pending_mask=1;c.dc_receive_fault=3;
  T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping);
  T(igh_mcan_startup_abort_prepare(&s,&p,20)==0);T(s.startup_abort && c.stopping);
  T(s.domain_count==(split?4U:2U));T(s.pending[0] && !s.expected[0]);
  now+=1000000;T(igh_mcan_stop_poll(&s,0)==IGH_ECAN_NOT_READY);T(!queued);
  T(c.sm_pending_mask==1 && c.dc_receive_fault==3); /* 原故障不能清除。 */
  ds[0].wc_state=EC_WC_COMPLETE;ds[0].working_counter=split?1:3;
  if(split){ds[1].wc_state=EC_WC_COMPLETE;ds[1].working_counter=1;}
  now+=1000000;T(igh_mcan_stop_poll(&s,0)==IGH_ECAN_NOT_READY);
  T(queued==(split?2U:1U));T(s.expected[0]==(split?1U:3U));
  for(unsigned b=0;b<128;b++)T(bytes[0][b]==0); /* 不创建/改写启动PDO。 */
  unsigned previous=queued;now+=1000000;
  T(igh_mcan_stop_poll(&s,0)==IGH_ECAN_NOT_READY);T(queued==previous); /* pending不重排 */
  info[0].al_state=2;now+=1000000;T(igh_mcan_stop_poll(&s,0)==IGH_ECAN_NOT_READY);
  T(igh_mcan_stop_poll(&s,1)==0);T(queued==previous);
 }
 startup_setup(0);c.business_started=1;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 startup_setup(0);c.startup_poll_started=0;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 startup_setup(0);bytes[1][100]=1;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping);
 startup_setup(0);c.slaves[0].rxpdo_view=shadow[0];p.writers[0].data=shadow[0];
 c.slaves[0].rx_domain_contiguous=0;for(unsigned u=0;u<8;u++)c.slaves[0].rx_offsets[u]=(7-u)*16;
 bytes[0][17]=1;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping);
 startup_setup(0);info[1].error_flag=1;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 startup_setup(0);info[1].vendor_id=789;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 startup_setup(0);info[1].al_state=3;T(igh_mcan_startup_abort_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 startup_setup(0);T(igh_mcan_startup_abort_prepare(&s,&p,20)==0);now+=20000000;
 T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);T(!queued);T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);
 for(int split=0;split<2;split++) {
  setup(split);c.business_started=1;c.dc_receive_fault=1;c.sm_pending_mask=3;c.split_cycle_collected=0;
  p.writers[0].frame_count=1;p.writers[0].frame_bytes=12;memset(bytes[0]+24,0xa5,12);
  uint8_t original[128];memcpy(original,bytes[0],128);
  T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
  T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);T(s.fault_stop && s.pending[0]);
  T(!memcmp(original,bytes[0],128)); /* prepare不得擦除在途命令。 */
  ack=0;for(unsigned n=0;n<4;n++){ds[n].wc_state=EC_WC_ZERO;ds[n].working_counter=0;}
  now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(!queued);
  T(!memcmp(original,bytes[0],128));
  ds[0].wc_state=EC_WC_COMPLETE;ds[0].working_counter=split?1:3;
  now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==1);
  T(!p.writers[0].frame_count && !p.writers[0].frame_bytes);
  T(!bytes[0][6] && !bytes[0][7]);for(unsigned b=24;b<128;b++)T(!bytes[0][b]);
  T(c.dc_receive_fault==1 && c.sm_pending_mask==3 && !c.split_cycle_collected);
  if(split)T(s.pending[1] && !s.expected[1]); /* 输入未确认没有被强制完成。 */
  unsigned before=queued;now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);T(queued==before);
  info[0].al_state=info[1].al_state=2;T(igh_mcan_stop_poll(&s,1)==0);
  T(c.dc_receive_fault==1 && c.stopping); /* 停机不清错，不恢复普通周期。 */
 }
 setup(1);c.business_started=1;c.pdo_queue_uncommitted=1;
 T(igh_mcan_fault_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping && !queued && !send_calls);
 setup(1);T(igh_mcan_fault_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 setup(1);c.business_started=1;info[0].al_state=4;T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);
 setup(1);c.business_started=1;info[0].al_state=0x14;T(igh_mcan_fault_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
 for(int split=0;split<2;split++) {
  setup(split);c.business_started=1;c.dc_receive_fault=1;ack=0;
  info[0].al_state=info[1].al_state=2;
  memset(bytes[0]+24,0x5a,40);p.writers[0].frame_count=2;p.writers[0].frame_bytes=40;
  uint8_t prior[128];memcpy(prior,bytes[0],128);
  for(unsigned n=0;n<4;n++){ds[n].working_counter=0;ds[n].wc_state=EC_WC_ZERO;}
  T(igh_mcan_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);
  T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);
  now+=1000000;T(igh_mcan_stop_poll(&s,0)==IGH_ECAN_NOT_READY);
  T(!queued && !memcmp(prior,bytes[0],128));
  T(s.retired_after_op==s.domain_count && c.dc_receive_fault==1);
  T(igh_mcan_stop_poll(&s,1)==0);T(c.stopping && !c.slaves_operational);
  T(!queued && !memcmp(prior,bytes[0],128));
  setup(split);c.business_started=1;info[0].al_state=2;info[0].error_flag=1;
  T(igh_mcan_fault_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping);
  setup(split);c.business_started=1;info[0].al_state=2;info[0].vendor_id=999;
  T(igh_mcan_fault_stop_prepare(&s,&p,20)==IGH_ECAN_NOT_READY);T(!c.stopping);
 }
 /* 故障停机接管后才出现SAFEOP+ERROR：不清错、不退役、不改写、不排旧包。 */
 for(int split=0;split<2;split++)for(int in_state=0;in_state<2;in_state++) {
  setup(split);c.business_started=1;c.dc_receive_fault=1;
  memset(bytes[0]+24,0x5a,24);p.writers[0].frame_count=2;p.writers[0].frame_bytes=24;
  uint8_t original[128];memcpy(original,bytes[0],128);
  T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);
  info[0].al_state=in_state?0x14:4;info[0].error_flag=!in_state;info[1].al_state=2;
  now+=1000000;T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_NOT_READY);
  T(s.al[0]==0x14 && s.al_error_seen[0] && s.al_error_polls==1);
  T(s.pending[0] && (!split || s.pending[1]));
  T(!queued && !memcmp(original,bytes[0],128));T(c.dc_receive_fault==1);
  info[0].al_state=2;info[0].error_flag=0;now+=1000000;
  T(igh_mcan_stop_poll(&s,1)==0);T(s.al_error_seen[0] && s.al_error_polls==1);
  T(!queued && !memcmp(original,bytes[0],128));
 }
 setup(1);c.business_started=1;T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);
 info[0].al_state=0x14;now+=20000000;
 T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);T(!queued);
 setup(1);c.business_started=1;T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);
 info[0].al_state=0x14;T(igh_mcan_stop_poll(&s,-1)==IGH_ECAN_FAIL);T(!queued);
 for(unsigned bad=0;bad<4;bad++) {
  setup(1);c.business_started=1;T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);
  info[0].al_state=bad==0?0x18:bad==1?0x12:bad==2?0x34:0x14;
  if(bad==3)info[0].vendor_id=999;
  T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_FAIL);T(!queued);
 }
 setup(1);c.business_started=1;T(igh_mcan_fault_stop_prepare(&s,&p,20)==0);now+=20000000;
 T(igh_mcan_stop_poll(&s,1)==IGH_ECAN_TIMEOUT);T(!queued);
 printf("stop actual implementation: %u assertions passed\n",assertions);return 0;
}
