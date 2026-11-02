#define _GNU_SOURCE
#include "../examples/common/igh_example_stop.h"
#include <assert.h>

static unsigned received, ended, stopped, closed, aborted, fault_stopped;
static int scenario;
static int fake_init(ecan_igh_context_t *c,const ecan_igh_config_t *cfg)
{memset(c,0,sizeof(*c));c->cycle_us=cfg->cycle_us;c->slave_count=1;return IGH_ECAN_OK;}
static int fake_port_init(igh_mcan_port_t *p,ecan_igh_context_t *c,const igh_mcan_port_config_t *cfg)
{memset(p,0,sizeof(*p));p->igh=c;p->config=*cfg;return IGH_ECAN_OK;}
static int fake_wait(ecan_igh_context_t *c,uint32_t ms) {(void)c;(void)ms;return scenario==5||scenario==6?IGH_ECAN_TIMEOUT:IGH_ECAN_OK;}
static int fake_begin(igh_mcan_port_t *p) {if(scenario==8 && ended)return IGH_ECAN_TIMEOUT;p->cycle_active=1;return IGH_ECAN_OK;}
static int fake_end(igh_mcan_port_t *p) {p->cycle_active=0;ended++;return scenario==7||scenario==9?IGH_ECAN_TIMEOUT:IGH_ECAN_OK;}
static void fake_close(ecan_igh_context_t *c) {(void)c;closed++;}
static int fake_receive(igh_mcan_port_t *p,igh_mcan_rx_frame_t *frames,size_t capacity,size_t *count)
{
    (void)p;received++;*count=0;
    if (received==3 && scenario==3) return IGH_ECAN_FAIL;
    int overflow=(scenario==1 && received==1)||(scenario==2 && received==3);
    if (overflow || received==3) {
        *count=overflow?capacity:1;
        memset(frames,0,*count*sizeof(*frames));
        for (size_t i=0;i<*count;i++) {
            frames[i].frame.can_id=0x456;frames[i].frame.data_len=1;
            frames[i].frame.dlc=1;frames[i].frame.data[0]=0xA5;
        }
    }
    return overflow?IGH_ECAN_NO_SPACE:IGH_ECAN_OK;
}
static int fake_stop(igh_mcan_port_t *p,int master,const char *cli,uint32_t timeout,igh_example_stop_result_t *r)
{(void)p;(void)master;(void)cli;(void)timeout;memset(r,0,sizeof(*r));stopped++;return scenario==4?IGH_ECAN_FAIL:IGH_ECAN_OK;}
static int fake_abort(igh_mcan_port_t *p,int master,const char *cli,uint32_t timeout,igh_example_stop_result_t *r)
{(void)p;(void)master;(void)cli;(void)timeout;memset(r,0,sizeof(*r));aborted++;return scenario==6?IGH_ECAN_FAIL:IGH_ECAN_OK;}
static int fake_fault_stop(igh_mcan_port_t *p,int master,const char *cli,uint32_t timeout,igh_example_stop_result_t *r)
{(void)p;(void)master;(void)cli;(void)timeout;memset(r,0,sizeof(*r));fault_stopped++;return scenario==9?IGH_ECAN_FAIL:IGH_ECAN_OK;}

#define main rx_dump_entry
#define ecan_igh_init fake_init
#define ecan_igh_wait_op_timeout fake_wait
#define ecan_igh_close fake_close
#define igh_mcan_port_init fake_port_init
#define igh_mcan_port_receive fake_receive
#define igh_mcan_port_cycle_begin_checked fake_begin
#define igh_mcan_port_cycle_end_checked fake_end
#define igh_example_stop_preop fake_stop
#define igh_example_startup_abort_preop fake_abort
#define igh_example_fault_stop_preop fake_fault_stop
#undef _DEFAULT_SOURCE
#ifndef ECAN_RX_DUMP_SOURCE
#define ECAN_RX_DUMP_SOURCE "../examples/mcan/mcan_rx_dump.c"
#endif
#include ECAN_RX_DUMP_SOURCE
#undef main

int main(int argc,char **argv)
{
    if (argc!=2) return 2;
    scenario=atoi(argv[1]);if(scenario<0||scenario>9)return 2;
    char *args[]={"rx_dump_test","--cycles","2","--cycle-us","1000","--diag-every","0","--no-mlock",NULL};
    int rc=rx_dump_entry(8,args);
    int expected=scenario==0?IGH_ECAN_EXIT_OK:IGH_ECAN_EXIT_FAIL;
    printf("HARNESS scenario=%d rc=%d expected=%d receives=%u ends=%u stops=%u closes=%u aborts=%u\n",
           scenario,rc,expected,received,ended,stopped,closed,aborted);
    if(scenario>=7)return rc==expected && received==1 && ended==1 && !stopped && closed==1 && !aborted && fault_stopped==1?0:1;
    if(scenario>=5)return rc==expected && received==0 && ended==0 && stopped==0 && closed==1 && aborted==1?0:1;
    return rc==expected && received==3 && ended==2 && stopped==(scenario==3?0U:1U) && closed==1?0:1;
}
