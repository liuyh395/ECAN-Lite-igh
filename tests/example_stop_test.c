#define _GNU_SOURCE
#include "igh_mcan_stop.h"
#include <spawn.h>
#include <signal.h>
#include <sys/wait.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "igh_rt_support.h"

static unsigned checks;
#define CHECK(c) do { ++checks; if (!(c)) {fprintf(stderr,"FAIL %u line %d: %s\n",checks,__LINE__,#c);exit(1);} } while (0)
static int scenario, waits, polls, kills, prepares;
static int fake_spawn(pid_t *, const char *, const posix_spawn_file_actions_t *,
                      const posix_spawnattr_t *, char *const[], char *const[]);
static pid_t fake_wait(pid_t,int *,int);
static int fake_kill(pid_t,int);
static void checked_clock_start(igh_rt_periodic_clock_t *c,uint32_t us,uint32_t spin,uint8_t catch_up)
{
    CHECK(us==100U && spin==0U && catch_up==0U);
    igh_rt_periodic_clock_start(c,us,spin,catch_up);
}
#define posix_spawnp fake_spawn
#define waitpid fake_wait
#define kill fake_kill
#define igh_rt_periodic_clock_start checked_clock_start
#include "../examples/common/igh_example_stop.h"
#undef igh_rt_periodic_clock_start
#undef posix_spawnp
#undef waitpid
#undef kill

int igh_mcan_stop_prepare(igh_mcan_stop_t *s,igh_mcan_port_t *p,uint32_t timeout)
{
    CHECK(timeout==20);prepares++;
    if (scenario==1) return IGH_ECAN_NOT_READY;
    s->port=p;s->active=1;s->positions[0]=4;s->positions[1]=7;
    p->igh->stopping=1;return IGH_ECAN_OK;
}
int igh_mcan_stop_poll(igh_mcan_stop_t *s,int request)
{
    CHECK(s->active);polls++;
    if (request<0) {s->result=IGH_ECAN_FAIL;return IGH_ECAN_FAIL;}
    if (scenario==5 || scenario==6) return IGH_ECAN_TIMEOUT;
    if (request>0 && polls>=2) return IGH_ECAN_OK;
    return IGH_ECAN_NOT_READY;
}
int igh_mcan_startup_abort_prepare(igh_mcan_stop_t *s,igh_mcan_port_t *p,uint32_t timeout)
{
    return igh_mcan_stop_prepare(s,p,timeout);
}
int igh_mcan_fault_stop_prepare(igh_mcan_stop_t *s,igh_mcan_port_t *p,uint32_t timeout)
{
    return igh_mcan_stop_prepare(s,p,timeout);
}
static int fake_spawn(pid_t *p,const char *file,const posix_spawn_file_actions_t *actions,
                      const posix_spawnattr_t *attr,char *const argv[],char *const env[])
{
    short flags=0;int policy=-1;struct sched_param sp={0};sigset_t mask,defaults;
    (void)actions;CHECK(env==environ);
    CHECK(!strcmp(file,"/trusted/ethercat"));
    CHECK(!strcmp(argv[1],"states") && !strcmp(argv[2],"-m") && !strcmp(argv[3],"2"));
    CHECK(!strcmp(argv[4],"-p") && !strcmp(argv[5],"4,7") && !strcmp(argv[6],"PREOP") && !argv[7]);
    CHECK(!posix_spawnattr_getflags(attr,&flags));
    CHECK(flags==(POSIX_SPAWN_SETSCHEDULER|POSIX_SPAWN_SETSCHEDPARAM|POSIX_SPAWN_SETSIGDEF|POSIX_SPAWN_SETSIGMASK));
    CHECK(!posix_spawnattr_getschedpolicy(attr,&policy) && policy==SCHED_OTHER);
    CHECK(!posix_spawnattr_getschedparam(attr,&sp) && sp.sched_priority==0);
    CHECK(!posix_spawnattr_getsigmask(attr,&mask) && !sigismember(&mask,SIGTERM));
    CHECK(!posix_spawnattr_getsigdefault(attr,&defaults) && sigismember(&defaults,SIGTERM) && sigismember(&defaults,SIGINT));
    if (scenario==2) return ENOENT;
    *p=123;return 0;
}
static pid_t fake_wait(pid_t p,int *status,int options)
{
    CHECK(p==123 && options==WNOHANG);waits++;
    if (scenario==3 && waits==1) {errno=EINTR;return -1;}
    if (scenario==4) {errno=ECHILD;return -1;}
    if (scenario==6 || (scenario==5 && !kills)) return 0;
    *status=scenario==7?(3<<8):0;return p;
}
static int fake_kill(pid_t p,int sig) {CHECK(p==123 && sig==SIGKILL);kills++;return 0;}

int main(void)
{
    ecan_igh_context_t ctx={0};igh_mcan_port_t port={0};igh_example_stop_result_t result;
    ctx.slave_count=2;ctx.cycle_us=1000;port.igh=&ctx;
    CHECK(igh_example_stop_preop(&port,-1,"/trusted/ethercat",20,&result)==IGH_ECAN_INVALID_ARG);
    CHECK(igh_example_stop_preop(&port,2,"",20,&result)==IGH_ECAN_INVALID_ARG);
    CHECK(!prepares);
    for (int period=0;period<2;period++)
    for (int startup=0;startup<3;startup++) for (scenario=0;scenario<8;scenario++) {
        ctx.cycle_us=period?250U:1000U;
        waits=polls=kills=prepares=0;ctx.stopping=0;
        int rc=startup==2?igh_example_fault_stop_preop(&port,2,"/trusted/ethercat",20,&result):
            startup==1?igh_example_startup_abort_preop(&port,2,"/trusted/ethercat",20,&result):
            igh_example_stop_preop(&port,2,"/trusted/ethercat",20,&result);
        CHECK(prepares==1);
        if (scenario==0 || scenario==3) CHECK(rc==IGH_ECAN_OK && polls>=2 && !kills);
        else if (scenario==1) CHECK(rc==IGH_ECAN_NOT_READY && !polls && !ctx.stopping);
        else if (scenario==5 || scenario==6) CHECK(rc==IGH_ECAN_TIMEOUT && kills==1);
        else CHECK(rc==IGH_ECAN_FAIL && !kills);
        CHECK(result.unreaped_child==(scenario==6?123:0));
        if (scenario==2) CHECK(result.cli_error==ENOENT && result.transaction.result==IGH_ECAN_FAIL);
        if (scenario==4) CHECK(result.cli_error==ECHILD);
        if (scenario!=1) CHECK(ctx.stopping==1);
    }
    printf("example stop: %u checks passed; no process spawned or device opened\n",checks);
    return 0;
}
