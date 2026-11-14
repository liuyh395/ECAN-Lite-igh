/* SPDX-License-Identifier: MIT */
#define _GNU_SOURCE
#define _DEFAULT_SOURCE

#include <inttypes.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

#include "dm_robot.h"
#include "igh_rt_support.h"
#include "igh_example_args.h"

#define TEST_CYCLE_US 1000U
#define TEST_WARMUP_CYCLES 32U
#define TEST_DRAIN_CYCLES 8U

static volatile sig_atomic_t stop_requested;

typedef struct {
    uint32_t cycles, bitrate, log_every;
    size_t slave, motor;
    uint8_t all, enable, keep_enabled;
    uint32_t command_id, feedback_id, motor_id;
    dm_motor_command_t command;
} test_options_t;

static void on_signal(int number)
{
    (void)number;
    stop_requested = 1;
}

static void usage(const char *program)
{
    printf("usage: %s [options]\n", program);
    printf("  --motor N          motor index 0..11 (default 0); or --all\n");
    printf("  --enable-motor     send enable before MIT commands\n");
    printf("  --keep-enabled     skip disable on exit (requires --enable-motor)\n");
    printf("  --cycles N         default 5000 cycles (about 5 seconds)\n");
    printf("  --slave N --bitrate N --log-every N\n");
    printf("  --command-id N --feedback-id N --motor-id N (single motor only)\n");
    printf("  --position F --velocity F --kp F --kd F --torque F\n");
    printf("Default MIT: position=0 velocity=0 kp=0 kd=0.2 torque=0 (damping).\n");
    printf("Classic CAN, DC 1ms, 1Mbps. Verify actual motor IDs and limits first.\n");
}

static int parse_args(int argc, char **argv, test_options_t *o)
{
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--motor") == 0 && i + 1 < argc)
            o->motor = igh_example_size(argv[++i]);
        else if (strcmp(argv[i], "--all") == 0) o->all = 1U;
        else if (strcmp(argv[i], "--enable-motor") == 0) o->enable = 1U;
        else if (strcmp(argv[i], "--keep-enabled") == 0) o->keep_enabled = 1U;
        else if (strcmp(argv[i], "--cycles") == 0 && i + 1 < argc)
            o->cycles = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--slave") == 0 && i + 1 < argc)
            o->slave = igh_example_size(argv[++i]);
        else if (strcmp(argv[i], "--bitrate") == 0 && i + 1 < argc)
            o->bitrate = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--log-every") == 0 && i + 1 < argc)
            o->log_every = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--command-id") == 0 && i + 1 < argc)
            o->command_id = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--feedback-id") == 0 && i + 1 < argc)
            o->feedback_id = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--motor-id") == 0 && i + 1 < argc)
            o->motor_id = igh_example_u32(argv[++i]);
        else if (strcmp(argv[i], "--position") == 0 && i + 1 < argc)
            o->command.position = igh_example_float(argv[++i]);
        else if (strcmp(argv[i], "--velocity") == 0 && i + 1 < argc)
            o->command.velocity = igh_example_float(argv[++i]);
        else if (strcmp(argv[i], "--kp") == 0 && i + 1 < argc)
            o->command.kp = igh_example_float(argv[++i]);
        else if (strcmp(argv[i], "--kd") == 0 && i + 1 < argc)
            o->command.kd = igh_example_float(argv[++i]);
        else if (strcmp(argv[i], "--torque") == 0 && i + 1 < argc)
            o->command.torque = igh_example_float(argv[++i]);
        else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            usage(argv[0]);
            return 1;
        } else return -1;
    }
    if (o->cycles == 0U || o->log_every == 0U || o->bitrate == 0U ||
        o->motor >= DM_ROBOT_MOTOR_COUNT || o->slave >= ECAN_IGH_MAX_SLAVES ||
        (o->keep_enabled && !o->enable)) return -1;
    if ((o->command_id != UINT32_MAX && o->command_id > 0x7FFU) ||
        (o->feedback_id != UINT32_MAX && o->feedback_id > 0x7FFU) ||
        (o->motor_id != UINT32_MAX && (o->motor_id == 0U || o->motor_id > 15U)) ||
        (o->all && (o->command_id != UINT32_MAX || o->feedback_id != UINT32_MAX ||
                    o->motor_id != UINT32_MAX))) return -1;
    return 0;
}

/* Reject out-of-range commands rather than silently saturating a real test. */
static int within_limits(const dm_motor_limits_t *l, const dm_motor_command_t *c)
{
    return c->position >= l->position_min && c->position <= l->position_max &&
           c->velocity >= l->velocity_min && c->velocity <= l->velocity_max &&
           c->kp >= l->kp_min && c->kp <= l->kp_max &&
           c->kd >= l->kd_min && c->kd <= l->kd_max &&
           c->torque >= l->torque_min && c->torque <= l->torque_max;
}

static void print_status(const dm_robot_t *robot, uint16_t mask)
{
    printf("cycles=%" PRIu64 "\n", robot->cycles_completed);
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        const dm_robot_motor_t *m = &robot->motors[i];
        if ((mask & (uint16_t)(1U << i)) == 0U) continue;
        printf("motor[%zu] CAN%u cmd=0x%03" PRIX32 " fb=0x%03" PRIX32
               " queued=%" PRIu64 " feedback_frames=%" PRIu64,
               i, m->config.channel, m->config.command_id, m->config.feedback_id,
               m->commands_sent, m->feedback_received);
        if (m->has_feedback)
            printf(" motor_id=%u state=0x%X pos=%.5f rad vel=%.5f rad/s "
                   "torque=%.5f Nm MOS=%uC rotor=%uC",
                   m->feedback.motor_id, m->feedback.error_state,
                   m->feedback.position, m->feedback.velocity, m->feedback.torque,
                   m->feedback.mos_temperature, m->feedback.rotor_temperature);
        else printf(" NO FEEDBACK");
        putchar('\n');
    }
    fflush(stdout);
}

/* Keep exchanging PDOs after disable so queued controls can leave the bridge.
 * A successful return is not a hardware acknowledgement from the motor. */
static int disable_and_drain(dm_robot_t *robot, igh_mcan_port_t *port,
                              uint16_t mask)
{
    igh_rt_periodic_clock_t clock;
    int failed = 0;
    if (igh_rt_periodic_clock_start_checked(&clock, TEST_CYCLE_US, 0U, 0U) != 0)
        return -1;
    for (unsigned i = 0; i < TEST_DRAIN_CYCLES; ++i) {
        int rc;
        if (igh_rt_periodic_clock_wait(&clock) == 0U) return -1;
        rc = i < 3U ? dm_robot_send_control_mask(robot, port, mask,
                                                 DM_MOTOR_CONTROL_DISABLE) :
                      dm_robot_cycle_mask(robot, port, 0U, NULL);
        if (rc != IGH_ECAN_OK) failed = 1;
    }
    return failed ? -1 : 0;
}

int main(int argc, char **argv)
{
    ecan_igh_config_t igh_config;
    igh_mcan_port_config_t mcan_config;
    ecan_igh_context_t igh;
    igh_mcan_port_t port;
    dm_robot_motor_config_t configs[DM_ROBOT_MOTOR_COUNT];
    dm_robot_t robot;
    test_options_t o = {
        .cycles = 5000U, .bitrate = 1000000U, .log_every = 1000U,
        .command_id = UINT32_MAX, .feedback_id = UINT32_MAX, .motor_id = UINT32_MAX,
        .command = {0.0F, 0.0F, 0.0F, 0.2F, 0.0F}
    };
    igh_rt_periodic_clock_t clock;
    igh_rt_period_stats_t periods;
    uint16_t mask;
    int may_be_enabled = 0;
    int final_rc = IGH_ECAN_EXIT_OK;
    int rc = parse_args(argc, argv, &o);
    if (rc != 0) {
        if (rc < 0) usage(argv[0]);
        return rc > 0 ? IGH_ECAN_EXIT_OK : IGH_ECAN_EXIT_ARG_ERROR;
    }
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    mask = o.all ? (uint16_t)((1U << DM_ROBOT_MOTOR_COUNT) - 1U) :
                   (uint16_t)(1U << o.motor);
    dm_robot_default_config(configs);
    /* Edit actual feedback IDs, motor IDs and model-specific limits here,
     * or in dm_robot_default_config(). All motors must be in MIT mode. */
    if (!o.all) {
        if (o.command_id != UINT32_MAX) configs[o.motor].command_id = o.command_id;
        if (o.feedback_id != UINT32_MAX) configs[o.motor].feedback_id = o.feedback_id;
        if (o.motor_id != UINT32_MAX) configs[o.motor].motor_id = (uint8_t)o.motor_id;
    }
    if (dm_robot_init(&robot, o.slave, configs) != 0) return IGH_ECAN_EXIT_ARG_ERROR;
    for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
        if ((mask & (uint16_t)(1U << i)) == 0U) continue;
        if (!within_limits(&configs[i].limits, &o.command) ||
            dm_robot_submit(&robot, i, &o.command) != 0) {
            fprintf(stderr, "invalid or out-of-range command for motor[%zu]\n", i);
            return IGH_ECAN_EXIT_ARG_ERROR;
        }
    }
    printf("DM motor test: slave=%zu mask=0x%03X bitrate=%" PRIu32
           " cycle=1000 us profile=%s warmup=%u\n", o.slave, (unsigned)mask,
           o.bitrate, ecan_igh_realtime_profile_name(
               ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE), TEST_WARMUP_CYCLES);
    printf("MIT: position=%.5f velocity=%.5f kp=%.5f kd=%.5f torque=%.5f\n",
           o.command.position, o.command.velocity, o.command.kp,
           o.command.kd, o.command.torque);
    printf("%s; Ctrl-C stops. No zero-position or clear-error command is sent.\n",
           o.enable ? "Enable requested" : "NO ENABLE: receive-only mode");
    print_status(&robot, mask);
    if (o.enable) {
        for (size_t i = 0; i < DM_ROBOT_MOTOR_COUNT; ++i) {
            uint8_t data[DM_MOTOR_PAYLOAD_SIZE];
            if ((mask & (uint16_t)(1U << i)) == 0U) continue;
            if (dm_motor_pack_command(&configs[i].limits, &o.command, data) != 0)
                return IGH_ECAN_EXIT_ARG_ERROR;
            printf("MIT TX template CAN%u ID=0x%03" PRIX32 " STD DLC=8 DATA=",
                   configs[i].channel, configs[i].command_id);
            for (size_t b = 0; b < sizeof(data); ++b)
                printf("%02X%s", data[b], b + 1U == sizeof(data) ? "\n" : " ");
        }
    }

    ecan_igh_default_config(&igh_config);
    igh_mcan_port_default_config(&mcan_config);
    if (ecan_igh_config_apply_realtime_profile(&igh_config,
            ECAN_IGH_PROFILE_DC_1MS_CROSS_CYCLE) != IGH_ECAN_OK ||
        ecan_igh_config_set_slave_count(&igh_config, o.slave + 1U) != IGH_ECAN_OK ||
        ecan_igh_config_set_cycle_us(&igh_config, TEST_CYCLE_US) != IGH_ECAN_OK ||
        ecan_igh_config_set_pdo_multiplier(&igh_config, 3U) != IGH_ECAN_OK ||
        dm_robot_apply_classic_can_config(&igh_config, &mcan_config,
                                          o.slave, o.bitrate) != 0)
        return IGH_ECAN_EXIT_ARG_ERROR;
    if (ecan_igh_init(&igh, &igh_config) != IGH_ECAN_OK) return IGH_ECAN_EXIT_FAIL;
    rc = igh_mcan_port_init(&port, &igh, &mcan_config);
    if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }
    rc = ecan_igh_wait_op_timeout(&igh, 10000U);
    if (rc != IGH_ECAN_OK) { final_rc = IGH_ECAN_EXIT_FAIL; goto close_igh; }
    for (unsigned i = 0; i < TEST_WARMUP_CYCLES && !stop_requested; ++i) {
        igh_mcan_rx_frame_t frames[DM_ROBOT_RX_CAPACITY];
        igh_mcan_port_warmup_result_t warmup;
        memset(&warmup, 0, sizeof(warmup));
        rc = igh_mcan_port_warmup_cycle_checked(&port, frames,
                                                DM_ROBOT_RX_CAPACITY, &warmup);
        if (rc != IGH_ECAN_OK && rc != IGH_ECAN_NO_SPACE) {
            fprintf(stderr, "warmup failed: %d\n", rc);
            final_rc = IGH_ECAN_EXIT_FAIL;
            goto close_igh;
        }
    }
    if (stop_requested) goto close_igh;
    igh_rt_period_stats_reset(&periods);
    if (igh_rt_periodic_clock_start_checked(&clock, TEST_CYCLE_US, 0U, 0U) != 0) {
        final_rc = IGH_ECAN_EXIT_FAIL;
        goto close_igh;
    }
    if (o.enable) {
        /* Set before transmission: a failed cycle may still reach some motors. */
        may_be_enabled = 1;
        rc = dm_robot_send_control_mask(&robot, &port, mask, DM_MOTOR_CONTROL_ENABLE);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "enable failed: %d\n", rc);
            final_rc = IGH_ECAN_EXIT_FAIL;
            goto disable_motor;
        }
    }
    for (uint32_t cycle = 0; cycle < o.cycles && !stop_requested; ++cycle) {
        uint64_t now = igh_rt_periodic_clock_wait(&clock);
        if (now == 0U) { final_rc = IGH_ECAN_EXIT_FAIL; break; }
        if (stop_requested) break;
        igh_rt_period_stats_sample(&periods, now, clock.cycle_ns);
        rc = dm_robot_cycle_mask(&robot, &port, o.enable ? mask : 0U, NULL);
        if (rc != IGH_ECAN_OK) {
            fprintf(stderr, "robot cycle failed at %" PRIu32 ": %d\n", cycle, rc);
            final_rc = IGH_ECAN_EXIT_FAIL;
            break;
        }
        if (cycle % o.log_every == 0U) print_status(&robot, mask);
    }
disable_motor:
    if (may_be_enabled && (!o.keep_enabled || final_rc != IGH_ECAN_EXIT_OK)) {
        if (disable_and_drain(&robot, &port, mask) != 0) {
            fprintf(stderr, "disable/drain FAILED; motor disable is not confirmed\n");
            final_rc = IGH_ECAN_EXIT_FAIL;
        } else printf("disable queued and PDO drain completed (not motor acknowledgement)\n");
    }
    print_status(&robot, mask);
    igh_rt_print_period_stats("period", &periods, (uint64_t)TEST_CYCLE_US * 1000U);
close_igh:
    ecan_igh_close(&igh);
    return final_rc;
}
