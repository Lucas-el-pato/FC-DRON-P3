/**
 ******************************************************************************
 * @file    balancer.c
 * @brief   Init, lazo realtime y tareas lentas del balancin (ver balancer.h).
 *
 *          Lazo (disparado por el DRDY del gyro, como el FC del dron):
 *            gyro    8 kHz  imu_read_gyro + bias
 *            filter  8 kHz  PT1 x2 sobre el eje del balancin
 *            control 2 kHz  seguridad -> bal_pid -> mezcla M1/M3 -> DShot
 *          Cola lenta: LINK (comandos PC), ATTITUDE (100 Hz), BATT.
 ******************************************************************************
 */

#include "balancer.h"
#include "bal_config.h"
#include "bal_link.h"
#include "bal_pid.h"

#include "filter.h"
#include "scheduler.h"

#include "console.h"
#include "driver_imu.h"
#include "driver_motors.h"
#include "sensors_battery.h"
#include "sensors_calib.h"
#include "sensors_scale.h"
#include "timebase.h"

#define BAL_RAD2DEG             57.2957795f
#define BAL_ATTITUDE_DT_S       0.01f     /* TASK_ATTITUDE a 100 Hz */
#define BAL_GYRO_ERR_DISARM     50u       /* lecturas fallidas seguidas */
#define BAL_FAILSAFE_PERIOD_MS  10u

/* El ESC aplica un comando DShot de configuracion tras ~6 repeticiones con
 * el motor parado; se mandan 10 como Betaflight (igual que arming.c).    */
#define BAL_DSHOT_CMD_REPEAT    10u
#define BAL_DSHOT_CMD_GAP_US    1000u
#define BAL_DSHOT_BUSY_TIMEOUT_US 200u

/* Motivos que impiden armar o que desarmaron (bitmask, se ve con "?"). */
#define BAL_FLAG_NO_GYRO        (1u << 0)
#define BAL_FLAG_NO_MOTORS      (1u << 1)
#define BAL_FLAG_CALIBRATING    (1u << 2)
#define BAL_FLAG_TILT           (1u << 3)
#define BAL_FLAG_PORT_CLOSED    (1u << 4)
#define BAL_FLAG_GYRO_ERROR     (1u << 5)

typedef struct {
    bool     has_gyro;
    bool     has_motors;
    volatile bool armed;

    float    ref_deg;          /* referencia, antihorario + */
    float    base_throttle;    /* 0..BAL_MOTOR_LIMIT        */
    float    angle_deg;        /* actitud, antihorario +    */
    float    angle_raw_deg;    /* antes de restar el trim   */
    float    trim_deg;         /* cero de montaje ("Z")     */
    float    acc_g[3];         /* ultimo accel (diagnostico) */
    float    rate_dps;         /* gyro filtrado, mismo signo */
    float    correction;       /* salida del PID (fraccion de motor) */
    uint16_t motor[MOTORS_COUNT];

    uint32_t flags;            /* BAL_FLAG_* actuales       */
    uint32_t last_disarm;      /* motivo del ultimo desarme */
    uint32_t gyro_err_run;
    uint32_t gyro_errors;
    uint32_t motor_drops;
    bool     calib_reported;
} bal_state_t;

/* Global para Live Expressions. */
bal_state_t g_balState;

static imu_sample_t      s_gyro_raw;
static sensors_imu_si_t  s_imu_si;
static sensors_attitude_t s_attitude;
static pt1_filter_t      s_gyro_lpf;
static pt1_filter_t      s_gyro_lpf2;

static float    s_att_gyro_sum[3];
static uint32_t s_att_gyro_count = 0u;

static sched_realtime_t s_realtime;
static sched_task_t     s_tasks[SCHED_MAX_TASKS];
static uint8_t          s_task_count = 0u;

/* ========================================================================= */
/* Helpers                                                                    */
/* ========================================================================= */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static float absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

/* Lleva a -180..+180: con la IMU invertida el roll crudo ronda +/-180 y
 * restar el trim no debe dejar saltos de 360.                          */
static float wrap180(float deg)
{
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static long x10(float v)
{
    return (long)(v * 10.0f + ((v < 0.0f) ? -0.5f : 0.5f));
}

static uint16_t bal_to_dshot(float v)
{
    v = clampf(v, 0.0f, BAL_MOTOR_LIMIT);
    uint32_t d = (uint32_t)BAL_DSHOT_MIN +
                 (uint32_t)(v * (float)(BAL_DSHOT_MAX - BAL_DSHOT_MIN) + 0.5f);
    if (d > BAL_DSHOT_MAX) {
        d = BAL_DSHOT_MAX;
    }
    return (uint16_t)d;
}

static void bal_send_motor_command(uint16_t cmd, uint8_t esc_mask)
{
    if (esc_mask == 0u) {
        return;
    }

    for (uint8_t i = 0u; i < BAL_DSHOT_CMD_REPEAT; ++i) {
        const uint32_t t0 = timebase_now();
        while (motors_output_busy() &&
               (timebase_elapsed_us(t0) < BAL_DSHOT_BUSY_TIMEOUT_US)) {
        }
        (void)motors_send_command4(cmd, esc_mask);
        timebase_delay_us(BAL_DSHOT_CMD_GAP_US);
    }
}

/* ========================================================================= */
/* Armado                                                                     */
/* ========================================================================= */

static void bal_disarm(uint32_t reason)
{
    bal_state_t *st = &g_balState;

    if (st->armed) {
        st->last_disarm = reason;
    }
    st->armed = false;
    for (uint8_t i = 0u; i < MOTORS_COUNT; ++i) {
        st->motor[i] = 0u;
    }
    (void)motors_disarm_all();
    bal_pid_reset();
    console_led_fail();   /* rojo = desarmado */
}

/* Devuelve 0 si armo, o los flags que lo impiden. */
static uint32_t bal_arm(void)
{
    bal_state_t *st = &g_balState;

    uint32_t block = st->flags & (BAL_FLAG_NO_GYRO | BAL_FLAG_NO_MOTORS |
                                  BAL_FLAG_CALIBRATING | BAL_FLAG_PORT_CLOSED);
    if (absf(st->angle_deg) > BAL_ARM_MAX_ANGLE_DEG) {
        block |= BAL_FLAG_TILT;
    }
    if (block != 0u) {
        return block;
    }
    if (st->armed) {
        return 0u;
    }

    /* Sentido de giro solo a los dos motores usados (motores parados). */
    const uint8_t reversed = (uint8_t)(BAL_MOTOR_REVERSED_MASK & BAL_MOTOR_MASK);
    const uint8_t normal = (uint8_t)(~BAL_MOTOR_REVERSED_MASK & BAL_MOTOR_MASK);
    bal_send_motor_command(MOTORS_DSHOT_CMD_SPIN_DIRECTION_REVERSED, reversed);
    bal_send_motor_command(MOTORS_DSHOT_CMD_SPIN_DIRECTION_NORMAL, normal);

    bal_pid_reset();
    st->last_disarm = 0u;
    st->armed = true;
    console_led_pass();   /* verde = armado */
    return 0u;
}

/* ========================================================================= */
/* Mezcla de 2 motores                                                        */
/* ========================================================================= */

/* u > 0 = girar antihorario (visto desde atras) = mas empuje en M3 (der.).
 * Si thr +/- u no entra en [IDLE, LIMIT] se corre thr para conservar el
 * diferencial, como el mixer del dron con el throttle.                    */
static void bal_mix(float u, bool armed, uint16_t out[MOTORS_COUNT])
{
    for (uint8_t i = 0u; i < MOTORS_COUNT; ++i) {
        out[i] = 0u;   /* M2 (CH3) y M4 (CH1) quedan siempre en 0 */
    }
    if (!armed) {
        return;
    }

    u = clampf(u, -BAL_CORR_LIMIT, BAL_CORR_LIMIT);
    const float a = absf(u);

    float thr = clampf(g_balState.base_throttle, BAL_IDLE, BAL_MOTOR_LIMIT);
    if ((thr + a) > BAL_MOTOR_LIMIT) {
        thr = BAL_MOTOR_LIMIT - a;
    }
    if ((thr - a) < BAL_IDLE) {
        thr = BAL_IDLE + a;
    }

    const float right = clampf(thr + u, BAL_IDLE, BAL_MOTOR_LIMIT);
    const float left = clampf(thr - u, BAL_IDLE, BAL_MOTOR_LIMIT);

    out[BAL_MOTOR_RIGHT_CH] = bal_to_dshot(right);
    out[BAL_MOTOR_LEFT_CH] = bal_to_dshot(left);
}

/* ========================================================================= */
/* Etapas realtime                                                            */
/* ========================================================================= */

static void bal_task_gyro(void)
{
    bal_state_t *st = &g_balState;

    if (!sensors_calib_done()) {
        /* Calibracion: gyro + accel; solo se usa el bias del gyro. */
        if (imu_read_sample(&s_gyro_raw) != IMU_OK) {
            st->gyro_errors++;
            return;
        }
        sensors_imu_scale(&s_gyro_raw, &s_imu_si);
        if (sensors_calib_feed(&s_gyro_raw)) {
            sensors_attitude_reset();
            bal_pid_reset();
            st->flags &= ~BAL_FLAG_CALIBRATING;
        }
        return;
    }

    if (imu_read_gyro(&s_gyro_raw) != IMU_OK) {
        st->gyro_errors++;
        st->gyro_err_run++;
        return;
    }
    st->gyro_err_run = 0u;

    sensors_calib_apply(&s_gyro_raw, false);
    sensors_imu_scale(&s_gyro_raw, &s_imu_si);

    s_att_gyro_sum[0] += s_imu_si.gx_rad_s;
    s_att_gyro_sum[1] += s_imu_si.gy_rad_s;
    s_att_gyro_sum[2] += s_imu_si.gz_rad_s;
    s_att_gyro_count++;
}

static void bal_task_filter(void)
{
    const float raw = BAL_CCW_SIGN * BAL_ROLL_SIGN * s_imu_si.gx_dps;
    g_balState.rate_dps = pt1_apply(&s_gyro_lpf2, pt1_apply(&s_gyro_lpf, raw));
}

static void bal_task_control(void)
{
    bal_state_t *st = &g_balState;

    /* Seguridad: cualquiera de estas desarma. */
    if (st->armed) {
        if (absf(st->angle_deg) > BAL_TILT_CUTOFF_DEG) {
            bal_disarm(BAL_FLAG_TILT);
        } else if ((st->flags & BAL_FLAG_PORT_CLOSED) != 0u) {
            bal_disarm(BAL_FLAG_PORT_CLOSED);
        } else if (st->gyro_err_run >= BAL_GYRO_ERR_DISARM) {
            bal_disarm(BAL_FLAG_GYRO_ERROR);
        }
    }

    if (st->armed) {
        st->correction = bal_pid_update(st->ref_deg, st->angle_deg, st->rate_dps);
    } else {
        st->correction = 0.0f;
    }

    /* Se escribe siempre (0 desarmado) para que los ESC no pierdan senal. */
    bal_mix(st->correction, st->armed, st->motor);
    if (st->has_motors && (motors_write4(st->motor, 0u) != MOT_OK)) {
        st->motor_drops++;
    }
}

/* Cada BAL_FAILSAFE_PERIOD_MS: estado del puerto USB. */
static void bal_task_port(void)
{
    if (console_port_open()) {
        g_balState.flags &= ~BAL_FLAG_PORT_CLOSED;
    } else {
        g_balState.flags |= BAL_FLAG_PORT_CLOSED;
    }
}

/* ========================================================================= */
/* Tareas lentas                                                              */
/* ========================================================================= */

static void bal_task_attitude(void)
{
    bal_state_t *st = &g_balState;

    if (!sensors_calib_done()) {
        return;
    }

    imu_sample_t sample;
    if (imu_read_sample(&sample) != IMU_OK) {
        st->gyro_errors++;
        return;
    }

    /* Solo bias del gyro: el accel NO se nivela a la posicion de encendido,
     * asi el cero del angulo es el horizonte real (gravedad).             */
    sensors_calib_apply(&sample, false);

    sensors_imu_si_t si;
    sensors_imu_scale(&sample, &si);

    float dt_s = BAL_ATTITUDE_DT_S;
    if (s_att_gyro_count > 0u) {
        const float inv = 1.0f / (float)s_att_gyro_count;
        si.gx_rad_s = s_att_gyro_sum[0] * inv;
        si.gy_rad_s = s_att_gyro_sum[1] * inv;
        si.gz_rad_s = s_att_gyro_sum[2] * inv;
        dt_s = (float)s_att_gyro_count / (float)BAL_GYRO_RATE_HZ;
    }
    s_att_gyro_sum[0] = 0.0f;
    s_att_gyro_sum[1] = 0.0f;
    s_att_gyro_sum[2] = 0.0f;
    s_att_gyro_count = 0u;

    sensors_attitude_update(&si, 0, dt_s, &s_attitude);

    st->acc_g[0] = si.ax_g;
    st->acc_g[1] = si.ay_g;
    st->acc_g[2] = si.az_g;
    st->angle_raw_deg = BAL_CCW_SIGN * BAL_ROLL_SIGN * s_attitude.roll_rad * BAL_RAD2DEG;
    st->angle_deg = wrap180(st->angle_raw_deg - st->trim_deg);
}

static void bal_reply_status(void)
{
    const bal_state_t *st = &g_balState;
    const sensors_battery_t *bat = sensors_battery_get();

    bal_link_reply("ST arm=%u ref=%ld ang=%ld raw=%ld trim=%ld rate=%ld thr=%ld cor=%ld | "
                   "M1=%u M3=%u | acc mg %ld %ld %ld | flags=0x%02lX last=0x%02lX | "
                   "vbat=%u.%02u\r\n",
                   (unsigned)st->armed,
                   x10(st->ref_deg),
                   x10(st->angle_deg),
                   x10(st->angle_raw_deg),
                   x10(st->trim_deg),
                   x10(st->rate_dps),
                   x10(st->base_throttle * 100.0f),
                   x10(st->correction * 100.0f),
                   (unsigned)st->motor[BAL_MOTOR_LEFT_CH],
                   (unsigned)st->motor[BAL_MOTOR_RIGHT_CH],
                   (long)(st->acc_g[0] * 1000.0f),
                   (long)(st->acc_g[1] * 1000.0f),
                   (long)(st->acc_g[2] * 1000.0f),
                   (unsigned long)st->flags,
                   (unsigned long)st->last_disarm,
                   (unsigned)(bat->voltage_cv / 100u),
                   (unsigned)(bat->voltage_cv % 100u));
}

static void bal_handle_command(const bal_cmd_t *cmd)
{
    bal_state_t *st = &g_balState;

    switch (cmd->type) {
    case BAL_CMD_ARM: {
        const uint32_t block = bal_arm();
        if (block == 0u) {
            bal_link_reply("OK ARM ref=%ld\r\n", x10(st->ref_deg));
        } else {
            bal_link_reply("ERR ARM flags=0x%02lX\r\n", (unsigned long)block);
        }
        break;
    }
    case BAL_CMD_STOP:
        bal_disarm(0u);
        bal_link_reply("OK STOP\r\n");
        break;
    case BAL_CMD_REF:
        st->ref_deg = clampf(cmd->value, -BAL_REF_LIMIT_DEG, BAL_REF_LIMIT_DEG);
        bal_link_reply("OK R %ld\r\n", x10(st->ref_deg));
        break;
    case BAL_CMD_THROTTLE:
        st->base_throttle = clampf(cmd->value * 0.01f, 0.0f, BAL_MOTOR_LIMIT);
        bal_link_reply("OK T %ld\r\n", x10(st->base_throttle * 100.0f));
        break;
    case BAL_CMD_STATUS:
        bal_reply_status();
        break;
    case BAL_CMD_ZERO:
        if (st->armed || !sensors_calib_done()) {
            bal_link_reply("ERR Z (desarmar y esperar calibracion)\r\n");
        } else {
            st->trim_deg = st->angle_raw_deg;
            st->angle_deg = 0.0f;
            bal_link_reply("OK Z trim=%ld\r\n", x10(st->trim_deg));
        }
        break;
    case BAL_CMD_INVALID:
    default:
        bal_link_reply("ERR cmd (A, D, R <deg>, T <pct>, Z, ?)\r\n");
        break;
    }
}

static void bal_task_link(void)
{
    bal_state_t *st = &g_balState;

    if (!st->calib_reported && st->has_gyro && sensors_calib_done()) {
        st->calib_reported = true;
        bal_link_reply("OK CAL listo, ref=0, mandar A para armar\r\n");
        return;
    }

    bal_cmd_t cmd;
    if (bal_link_poll(&cmd)) {
        bal_handle_command(&cmd);
    }
}

static void bal_task_battery(void)
{
    sensors_battery_update();
}

/* ========================================================================= */
/* Init                                                                       */
/* ========================================================================= */

static void bal_task_add(const char *name, sched_task_fn fn,
                         uint32_t period_us, uint8_t prio)
{
    if (s_task_count >= SCHED_MAX_TASKS || fn == 0) {
        return;
    }

    sched_task_t *t = &s_tasks[s_task_count++];
    t->name = name;
    t->fn = fn;
    t->period_us = period_us;
    t->static_prio = prio;
}

static void bal_init_state(void)
{
    bal_state_t *st = &g_balState;

    st->has_gyro = false;
    st->has_motors = false;
    st->armed = false;
    st->ref_deg = 0.0f;   /* boot: horizonte */
    st->base_throttle = BAL_BASE_THROTTLE;
    st->angle_deg = 0.0f;
    st->angle_raw_deg = 0.0f;
    st->trim_deg = BAL_ANGLE_TRIM_DEG;
    st->acc_g[0] = 0.0f;
    st->acc_g[1] = 0.0f;
    st->acc_g[2] = 0.0f;
    st->rate_dps = 0.0f;
    st->correction = 0.0f;
    for (uint8_t i = 0u; i < MOTORS_COUNT; ++i) {
        st->motor[i] = 0u;
    }
    st->flags = 0u;
    st->last_disarm = 0u;
    st->gyro_err_run = 0u;
    st->gyro_errors = 0u;
    st->motor_drops = 0u;
    st->calib_reported = false;
}

void bal_run(void)
{
    bal_state_t *st = &g_balState;

    /* --- fase 1: timebase + consola --- */
    (void)timebase_init();
    bal_init_state();
    console_led_fail();
    console_init();
    console_banner("Balancin 2 motores (M1 izq / M3 der) - PC por USB");
    console_printf("Control %u Hz, motores max %u%%, ref 0 = horizonte, "
                   "antihorario + visto desde atras\r\n",
                   (unsigned)BAL_PID_RATE_HZ,
                   (unsigned)(BAL_MOTOR_LIMIT * 100.0f + 0.5f));

    /* --- fase 2: IMU + calibracion del gyro --- */
    uint8_t who = 0u;
    if ((imu_check_who_am_i(&who) == IMU_OK) && (imu_init() == IMU_OK)) {
        st->has_gyro = true;
        st->flags |= BAL_FLAG_CALIBRATING;
        sensors_calib_start(SENSORS_CALIB_SAMPLES);
        console_result(true, "IMU LSM6DSV16X");
        console_printf("Calibrando gyro (%u ms): no mover el balancin\r\n",
                       (unsigned)((SENSORS_CALIB_SAMPLES * 1000u) / BAL_GYRO_RATE_HZ));
    } else {
        st->flags |= BAL_FLAG_NO_GYRO;
        console_printf("IMU WHO=0x%02X\r\n", (unsigned)who);
        console_result(false, "IMU - no se puede armar");
    }
    sensors_attitude_reset();

    sensors_battery_init();
    sensors_battery_update();

    /* --- fase 3: motores (DShot300 en los 4 canales, M2/M4 siempre 0) --- */
    if (motors_init_all(MOTORS_PROTO_DSHOT300) == MOT_OK) {
        st->has_motors = true;
        (void)motors_disarm_all();
        console_result(true, "Motores DShot300 (M1 + M3; M2/M4 deshabilitados)");
    } else {
        st->flags |= BAL_FLAG_NO_MOTORS;
        console_result(false, "Motores - no se puede armar");
    }
    console_led_fail();   /* console_result pinta el LED; rojo = desarmado */

    /* --- fase 4: control + scheduler --- */
    const float filter_dt = 1.0f / (float)BAL_GYRO_RATE_HZ;
    pt1_init(&s_gyro_lpf, BAL_GYRO_LPF_HZ, filter_dt);
    pt1_init(&s_gyro_lpf2, BAL_GYRO_LPF2_HZ, filter_dt);
    bal_pid_init(1.0f / (float)BAL_PID_RATE_HZ);
    bal_link_init();
    bal_task_port();

    s_realtime.gyro_stage = bal_task_gyro;
    s_realtime.filter_stage = bal_task_filter;
    s_realtime.pid_stage = bal_task_control;
    s_realtime.rx_check = 0;
    s_realtime.failsafe_check = bal_task_port;
    s_realtime.filter_denom = 1u;
    s_realtime.pid_denom = BAL_PID_DENOM;
    s_realtime.failsafe_period_ms = BAL_FAILSAFE_PERIOD_MS;
    s_realtime.pid_period_us = 1000000u / BAL_PID_RATE_HZ;
    s_realtime.gyro_enabled = st->has_gyro;

    s_task_count = 0u;
    bal_task_add("LINK", bal_task_link, 5000u, 4u);
    if (st->has_gyro) {
        bal_task_add("ATTITUDE", bal_task_attitude, 10000u, 3u);
    }
    bal_task_add("BATT", bal_task_battery, 100000u, 1u);

    scheduler_init(&s_realtime, s_tasks, s_task_count);

    console_printf("Comandos: A armar, D parar, R <deg> referencia, "
                   "T <pct> throttle base (default %u%%), Z cero (nivelado), ? estado\r\n",
                   (unsigned)(BAL_BASE_THROTTLE * 100.0f + 0.5f));

    scheduler_run();   /* no retorna */
}
