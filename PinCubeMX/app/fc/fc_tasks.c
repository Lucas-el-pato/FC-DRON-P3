/**
 ******************************************************************************
 * @file    fc_tasks.c
 * @brief   Etapas realtime + tabla de tareas lentas (ver fc_tasks.h).
 ******************************************************************************
 */

#include "fc_tasks.h"
#include "fc_rc.h"
#include "fc_state.h"

#include "arming.h"
#include "failsafe.h"
#include "filter.h"
#include "mixer.h"
#include "pid.h"

#include "driver_baro.h"
#include "driver_crsf.h"
#include "driver_imu.h"
#include "driver_mag.h"
#include "driver_motors.h"
#include "sensors_scale.h"
#include "telemtry.h"
#include "console.h"

#if FC_ENABLE_GPS
#include "driver_gps.h"
#endif

#define FC_MAG_TIMEOUT_MS   8u
#define FC_ATTITUDE_DT_S    0.01f    /* TASK_ATTITUDE corre a 100 Hz */

static pt1_filter_t s_gyro_lpf[PID_AXIS_COUNT];
static sched_realtime_t s_realtime;
static sched_task_t s_tasks[SCHED_MAX_TASKS];
static uint8_t s_task_count = 0u;

/* ========================================================================= */
/* Etapas realtime                                                            */
/* ========================================================================= */

void fc_task_gyro(void)
{
    fc_state_t *st = fc_state();

    if (imu_read_gyro(&st->gyro_raw) != IMU_OK) {
        st->gyro_errors++;
        return;
    }

    sensors_imu_scale(&st->gyro_raw, &st->imu_si);
    st->gyro_reads++;
}

void fc_task_filter(void)
{
    fc_state_t *st = fc_state();

    st->gyro_filt_dps[PID_AXIS_ROLL] =
        FC_GYRO_ROLL_SIGN * pt1_apply(&s_gyro_lpf[PID_AXIS_ROLL], st->imu_si.gx_dps);
    st->gyro_filt_dps[PID_AXIS_PITCH] =
        FC_GYRO_PITCH_SIGN * pt1_apply(&s_gyro_lpf[PID_AXIS_PITCH], st->imu_si.gy_dps);
    st->gyro_filt_dps[PID_AXIS_YAW] =
        FC_GYRO_YAW_SIGN * pt1_apply(&s_gyro_lpf[PID_AXIS_YAW], st->imu_si.gz_dps);
}

void fc_task_pid(void)
{
    fc_state_t *st = fc_state();
    const fc_rc_t *rc = fc_rc_get();

    /* 1. Armado / desarmado con el estado RC mas reciente. */
    arming_update(rc->arm_switch, rc->throttle, rc->link_ok,
                  false, failsafe_active());

    const bool armed = arming_is_armed();

    /* 2. PID (etapa 1: passthrough, ver FC_ENABLE_PID). */
#if FC_ENABLE_PID
    if (armed) {
        pid_update(rc->setpoint_dps, st->gyro_filt_dps, rc->throttle, st->pid_out);
    } else {
        st->pid_out[PID_AXIS_ROLL] = 0.0f;
        st->pid_out[PID_AXIS_PITCH] = 0.0f;
        st->pid_out[PID_AXIS_YAW] = 0.0f;
    }
#else
    st->pid_out[PID_AXIS_ROLL] = 0.0f;
    st->pid_out[PID_AXIS_PITCH] = 0.0f;
    st->pid_out[PID_AXIS_YAW] = 0.0f;
#endif

    /* 3. Mixer + salida DShot a los 4 ESCs. Se escribe siempre (tambien
     * desarmado, con throttle 0) para que los ESC no pierdan la senal.    */
    mixer_run(rc->throttle, st->pid_out, armed, st->motor);

    const mot_status_t mst = motors_write4(st->motor, 0u);
    if (mst == MOT_OK) {
        st->motor_frames++;
    } else {
        st->motor_drops++;
    }
}

void fc_task_rx(void)
{
    if (fc_rc_poll()) {
        fc_state()->rx_frames++;
    }
}

void fc_task_failsafe(void)
{
    fc_rc_update_link();

    const fc_rc_t *rc = fc_rc_get();
    if (failsafe_update(rc->link_ok, arming_is_armed())) {
        arming_disarm();
    }
}

/* ========================================================================= */
/* Tareas lentas                                                              */
/* ========================================================================= */

static void fc_task_attitude(void)
{
    fc_state_t *st = fc_state();

    /* Lectura completa (gyro + accel): el filtro complementario necesita el
     * acelerometro, que el lazo realtime no lee.                          */
    imu_sample_t sample;
    if (imu_read_sample(&sample) != IMU_OK) {
        st->gyro_errors++;
        return;
    }

    sensors_imu_si_t si;
    sensors_imu_scale(&sample, &si);
    sensors_attitude_update(&si,
                            st->has_mag ? &st->mag_si : 0,
                            FC_ATTITUDE_DT_S,
                            &st->attitude);
}

static void fc_task_baro(void)
{
    fc_state_t *st = fc_state();

    baro_sample_t raw;
    if (baro_read_sample(&raw) != BARO_OK) {
        return;
    }
    if (sensors_baro_update(&raw, &st->baro_si) == SENSORS_SCALE_OK) {
        st->baro_updates++;
    }
}

#if FC_ENABLE_MAG
static void fc_task_mag(void)
{
    fc_state_t *st = fc_state();

    mag_sample_t raw;
    if (mag_read_sample(&raw, FC_MAG_TIMEOUT_MS) != MAG_OK) {
        return;
    }
    sensors_mag_scale(&raw, &st->mag_si);
    st->mag_updates++;
}
#endif

static void fc_task_esc_telem(void)
{
    telemtry_data_t data;
    (void)telemtry_poll(&data);   /* g_telemLast queda con el ultimo valido */
}

#if FC_ENABLE_TELEM_TX
static void fc_task_telem_attitude(void)
{
    const fc_state_t *st = fc_state();
    (void)crsf_send_attitude(st->attitude.pitch_rad,
                             st->attitude.roll_rad,
                             st->attitude.yaw_rad);
}

static void fc_task_telem_baro(void)
{
    const fc_state_t *st = fc_state();
    if (!st->baro_si.valid) {
        return;
    }
    (void)crsf_send_baro_altitude(st->baro_si.alt_dm);
    (void)crsf_send_vario(st->baro_si.vspeed_cm_s);
}
#endif

#if FC_ENABLE_GPS
static void fc_task_gps(void)
{
    gps_nmea_sentence_t s;
    (void)gps_read_sentence(&s, 1u);
}
#endif

static void fc_task_log(void)
{
    const fc_state_t *st = fc_state();
    const fc_rc_t *rc = fc_rc_get();

    /* Con el drone armado no se imprime: console_print puede bloquear hasta
     * 160 ms si el host deja de leer el CDC.                               */
    if (arming_is_armed()) {
        return;
    }

    console_printf("RC thr=%4u r=%4u p=%4u y=%4u arm=%u link=%u age=%lums | "
                   "M %4u %4u %4u %4u | flags=0x%02lX fs=%s | pid_it=%lu drops=%lu\r\n",
                   (unsigned)rc->raw[FC_RC_CH_THROTTLE],
                   (unsigned)rc->raw[FC_RC_CH_ROLL],
                   (unsigned)rc->raw[FC_RC_CH_PITCH],
                   (unsigned)rc->raw[FC_RC_CH_YAW],
                   (unsigned)rc->arm_switch,
                   (unsigned)rc->link_ok,
                   (unsigned long)rc->age_ms,
                   (unsigned)st->motor[0], (unsigned)st->motor[1],
                   (unsigned)st->motor[2], (unsigned)st->motor[3],
                   (unsigned long)arming_disable_flags(),
                   failsafe_state_name(failsafe_get_state()),
                   (unsigned long)scheduler_pid_iterations(),
                   (unsigned long)st->motor_drops);
}

/* ========================================================================= */
/* Registro                                                                   */
/* ========================================================================= */

static void fc_task_add(const char *name, sched_task_fn fn,
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

void fc_tasks_init(void)
{
    fc_state_t *st = fc_state();

    /* Filtros del gyro: dt del lazo de filtrado. */
    const float filter_dt = (float)FC_FILTER_DENOM / (float)FC_GYRO_RATE_HZ;
    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        pt1_init(&s_gyro_lpf[i], FC_GYRO_LPF_HZ, filter_dt);
    }

    pid_init(1.0f / (float)FC_PID_RATE_HZ);
    mixer_init();

    /* --- Cadena realtime --- */
    s_realtime.gyro_stage = fc_task_gyro;
    s_realtime.filter_stage = fc_task_filter;
    s_realtime.pid_stage = fc_task_pid;
    s_realtime.rx_check = fc_task_rx;
    s_realtime.failsafe_check = fc_task_failsafe;
    s_realtime.filter_denom = FC_FILTER_DENOM;
    s_realtime.pid_denom = FC_PID_DENOM;
    s_realtime.failsafe_period_ms = FAILSAFE_PERIOD_MS;
    s_realtime.pid_period_us = 1000000u / FC_PID_RATE_HZ;
    s_realtime.gyro_enabled = st->has_gyro;

    /* --- Cola lenta (periodo en us, prioridad 1 = baja .. 5 = alta) --- */
    s_task_count = 0u;

    /* RX tambien en la cola: si el gyro no esta, el enlace igual se atiende. */
    fc_task_add("RX", fc_task_rx, 2000u, 5u);

    if (st->has_gyro) {
        fc_task_add("ATTITUDE", fc_task_attitude, 10000u, 3u);
    }
    if (st->has_baro) {
        fc_task_add("BARO", fc_task_baro, 20000u, 2u);
    }
#if FC_ENABLE_MAG
    if (st->has_mag) {
        fc_task_add("MAG", fc_task_mag, 100000u, 2u);
    }
#endif
    fc_task_add("ESC_TELEM", fc_task_esc_telem, 10000u, 2u);

#if FC_ENABLE_TELEM_TX
    fc_task_add("TELEM_ATT", fc_task_telem_attitude, 50000u, 1u);
    if (st->has_baro) {
        fc_task_add("TELEM_BARO", fc_task_telem_baro, 200000u, 1u);
    }
#endif
#if FC_ENABLE_GPS
    fc_task_add("GPS", fc_task_gps, 100000u, 1u);
#endif
    fc_task_add("LOG", fc_task_log, 500000u, 1u);
}

const sched_realtime_t *fc_tasks_realtime(void)
{
    return &s_realtime;
}

sched_task_t *fc_tasks_table(uint8_t *count)
{
    if (count != 0) {
        *count = s_task_count;
    }
    return s_tasks;
}
