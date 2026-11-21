/**
 ******************************************************************************
 * @file    fc_tasks.c
 * @brief   Etapas realtime + tabla de tareas lentas (ver fc_tasks.h).
 ******************************************************************************
 */

#include "fc_tasks.h"
#include "fc_blackbox.h"
#include "fc_rc.h"
#include "fc_state.h"
#include "fc_telem.h"

#include "arming.h"
#include "failsafe.h"
#include "filter.h"
#include "mixer.h"
#include "pid.h"
#include "rpm_filter.h"
#include "dyn_notch.h"

#include "driver_baro.h"
#include "driver_imu.h"
#include "driver_mag.h"
#include "driver_motors.h"
#include "sensors_calib.h"
#include "sensors_battery.h"
#include "sensors_scale.h"
#include "telemtry.h"
#include "timebase.h"
#include "console.h"

#if FC_ENABLE_GPS
#include "driver_gps.h"
#endif

#define FC_ATTITUDE_DT_S    (1.0f / (float)FC_ATTITUDE_RATE_HZ)
#define FC_RAD2DEG          57.2957795f

static pt1_filter_t s_gyro_lpf[PID_AXIS_COUNT];    /* gyro_lpf1 de BF */
static pt1_filter_t s_gyro_lpf2[PID_AXIS_COUNT];   /* gyro_lpf2 de BF */
static float s_gyro_peak_dps[PID_AXIS_COUNT];

#if FC_GYRO_NOTCH_ENABLE
/* Notch por motor: un juego de coeficientes por armonico (comun a los 3
 * ejes) y un estado por eje y armonico. Los coeficientes los recalcula
 * fc_notch_track() desde el PID; el filtro corre a 8 kHz.               */
static biquad_coeff_t s_notch_coeff[FC_GYRO_NOTCH_HARMONICS];
static biquad_state_t s_notch_state[FC_GYRO_NOTCH_HARMONICS][PID_AXIS_COUNT];
static pt1_filter_t   s_notch_dshot_lpf;   /* DShot medio suavizado      */
static float          s_notch_center_hz;   /* centro aplicado (diagnost.) */
#endif

#define FC_RPM_FILTER_ON  (FC_MOTOR_BIDIR && FC_GYRO_RPM_FILTER)

#if FC_RPM_FILTER_ON
/* Filtro RPM activo (bidireccional arrancado). Si no, notch estimado.   */
static bool     s_rpm_active = false;
static uint32_t s_rpm_frames = 0u;                 /* frames ya leidos   */
static uint32_t s_rpm_ok[4];                       /* respuestas validas */
static uint16_t s_rpm_stale[4];                    /* frames sin validas */
#endif

/* Gyro acumulado a 8 kHz entre dos pasadas de la actitud: la actitud
 * integra el promedio y no una muestra suelta (que trae aliasing).       */
static float    s_att_gyro_sum[3];
static uint32_t s_att_gyro_count = 0u;

/* Ultima muestra completa sin calibrar (gyro + accel). La actitud saca
 * el accel de aca: no vuelve a usar el SPI1, que es del stream DMA.     */
static imu_sample_t s_last_raw;
static bool         s_have_raw = false;

/* HAL_GetTick() de la ultima muestra del mag (la escribe la tarea MAG). */
static uint32_t s_mag_last_ms = 0u;
static sched_realtime_t s_realtime;
static sched_task_t s_tasks[SCHED_MAX_TASKS];
static uint8_t s_task_count = 0u;

/* ========================================================================= */
/* Etapas realtime                                                            */
/* ========================================================================= */

/* Disparo de la cadena realtime: muestra nueva del stream (DMA). Queda en
 * st->gyro_raw (gyro + accel, raw) para fc_task_gyro.                    */
static bool fc_gyro_sample_ready(void)
{
    return imu_stream_take(&fc_state()->gyro_raw);
}

/* Mientras calibra alimenta la ventana de calibracion con gyro + accel. */
static void fc_task_gyro_calibrate(fc_state_t *st)
{
    sensors_imu_scale(&st->gyro_raw, &st->imu_si);

    if (sensors_calib_feed(&st->gyro_raw)) {
        /* La actitud arranca de nuevo desde el accel ya nivelado. */
        sensors_attitude_reset();
        pid_reset();
        arming_disable_clear(ARMING_DISABLED_CALIBRATING);
    }
}

void fc_task_gyro(void)
{
    fc_state_t *st = fc_state();

    st->gyro_reads++;
    s_last_raw = st->gyro_raw;
    s_have_raw = true;

    if (!sensors_calib_done()) {
        fc_task_gyro_calibrate(st);
        return;
    }

    sensors_calib_apply(&st->gyro_raw, false);
    sensors_imu_scale(&st->gyro_raw, &st->imu_si);
    st->gyro_reads++;

    /* Acumulado para la actitud (gyroGetAccumulationAverage de BF). */
    s_att_gyro_sum[0] += st->imu_si.gx_rad_s;
    s_att_gyro_sum[1] += st->imu_si.gy_rad_s;
    s_att_gyro_sum[2] += st->imu_si.gz_rad_s;
    s_att_gyro_count++;
}

static void fc_attitude_update(void);

#if FC_GYRO_NOTCH_ENABLE
static void fc_notch_set_center(float center_hz)
{
    const float dt = (float)FC_FILTER_DENOM / (float)FC_GYRO_RATE_HZ;
    for (uint8_t h = 0u; h < FC_GYRO_NOTCH_HARMONICS; ++h) {
        biquad_notch_coeff(&s_notch_coeff[h], center_hz * (float)(h + 1u),
                           FC_GYRO_NOTCH_Q, dt);
    }
    s_notch_center_hz = center_hz;
}

/* Llamada desde el PID (2 kHz) con los DShot recien calculados: mueve el
 * centro del notch a la frecuencia de giro estimada de los motores.     */
static void fc_notch_track(const uint16_t motor[4], bool armed)
{
    float dshot = 0.0f;
    if (armed) {
        dshot = 0.25f * ((float)motor[0] + (float)motor[1] +
                         (float)motor[2] + (float)motor[3]);
    }
    dshot = pt1_apply(&s_notch_dshot_lpf, dshot);

    float center = FC_GYRO_NOTCH_HZ_PER_DSHOT * dshot + FC_GYRO_NOTCH_OFFSET_HZ;
    if (center < FC_GYRO_NOTCH_MIN_HZ) {
        center = FC_GYRO_NOTCH_MIN_HZ;
    } else if (center > FC_GYRO_NOTCH_MAX_HZ) {
        center = FC_GYRO_NOTCH_MAX_HZ;
    }

    /* sinf/cosf solo si el centro se movio de verdad. */
    const float diff = center - s_notch_center_hz;
    if ((diff > 0.5f) || (diff < -0.5f)) {
        fc_notch_set_center(center);
    }
}
#endif

#if FC_RPM_FILTER_ON
/* Una vez por frame nuevo (2 kHz): velocidad de cada rotor al filtro RPM.
 * eRPM / 60 = Hz electricos; / pares de polos = Hz del rotor.          */
static void fc_rpm_feed(void)
{
    const motors_bidir_stats_t *bd = motors_bidir_stats();
    if ((bd == NULL) || (bd->frames == s_rpm_frames)) {
        return;
    }
    s_rpm_frames = bd->frames;

    const float erpm_to_hz = 1.0f / (60.0f * ((float)TELEMTRY_MOTOR_POLES / 2.0f));
    float hz[4];
    for (uint8_t m = 0u; m < 4u; ++m) {
        if (bd->ok[m] != s_rpm_ok[m]) {
            s_rpm_ok[m] = bd->ok[m];
            s_rpm_stale[m] = 0u;
        } else if (s_rpm_stale[m] < 0xFFFFu) {
            s_rpm_stale[m]++;
        }
        hz[m] = (s_rpm_stale[m] > FC_RPM_STALE_FRAMES) ? 0.0f
                                                        : (float)bd->erpm[m] * erpm_to_hz;
    }
    rpm_filter_set_motor_hz(hz);
}
#endif

/* Notch de motores de un eje: filtro RPM si esta activo, si no el estimado. */
static float fc_gyro_notch(uint8_t axis, float x)
{
#if FC_RPM_FILTER_ON
    if (s_rpm_active) {
        return rpm_filter_apply(axis, x);
    }
#endif
#if FC_GYRO_NOTCH_ENABLE
    for (uint8_t h = 0u; h < FC_GYRO_NOTCH_HARMONICS; ++h) {
        x = biquad_apply(&s_notch_coeff[h], &s_notch_state[h][axis], x);
    }
#else
    (void)axis;
#endif
    return x;
}

float fc_gyro_notch_hz(void)
{
#if FC_RPM_FILTER_ON
    if (s_rpm_active) {
        return rpm_filter_mean_hz();
    }
#endif
#if FC_GYRO_NOTCH_ENABLE
    return s_notch_center_hz;
#else
    return 0.0f;
#endif
}

void fc_task_filter(void)
{
    fc_state_t *st = fc_state();

    const float raw[PID_AXIS_COUNT] = {
        FC_GYRO_ROLL_SIGN * st->imu_si.gx_dps,
        FC_GYRO_PITCH_SIGN * st->imu_si.gy_dps,
        FC_GYRO_YAW_SIGN * st->imu_si.gz_dps,
    };

#if FC_RPM_FILTER_ON
    if (s_rpm_active) {
        rpm_filter_step();   /* 2 de los 12 juegos de coeficientes */
    }
#endif

    /* Notches de motores (filtro RPM o estimado). */
    float pre[PID_AXIS_COUNT];
    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
        pre[i] = fc_gyro_notch(i, raw[i]);
    }
#if FC_GYRO_DYN_NOTCH
    /* El analisis ve el gyro ANTES del notch dinamico. */
    dyn_notch_accumulate(pre);
#endif

    for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
#if FC_GYRO_DYN_NOTCH
        const float x = dyn_notch_apply(i, pre[i]);
#else
        const float x = pre[i];
#endif
        const float f = pt1_apply(&s_gyro_lpf2[i], pt1_apply(&s_gyro_lpf[i], x));
        st->gyro_filt_dps[i] = f;

        /* Pico desde el ultimo log: con el dron quieto es el ruido que ve el PID. */
        const float a = (f < 0.0f) ? -f : f;
        if (a > s_gyro_peak_dps[i]) {
            s_gyro_peak_dps[i] = a;
        }
    }

#if FC_GYRO_DYN_NOTCH
    /* SDFT + analisis de un eje en la muestra siguiente al PID (liviana):
     * cada FC_PID_DENOM muestras = 2 kHz, con el promedio de esas muestras. */
    if ((scheduler_gyro_samples() % FC_PID_DENOM) == 1u) {
        dyn_notch_update();
    }
#endif

#if FC_MOTOR_BIDIR
    /* eRPM del ultimo frame en la muestra justo antes del PID: la respuesta
     * del ESC termina ~250 us despues del PID y esta muestra llega a los
     * 375 us. Asi la decodificacion no alarga el ciclo del PID.          */
    if ((scheduler_gyro_samples() % FC_PID_DENOM) == (FC_PID_DENOM - 1u)) {
        const uint32_t t_dec = timebase_now();
        motors_bidir_service();
#if FC_RPM_FILTER_ON
        if (s_rpm_active) {
            fc_rpm_feed();
        }
#endif
        const uint32_t dec_us = timebase_elapsed_us(t_dec);
        if (dec_us > st->motor_us_max) {
            st->motor_us_max = dec_us;
        }
    }
#endif

    /* Actitud a 1 kHz, en una muestra sin PID (el PID corre cuando el
     * contador es multiplo de FC_PID_DENOM) para repartir la carga.     */
    if ((scheduler_gyro_samples() % FC_ATTITUDE_DENOM) == FC_ATTITUDE_PHASE) {
        fc_attitude_update();
    }
}

#if FC_MOTOR_BIDIR
/* Retiene las lineas de los ESC en bajo mientras los ESC estan apagados o
 * arrancando (ver FC_ESC_BOOT_MS). Solo desarmado.                       */
static void fc_esc_power_gate(void)
{
    static bool     s_powered = false;
    static uint32_t s_powered_ms = 0u;

    if (!motors_bidir_active() || arming_is_armed()) {
        return;
    }

    const uint32_t now_ms = HAL_GetTick();
    const bool powered = sensors_battery_get()->voltage_cv >= FC_ESC_POWER_MIN_CV;

    if (!powered) {
        s_powered = false;
        if (!motors_bidir_held()) {
            motors_bidir_hold(true);
            arming_disable_set(ARMING_DISABLED_NO_MOTORS);
        }
        return;
    }
    if (!s_powered) {
        s_powered = true;
        s_powered_ms = now_ms;
    }
    if (motors_bidir_held() && ((now_ms - s_powered_ms) >= FC_ESC_BOOT_MS)) {
        motors_bidir_hold(false);
        arming_disable_clear(ARMING_DISABLED_NO_MOTORS);
    }
}
#endif

void fc_task_pid(void)
{
    fc_state_t *st = fc_state();
    const fc_rc_t *rc = fc_rc_get();

#if FC_MOTOR_BIDIR
    fc_esc_power_gate();
#endif

    /* 1. Armado / desarmado con el estado RC mas reciente. */
    arming_update(rc->arm_switch, rc->throttle, rc->link_ok,
                  false, failsafe_active());

    const bool armed = arming_is_armed();

    /* 2. PID ANGLE (etapa 1: passthrough, ver FC_ENABLE_PID). */
#if FC_ENABLE_PID
    if (armed) {
        pid_update(rc->stick, rc->setpoint_dps[PID_AXIS_YAW], st->angle_deg,
                   st->gyro_filt_dps, rc->throttle, st->pid_out);
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
#if FC_GYRO_NOTCH_ENABLE
#if FC_RPM_FILTER_ON
    if (!s_rpm_active)
#endif
    {
        fc_notch_track(st->motor, armed);
    }
#endif

    /* Con DShot bidireccional incluye decodificar la respuesta anterior.
     * Retenido (ESC apagados o arrancando): no se manda nada.            */
    if (!motors_bidir_held()) {
        const uint32_t t_mot = timebase_now();
        const mot_status_t mst = motors_write4(st->motor, 0u);
        const uint32_t mot_us = timebase_elapsed_us(t_mot);
        if (mot_us > st->motor_us_max) {
            st->motor_us_max = mot_us;
        }
        if (mst == MOT_OK) {
            st->motor_frames++;
        } else {
            st->motor_drops++;
        }
    }

    /* 4. Blackbox: copia el estado de este PID al ring (no bloquea). */
    blackbox_record();
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

/* Actitud + angulos del lazo de nivel. Corre desde fc_task_filter a
 * FC_ATTITUDE_RATE_HZ (antes TASK_ATTITUDE, tarea lenta a 100 Hz).       */
static void fc_attitude_update(void)
{
    fc_state_t *st = fc_state();

    /* Sin calibrar no hay nivel de referencia: no se integra nada. */
    if (!sensors_calib_done()) {
        return;
    }

    /* Ultima muestra del stream (el DMA ya trae gyro + accel en cada INT1).
     * El filtro complementario necesita el accel; el gyro se reemplaza
     * abajo por el promedio acumulado.                                    */
    if (!s_have_raw) {
        return;
    }
    imu_sample_t sample = s_last_raw;

    sensors_calib_apply(&sample, true);

    sensors_imu_si_t si;
    sensors_imu_scale(&sample, &si);

    /* Gyro: promedio de las muestras desde la pasada anterior, y dt = el
     * tiempo que cubren (exacto aunque la tarea se haya atrasado). El accel
     * ya llega filtrado a ~10 Hz por el LPF2 de la IMU.                    */
    float dt_s = FC_ATTITUDE_DT_S;
    if (s_att_gyro_count > 0u) {
        const float inv = 1.0f / (float)s_att_gyro_count;
        si.gx_rad_s = s_att_gyro_sum[0] * inv;
        si.gy_rad_s = s_att_gyro_sum[1] * inv;
        si.gz_rad_s = s_att_gyro_sum[2] * inv;
        dt_s = (float)s_att_gyro_count / (float)FC_GYRO_RATE_HZ;
    }
    s_att_gyro_sum[0] = 0.0f;
    s_att_gyro_sum[1] = 0.0f;
    s_att_gyro_sum[2] = 0.0f;
    s_att_gyro_count = 0u;

    /* Mag solo con una muestra fresca: el yaw se corrige hacia el rumbo
     * del mag en cada pasada. Sin mag (o vencido) el yaw sigue con gyro. */
    const bool mag_fresh = FC_ENABLE_MAG && st->has_mag && (st->mag_updates > 0u) &&
                           ((HAL_GetTick() - s_mag_last_ms) <= FC_MAG_FRESH_MS);
    sensors_attitude_update(&si, mag_fresh ? &st->mag_si : 0, dt_s, &st->attitude);

    /* Angulos para el lazo de nivel, con el mismo signo que el gyro del PID. */
    st->angle_deg[PID_AXIS_ROLL] = FC_GYRO_ROLL_SIGN * st->attitude.roll_rad * FC_RAD2DEG;
    st->angle_deg[PID_AXIS_PITCH] = FC_GYRO_PITCH_SIGN * st->attitude.pitch_rad * FC_RAD2DEG;
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
/* No bloquea: avanza la maquina de estados I2C (driver_mag async) y toma
 * la muestra nueva si la hay. La transferencia corre por interrupcion.  */
static void fc_task_mag(void)
{
    fc_state_t *st = fc_state();

    mag_async_poll();

    mag_sample_t raw;
    if (!mag_async_take(&raw)) {
        return;
    }
    sensors_mag_scale(&raw, &st->mag_si);
    s_mag_last_ms = HAL_GetTick();
    st->mag_updates++;
}
#endif

bool fc_mag_ok(void)
{
    const fc_state_t *st = fc_state();
    return FC_ENABLE_MAG && st->has_mag && (st->mag_updates > 0u) &&
           ((HAL_GetTick() - s_mag_last_ms) <= FC_MAG_OK_MS);
}

static void fc_task_battery(void)
{
    sensors_battery_update();
}

static void fc_task_esc_telem(void)
{
    telemtry_data_t data;
    (void)telemtry_poll(&data);   /* g_telemLast queda con el ultimo valido */
}

#if FC_ENABLE_GPS
static void fc_task_gps(void)
{
    gps_process();   /* vacia el ring de la ISR y parsea NMEA */
}
#endif

/* Ultima ventana del lazo. Tiempos en us; con el dron quieto y desarmado
 * pid_int deberia quedar cerca de 500 y missed en 0.                     */
static void fc_task_log_loop(const fc_loop_stats_t *lp)
{
    const sched_stats_t *sc = &lp->sched;

    if (lp->windows == 0u) {
        console_printf("LOOP sin ventana todavia\r\n");
        return;
    }

    console_printf("LOOP win=%lums gyro=%lu pid=%lu | max us: rt=%lu pid=%lu "
                   "pid_int=%lu | slow max=%lu us (%s) runs=%lu forced=%lu\r\n",
                   (unsigned long)(sc->window_us / 1000u),
                   (unsigned long)sc->gyro_samples,
                   (unsigned long)sc->pid_runs,
                   (unsigned long)sc->realtime_max_us,
                   (unsigned long)sc->pid_max_us,
                   (unsigned long)sc->pid_interval_max_us,
                   (unsigned long)sc->slow_task_max_us,
                   scheduler_task_name(sc->slow_task_index),
                   (unsigned long)sc->slow_runs,
                   (unsigned long)sc->slow_forced);
    console_printf("DRDY edges=%lu late=%lu missed=%lu | drdy-samples=%lu\r\n",
                   (unsigned long)lp->drdy_edges,
                   (unsigned long)lp->drdy_late_events,
                   (unsigned long)lp->drdy_missed,
                   (unsigned long)lp->drdy_minus_samples);
    console_printf("IMU %s samples=%lu ovr=%lu skip=%lu err=%lu rec=%lu\r\n",
                   IMU_USE_DMA ? "DMA" : "POLL",
                   (unsigned long)lp->imu.samples,
                   (unsigned long)lp->imu.overruns,
                   (unsigned long)lp->imu.skipped,
                   (unsigned long)lp->imu.spi_errors,
                   (unsigned long)lp->imu.recoveries);
}

static void fc_task_log(void)
{
    const fc_state_t *st = fc_state();
    const fc_rc_t *rc = fc_rc_get();

#if !FC_ENABLE_TELEM_TX
    /* Sin telemetria nadie mas cierra la ventana: la toma el log. */
    fc_tasks_loop_stats_take();
#endif

    /* Con el drone armado no se imprime: console_print puede bloquear hasta
     * 160 ms si el host deja de leer el CDC.                               */
    if (arming_is_armed() && !FC_LOG_WHEN_ARMED) {
        return;
    }

    /* Valores x10 en enteros: nano.specs no imprime floats. */
    if (sensors_calib_done()) {
        console_printf("CAL ok rst=%lu | ang x0.1deg R=%5ld P=%5ld | "
                       "gyro x0.1dps R=%5ld P=%5ld Y=%5ld\r\n",
                       (unsigned long)sensors_calib_restarts(),
                       (long)(st->angle_deg[PID_AXIS_ROLL] * 10.0f),
                       (long)(st->angle_deg[PID_AXIS_PITCH] * 10.0f),
                       (long)(st->gyro_filt_dps[PID_AXIS_ROLL] * 10.0f),
                       (long)(st->gyro_filt_dps[PID_AXIS_PITCH] * 10.0f),
                       (long)(st->gyro_filt_dps[PID_AXIS_YAW] * 10.0f));
        console_printf("PID sp dps R=%5ld P=%5ld Y=%5ld | out x1000 R=%5ld P=%5ld Y=%5ld\r\n",
                       (long)pid_setpoint_dps(PID_AXIS_ROLL),
                       (long)pid_setpoint_dps(PID_AXIS_PITCH),
                       (long)pid_setpoint_dps(PID_AXIS_YAW),
                       (long)(st->pid_out[PID_AXIS_ROLL] * 1000.0f),
                       (long)(st->pid_out[PID_AXIS_PITCH] * 1000.0f),
                       (long)(st->pid_out[PID_AXIS_YAW] * 1000.0f));
        /* Ruido: D en x1000 de motor y pico de gyro filtrado desde el ultimo
         * log. Con el dron quieto, D grande = zumbido/calor por ruido en D. */
        console_printf("NOISE D x1000 R=%4ld P=%4ld Y=%4ld | gyro pk x0.1dps R=%5ld P=%5ld Y=%5ld | notch %ld Hz\r\n",
                       (long)pid_dterm(PID_AXIS_ROLL),
                       (long)pid_dterm(PID_AXIS_PITCH),
                       (long)pid_dterm(PID_AXIS_YAW),
                       (long)(s_gyro_peak_dps[PID_AXIS_ROLL] * 10.0f),
                       (long)(s_gyro_peak_dps[PID_AXIS_PITCH] * 10.0f),
                       (long)(s_gyro_peak_dps[PID_AXIS_YAW] * 10.0f),
                       (long)fc_gyro_notch_hz());
        for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
            s_gyro_peak_dps[i] = 0.0f;
        }
    } else if (st->has_gyro) {
        console_printf("CAL calibrando IMU, no mover el dron (rst=%lu)\r\n",
                       (unsigned long)sensors_calib_restarts());
    }

#if FC_ENABLE_MAG
    if (st->has_mag) {
        mag_async_stats_t ms;
        mag_async_stats(&ms);
        /* Rumbo 0..360 en sentido horario desde el norte (con declinacion). */
        long hdg = (long)(st->attitude.yaw_rad * FC_RAD2DEG);
        if (hdg < 0) {
            hdg += 360;
        }
        console_printf("MAG ok=%u yaw=%3ld deg | uT x=%ld y=%ld z=%ld | n=%lu "
                       "notrdy=%lu err=%lu rec=%lu\r\n",
                       (unsigned)fc_mag_ok(), hdg,
                       (long)st->mag_si.mx_uT, (long)st->mag_si.my_uT,
                       (long)st->mag_si.mz_uT,
                       (unsigned long)ms.samples, (unsigned long)ms.not_ready,
                       (unsigned long)ms.errors, (unsigned long)ms.recoveries);
    }
#endif

    const sensors_battery_t *bat = sensors_battery_get();
    console_printf("VBAT %u.%02uV %uS %u%% (adc=%u)\r\n",
                   (unsigned)(bat->voltage_cv / 100u),
                   (unsigned)(bat->voltage_cv % 100u),
                   (unsigned)bat->cells,
                   (unsigned)bat->percent,
                   (unsigned)bat->raw);

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

#if FC_ENABLE_GPS
    const gps_data_t *g = gps_get();
    gps_stats_t gs;
    gps_stats_get(&gs);
    console_printf("GPS fix=%u q=%u sats=%u hdop=%u.%02u lat=%ld lon=%ld (x1e-7) "
                   "alt=%ldcm | ok=%lu bad=%lu ore=%lu ring=%lu\r\n",
                   (unsigned)g->valid, (unsigned)g->fix_quality,
                   (unsigned)g->sats,
                   (unsigned)(g->hdop_x100 / 100u), (unsigned)(g->hdop_x100 % 100u),
                   (long)g->lat_e7, (long)g->lon_e7, (long)g->alt_cm,
                   (unsigned long)gs.sentences_ok, (unsigned long)gs.sentences_bad,
                   (unsigned long)gs.uart_overruns, (unsigned long)gs.ring_overflows);
#endif

    fc_task_log_loop(&st->loop);
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
        pt1_init(&s_gyro_lpf2[i], FC_GYRO_LPF2_HZ, filter_dt);
        s_gyro_peak_dps[i] = 0.0f;
    }

#if FC_GYRO_NOTCH_ENABLE
    /* El tracker corre en el PID (2 kHz). Arranca en el minimo. */
    pt1_init(&s_notch_dshot_lpf, FC_GYRO_NOTCH_TRACK_HZ, 1.0f / (float)FC_PID_RATE_HZ);
    fc_notch_set_center(FC_GYRO_NOTCH_MIN_HZ);
    for (uint8_t h = 0u; h < FC_GYRO_NOTCH_HARMONICS; ++h) {
        for (uint8_t i = 0u; i < PID_AXIS_COUNT; ++i) {
            biquad_reset(&s_notch_state[h][i], 0.0f);
        }
    }
#endif

#if FC_RPM_FILTER_ON
    /* Despues de motors_init_all_bidir (fase 3): si el bidireccional no
     * arranco no hay eRPM y queda el notch estimado.                   */
    rpm_filter_init((float)FC_GYRO_RATE_HZ / (float)FC_FILTER_DENOM, (float)FC_PID_RATE_HZ);
    s_rpm_active = motors_bidir_active();
    s_rpm_frames = 0u;
    for (uint8_t m = 0u; m < 4u; ++m) {
        s_rpm_ok[m] = 0u;
        s_rpm_stale[m] = 0xFFFFu;
    }
#endif

#if FC_GYRO_DYN_NOTCH
    /* SDFT a FC_GYRO_RATE_HZ / FC_PID_DENOM (2 kHz): promedio de 4 muestras. */
    dyn_notch_init((float)FC_GYRO_RATE_HZ / (float)FC_FILTER_DENOM, (uint8_t)FC_PID_DENOM);
#endif

    pid_init(1.0f / (float)FC_PID_RATE_HZ);
    mixer_init();

    /* --- Cadena realtime --- */
    s_realtime.sample_ready = fc_gyro_sample_ready;
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

    /* La actitud ya no es tarea lenta: corre en fc_task_filter a 1 kHz. */
    if (st->has_baro) {
        fc_task_add("BARO", fc_task_baro, 20000u, 2u);
    }
#if FC_ENABLE_MAG
    if (st->has_mag) {
        mag_async_start();
        fc_task_add("MAG", fc_task_mag, FC_MAG_PERIOD_US, 2u);
    }
#endif
    fc_task_add("BATT", fc_task_battery, 100000u, 2u);
#if FC_ENABLE_BLACKBOX
    /* BB_WRITE nunca bloquea; BB_FILE solo trabaja desarmado (cierra y
     * reserva archivos con FatFs, puede tardar cientos de ms).          */
    fc_task_add("BB_WRITE", blackbox_task_write, 1000u, 3u);
    fc_task_add("BB_FILE", blackbox_task_file, 100000u, 1u);
#endif
    fc_task_add("ESC_TELEM", fc_task_esc_telem, 10000u, 2u);

#if FC_ENABLE_TELEM_TX
    fc_task_add("TELEM_ATT", fc_telem_attitude, FC_TELEM_ATT_PERIOD_US, 1u);
    if (st->has_baro) {
        fc_task_add("TELEM_BARO", fc_telem_baro, FC_TELEM_BARO_PERIOD_US, 1u);
    }
    fc_task_add("TELEM_BATT", fc_telem_battery, FC_TELEM_BATT_PERIOD_US, 1u);
    fc_task_add("TELEM_MODE", fc_telem_flight_mode, FC_TELEM_MODE_PERIOD_US, 1u);
    fc_task_add("TELEM_ESC", fc_telem_esc, FC_TELEM_ESC_PERIOD_US, 1u);
    fc_task_add("TELEM_DBG", fc_telem_debug, FC_TELEM_DEBUG_PERIOD_US, 1u);
#if FC_ENABLE_GPS
    fc_task_add("TELEM_GPS", fc_telem_gps, FC_TELEM_GPS_PERIOD_US, 1u);
#endif
#endif
#if FC_ENABLE_GPS
    /* Prioridad 2: si se atrasa mas de ~44 ms el ring se llena. */
    fc_task_add("GPS", fc_task_gps, FC_GPS_PERIOD_US, 2u);
#endif
    fc_task_add("LOG", fc_task_log, 500000u, 1u);
}

void fc_tasks_loop_stats_take(void)
{
    fc_loop_stats_t *lp = &fc_state()->loop;

    scheduler_stats_take(&lp->sched);

    imu_drdy_stats_t drdy;
    imu_gyro_drdy_stats_take(&drdy);
    lp->drdy_edges = drdy.edges;
    lp->drdy_late_events = drdy.late_events;
    lp->drdy_missed = imu_gyro_drdy_missed_estimate(drdy.late_cyc, drdy.late_events);

    /* Con DMA el ISR lee cada flanco aunque el lazo se atrase: esta resta
     * crece con las muestras que el lazo no alcanzo a tomar (imu.skipped).
     * En polling (IMU_USE_DMA = 0) queda en 0..1.                        */
    lp->drdy_minus_samples = imu_gyro_drdy_count() - scheduler_gyro_samples();

    imu_stream_stats_take(&lp->imu);
    /* gyro_errors (pagina 0 / log) sigue contando lecturas fallidas. */
    fc_state()->gyro_errors += lp->imu.spi_errors;

    lp->windows++;
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
