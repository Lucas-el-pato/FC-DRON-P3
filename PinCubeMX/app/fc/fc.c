/**
 ******************************************************************************
 * @file    fc.c
 * @brief   Flight controller: init por fases + entrada al scheduler.
 *
 *          Espeja la secuencia de Betaflight (fc/init.c) adaptada a CubeMX,
 *          que ya inicializo clocks y perifericos en main.c:
 *
 *            fase 1  timebase (DWT) + consola USB CDC
 *            fase 2  sensores: IMU (obligatorio), baro y mag (opcionales)
 *            fase 3  actuadores y enlaces: motores DShot, CRSF, telemetria ESC
 *            fase 4  control: PID, mixer, arming, failsafe, tabla de tareas
 *            run     scheduler_run() -- no retorna
 *
 *          Sin gyro no se arma nunca (ARMING_DISABLED_NO_GYRO), pero el
 *          scheduler igual corre para poder diagnosticar por consola.
 ******************************************************************************
 */

#include "fc.h"
#include "fc_rc.h"
#include "fc_state.h"
#include "fc_tasks.h"

#include "arming.h"
#include "failsafe.h"
#include "scheduler.h"

#include "console.h"
#include "driver_baro.h"
#include "driver_imu.h"
#include "driver_mag.h"
#include "driver_motors.h"
#include "sensors_scale.h"
#include "telemtry.h"
#include "timebase.h"

/* Protocolo de salida a los ESC. DShot300 es el mas tolerante al cableado. */
#define FC_MOTOR_PROTOCOL       MOTORS_PROTO_DSHOT300

/* Muestras para capturar la presion de referencia del baro. */
#define FC_BARO_REF_SAMPLES     50u

static void fc_init_phase1(void)
{
    (void)timebase_init();
    fc_state_init();
    /* arming_init() antes que los sensores: las fases 2 y 3 setean flags de
     * bloqueo (gyro/motores ausentes) y no deben perderse.                 */
    arming_init();
    console_init();
    console_banner("FC Dron - stack de vuelo (RC CRSF -> 4 motores)");
    console_printf("Gyro %u Hz, PID %u Hz, PID %s\r\n",
                   (unsigned)FC_GYRO_RATE_HZ,
                   (unsigned)FC_PID_RATE_HZ,
                   FC_ENABLE_PID ? "ON" : "OFF (passthrough)");
}

static void fc_init_phase2_sensors(void)
{
    fc_state_t *st = fc_state();

    /* --- IMU: obligatoria para volar --- */
    uint8_t who = 0u;
    if ((imu_check_who_am_i(&who) == IMU_OK) && (imu_init() == IMU_OK)) {
        st->has_gyro = true;
        console_result(true, "IMU LSM6DSV16X");
    } else {
        st->has_gyro = false;
        arming_disable_set(ARMING_DISABLED_NO_GYRO);
        console_printf("IMU WHO=0x%02X\r\n", (unsigned)who);
        console_result(false, "IMU - no se puede armar");
    }

    /* --- Baro: opcional (altitud y vario para telemetria) --- */
    uint8_t chip = 0u;
    if ((baro_check_chip_id(&chip) == BARO_OK) && (baro_init() == BARO_OK) &&
        (sensors_baro_calib_load() == SENSORS_SCALE_OK) &&
        (sensors_baro_set_reference(FC_BARO_REF_SAMPLES) == SENSORS_SCALE_OK)) {
        st->has_baro = true;
        console_result(true, "BARO BMP388");
    } else {
        st->has_baro = false;
        console_result(false, "BARO (se sigue sin altitud)");
    }

    /* --- Mag: opcional (heading) --- */
    uint8_t id = 0u;
    if ((mag_check_product_id(&id) == MAG_OK) && (mag_init() == MAG_OK)) {
        st->has_mag = true;
        console_result(true, "MAG MMC5983MA");
    } else {
        st->has_mag = false;
        console_result(false, "MAG (se sigue sin heading)");
    }

    sensors_attitude_reset();
}

static void fc_init_phase3_actuators(void)
{
    /* Enlace RC + telemetria del ESC. */
    fc_rc_init();
    (void)telemtry_init();

    /* Salida a los 4 ESC. Sin motores no se arma. */
    if (motors_init_all(FC_MOTOR_PROTOCOL) == MOT_OK) {
        console_result(true, "Motores DShot300 (M1..M4)");
    } else {
        arming_disable_set(ARMING_DISABLED_NO_MOTORS);
        console_result(false, "Motores - no se puede armar");
    }
}

static void fc_init_phase4_control(void)
{
    failsafe_init();
    fc_tasks_init();

    uint8_t count = 0u;
    sched_task_t *tasks = fc_tasks_table(&count);
    scheduler_init(fc_tasks_realtime(), tasks, count);

    console_printf("Scheduler listo: %u tareas lentas. Armar: throttle abajo "
                   "+ AUX1 arriba (esperar %u ms de boot).\r\n",
                   (unsigned)count, (unsigned)ARMING_BOOT_GRACE_MS);
}

void fc_run(void)
{
    fc_init_phase1();
    fc_init_phase2_sensors();
    fc_init_phase3_actuators();
    fc_init_phase4_control();

    scheduler_run();   /* no retorna */
}
