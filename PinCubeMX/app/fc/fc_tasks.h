/**
 ******************************************************************************
 * @file    fc_tasks.h
 * @brief   Tabla de tareas del FC y etapas del lazo realtime (equivalente a
 *          fc/tasks.c de Betaflight).
 *
 *          Etapas realtime (disparadas por el DRDY del gyro, 8 kHz):
 *            gyro   -> muestra del stream DMA (imu_stream_take) + escalado
 *            filter -> notch por motor + 2 PT1 sobre los 3 ejes (cada
 *                      FC_FILTER_DENOM muestras); cada FC_ATTITUDE_DENOM
 *                      muestras tambien la actitud (1 kHz)
 *            pid    -> RC -> arming -> PID -> mixer -> motors_write4
 *                      (cada FC_PID_DENOM muestras: 8 kHz / 4 = 2 kHz)
 *
 *          Cola lenta (una por pasada, elegida por el scheduler):
 *            RX CRSF, baro, telemetria ESC, telemetria CRSF, log.
 *
 *          Ninguna tarea lenta bloquea: el GPS (USART3) y el mag (I2C1)
 *          trabajan por interrupcion.
 ******************************************************************************
 */

#ifndef FC_FC_TASKS_H_
#define FC_FC_TASKS_H_

#include "scheduler.h"
#include "driver_imu.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------- */
/* Configuracion del lazo                                                     */
/* ------------------------------------------------------------------------- */

/* ODR del gyro (driver_imu lo configura en 8 kHz). */
#define FC_GYRO_RATE_HZ     IMU_ODR_HZ

/* Divisores del lazo: filtro a 8 kHz, PID/motores a 2 kHz. */
#define FC_FILTER_DENOM     1u
#define FC_PID_DENOM        4u
#define FC_PID_RATE_HZ      (FC_GYRO_RATE_HZ / FC_PID_DENOM)

/* Cutoff del pasabajos del gyro que entra al PID. */
#define FC_GYRO_LPF_HZ      150.0f

/* Segundo PT1 en cascada (gyro_lpf2 de BF): atenua mas la vibracion de los
 * motores (no hay filtro RPM ni notch dinamico).                           */
#ifndef FC_GYRO_LPF2_HZ
#define FC_GYRO_LPF2_HZ     300.0f
#endif

/* Filtro RPM (app/control/rpm_filter.c): un notch por motor y armonico,
 * centrado en la eRPM medida por DShot bidireccional. Necesita
 * FC_MOTOR_BIDIR = 1; si el bidireccional no arranco, se usa el notch
 * estimado de abajo. Ajustes (Q, minimo, armonicos) en rpm_filter.h.    */
#ifndef FC_GYRO_RPM_FILTER
#define FC_GYRO_RPM_FILTER          1
#endif
/* Notch dinamico (app/control/dyn_notch.c): un notch por eje sobre el pico
 * que queda despues del filtro RPM (resonancias del frame / banco). La
 * SDFT se actualiza en la muestra de gyro siguiente al PID (fase 1 de
 * FC_PID_DENOM), una muestra liviana. Ajustes en dyn_notch.h.           */
#ifndef FC_GYRO_DYN_NOTCH
#define FC_GYRO_DYN_NOTCH           1
#endif

/* Un motor sin respuesta valida durante esta cantidad de frames (2 kHz)
 * deja de filtrarse (su notch se apaga) hasta que vuelva a contestar.    */
#define FC_RPM_STALE_FRAMES         100u    /* 50 ms */

/* Notch estimado: solo si el filtro RPM no esta activo.
 * Notch que sigue a los motores (antes de los PT1, a 8 kHz). Como el D se
 * calcula sobre el gyro filtrado, el notch limpia tambien el D-term.
 *
 * La frecuencia sale del DShot medio de los 4 motores (no hay RPM por
 * telemetria rapida). Ajuste de los blackbox BB001-BB003 (banco, 2026-11):
 * pico de pitch/yaw = 0.222 * dshot_medio + 35 Hz (error +/-8.5 Hz):
 *   DShot 420 -> ~128 Hz,  520 -> ~150 Hz,  670 -> ~184 Hz,  880 -> ~230 Hz
 * Si se cambian helices, motores o bateria, rehacer el ajuste con un log.
 * El 2.o armonico (2 x f, visible en roll) lleva un segundo notch.       */
#ifndef FC_GYRO_NOTCH_ENABLE
#define FC_GYRO_NOTCH_ENABLE        1
#endif
#ifndef FC_GYRO_NOTCH_HZ_PER_DSHOT
#define FC_GYRO_NOTCH_HZ_PER_DSHOT  0.222f
#endif
#ifndef FC_GYRO_NOTCH_OFFSET_HZ
#define FC_GYRO_NOTCH_OFFSET_HZ     35.0f
#endif
/* Rango del centro: debajo del minimo el notch se queda en el minimo (con
 * el dron apoyado el ruido no importa y un notch muy bajo agrega retardo). */
#ifndef FC_GYRO_NOTCH_MIN_HZ
#define FC_GYRO_NOTCH_MIN_HZ        100.0f
#endif
#ifndef FC_GYRO_NOTCH_MAX_HZ
#define FC_GYRO_NOTCH_MAX_HZ        350.0f
#endif
/* Q = centro / ancho de banda. 3 cubre los +/-20 Hz de diferencia entre
 * motores; retardo extra ~0.4 ms a frecuencias de control.              */
#ifndef FC_GYRO_NOTCH_Q
#define FC_GYRO_NOTCH_Q             3.0f
#endif
/* 1 = solo fundamental, 2 = fundamental + 2.o armonico. */
#ifndef FC_GYRO_NOTCH_HARMONICS
#define FC_GYRO_NOTCH_HARMONICS     2u
#endif
/* Suavizado del DShot medio antes de calcular el centro (las correcciones
 * del PID mueven cada motor rapido; el giro real sigue mas lento).       */
#ifndef FC_GYRO_NOTCH_TRACK_HZ
#define FC_GYRO_NOTCH_TRACK_HZ      20.0f
#endif

/* Actitud (filtro complementario) y lazo de nivel: corren en la cadena
 * realtime a FC_GYRO_RATE_HZ / FC_ATTITUDE_DENOM (1 kHz), en una muestra
 * de gyro donde no corre el PID para repartir la carga. Antes era una
 * tarea lenta a 100 Hz y el setpoint de ANGLE cambiaba en escalones de
 * 10 ms.                                                                */
#define FC_ATTITUDE_DENOM   8u
#define FC_ATTITUDE_PHASE   2u
#define FC_ATTITUDE_RATE_HZ (FC_GYRO_RATE_HZ / FC_ATTITUDE_DENOM)

/* 1 = modo ANGLE (stick -> angulo -> PID -> mixer). 0 = passthrough: solo
 * throttle a los 4 motores; los sticks de roll/pitch/yaw no hacen nada.     */
#ifndef FC_ENABLE_PID
#define FC_ENABLE_PID       1
#endif

/* Mag MMC5983MA por I2C1 con interrupcion (driver_mag, modo async): no
 * bloquea. La tarea MAG avanza la maquina de estados cada 20 ms y la
 * actitud corrige el yaw hacia el rumbo del mag. Calibrar con
 * TEST_SELECT_MAG (offsets en sensors_scale.h).                           */
#ifndef FC_ENABLE_MAG
#define FC_ENABLE_MAG       1
#endif
#define FC_MAG_PERIOD_US    20000u
/* Muestra del mag mas vieja que esto -> la actitud sigue solo con gyro. */
#define FC_MAG_FRESH_MS     100u
/* Sin muestra en este tiempo -> fc_mag_ok() = false (telemetria / GCS). */
#define FC_MAG_OK_MS        500u
/* GPS por USART3 con RX por interrupcion: no bloquea. Agrega la tarea GPS
 * (parseo NMEA) y TELEM_GPS (frame CRSF 0x02).                             */
#ifndef FC_ENABLE_GPS
#define FC_ENABLE_GPS       1
#endif
#define FC_GPS_PERIOD_US    20000u   /* ~230 B por pasada, ring de 512 */

/* 1 = el log USB sigue imprimiendo con el dron armado. SOLO en banco, sin
 * helices y con el host leyendo el CDC (si no, console_print puede bloquear
 * el lazo hasta 160 ms).                                                    */
#ifndef FC_LOG_WHEN_ARMED
#define FC_LOG_WHEN_ARMED   0
#endif

/* 1 = DShot300 bidireccional (driver_dshot_bb.c: TIM1 + DMA2 sobre GPIO):
 * los ESC (AM32) contestan su eRPM en cada frame y va al blackbox.
 * 0 = DShot300 normal por TIM2 (el de siempre), sin RPM.
 * Probar SIN HELICES: los 4 motores tienen que responder igual que antes
 * y el blackbox mostrar eRPM con pocas respuestas malas.                 */
#ifndef FC_MOTOR_BIDIR
#define FC_MOTOR_BIDIR      1
#endif
/* Con DShot bidireccional las lineas de los ESC quedan en BAJO (sin
 * frames, sin poder armar) hasta ver la bateria arriba de
 * FC_ESC_POWER_MIN_CV durante FC_ESC_BOOT_MS. El bootloader de AM32 no
 * arranca el firmware si al encenderse ve la linea en alto (reposo del
 * DShot invertido); asi tambien anda si el FC se alimenta antes (USB o
 * ST-Link) y la bateria se conecta despues. Si la bateria se desconecta
 * con el dron desarmado, vuelve a retener.                                */
#define FC_ESC_POWER_MIN_CV 600u     /* 6 V: debajo, los ESC estan apagados */
#define FC_ESC_BOOT_MS      500u     /* el bootloader mira la linea ~40 ms */

/* Telemetria CRSF hacia la radio (attitude / altitud / vario). */
#ifndef FC_ENABLE_TELEM_TX
#define FC_ENABLE_TELEM_TX  1
#endif

/* ------------------------------------------------------------------------- */
/* Mapeo de ejes IMU -> ejes de vuelo.                                        */
/* Marco de Betaflight (el que asume el mixer):                              */
/*   roll+  = ala derecha abajo                                               */
/*   pitch+ = nariz abajo                                                     */
/*   yaw+   = antihorario visto desde arriba                                  */
/* VERIFICAR SIN HELICES con el log: gyro y angulo deben dar positivo en esos */
/* sentidos. Si un eje sale invertido, cambiar su signo aca (se aplica al     */
/* gyro y al angulo). Un signo mal = realimentacion positiva = motores locos. */
/* ------------------------------------------------------------------------- */
#define FC_GYRO_ROLL_SIGN    (-1.0f)   /* verificado en banco: roll invertido */
#define FC_GYRO_PITCH_SIGN   (-1.0f)   /* verificado en GCS: nariz abajo daba pitch < 0 */
#define FC_GYRO_YAW_SIGN     (+1.0f)

/* Prepara filtros, PID, mixer, arming, failsafe y la tabla de tareas.
 * has_gyro/has_baro/has_mag vienen del init de fc.c.                        */
void fc_tasks_init(void);

/* Config realtime + tabla, para pasarle a scheduler_init(). */
const sched_realtime_t *fc_tasks_realtime(void);
sched_task_t *fc_tasks_table(uint8_t *count);

/* Cierra la ventana de estadisticas del lazo (scheduler + INT1 del gyro) y
 * la guarda en fc_state()->loop. Un solo llamador: fc_telem_debug (o el
 * log si FC_ENABLE_TELEM_TX = 0).                                         */
void fc_tasks_loop_stats_take(void);

/* Etapas expuestas para tests de banco. */
void fc_task_gyro(void);
void fc_task_filter(void);

/* Centro actual del notch de motores en Hz (0 si esta deshabilitado). Con
 * el filtro RPM activo: fundamental promedio de los 4 motores.           */
float fc_gyro_notch_hz(void);

/* true si el mag esta habilitado y entrega muestras (ultima <= FC_MAG_OK_MS). */
bool fc_mag_ok(void);
void fc_task_pid(void);
void fc_task_rx(void);
void fc_task_failsafe(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_FC_TASKS_H_ */
