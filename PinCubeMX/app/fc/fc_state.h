/**
 ******************************************************************************
 * @file    fc_state.h
 * @brief   Estado de vuelo y contadores de diagnostico (equivalente a
 *          fc/runtime_config.c de Betaflight).
 *
 *          El flag de armado vive en app/control/arming.c (es quien lo maneja);
 *          aca queda el modo de vuelo y las estadisticas que consume el log de
 *          consola y la telemetria.
 ******************************************************************************
 */

#ifndef FC_FC_STATE_H_
#define FC_FC_STATE_H_

#include "sensors_scale.h"
#include "pid.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    FC_MODE_ACRO = 0,      /* rate puro (unico modo implementado) */
    FC_MODE_ANGLE          /* reservado: necesita accel + fusion  */
} fc_flight_mode_t;

typedef struct {
    /* Sensores del lazo. */
    imu_sample_t       gyro_raw;
    sensors_imu_si_t   imu_si;
    float              gyro_filt_dps[PID_AXIS_COUNT];
    sensors_mag_si_t   mag_si;
    sensors_baro_si_t  baro_si;
    sensors_attitude_t attitude;

    /* Salidas. */
    float    pid_out[PID_AXIS_COUNT];
    uint16_t motor[4];

    /* Contadores. */
    uint32_t gyro_reads;
    uint32_t gyro_errors;
    uint32_t motor_frames;
    uint32_t motor_drops;     /* frames descartados por DMA ocupado */
    uint32_t rx_frames;
    uint32_t baro_updates;
    uint32_t mag_updates;

    /* Sensores presentes al arrancar. */
    bool has_gyro;
    bool has_baro;
    bool has_mag;

    fc_flight_mode_t mode;
} fc_state_t;

/* Estado global unico (Live Expressions / log). */
extern fc_state_t g_fcState;

void fc_state_init(void);
fc_state_t *fc_state(void);

#ifdef __cplusplus
}
#endif

#endif /* FC_FC_STATE_H_ */
