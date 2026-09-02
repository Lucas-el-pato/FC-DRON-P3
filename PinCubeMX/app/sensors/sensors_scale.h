/**
 ******************************************************************************
 * @file    sensors_scale.h
 * @brief   Escalado de sensores crudos a unidades fisicas + fusion minima
 *          de actitud (filtro complementario) para telemetria CRSF.
 *
 *          No toca los drivers de bus (IMU/baro/mag siguen devolviendo raw).
 *          Este modulo consume esas lecturas y produce:
 *            - aceleracion en g, giro en dps / rad/s
 *            - campo magnetico en uT y heading
 *            - presion compensada en Pa, temperatura en C
 *            - altitud relativa en m / dm y velocidad vertical en cm/s
 *            - roll / pitch / yaw en radianes (-pi..+pi)
 *
 *          Escalas (config actual de imu_init):
 *            XL +/-4 g  -> 0.122 mg/LSB
 *            G  +/-500 dps -> 17.50 mdps/LSB
 *
 *          Magnetometro MMC5983MA 18-bit:
 *            sensibilidad 16384 counts/Gauss; 1 Gauss = 100 uT
 *            (el driver ya resto el offset de centro 2^17)
 *
 *          Barometro BMP388:
 *            coeficientes NVM 0x31..0x45 + compensacion float del datasheet.
 *            Altitud relativa contra p0 capturada al arrancar.
 *
 *          Fusion (sin EKF):
 *            roll/pitch = complemento accel (2%) + giro (98%)
 *            yaw       = heading mag compensado por inclinacion
 ******************************************************************************
 */

#ifndef TESTS_INC_SENSORS_SCALE_H_
#define TESTS_INC_SENSORS_SCALE_H_

#include "driver_imu.h"
#include "driver_baro.h"
#include "driver_mag.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Escalas LSM6DSV16X (datasheet Rev 4, tablas de sensibilidad). */
#define SENSORS_IMU_ACC_MG_PER_LSB   0.122f   /* +/-4 g  */
#define SENSORS_IMU_GYRO_MDPS_PER_LSB 17.50f  /* +/-500 dps */

/* MMC5983MA: 18-bit, 16384 counts/Gauss. */
#define SENSORS_MAG_COUNTS_PER_GAUSS 16384.0f

/* Filtro complementario: peso del acelerometro. */
#define SENSORS_COMP_ALPHA  0.02f

/* Pasabajos de altitud para derivar vario. */
#define SENSORS_ALT_LP_ALPHA 0.2f

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float gx_rad_s;
    float gy_rad_s;
    float gz_rad_s;
} sensors_imu_si_t;

typedef struct {
    float mx_uT;
    float my_uT;
    float mz_uT;
} sensors_mag_si_t;

typedef struct {
    float press_pa;
    float temp_c;
    float alt_m;          /* relativa a p0 */
    int32_t alt_dm;       /* decimetros relativos */
    int16_t vspeed_cm_s;  /* positivo = subiendo */
    bool valid;
} sensors_baro_si_t;

typedef struct {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} sensors_attitude_t;

typedef enum {
    SENSORS_SCALE_OK = 0,
    SENSORS_SCALE_ERR_PARAM,
    SENSORS_SCALE_ERR_BARO,
    SENSORS_SCALE_ERR_NOT_READY
} sensors_scale_status_t;

/* ------------------------------------------------------------------------- */
/* Escala IMU raw -> g / dps / rad/s.                                         */
/* ------------------------------------------------------------------------- */
void sensors_imu_scale(const imu_sample_t *raw, sensors_imu_si_t *out);

/* ------------------------------------------------------------------------- */
/* Escala mag raw (ya con offset restado) -> uT.                              */
/* ------------------------------------------------------------------------- */
void sensors_mag_scale(const mag_sample_t *raw, sensors_mag_si_t *out);

/* ------------------------------------------------------------------------- */
/* Baro: lee NVM, convierte a coeficientes float. Llamar una vez post-init.   */
/* ------------------------------------------------------------------------- */
sensors_scale_status_t sensors_baro_calib_load(void);

/* ------------------------------------------------------------------------- */
/* Compensa una muestra raw; actualiza altitud relativa y vario filtrado.     */
/* Requiere sensors_baro_calib_load() + sensors_baro_set_reference() previos. */
/* ------------------------------------------------------------------------- */
sensors_scale_status_t sensors_baro_update(const baro_sample_t *raw,
                                           sensors_baro_si_t *out);

/* ------------------------------------------------------------------------- */
/* Captura p0 como promedio de n muestras compensadas (bloqueante).           */
/* ------------------------------------------------------------------------- */
sensors_scale_status_t sensors_baro_set_reference(uint8_t n_samples);

/* ------------------------------------------------------------------------- */
/* Fusion: alimentar con cada muestra IMU (y mag cuando haya).                */
/* dt_s = segundos desde la llamada anterior (tipicamente 0.01).              */
/* Si mag == NULL solo actualiza roll/pitch; yaw se conserva.                 */
/* ------------------------------------------------------------------------- */
void sensors_attitude_update(const sensors_imu_si_t *imu,
                             const sensors_mag_si_t *mag,
                             float dt_s,
                             sensors_attitude_t *out);

/* ------------------------------------------------------------------------- */
/* Resetea el estado interno del filtro (roll/pitch/yaw = 0).                 */
/* ------------------------------------------------------------------------- */
void sensors_attitude_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_SENSORS_SCALE_H_ */
