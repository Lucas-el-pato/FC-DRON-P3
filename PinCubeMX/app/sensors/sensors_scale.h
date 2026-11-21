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
 *            roll/pitch = gyro + correccion del accel (tau 0.49 s)
 *            yaw        = gyro + correccion hacia el rumbo del mag compensado
 *                         por inclinacion (tau SENSORS_MAG_TAU_S); sin mag,
 *                         solo gyro (deriva)
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

/* Calibracion de hierro duro / blando (cuentas crudas del driver). Se
 * obtienen con TEST_SELECT_MAG (tests/test_mag.c): girar el dron en todas
 * las direcciones y copiar los valores sugeridos.
 *   offset = (max + min) / 2       escala = rango_medio / rango_del_eje
 * Valores actuales: TEST_SELECT_MAG por USB (2026-11-20), dron completo con
 * la bateria montada. Campo medido ~2800 cuentas (~17 uT).                 */
#ifndef SENSORS_MAG_OFFSET_X
#define SENSORS_MAG_OFFSET_X    (-440)
#endif
#ifndef SENSORS_MAG_OFFSET_Y
#define SENSORS_MAG_OFFSET_Y    2403
#endif
#ifndef SENSORS_MAG_OFFSET_Z
#define SENSORS_MAG_OFFSET_Z    (-262)
#endif
#ifndef SENSORS_MAG_SCALE_X
#define SENSORS_MAG_SCALE_X     1.017f
#endif
#ifndef SENSORS_MAG_SCALE_Y
#define SENSORS_MAG_SCALE_Y     0.939f
#endif
#ifndef SENSORS_MAG_SCALE_Z
#define SENSORS_MAG_SCALE_Z     1.049f
#endif

/* Orientacion del mag respecto de la IMU (se aplica despues de la
 * calibracion). Verificar en el banco apuntando la nariz al N/E/S/O: el
 * rumbo tiene que crecer en sentido horario visto desde arriba.          */
#ifndef SENSORS_MAG_SWAP_XY
#define SENSORS_MAG_SWAP_XY     0
#endif
#ifndef SENSORS_MAG_SIGN_X
#define SENSORS_MAG_SIGN_X      1.0f
#endif
#ifndef SENSORS_MAG_SIGN_Y
#define SENSORS_MAG_SIGN_Y      (-1.0f)   /* banco 2026-11-20: rumbo espejado */
#endif
#ifndef SENSORS_MAG_SIGN_Z
#define SENSORS_MAG_SIGN_Z      (-1.0f)   /* chip con z hacia arriba (el campo en
                                            * Asuncion apunta hacia arriba y medi +z) */
#endif

/* Declinacion magnetica (norte verdadero = magnetico + declinacion; oeste
 * negativo). Asuncion ~ -15 grados: confirmar con la calculadora WMM de
 * NOAA (ngdc.noaa.gov/geomag/calculators/magcalc.shtml).                  */
#ifndef SENSORS_MAG_DECLINATION_DEG
#define SENSORS_MAG_DECLINATION_DEG  (-15.0f)
#endif

/* Constante de tiempo con la que el yaw (integrado por gyro) se corrige
 * hacia el rumbo del mag: filtra el ruido del mag y la deriva del gyro.  */
#ifndef SENSORS_MAG_TAU_S
#define SENSORS_MAG_TAU_S       1.0f
#endif

/* Signo de la integracion del gyro en el yaw (1 o -1). Tiene que girar en
 * el mismo sentido que el rumbo del mag; verificar en el banco.          */
#ifndef SENSORS_YAW_GYRO_SIGN
#define SENSORS_YAW_GYRO_SIGN   (-1.0f)   /* gyro z hacia arriba: + = antihorario */
#endif

/* Filtro complementario: constante de tiempo de la correccion del
 * acelerometro. El peso por paso sale de dt (alpha = dt / (tau + dt)) para
 * que el filtro se comporte igual a 100 Hz o a 1 kHz. 0.49 s equivale al
 * alpha fijo de 0.02 que se usaba a 100 Hz.                               */
#define SENSORS_COMP_TAU_S  0.49f

/* El accel solo corrige la actitud si su modulo esta cerca de 1 g. */
#define SENSORS_ACC_TRUST_MIN_G  0.85f
#define SENSORS_ACC_TRUST_MAX_G  1.15f

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
/* Escala mag raw (centro 2^17 ya restado) -> uT: aplica los offsets y      */
/* escalas de calibracion y la orientacion respecto de la IMU.                */
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
/* El yaw siempre se integra con el gyro. Si mag != NULL se corrige hacia el */
/* rumbo del mag (pasar mag en cada llamada mientras la muestra este fresca). */
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
