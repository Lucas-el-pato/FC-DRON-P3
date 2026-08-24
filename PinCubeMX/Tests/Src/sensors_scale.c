/**
 ******************************************************************************
 * @file    sensors_scale.c
 * @brief   Escalado IMU/mag, compensacion BMP388 y filtro complementario.
 ******************************************************************************
 */

#include "sensors_scale.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

#define BARO_REG_CALIB_DATA  0x31u
#define BARO_CALIB_LEN       21u
#define DEG2RAD              0.01745329252f

/* ------------------------------------------------------------------------- */
/* Estado interno                                                             */
/* ------------------------------------------------------------------------- */
typedef struct {
    float par_t1, par_t2, par_t3;
    float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6;
    float par_p7, par_p8, par_p9, par_p10, par_p11;
    bool loaded;
} baro_calib_t;

static baro_calib_t s_baro_cal = { 0 };
static float s_p0_pa = 101325.0f;
static bool  s_p0_set = false;
static float s_alt_filt_m = 0.0f;
static float s_alt_prev_filt_m = 0.0f;
static bool  s_alt_filt_init = false;
static uint32_t s_alt_last_ms = 0u;

static float s_roll = 0.0f;
static float s_pitch = 0.0f;
static float s_yaw = 0.0f;
static bool  s_att_init = false;

/* ------------------------------------------------------------------------- */
/* Helpers                                                                    */
/* ------------------------------------------------------------------------- */
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

static float wrap_pi(float a)
{
    const float pi = 3.14159265f;
    while (a > pi) {
        a -= 2.0f * pi;
    }
    while (a < -pi) {
        a += 2.0f * pi;
    }
    return a;
}

static int16_t read_s16_le(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint16_t read_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

/* ------------------------------------------------------------------------- */
/* IMU                                                                        */
/* ------------------------------------------------------------------------- */
void sensors_imu_scale(const imu_sample_t *raw, sensors_imu_si_t *out)
{
    if (raw == NULL || out == NULL) {
        return;
    }
    out->ax_g = (float)raw->ax * (SENSORS_IMU_ACC_MG_PER_LSB * 0.001f);
    out->ay_g = (float)raw->ay * (SENSORS_IMU_ACC_MG_PER_LSB * 0.001f);
    out->az_g = (float)raw->az * (SENSORS_IMU_ACC_MG_PER_LSB * 0.001f);

    out->gx_dps = (float)raw->gx * (SENSORS_IMU_GYRO_MDPS_PER_LSB * 0.001f);
    out->gy_dps = (float)raw->gy * (SENSORS_IMU_GYRO_MDPS_PER_LSB * 0.001f);
    out->gz_dps = (float)raw->gz * (SENSORS_IMU_GYRO_MDPS_PER_LSB * 0.001f);

    out->gx_rad_s = out->gx_dps * DEG2RAD;
    out->gy_rad_s = out->gy_dps * DEG2RAD;
    out->gz_rad_s = out->gz_dps * DEG2RAD;
}

/* ------------------------------------------------------------------------- */
/* Magnetometro                                                               */
/* ------------------------------------------------------------------------- */
void sensors_mag_scale(const mag_sample_t *raw, sensors_mag_si_t *out)
{
    if (raw == NULL || out == NULL) {
        return;
    }
    /* counts / 16384 * 100 uT/Gauss */
    const float k = 100.0f / SENSORS_MAG_COUNTS_PER_GAUSS;
    out->mx_uT = (float)raw->x * k;
    out->my_uT = (float)raw->y * k;
    out->mz_uT = (float)raw->z * k;
}

/* ------------------------------------------------------------------------- */
/* Barometro BMP388                                                           */
/* ------------------------------------------------------------------------- */
sensors_scale_status_t sensors_baro_calib_load(void)
{
    uint8_t raw[BARO_CALIB_LEN];
    if (baro_read_burst(BARO_REG_CALIB_DATA, raw, BARO_CALIB_LEN) != BARO_OK) {
        s_baro_cal.loaded = false;
        return SENSORS_SCALE_ERR_BARO;
    }

    /* Layout datasheet BMP388 Rev 1.5, tabla de NVM_PAR. */
    const uint16_t nvm_t1 = read_u16_le(&raw[0]);
    const uint16_t nvm_t2 = read_u16_le(&raw[2]);
    const int8_t   nvm_t3 = (int8_t)raw[4];
    const int16_t  nvm_p1 = read_s16_le(&raw[5]);
    const int16_t  nvm_p2 = read_s16_le(&raw[7]);
    const int8_t   nvm_p3 = (int8_t)raw[9];
    const int8_t   nvm_p4 = (int8_t)raw[10];
    const uint16_t nvm_p5 = read_u16_le(&raw[11]);
    const uint16_t nvm_p6 = read_u16_le(&raw[13]);
    const int8_t   nvm_p7 = (int8_t)raw[15];
    const int8_t   nvm_p8 = (int8_t)raw[16];
    const int16_t  nvm_p9 = read_s16_le(&raw[17]);
    const int8_t   nvm_p10 = (int8_t)raw[19];
    const int8_t   nvm_p11 = (int8_t)raw[20];

    /* Conversion a coeficientes float (API Bosch BMP3). */
    s_baro_cal.par_t1 = ((float)nvm_t1) / 0.00390625f;           /* / 2^-8 */
    s_baro_cal.par_t2 = ((float)nvm_t2) / 1073741824.0f;         /* / 2^30 */
    s_baro_cal.par_t3 = ((float)nvm_t3) / 281474976710656.0f;    /* / 2^48 */

    s_baro_cal.par_p1 = (((float)nvm_p1) - 16384.0f) / 1048576.0f;
    s_baro_cal.par_p2 = (((float)nvm_p2) - 16384.0f) / 536870912.0f;
    s_baro_cal.par_p3 = ((float)nvm_p3) / 4294967296.0f;
    s_baro_cal.par_p4 = ((float)nvm_p4) / 137438953472.0f;
    s_baro_cal.par_p5 = ((float)nvm_p5) / 0.125f;
    s_baro_cal.par_p6 = ((float)nvm_p6) / 64.0f;
    s_baro_cal.par_p7 = ((float)nvm_p7) / 256.0f;
    s_baro_cal.par_p8 = ((float)nvm_p8) / 32768.0f;
    s_baro_cal.par_p9 = ((float)nvm_p9) / 281474976710656.0f;
    s_baro_cal.par_p10 = ((float)nvm_p10) / 281474976710656.0f;
    s_baro_cal.par_p11 = ((float)nvm_p11) / 36893488147419103232.0f;

    s_baro_cal.loaded = true;
    s_p0_set = false;
    s_alt_filt_init = false;
    return SENSORS_SCALE_OK;
}

static void baro_compensate(uint32_t press_raw, uint32_t temp_raw,
                            float *temp_c, float *press_pa)
{
    /* Temperatura linealizada. */
    /* API float Bosch BMP3: t_lin ya queda en grados C. */
    float partial_data1 = ((float)temp_raw) - s_baro_cal.par_t1;
    float partial_data2 = partial_data1 * s_baro_cal.par_t2;
    float t_lin = partial_data2 + (partial_data1 * partial_data1) * s_baro_cal.par_t3;
    *temp_c = t_lin;

    /* Presion compensada (Pa). */
    float uncomp = (float)press_raw;
    float pd1 = s_baro_cal.par_p6 * t_lin;
    float pd2 = s_baro_cal.par_p7 * (t_lin * t_lin);
    float pd3 = s_baro_cal.par_p8 * (t_lin * t_lin * t_lin);
    float out1 = s_baro_cal.par_p5 + pd1 + pd2 + pd3;

    pd1 = s_baro_cal.par_p2 * t_lin;
    pd2 = s_baro_cal.par_p3 * (t_lin * t_lin);
    pd3 = s_baro_cal.par_p4 * (t_lin * t_lin * t_lin);
    float out2 = uncomp * (s_baro_cal.par_p1 + pd1 + pd2 + pd3);

    pd1 = uncomp * uncomp;
    pd2 = s_baro_cal.par_p9 + s_baro_cal.par_p10 * t_lin;
    pd3 = pd1 * pd2;
    float pd4 = pd3 + (uncomp * uncomp * uncomp) * s_baro_cal.par_p11;
    *press_pa = out1 + out2 + pd4;
}

static float press_to_alt_m(float press_pa)
{
    if (!s_p0_set || s_p0_pa < 1.0f || press_pa < 1.0f) {
        return 0.0f;
    }
    return 44330.0f * (1.0f - powf(press_pa / s_p0_pa, 0.1902949f));
}

sensors_scale_status_t sensors_baro_set_reference(uint8_t n_samples)
{
    if (!s_baro_cal.loaded) {
        return SENSORS_SCALE_ERR_NOT_READY;
    }
    if (n_samples == 0u) {
        n_samples = 1u;
    }

    float sum = 0.0f;
    uint8_t ok = 0u;
    for (uint8_t i = 0u; i < n_samples; ++i) {
        baro_sample_t raw = { 0 };
        if (baro_read_sample(&raw) != BARO_OK) {
            continue;
        }
        float t_c = 0.0f;
        float p_pa = 0.0f;
        baro_compensate(raw.press_raw, raw.temp_raw, &t_c, &p_pa);
        if (p_pa > 1000.0f && p_pa < 120000.0f) {
            sum += p_pa;
            ok++;
        }
        HAL_Delay(25u); /* ODR baro ~50 Hz */
    }
    if (ok == 0u) {
        return SENSORS_SCALE_ERR_BARO;
    }
    s_p0_pa = sum / (float)ok;
    s_p0_set = true;
    s_alt_filt_init = false;
    return SENSORS_SCALE_OK;
}

sensors_scale_status_t sensors_baro_update(const baro_sample_t *raw,
                                           sensors_baro_si_t *out)
{
    if (raw == NULL || out == NULL) {
        return SENSORS_SCALE_ERR_PARAM;
    }
    if (!s_baro_cal.loaded) {
        out->valid = false;
        return SENSORS_SCALE_ERR_NOT_READY;
    }

    float t_c = 0.0f;
    float p_pa = 0.0f;
    baro_compensate(raw->press_raw, raw->temp_raw, &t_c, &p_pa);

    out->temp_c = t_c;
    out->press_pa = p_pa;
    out->alt_m = press_to_alt_m(p_pa);

    /* Vario: derivada de la altitud filtrada (pasabajos alfa=0.2). */
    const uint32_t now = HAL_GetTick();
    if (!s_alt_filt_init) {
        s_alt_filt_m = out->alt_m;
        s_alt_prev_filt_m = out->alt_m;
        s_alt_filt_init = true;
        s_alt_last_ms = now;
        out->vspeed_cm_s = 0;
    } else {
        s_alt_filt_m = (SENSORS_ALT_LP_ALPHA * out->alt_m) +
                       ((1.0f - SENSORS_ALT_LP_ALPHA) * s_alt_filt_m);
        float dt = (float)(now - s_alt_last_ms) * 0.001f;
        if (dt < 0.001f) {
            dt = 0.001f;
        }
        float vs_m_s = (s_alt_filt_m - s_alt_prev_filt_m) / dt;
        s_alt_prev_filt_m = s_alt_filt_m;
        s_alt_last_ms = now;

        float vs_cm = clampf(vs_m_s * 100.0f, -32768.0f, 32767.0f);
        out->vspeed_cm_s = (int16_t)lroundf(vs_cm);
    }

    /* Usar altitud filtrada para el reporte. */
    out->alt_m = s_alt_filt_m;
    out->alt_dm = (int32_t)lroundf(s_alt_filt_m * 10.0f);
    out->valid = true;
    return SENSORS_SCALE_OK;
}

/* ------------------------------------------------------------------------- */
/* Actitud: filtro complementario + heading mag                               */
/* ------------------------------------------------------------------------- */
void sensors_attitude_reset(void)
{
    s_roll = 0.0f;
    s_pitch = 0.0f;
    s_yaw = 0.0f;
    s_att_init = false;
}

void sensors_attitude_update(const sensors_imu_si_t *imu,
                             const sensors_mag_si_t *mag,
                             float dt_s,
                             sensors_attitude_t *out)
{
    if (imu == NULL || out == NULL) {
        return;
    }
    if (dt_s < 0.0001f) {
        dt_s = 0.0001f;
    }
    if (dt_s > 0.1f) {
        dt_s = 0.1f;
    }

    /* Actitud del acelerometro. */
    float roll_acc = atan2f(imu->ay_g, imu->az_g);
    float pitch_acc = atan2f(-imu->ax_g,
                             sqrtf(imu->ay_g * imu->ay_g + imu->az_g * imu->az_g));

    if (!s_att_init) {
        s_roll = roll_acc;
        s_pitch = pitch_acc;
        s_att_init = true;
    } else {
        /* Complementario: 98% giro + 2% accel. */
        const float a = SENSORS_COMP_ALPHA;
        s_roll = (1.0f - a) * (s_roll + imu->gx_rad_s * dt_s) + a * roll_acc;
        s_pitch = (1.0f - a) * (s_pitch + imu->gy_rad_s * dt_s) + a * pitch_acc;
    }

    /* Yaw desde magnetometro (tilt-compensated) si hay muestra. */
    if (mag != NULL) {
        const float cr = cosf(s_roll);
        const float sr = sinf(s_roll);
        const float cp = cosf(s_pitch);
        const float sp = sinf(s_pitch);

        float mx = mag->mx_uT;
        float my = mag->my_uT;
        float mz = mag->mz_uT;

        float xh = mx * cp + my * sr * sp + mz * cr * sp;
        float yh = my * cr - mz * sr;
        s_yaw = atan2f(-yh, xh); /* -yh para NED/heading convencional */
        s_yaw = wrap_pi(s_yaw);
    } else {
        /* Integrar yaw con gz si no hay mag. */
        s_yaw = wrap_pi(s_yaw + imu->gz_rad_s * dt_s);
    }

    s_roll = wrap_pi(s_roll);
    s_pitch = wrap_pi(s_pitch);

    out->roll_rad = s_roll;
    out->pitch_rad = s_pitch;
    out->yaw_rad = s_yaw;
}
