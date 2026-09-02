/**
 ******************************************************************************
 * @file    test_crsf_telem.c
 * @brief   Telemetria CRSF por UART4 hacia SuperD ELRS.
 *
 *          Inicializa IMU + baro + mag + CRSF (DMA RX).
 *          Loop cooperativo (sin HAL_Delay que bloquee el RX):
 *            - crsf_poll_frame() cada iteracion (canales RC)
 *            - IMU ~100 Hz -> filtro complementario
 *            - Attitude 0x1E @ 20 Hz
 *            - Mag @ 10 Hz (heading)
 *            - Baro 0x09 + Vario 0x07 @ 5 Hz
 *            - console_printf @ 2 Hz (debug USB CDC)
 *
 *          Verificacion EdgeTX:
 *            Telem Ratio 1:8 o 1:4, Discover new sensors ->
 *            Ptch, Roll, Yaw, Alt, VSpd.
 ******************************************************************************
 */

#include "test_crsf_telem.h"
#include "console.h"
#include "driver_crsf.h"
#include "driver_imu.h"
#include "driver_baro.h"
#include "driver_mag.h"
#include "sensors_scale.h"

#include <math.h>

#define PERIOD_IMU_MS       10u   /* 100 Hz */
#define PERIOD_ATT_MS       50u   /* 20 Hz  */
#define PERIOD_MAG_MS       100u  /* 10 Hz  */
#define PERIOD_BARO_MS      200u  /* 5 Hz   */
#define PERIOD_LOG_MS       500u  /* 2 Hz   */
#define MAG_TIMEOUT_MS      50u
#define RAD2DEG             57.29577951f

static bool crsf_telem_init(void)
{
    console_banner("CRSF Telem IMU+Baro+Mag -> SuperD");

    uint8_t who = 0u;
    if (imu_check_who_am_i(&who) != IMU_OK || imu_init() != IMU_OK) {
        console_printf("IMU WHO=0x%02X fallo\r\n", (unsigned)who);
        console_result(false, "IMU init");
        return false;
    }
    console_result(true, "IMU OK");

    uint8_t chip = 0u;
    if (baro_check_chip_id(&chip) != BARO_OK || baro_init() != BARO_OK) {
        console_printf("BARO CHIP=0x%02X fallo\r\n", (unsigned)chip);
        console_result(false, "BARO init");
        return false;
    }
    if (sensors_baro_calib_load() != SENSORS_SCALE_OK) {
        console_result(false, "BARO calib NVM");
        return false;
    }
    console_print("Capturando p0 baro (50 muestras)...\r\n");
    if (sensors_baro_set_reference(50u) != SENSORS_SCALE_OK) {
        console_result(false, "BARO p0");
        return false;
    }
    console_result(true, "BARO OK");

    uint8_t id = 0u;
    if (mag_check_product_id(&id) != MAG_OK || mag_init() != MAG_OK) {
        console_printf("MAG ID=0x%02X fallo\r\n", (unsigned)id);
        console_result(false, "MAG init");
        return false;
    }
    console_result(true, "MAG OK");

    crsf_init();
    sensors_attitude_reset();
    console_result(true, "CRSF DMA RX + TX listo");
    console_print("Enviando Attitude@20Hz Baro@5Hz. Telem Ratio ELRS: 1:8/1:4\r\n");
    return true;
}

void test_crsf_telem_run(void)
{
    if (!crsf_telem_init()) {
        while (1) {
            console_led_fail();
            HAL_Delay(500u);
        }
    }

    console_led_pass();

    uint32_t t_imu = 0u;
    uint32_t t_att = 0u;
    uint32_t t_mag = 0u;
    uint32_t t_baro = 0u;
    uint32_t t_log = 0u;
    uint32_t last_imu_ms = HAL_GetTick();

    sensors_attitude_t att = { 0 };
    sensors_baro_si_t baro_si = { 0 };
    sensors_mag_si_t mag_si = { 0 };
    bool mag_valid = false;
    crsf_channels_t last_ch = { 0 };
    bool have_rc = false;
    uint32_t att_tx_ok = 0u;
    uint32_t baro_tx_ok = 0u;

    while (1) {
        const uint32_t now = HAL_GetTick();

        /* 1) Consumir RC del ring DMA. */
        crsf_frame_t frame;
        crsf_status_t st = crsf_poll_frame(&frame);
        if (st == CRSF_OK && frame.type == CRSF_TYPE_RC_CHANNELS) {
            if (crsf_decode_channels(&frame, &last_ch) == CRSF_OK) {
                have_rc = true;
            }
        }

        /* 2) IMU -> fusion. */
        if ((now - t_imu) >= PERIOD_IMU_MS) {
            t_imu = now;
            imu_sample_t imu_raw = { 0 };
            if (imu_read_sample(&imu_raw) == IMU_OK) {
                sensors_imu_si_t imu_si;
                sensors_imu_scale(&imu_raw, &imu_si);

                float dt = (float)(now - last_imu_ms) * 0.001f;
                last_imu_ms = now;
                sensors_attitude_update(&imu_si,
                                        mag_valid ? &mag_si : NULL,
                                        dt,
                                        &att);
                /* Usar mag solo una vez por muestra. */
                mag_valid = false;
            }
        }

        /* 3) Magnetometro @ 10 Hz. */
        if ((now - t_mag) >= PERIOD_MAG_MS) {
            t_mag = now;
            mag_sample_t mag_raw = { 0 };
            if (mag_read_sample(&mag_raw, MAG_TIMEOUT_MS) == MAG_OK) {
                sensors_mag_scale(&mag_raw, &mag_si);
                mag_valid = true;
            }
        }

        /* 4) Attitude CRSF @ 20 Hz. */
        if ((now - t_att) >= PERIOD_ATT_MS) {
            t_att = now;
            if (crsf_send_attitude(att.pitch_rad, att.roll_rad, att.yaw_rad) == CRSF_OK) {
                att_tx_ok++;
            }
        }

        /* 5) Baro + vario @ 5 Hz. */
        if ((now - t_baro) >= PERIOD_BARO_MS) {
            t_baro = now;
            baro_sample_t baro_raw = { 0 };
            if (baro_read_sample(&baro_raw) == BARO_OK &&
                sensors_baro_update(&baro_raw, &baro_si) == SENSORS_SCALE_OK) {
                if (crsf_send_baro_altitude(baro_si.alt_dm) == CRSF_OK) {
                    baro_tx_ok++;
                }
                (void)crsf_send_vario(baro_si.vspeed_cm_s);
            }
        }

        /* 6) Log USB @ 2 Hz. */
        if ((now - t_log) >= PERIOD_LOG_MS) {
            t_log = now;
            console_printf(
                "ATT deg R=%.1f P=%.1f Y=%.1f | ALT=%.2fm VSpd=%d cm/s "
                "P=%.1fhPa | TX att=%lu baro=%lu RX ok=%lu crc=%lu | RC=%s\r\n",
                (double)(att.roll_rad * RAD2DEG),
                (double)(att.pitch_rad * RAD2DEG),
                (double)(att.yaw_rad * RAD2DEG),
                (double)baro_si.alt_m,
                (int)baro_si.vspeed_cm_s,
                (double)(baro_si.press_pa * 0.01f),
                (unsigned long)att_tx_ok,
                (unsigned long)baro_tx_ok,
                (unsigned long)g_crsfValidCount,
                (unsigned long)g_crsfCrcErrCount,
                have_rc ? "yes" : "no");

            if (have_rc) {
                console_led_pass();
            } else {
                /* Sin RC aun: LED fail suave, pero seguimos mandando telem. */
                console_led_fail();
            }
        }
    }
}
