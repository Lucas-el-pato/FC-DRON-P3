/**
 ******************************************************************************
 * @file    test_telemetry.c
 * @brief   Combined IMU + baro + mag telemetry for the web dashboard.
 *
 *          Emits one newline-delimited JSON object per sample, e.g.:
 *          {"t_ms":12345,"imu":{"gx":1,"gy":2,"gz":3,"ax":4,"ay":5,"az":6,
 *           "st":0},"baro":{"press_raw":123,"temp_raw":456,"st":0},
 *           "mag":{"x":7,"y":8,"z":9,"st":0}}
 ******************************************************************************
 */

#include "test_telemetry.h"
#include "console.h"
#include "driver_imu.h"
#include "driver_baro.h"
#include "driver_mag.h"

#include <stdio.h>

#define TELEMETRY_LOOP_DELAY_MS  50u
#define TELEMETRY_MAG_TIMEOUT_MS 50u

static bool telemetry_init_sensors(void)
{
    console_banner("Telemetry IMU+Baro+Mag");

    uint8_t who = 0u;
    imu_status_t imu_st = imu_check_who_am_i(&who);
    console_printf("IMU WHO_AM_I = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)who, (unsigned)IMU_WHO_AM_I_VALUE);
    if (imu_st != IMU_OK) {
        console_result(false, "IMU WHO_AM_I fallo");
        return false;
    }

    imu_st = imu_init();
    if (imu_st != IMU_OK) {
        console_printf("imu_init fallo (codigo=%d)\r\n", (int)imu_st);
        console_result(false, "fallo en imu_init");
        return false;
    }
    console_result(true, "IMU OK");

    uint8_t chip = 0u;
    baro_status_t baro_st = baro_check_chip_id(&chip);
    console_printf("BARO CHIP_ID = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)chip, (unsigned)BARO_CHIP_ID_VALUE);
    if (baro_st != BARO_OK) {
        console_result(false, "BARO CHIP_ID fallo");
        return false;
    }

    baro_st = baro_init();
    if (baro_st != BARO_OK) {
        console_printf("baro_init fallo (codigo=%d)\r\n", (int)baro_st);
        console_result(false, "fallo en baro_init");
        return false;
    }
    console_result(true, "BARO OK");

    uint8_t id = 0u;
    mag_status_t mag_st = mag_check_product_id(&id);
    console_printf("MAG PRODUCT_ID = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)id, (unsigned)MAG_PRODUCT_ID_VALUE);
    if (mag_st != MAG_OK) {
        console_result(false, "MAG PRODUCT_ID fallo");
        return false;
    }

    mag_st = mag_init();
    if (mag_st != MAG_OK) {
        console_printf("mag_init fallo (codigo=%d)\r\n", (int)mag_st);
        console_result(false, "fallo en mag_init");
        return false;
    }
    console_result(true, "MAG OK");

    console_print("\r\nTELEMETRY_JSON_START\r\n");
    return true;
}

void test_telemetry_run(void)
{
    if (!telemetry_init_sensors()) {
        while (1) {
            console_led_fail();
            HAL_Delay(500u);
        }
    }

    console_led_pass();

    while (1) {
        imu_sample_t imu = { 0 };
        baro_sample_t baro = { 0 };
        mag_sample_t mag = { 0 };

        imu_status_t imu_st = imu_read_sample(&imu);
        baro_status_t baro_st = baro_read_sample(&baro);
        mag_status_t mag_st = mag_read_sample(&mag, TELEMETRY_MAG_TIMEOUT_MS);

        char line[256];
        int n = snprintf(line, sizeof(line),
            "{\"t_ms\":%lu,"
            "\"imu\":{\"gx\":%d,\"gy\":%d,\"gz\":%d,\"ax\":%d,\"ay\":%d,\"az\":%d,\"st\":%d},"
            "\"baro\":{\"press_raw\":%lu,\"temp_raw\":%lu,\"st\":%d},"
            "\"mag\":{\"x\":%ld,\"y\":%ld,\"z\":%ld,\"st\":%d}}\r\n",
            (unsigned long)HAL_GetTick(),
            (int)imu.gx, (int)imu.gy, (int)imu.gz,
            (int)imu.ax, (int)imu.ay, (int)imu.az, (int)imu_st,
            (unsigned long)baro.press_raw, (unsigned long)baro.temp_raw, (int)baro_st,
            (long)mag.x, (long)mag.y, (long)mag.z, (int)mag_st);

        if (n > 0 && (size_t)n < sizeof(line)) {
            console_print(line);
        }

        if (imu_st != IMU_OK || baro_st != BARO_OK || mag_st != MAG_OK) {
            console_led_fail();
        } else {
            console_led_pass();
        }

        HAL_Delay(TELEMETRY_LOOP_DELAY_MS);
    }
}
