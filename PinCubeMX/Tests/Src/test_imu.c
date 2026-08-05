/**
 ******************************************************************************
 * @file    test_imu.c
 * @brief   Test del IMU LSM6DSV16X: WHO_AM_I + lectura periodica de XL/G.
 ******************************************************************************
 */

#include "console.h"
#include "driver_imu.h"

void test_imu_run(void)
{
    console_banner("IMU LSM6DSV16X");

    /* 1. Verificacion WHO_AM_I. */
    uint8_t who = 0u;
    imu_status_t st = imu_check_who_am_i(&who);
    console_printf("WHO_AM_I leido = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)who, (unsigned)IMU_WHO_AM_I_VALUE);

    if (st != IMU_OK) {
        console_result(false, "WHO_AM_I incorrecto o SPI sin respuesta");
        /* Loop con LED rojo. */
        while (1) {
            HAL_Delay(500u);
        }
    }

    /* 2. Init completo del IMU. */
    st = imu_init();
    if (st != IMU_OK) {
        console_printf("imu_init fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo en imu_init");
        while (1) {
            HAL_Delay(500u);
        }
    }

    console_result(true, "IMU inicializada correctamente");

    /* 3. Bucle de lectura periodica.
     * Cada display promedia N=32 muestras (a 120 Hz ODR -> ~270 ms de
     * captura). El delay extra al final solo asegura un ritmo de display
     * comodo para leer en PuTTY.                                            */
    const uint8_t kAvgSamples = 32u;
    while (1) {
        imu_sample_t s = { 0 };
        st = imu_read_sample_avg(&s, kAvgSamples);
        if (st == IMU_OK) {
            console_printf("GYRO x=%6d y=%6d z=%6d | ACCEL x=%6d y=%6d z=%6d\r\n",
                           s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
        } else {
            console_printf("imu_read_sample_avg fallo (codigo=%d)\r\n", (int)st);
            console_led_fail();
        }
        HAL_Delay(100u);
    }
}
