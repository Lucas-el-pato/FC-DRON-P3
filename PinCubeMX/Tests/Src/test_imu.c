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

    /* 3. Verificar INT1_CTRL.INT1_DRDY_G y que EXTI2 dispara al ODR. */
    uint8_t int1_ctrl = 0u;
    (void)imu_read_reg(IMU_REG_INT1_CTRL, &int1_ctrl);
    console_printf("INT1_CTRL = 0x%02X (esperado 0x%02X = INT1_DRDY_G)\r\n",
                   (unsigned)int1_ctrl, (unsigned)IMU_INT1_DRDY_G);

    const uint32_t t0 = HAL_GetTick();
    uint32_t n_ok = 0u;
    uint32_t n_to = 0u;
    while ((HAL_GetTick() - t0) < 100u) {
        if (imu_wait_gyro_drdy(5u) != IMU_OK) {
            n_to++;
            continue;
        }
        imu_sample_t s = { 0 };
        if (imu_read_sample(&s) == IMU_OK) {
            n_ok++;
        }
    }
    console_printf("muestras INT1 en 100 ms = %lu timeout=%lu (ODR 7.68 kHz -> ~768)\r\n",
                   (unsigned long)n_ok, (unsigned long)n_to);

    /* 4. Bucle de lectura periodica, sincronizado a INT1 gyro DRDY.
     * Cada display promedia N=32 muestras (a 7.68 kHz ODR -> ~4.2 ms de
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
