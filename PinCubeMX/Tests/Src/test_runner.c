/**
 ******************************************************************************
 * @file    test_runner.c
 * @brief   Despacha al test seleccionado por el #define en test_runner.h.
 ******************************************************************************
 */

#include "test_runner.h"
#include "console.h"

/* Verifica que no se haya seleccionado mas de un test a la vez. */
#define TEST_RUNNER_COUNT \
    ( (defined(TEST_SELECT_IMU)      ? 1 : 0) \
    + (defined(TEST_SELECT_IMU_DIAG) ? 1 : 0) \
    + (defined(TEST_SELECT_BARO)     ? 1 : 0) \
    + (defined(TEST_SELECT_MAG)      ? 1 : 0) \
    + (defined(TEST_SELECT_GPS)      ? 1 : 0) \
    + (defined(TEST_SELECT_RC)       ? 1 : 0) \
    + (defined(TEST_SELECT_MOTORS)   ? 1 : 0) )

#if TEST_RUNNER_COUNT > 1
#error "Hay mas de un TEST_SELECT_xxx definido. Dejar solo uno."
#endif

void test_runner_run(void)
{
    console_init();

#if defined(TEST_SELECT_IMU)
    test_imu_run();
#elif defined(TEST_SELECT_IMU_DIAG)
    test_imu_diag_run();
#elif defined(TEST_SELECT_BARO)
    test_baro_run();
#elif defined(TEST_SELECT_MAG)
    test_mag_run();
#elif defined(TEST_SELECT_GPS)
    test_gps_run();
#elif defined(TEST_SELECT_RC)
    test_rc_run();
#elif defined(TEST_SELECT_MOTORS)
    test_motors_run();
#else
    console_print("\r\nERROR: ningun TEST_SELECT_xxx definido en test_runner.h\r\n");
    while (1) {
        console_led_fail();
        HAL_Delay(250u);
        console_led_off();
        HAL_Delay(250u);
    }
#endif
}
