/**
 ******************************************************************************
 * @file    test_runner.c
 * @brief   Despacha al test seleccionado por el #define en test_runner.h.
 ******************************************************************************
 */

#include "test_runner.h"
#include "console.h"
#include "timebase.h"

#if TEST_RUNNER_COUNT > 1
#error "Hay mas de un TEST_SELECT_xxx definido. Dejar solo uno."
#endif

void test_runner_run(void)
{
    (void)timebase_init();

#if defined(TEST_SELECT_IMU)
    /* El test IMU enciende el LED y despues llama console_init(), para no
     * esperar DTR USB antes de indicar que el sensor ya esta listo. */
    test_imu_run();
    return;
#endif

    console_init();

#if defined(TEST_SELECT_IMU_DIAG)
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
#elif defined(TEST_SELECT_TELEMETRY)
    test_telemetry_run();
#elif defined(TEST_SELECT_CRSF_TELEM)
    test_crsf_telem_run();
#elif defined(TEST_SELECT_SD)
    test_sd_run();
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
