/**
 ******************************************************************************
 * @file    test_baro.c
 * @brief   Test del barometro BMP388: CHIP_ID + presion/temperatura crudas.
 ******************************************************************************
 */

#include "console.h"
#include "driver_baro.h"

void test_baro_run(void)
{
    console_banner("Barometro BMP388");

    uint8_t chip = 0u;
    baro_status_t st = baro_check_chip_id(&chip);
    console_printf("CHIP_ID leido = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)chip, (unsigned)BARO_CHIP_ID_VALUE);

    if (st != BARO_OK) {
        console_result(false, "CHIP_ID incorrecto o SPI sin respuesta");
        while (1) { HAL_Delay(500u); }
    }

    st = baro_init();
    if (st != BARO_OK) {
        console_printf("baro_init fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo en baro_init");
        while (1) { HAL_Delay(500u); }
    }

    console_result(true, "BMP388 inicializado correctamente");

    while (1) {
        baro_sample_t s = { 0 };
        st = baro_read_sample(&s);
        if (st == BARO_OK) {
            console_printf("PRESS_raw=%lu  TEMP_raw=%lu\r\n",
                           (unsigned long)s.press_raw,
                           (unsigned long)s.temp_raw);
        } else {
            console_printf("baro_read_sample fallo (codigo=%d)\r\n", (int)st);
            console_led_fail();
        }
        HAL_Delay(500u);
    }
}
