/**
 ******************************************************************************
 * @file    test_mag.c
 * @brief   Test del magnetometro MMC5983MA: PRODUCT_ID + lecturas XYZ.
 ******************************************************************************
 */

#include "console.h"
#include "driver_mag.h"

void test_mag_run(void)
{
    console_banner("Magnetometro MMC5983MA");

    uint8_t id = 0u;
    mag_status_t st = mag_check_product_id(&id);
    console_printf("PRODUCT_ID leido = 0x%02X (esperado 0x%02X)\r\n",
                   (unsigned)id, (unsigned)MAG_PRODUCT_ID_VALUE);

    if (st != MAG_OK) {
        console_result(false, "PRODUCT_ID incorrecto o I2C sin respuesta");
        while (1) { HAL_Delay(500u); }
    }

    st = mag_init();
    if (st != MAG_OK) {
        console_printf("mag_init fallo (codigo=%d)\r\n", (int)st);
        console_result(false, "fallo en mag_init");
        while (1) { HAL_Delay(500u); }
    }

    console_result(true, "MMC5983MA inicializado correctamente");

    while (1) {
        mag_sample_t s = { 0 };
        st = mag_read_sample(&s, 50u);
        if (st == MAG_OK) {
            console_printf("MAG x=%7ld y=%7ld z=%7ld\r\n",
                           (long)s.x, (long)s.y, (long)s.z);
        } else {
            console_printf("mag_read_sample fallo (codigo=%d)\r\n", (int)st);
            console_led_fail();
        }
        HAL_Delay(500u);
    }
}
