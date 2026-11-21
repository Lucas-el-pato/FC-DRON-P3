/**
 ******************************************************************************
 * @file    test_mag.c
 * @brief   Test del magnetometro MMC5983MA: PRODUCT_ID + calibracion.
 *
 *          Uso: flashear con TEST_SELECT_MAG, abrir la consola USB y girar
 *          el dron despacio en TODAS las direcciones (8 horizontal, de
 *          costado, boca abajo) durante ~30 s, lejos de metales e imanes y
 *          con la bateria conectada (los cables de potencia cuentan).
 *          Cada 1 s imprime los offsets y escalas sugeridos: copiar la
 *          ultima linea MACROS a sensors_scale.h (o como -D en CMake).
 *          El rumbo (dron plano) usa la calibracion ya compilada.
 ******************************************************************************
 */

#include "console.h"
#include "driver_mag.h"
#include "sensors_scale.h"

#include <math.h>

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

    console_printf("Girar el dron en todas las direcciones ~30 s...\r\n");

    int32_t mn[3] = { INT32_MAX, INT32_MAX, INT32_MAX };
    int32_t mx[3] = { INT32_MIN, INT32_MIN, INT32_MIN };
    uint32_t n = 0u;
    uint32_t fails = 0u;
    uint32_t t_print = HAL_GetTick();

    while (1) {
        /* ~50 Hz: medicion de 8 ms + espera. */
        mag_sample_t s = { 0 };
        st = mag_read_sample(&s, 50u);
        if (st != MAG_OK) {
            fails++;
            console_led_fail();
            HAL_Delay(20u);
            continue;
        }
        const int32_t v[3] = { s.x, s.y, s.z };
        for (uint8_t i = 0u; i < 3u; ++i) {
            if (v[i] < mn[i]) {
                mn[i] = v[i];
            }
            if (v[i] > mx[i]) {
                mx[i] = v[i];
            }
        }
        n++;
        HAL_Delay(10u);

        if ((HAL_GetTick() - t_print) < 1000u) {
            continue;
        }
        t_print = HAL_GetTick();

        /* Rumbo con el dron plano, con la calibracion compilada. */
        sensors_mag_si_t si;
        sensors_mag_scale(&s, &si);
        float hdg = atan2f(-si.my_uT, si.mx_uT) * 57.2957795f + SENSORS_MAG_DECLINATION_DEG;
        while (hdg < 0.0f) {
            hdg += 360.0f;
        }
        while (hdg >= 360.0f) {
            hdg -= 360.0f;
        }

        int32_t off[3];
        int32_t rng[3];
        for (uint8_t i = 0u; i < 3u; ++i) {
            off[i] = (mx[i] + mn[i]) / 2;
            rng[i] = (mx[i] - mn[i]) / 2;
        }
        const int32_t avg = (rng[0] + rng[1] + rng[2]) / 3;

        console_printf("RAW x=%7ld y=%7ld z=%7ld | rumbo plano=%3ld deg | n=%lu fail=%lu\r\n",
                       (long)s.x, (long)s.y, (long)s.z, (long)hdg,
                       (unsigned long)n, (unsigned long)fails);
        console_printf("RANGO x=%ld..%ld y=%ld..%ld z=%ld..%ld (cuentas)\r\n",
                       (long)mn[0], (long)mx[0], (long)mn[1], (long)mx[1],
                       (long)mn[2], (long)mx[2]);
        if (rng[0] > 0 && rng[1] > 0 && rng[2] > 0) {
            /* Escala x1000 en enteros (nano.specs no imprime floats). */
            console_printf("MACROS -DSENSORS_MAG_OFFSET_X=%ld -DSENSORS_MAG_OFFSET_Y=%ld "
                           "-DSENSORS_MAG_OFFSET_Z=%ld -DSENSORS_MAG_SCALE_X=%ld.%03ldf "
                           "-DSENSORS_MAG_SCALE_Y=%ld.%03ldf -DSENSORS_MAG_SCALE_Z=%ld.%03ldf\r\n",
                           (long)off[0], (long)off[1], (long)off[2],
                           (long)((avg * 1000) / rng[0] / 1000), (long)((avg * 1000) / rng[0] % 1000),
                           (long)((avg * 1000) / rng[1] / 1000), (long)((avg * 1000) / rng[1] % 1000),
                           (long)((avg * 1000) / rng[2] / 1000), (long)((avg * 1000) / rng[2] % 1000));
        }
    }
}
