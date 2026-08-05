/**
 ******************************************************************************
 * @file    test_gps.c
 * @brief   Test del GPS HGLRC M100 por USART3: recepcion NMEA + checksum.
 ******************************************************************************
 */

#include "console.h"
#include "driver_gps.h"

void test_gps_run(void)
{
    console_banner("GPS HGLRC M100 (NMEA 115200)");

    gps_init();

    /* Ventana de captura inicial de 3 s. */
    uint16_t v = 0u, inv = 0u;
    console_print("Capturando sentencias NMEA durante 3 segundos...\r\n");
    gps_count_sentences(3000u, &v, &inv);
    console_printf("Sentencias validas:%u  invalidas:%u\r\n",
                   (unsigned)v, (unsigned)inv);

    bool pass = (v > 0u);
    console_result(pass, pass ? "GPS detectado" : "Sin sentencias NMEA validas");

    if (!pass) {
        while (1) { HAL_Delay(500u); }
    }

    /* Bucle de impresion continua: una sentencia por iteracion. */
    while (1) {
        gps_nmea_sentence_t s;
        gps_status_t st = gps_read_sentence(&s, 2000u);
        if (st == GPS_OK) {
            console_printf("$%s*%02X [OK]\r\n",
                           s.body, (unsigned)s.checksum_rx);
        } else if (st == GPS_ERR_CHECKSUM) {
            console_printf("$%s*%02X [BAD CRC, calc=%02X]\r\n",
                           s.body, (unsigned)s.checksum_rx,
                           (unsigned)s.checksum_calc);
        } else {
            console_print("(timeout esperando sentencia)\r\n");
        }
    }
}
