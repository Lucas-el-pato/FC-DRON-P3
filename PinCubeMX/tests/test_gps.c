/**
 ******************************************************************************
 * @file    test_gps.c
 * @brief   Test del GPS HGLRC M100 por USART3: RX por IRQ + parser NMEA.
 *
 *          Cada 1 s imprime fix, satelites, HDOP, posicion, altitud,
 *          velocidad, rumbo y los contadores del driver. Probar al aire
 *          libre: el arranque en frio tarda ~30 s - 1 min en dar fix.
 *
 *          Diagnostico rapido:
 *            bytes = 0            -> no llega nada: cableado TX/RX o energia
 *            bytes > 0, ok = 0    -> baud equivocado (el modulo a 9600?)
 *            ore / ring > 0       -> se pierden bytes (IRQ o tarea lenta)
 ******************************************************************************
 */

#include "console.h"
#include "driver_gps.h"

static void test_gps_print(void)
{
    const gps_data_t *g = gps_get();
    gps_stats_t s;
    gps_stats_get(&s);

    const int32_t lat = g->lat_e7;
    const int32_t lon = g->lon_e7;
    const uint32_t alat = (uint32_t)((lat < 0) ? -lat : lat);
    const uint32_t alon = (uint32_t)((lon < 0) ? -lon : lon);

    /* nano.specs no imprime floats: grados con 7 decimales a mano. */
    console_printf("fix=%u q=%u sats=%2u hdop=%u.%02u | %s%lu.%07lu %s%lu.%07lu | "
                   "alt=%ld.%02ldm v=%u.%02um/s rumbo=%u.%02u | utc=%06lu\r\n",
                   (unsigned)g->valid, (unsigned)g->fix_quality, (unsigned)g->sats,
                   (unsigned)(g->hdop_x100 / 100u), (unsigned)(g->hdop_x100 % 100u),
                   (lat < 0) ? "-" : "", (unsigned long)(alat / 10000000u),
                   (unsigned long)(alat % 10000000u),
                   (lon < 0) ? "-" : "", (unsigned long)(alon / 10000000u),
                   (unsigned long)(alon % 10000000u),
                   (long)(g->alt_cm / 100), (long)((g->alt_cm < 0 ? -g->alt_cm : g->alt_cm) % 100),
                   (unsigned)(g->speed_cms / 100u), (unsigned)(g->speed_cms % 100u),
                   (unsigned)(g->course_cdeg / 100u), (unsigned)(g->course_cdeg % 100u),
                   (unsigned long)g->utc_hhmmss);
    console_printf("    bytes=%lu ok=%lu bad=%lu ore=%lu err=%lu ring=%lu\r\n",
                   (unsigned long)s.rx_bytes, (unsigned long)s.sentences_ok,
                   (unsigned long)s.sentences_bad, (unsigned long)s.uart_overruns,
                   (unsigned long)s.uart_errors, (unsigned long)s.ring_overflows);
}

void test_gps_run(void)
{
    console_banner("GPS HGLRC M100 (NMEA 115200, RX por IRQ)");

    gps_init();

    /* 3 s para ver si llegan sentencias validas. */
    const uint32_t t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < 3000u) {
        gps_process();
        HAL_Delay(10u);
    }

    gps_stats_t s;
    gps_stats_get(&s);
    const bool pass = (s.sentences_ok > 0u);
    console_printf("Sentencias validas:%lu invalidas:%lu bytes:%lu\r\n",
                   (unsigned long)s.sentences_ok, (unsigned long)s.sentences_bad,
                   (unsigned long)s.rx_bytes);
    console_result(pass, pass ? "GPS detectado"
                              : (s.rx_bytes > 0u ? "Llegan bytes pero sin NMEA valido (baud?)"
                                                 : "No llega nada por USART3"));

    uint32_t last = HAL_GetTick();
    while (1) {
        gps_process();
        if ((HAL_GetTick() - last) >= 1000u) {
            last += 1000u;
            test_gps_print();
        }
        HAL_Delay(10u);
    }
}
