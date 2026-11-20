/**
 ******************************************************************************
 * @file    driver_gps.h
 * @brief   Driver del GPS HGLRC M100 (modulo u-blox MAX-M10M) por USART3.
 *
 *          Pines (115200 8N1, configurado por MX_USART3_UART_Init):
 *            TX (PB10) -> al GPS RX   (no se usa por ahora)
 *            RX (PB11) <- del GPS TX
 *
 *          Recepcion NO bloqueante: USART3_RX no tiene stream DMA libre en el
 *          F405 (DMA1_Stream1 es de DShot), asi que cada byte entra por la
 *          interrupcion RXNE a un ring buffer. gps_process() (tarea lenta)
 *          vacia el ring y parsea NMEA:
 *            $GxGGA -> hora, lat, lon, calidad de fix, satelites, HDOP, altitud
 *            $GxRMC -> estado A/V, lat, lon, velocidad, rumbo
 *          Cualquier talker (GP, GN, GL...). Checksum XOR obligatorio.
 *          Coordenadas en enteros (grados x 1e7), sin float ni double.
 *
 *          Basado en el parser tipo TinyGPS usado en Comms_CCTE-2026
 *          (MAX-M10S), con dos correcciones: el fix se invalida si deja de
 *          llegar (GPS_FIX_TIMEOUT_MS) y la ISR no toca los datos publicados.
 *
 *          El handler USART3_IRQHandler vive en driver_gps.c (el IRQ de USART3
 *          no esta habilitado en el .ioc). Si algun dia se habilita en CubeMX,
 *          borrar el handler de driver_gps.c y llamar a gps_uart_irq() desde
 *          el USER CODE de stm32f4xx_it.c.
 ******************************************************************************
 */

#ifndef DRIVERS_GPS_DRIVER_GPS_H_
#define DRIVERS_GPS_DRIVER_GPS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Ring de recepcion (potencia de 2). A 115200 entran ~11.5 kB/s: 512 B
 * aguantan ~44 ms sin que la tarea GPS lo vacie.                         */
#ifndef GPS_RX_RING_SIZE
#define GPS_RX_RING_SIZE      512u
#endif

/* Sin sentencia con fix durante este tiempo -> valid = false. */
#ifndef GPS_FIX_TIMEOUT_MS
#define GPS_FIX_TIMEOUT_MS    2000u
#endif

/* Calidad minima para considerar valida una posicion. Bajo techo el modulo
 * sigue reportando "fix" con pocos satelites y HDOP alto, y la posicion se
 * va a la deriva (recorrido fantasma): eso se descarta como sin fix.
 * Hacen falta GPS_GOOD_STREAK GGA buenas seguidas para volver a valid.   */
#ifndef GPS_MIN_SATS
#define GPS_MIN_SATS          8u      /* 6 dejaba pasar deriva bajo techo */
#endif
#ifndef GPS_MAX_HDOP_X100
#define GPS_MAX_HDOP_X100     250u    /* HDOP 2.5 */
#endif
#ifndef GPS_GOOD_STREAK
#define GPS_GOOD_STREAK       3u      /* ~3 s a 1 Hz */
#endif

/* Prioridad NVIC del USART3. Numericamente mayor que EXTI2 (gyro, 5): el
 * GPS nunca demora al IMU. A 115200 hay ~87 us por byte para atenderlo.  */
#ifndef GPS_NVIC_PRIO
#define GPS_NVIC_PRIO         6u
#endif

/* Datos publicados (solo los escribe gps_process). */
typedef struct {
    bool     valid;          /* fix de buena calidad y no vencido         */
    uint8_t  fix_quality;    /* GGA campo 6: 0 sin fix, 1 GPS, 2 DGPS...  */
    uint8_t  sats;           /* satelites usados (GGA)                    */
    uint16_t hdop_x100;      /* HDOP x100 (GGA)                           */
    int32_t  lat_e7;         /* grados x 1e7, + = Norte                   */
    int32_t  lon_e7;         /* grados x 1e7, + = Este                    */
    int32_t  alt_cm;         /* altitud MSL en cm (GGA)                   */
    uint16_t speed_cms;      /* velocidad sobre el suelo, cm/s (RMC)      */
    uint16_t course_cdeg;    /* rumbo sobre el suelo, grados x100 (RMC)   */
    uint32_t utc_hhmmss;     /* hora UTC como entero hhmmss               */
    uint32_t last_fix_ms;    /* HAL_GetTick() de la ultima posicion       */
    uint32_t last_rx_ms;     /* HAL_GetTick() de la ultima sentencia ok   */
} gps_data_t;

/* Contadores de diagnostico (desde gps_init). */
typedef struct {
    uint32_t rx_bytes;       /* bytes sacados del ring                    */
    uint32_t sentences_ok;   /* checksum correcto                         */
    uint32_t sentences_bad;  /* checksum incorrecto / malformadas         */
    uint32_t uart_overruns;  /* ORE: un byte se perdio en el UART         */
    uint32_t uart_errors;    /* ruido / framing                           */
    uint32_t ring_overflows; /* bytes descartados con el ring lleno       */
} gps_stats_t;

/* Resetea el estado y habilita la RX por interrupcion en USART3.
 * USART3 ya esta inicializado por MX_USART3_UART_Init().                  */
void gps_init(void);

/* Vacia el ring y parsea. No bloquea; llamarlo cada <= 40 ms. */
void gps_process(void);

/* Ultimo dato publicado. */
const gps_data_t *gps_get(void);

/* Contadores (copia, los de la ISR pueden cambiar mientras se leen). */
void gps_stats_get(gps_stats_t *out);

/* Cuerpo del IRQ (lo llama USART3_IRQHandler). */
void gps_uart_irq(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_GPS_DRIVER_GPS_H_ */
