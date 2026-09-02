/**
 ******************************************************************************
 * @file    driver_gps.h
 * @brief   Driver del GPS HGLRC M100 (modulo MAX-M10M) por USART3 @ 115200.
 *
 *          Pines:
 *            TX (PB10) -> al GPS RX
 *            RX (PB11) <- del GPS TX
 *          El modulo arranca emitiendo sentencias NMEA estandar:
 *            $GxRMC, $GxGGA, $GxGSA, etc., terminadas en CR LF.
 *          El test valida que llegan tramas y que el checksum es correcto.
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_GPS_H_
#define TESTS_INC_DRIVER_GPS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_NMEA_MAX_LEN    96u  /* sentencias NMEA cortas; descartamos largas */

typedef enum {
    GPS_OK = 0,
    GPS_ERR_UART,
    GPS_ERR_TIMEOUT,
    GPS_ERR_CHECKSUM,
    GPS_ERR_OVERFLOW
} gps_status_t;

/* Estructura con una sentencia NMEA capturada (sin '$' inicial ni '*XX'). */
typedef struct {
    char    body[GPS_NMEA_MAX_LEN];  /* contenido sin '$' ni '*CS' */
    uint16_t length;                 /* longitud de body */
    uint8_t  checksum_rx;            /* checksum recibido */
    uint8_t  checksum_calc;          /* checksum calculado localmente */
    bool     valid;                  /* checksum_rx == checksum_calc */
} gps_nmea_sentence_t;

/* ------------------------------------------------------------------------- */
/* Inicializa el driver (en este caso, simplemente reinicia el estado interno).*/
/* USART3 ya esta inicializado por MX_USART3_UART_Init().                     */
/* ------------------------------------------------------------------------- */
void gps_init(void);

/* ------------------------------------------------------------------------- */
/* Lee una sentencia NMEA completa (espera el '$', luego acumula hasta CR/LF),*/
/* valida el checksum XOR y la devuelve en out.                                */
/* Devuelve GPS_OK si la sentencia es completa y con checksum correcto.       */
/* GPS_ERR_TIMEOUT si pasa timeout_ms sin recibir una sentencia completa.     */
/* ------------------------------------------------------------------------- */
gps_status_t gps_read_sentence(gps_nmea_sentence_t *out, uint32_t timeout_ms);

/* ------------------------------------------------------------------------- */
/* Cuenta cuantas sentencias validas y cuantas invalidas se reciben durante   */
/* window_ms milisegundos. Util para el test de presencia del GPS.            */
/* ------------------------------------------------------------------------- */
void gps_count_sentences(uint32_t window_ms, uint16_t *valid, uint16_t *invalid);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_GPS_H_ */
