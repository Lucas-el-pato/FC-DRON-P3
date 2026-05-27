/**
 ******************************************************************************
 * @file    driver_crsf.h
 * @brief   Driver CRSF (Crossfire / ELRS) sobre UART4 @ 420000.
 *
 *          Pines:
 *            TX (PA0)  -> al receptor
 *            RX (PA1)  <- del receptor
 *
 *          Estructura de un frame CRSF:
 *            [device_addr][len][type][payload...][crc8]
 *              device_addr = 0xC8 (flight controller) o 0xEE (handset)
 *              len         = longitud del bloque desde 'type' hasta 'crc8'
 *              type        = tipo de paquete (0x16 = RC channels)
 *              payload     = (len - 2) bytes
 *              crc8        = CRC8 con polinomio 0xD5 sobre [type + payload]
 *
 *          Tipo 0x16 (RC Channels Packed): 16 canales de 11 bits cada uno
 *          empaquetados little-endian (22 bytes de payload).
 *          Rango tipico de canal:  172 .. 1811 (centro ~992).
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_CRSF_H_
#define TESTS_INC_DRIVER_CRSF_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CRSF_ADDR_FC            0xC8u
#define CRSF_TYPE_RC_CHANNELS   0x16u
#define CRSF_MAX_PAYLOAD        62u   /* maximo frame: 64 bytes total */
#define CRSF_CHANNELS_COUNT     16u

typedef enum {
    CRSF_OK = 0,
    CRSF_ERR_UART,
    CRSF_ERR_TIMEOUT,
    CRSF_ERR_BAD_ADDR,
    CRSF_ERR_BAD_LEN,
    CRSF_ERR_CRC
} crsf_status_t;

typedef struct {
    uint8_t  device_addr;
    uint8_t  len;
    uint8_t  type;
    uint8_t  payload[CRSF_MAX_PAYLOAD];
    uint8_t  payload_len;        /* = len - 2 */
    uint8_t  crc_rx;
    uint8_t  crc_calc;
    bool     valid;
} crsf_frame_t;

/* Canales RC ya desempaquetados (rango ~172..1811). */
typedef struct {
    uint16_t ch[CRSF_CHANNELS_COUNT];
} crsf_channels_t;

/* ------------------------------------------------------------------------- */
/* Inicializa el driver (limpia el estado / flags de UART4).                  */
/* La inicializacion de UART4 ya la hizo MX_UART4_Init() a 420000 bps.        */
/* ------------------------------------------------------------------------- */
void crsf_init(void);

/* ------------------------------------------------------------------------- */
/* Lee un frame CRSF completo con timeout.                                    */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_read_frame(crsf_frame_t *out, uint32_t timeout_ms);

/* ------------------------------------------------------------------------- */
/* Decodifica un frame de canales RC (type=0x16) a 16 valores de 11 bits.     */
/* Retorna CRSF_OK si frame->type == 0x16 y payload_len == 22.                */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_decode_channels(const crsf_frame_t *frame, crsf_channels_t *out);

/* ------------------------------------------------------------------------- */
/* CRC8 con polinomio 0xD5 (sin shift) usado por CRSF. Util en tests.         */
/* ------------------------------------------------------------------------- */
uint8_t crsf_crc8(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_CRSF_H_ */
