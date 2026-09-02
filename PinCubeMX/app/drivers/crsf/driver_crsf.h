/**
 ******************************************************************************
 * @file    driver_crsf.h
 * @brief   Driver CRSF (Crossfire / ELRS) sobre UART4 @ 420000.
 *
 *          Pines:
 *            TX (PA0)  -> pad RX del receptor SuperD
 *            RX (PA1)  <- pad TX del receptor SuperD
 *
 *          Estructura de un frame CRSF:
 *            [device_addr][len][type][payload...][crc8]
 *              device_addr = 0xC8 (flight controller / sync)
 *              len         = longitud del bloque desde 'type' hasta 'crc8'
 *                            (= payload_len + 2)
 *              type        = tipo de paquete
 *              payload     = (len - 2) bytes
 *              crc8        = CRC8 polinomio 0xD5 sobre [type + payload]
 *                            (NO incluye sync ni len)
 *
 *          Recepcion: DMA circular en DMA1_Stream2 Canal 4 (UART4_RX).
 *          Transmision: HAL_UART_Transmit bloqueante (~240 us por attitude).
 *
 * -------------------------------------------------------------------------
 * TIPOS DE FRAME SOPORTADOS
 * -------------------------------------------------------------------------
 *
 * Tipo 0x16 (RC Channels Packed) -- RECEPCION:
 *   16 canales x 11 bits = 22 bytes de payload, LITTLE-endian bit packing.
 *   Rango tipico de canal: 172 .. 1811 (centro ~992).
 *
 * Tipo 0x1E (Attitude) -- TRANSMISION:
 *   Payload 6 bytes, frame total 10 bytes, BIG-endian:
 *     [0xC8][0x08][0x1E][pitch_hi][pitch_lo][roll_hi][roll_lo]
 *                       [yaw_hi][yaw_lo][crc8]
 *   Cada eje: int16 = radianes * 10000. Rango obligatorio -pi..+pi
 *   (pi * 10000 = 31416, cabe en int16).
 *
 * Tipo 0x09 (Barometric Altitude) -- TRANSMISION:
 *   Payload 2 bytes (vspeed opcional omitido a proposito), frame total 6:
 *     [0xC8][0x04][0x09][alt_hi][alt_lo][crc8]
 *   uint16 BIG-endian empaquetado:
 *     MSB=0 -> valor = decimetros + offset 10000
 *              (10000 = 0 m, rango -1000 m .. +2276.7 m)
 *     MSB=1 -> 15 bits bajos = metros sin offset
 *   Decision: se envia SOLO altitud (2 bytes). La velocidad vertical va
 *   por el frame 0x07, porque la spec TBS (int8 log) y ELRS (int16) no
 *   coinciden en el campo opcional de 0x09.
 *
 * Tipo 0x07 (Variometer) -- TRANSMISION:
 *   Payload 2 bytes, frame total 6 bytes, BIG-endian:
 *     [0xC8][0x04][0x07][vs_hi][vs_lo][crc8]
 *   int16 = velocidad vertical en cm/s (positivo = subiendo).
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
#define CRSF_TYPE_VARIO         0x07u
#define CRSF_TYPE_BARO_ALTITUDE 0x09u
#define CRSF_TYPE_RC_CHANNELS   0x16u
#define CRSF_TYPE_ATTITUDE      0x1Eu
#define CRSF_MAX_PAYLOAD        62u   /* maximo frame: 64 bytes total */
#define CRSF_CHANNELS_COUNT     16u

typedef enum {
    CRSF_OK = 0,
    CRSF_ERR_UART,
    CRSF_ERR_TIMEOUT,
    CRSF_ERR_BAD_ADDR,
    CRSF_ERR_BAD_LEN,
    CRSF_ERR_CRC,
    CRSF_ERR_NONE     /* poll: sin frame completo aun */
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

/* Contadores de diagnostico (visibles en Live Expressions / console). */
extern volatile uint32_t g_crsfByteCount;
extern volatile uint32_t g_crsfValidCount;
extern volatile uint32_t g_crsfCrcErrCount;
extern volatile uint32_t g_crsfTxCount;

/* ------------------------------------------------------------------------- */
/* Inicializa el driver: limpia flags UART4 y arranca DMA circular RX.        */
/* La inicializacion de UART4 ya la hizo MX_UART4_Init() a 420000 bps.        */
/* ------------------------------------------------------------------------- */
void crsf_init(void);

/* ------------------------------------------------------------------------- */
/* Poll no bloqueante: consume el ring DMA.                                   */
/* Retorna CRSF_OK si hay un frame con CRC valido, CRSF_ERR_NONE si aun no    */
/* hay frame completo, o un codigo de error (CRC/len).                        */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_poll_frame(crsf_frame_t *out);

/* ------------------------------------------------------------------------- */
/* Lee un frame CRSF completo con timeout (wrapper sobre crsf_poll_frame).    */
/* Conserva la firma y semantica original para no romper test_rc.c.           */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_read_frame(crsf_frame_t *out, uint32_t timeout_ms);

/* ------------------------------------------------------------------------- */
/* Decodifica un frame de canales RC (type=0x16) a 16 valores de 11 bits.     */
/* Retorna CRSF_OK si frame->type == 0x16 y payload_len == 22.                */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_decode_channels(const crsf_frame_t *frame, crsf_channels_t *out);

/* ------------------------------------------------------------------------- */
/* CRC8 con polinomio 0xD5 (sin shift) usado por CRSF.                        */
/* ------------------------------------------------------------------------- */
uint8_t crsf_crc8(const uint8_t *data, uint16_t len);

/* ------------------------------------------------------------------------- */
/* Empaqueta altitud relativa en decimetros al formato packed de 0x09.        */
/* ------------------------------------------------------------------------- */
uint16_t crsf_pack_altitude(int32_t altitude_dm);

/* ------------------------------------------------------------------------- */
/* Transmite un frame generico: [0xC8][len][type][payload][crc8].             */
/* len = payload_len + 2. Timeout TX = 5 ms.                                  */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_send_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len);

/* ------------------------------------------------------------------------- */
/* Helpers de telemetria (radianes / dm / cm/s -> frames BIG-endian).         */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_send_attitude(float pitch_rad, float roll_rad, float yaw_rad);
crsf_status_t crsf_send_baro_altitude(int32_t altitude_dm);
crsf_status_t crsf_send_vario(int16_t vspeed_cm_s);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_CRSF_H_ */
