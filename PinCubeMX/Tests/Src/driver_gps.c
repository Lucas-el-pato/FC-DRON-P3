/**
 ******************************************************************************
 * @file    driver_gps.c
 * @brief   Driver del GPS M100 (NMEA) por USART3, polling.
 ******************************************************************************
 */

#include "driver_gps.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

#include <string.h>
#include <ctype.h>

extern UART_HandleTypeDef huart3;

/* ------------------------------------------------------------------------- */
/* Helpers internos.                                                          */
/* ------------------------------------------------------------------------- */
static int hex2nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

/* Lee un byte por UART con timeout (en ms). Devuelve HAL_OK / HAL_TIMEOUT. */
static HAL_StatusTypeDef gps_read_byte(uint8_t *b, uint32_t timeout_ms)
{
    return HAL_UART_Receive(&huart3, b, 1u, timeout_ms);
}

/* ------------------------------------------------------------------------- */
/* gps_init                                                                   */
/* ------------------------------------------------------------------------- */
void gps_init(void)
{
    /* USART3 ya esta inicializado por MX_USART3_UART_Init() en main.c.
     * Limpiamos eventuales errores y descartamos lo que haya en el buffer.   */
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    __HAL_UART_CLEAR_NEFLAG(&huart3);
    __HAL_UART_CLEAR_FEFLAG(&huart3);
    __HAL_UART_CLEAR_PEFLAG(&huart3);
}

/* ------------------------------------------------------------------------- */
/* gps_read_sentence                                                          */
/*                                                                            */
/* Estados:                                                                   */
/*   0 - esperando '$' de inicio                                              */
/*   1 - acumulando body hasta '*'                                            */
/*   2 - leyendo primer digito del checksum                                   */
/*   3 - leyendo segundo digito del checksum                                  */
/*   4 - esperando CR / LF final                                              */
/* ------------------------------------------------------------------------- */
gps_status_t gps_read_sentence(gps_nmea_sentence_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return GPS_ERR_UART;
    }
    memset(out, 0, sizeof(*out));

    uint32_t t0 = HAL_GetTick();
    uint8_t  state = 0u;
    uint8_t  xor_acc = 0u;
    uint16_t idx = 0u;
    int      cs_hi = -1;
    int      cs_lo = -1;

    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t b;
        HAL_StatusTypeDef st = gps_read_byte(&b, 50u);
        if (st == HAL_TIMEOUT) {
            continue; /* sigue intentando hasta agotar timeout_ms global */
        }
        if (st != HAL_OK) {
            /* Errores de frame/overrun: limpiamos y seguimos. */
            __HAL_UART_CLEAR_OREFLAG(&huart3);
            __HAL_UART_CLEAR_NEFLAG(&huart3);
            __HAL_UART_CLEAR_FEFLAG(&huart3);
            __HAL_UART_CLEAR_PEFLAG(&huart3);
            state = 0u;
            idx = 0u;
            xor_acc = 0u;
            continue;
        }

        switch (state) {
        case 0:
            if (b == '$') {
                state = 1u;
                idx = 0u;
                xor_acc = 0u;
            }
            break;

        case 1:
            if (b == '*') {
                state = 2u;
            } else if (b == '\r' || b == '\n') {
                /* Sentencia sin checksum -> descartar. */
                state = 0u;
                idx = 0u;
                xor_acc = 0u;
            } else {
                if (idx >= (GPS_NMEA_MAX_LEN - 1u)) {
                    return GPS_ERR_OVERFLOW;
                }
                out->body[idx++] = (char)b;
                xor_acc ^= b;
            }
            break;

        case 2:
            cs_hi = hex2nibble((char)b);
            if (cs_hi < 0) {
                state = 0u;
                idx = 0u;
                xor_acc = 0u;
                break;
            }
            state = 3u;
            break;

        case 3:
            cs_lo = hex2nibble((char)b);
            if (cs_lo < 0) {
                state = 0u;
                idx = 0u;
                xor_acc = 0u;
                break;
            }
            out->body[idx] = '\0';
            out->length = idx;
            out->checksum_rx   = (uint8_t)((cs_hi << 4) | cs_lo);
            out->checksum_calc = xor_acc;
            out->valid = (out->checksum_rx == out->checksum_calc);
            state = 4u;
            break;

        case 4:
            if (b == '\r' || b == '\n') {
                /* Devuelve incluso si checksum es invalido (el caller decide).*/
                return out->valid ? GPS_OK : GPS_ERR_CHECKSUM;
            }
            /* Cualquier otra cosa: invalido, reiniciar. */
            state = 0u;
            idx = 0u;
            xor_acc = 0u;
            break;

        default:
            state = 0u;
            break;
        }
    }

    return GPS_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------- */
/* gps_count_sentences                                                        */
/* ------------------------------------------------------------------------- */
void gps_count_sentences(uint32_t window_ms, uint16_t *valid, uint16_t *invalid)
{
    uint16_t v = 0u;
    uint16_t i = 0u;
    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) < window_ms) {
        gps_nmea_sentence_t s;
        gps_status_t st = gps_read_sentence(&s, 200u);
        if (st == GPS_OK) {
            v++;
        } else if (st == GPS_ERR_CHECKSUM) {
            i++;
        }
        /* timeout / otros: simplemente seguimos intentando. */
    }

    if (valid != NULL)   *valid   = v;
    if (invalid != NULL) *invalid = i;
}
