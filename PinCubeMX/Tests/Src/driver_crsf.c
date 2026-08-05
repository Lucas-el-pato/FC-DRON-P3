/**
 ******************************************************************************
 * @file    driver_crsf.c
 * @brief   Driver CRSF (ELRS / Crossfire) por UART4 @ 420000, polling.
 ******************************************************************************
 */

#include "driver_crsf.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

#include <string.h>

extern UART_HandleTypeDef huart4;

/* ------------------------------------------------------------------------- */
/* Tabla precomputada CRC8 con polinomio 0xD5 (DVB-S2 sin reversal).          */
/* Generada con:                                                              */
/*   crc = byte;                                                              */
/*   for(i=0; i<8; i++) crc = (crc & 0x80) ? (crc<<1) ^ 0xD5 : (crc<<1);      */
/* ------------------------------------------------------------------------- */
static const uint8_t crsf_crc_table[256] = {
    0x00,0xD5,0x7F,0xAA,0xFE,0x2B,0x81,0x54,0x29,0xFC,0x56,0x83,0xD7,0x02,0xA8,0x7D,
    0x52,0x87,0x2D,0xF8,0xAC,0x79,0xD3,0x06,0x7B,0xAE,0x04,0xD1,0x85,0x50,0xFA,0x2F,
    0xA4,0x71,0xDB,0x0E,0x5A,0x8F,0x25,0xF0,0x8D,0x58,0xF2,0x27,0x73,0xA6,0x0C,0xD9,
    0xF6,0x23,0x89,0x5C,0x08,0xDD,0x77,0xA2,0xDF,0x0A,0xA0,0x75,0x21,0xF4,0x5E,0x8B,
    0x9D,0x48,0xE2,0x37,0x63,0xB6,0x1C,0xC9,0xB4,0x61,0xCB,0x1E,0x4A,0x9F,0x35,0xE0,
    0xCF,0x1A,0xB0,0x65,0x31,0xE4,0x4E,0x9B,0xE6,0x33,0x99,0x4C,0x18,0xCD,0x67,0xB2,
    0x39,0xEC,0x46,0x93,0xC7,0x12,0xB8,0x6D,0x10,0xC5,0x6F,0xBA,0xEE,0x3B,0x91,0x44,
    0x6B,0xBE,0x14,0xC1,0x95,0x40,0xEA,0x3F,0x42,0x97,0x3D,0xE8,0xBC,0x69,0xC3,0x16,
    0xEF,0x3A,0x90,0x45,0x11,0xC4,0x6E,0xBB,0xC6,0x13,0xB9,0x6C,0x38,0xED,0x47,0x92,
    0xBD,0x68,0xC2,0x17,0x43,0x96,0x3C,0xE9,0x94,0x41,0xEB,0x3E,0x6A,0xBF,0x15,0xC0,
    0x4B,0x9E,0x34,0xE1,0xB5,0x60,0xCA,0x1F,0x62,0xB7,0x1D,0xC8,0x9C,0x49,0xE3,0x36,
    0x19,0xCC,0x66,0xB3,0xE7,0x32,0x98,0x4D,0x30,0xE5,0x4F,0x9A,0xCE,0x1B,0xB1,0x64,
    0x72,0xA7,0x0D,0xD8,0x8C,0x59,0xF3,0x26,0x5B,0x8E,0x24,0xF1,0xA5,0x70,0xDA,0x0F,
    0x20,0xF5,0x5F,0x8A,0xDE,0x0B,0xA1,0x74,0x09,0xDC,0x76,0xA3,0xF7,0x22,0x88,0x5D,
    0xD6,0x03,0xA9,0x7C,0x28,0xFD,0x57,0x82,0xFF,0x2A,0x80,0x55,0x01,0xD4,0x7E,0xAB,
    0x84,0x51,0xFB,0x2E,0x7A,0xAF,0x05,0xD0,0xAD,0x78,0xD2,0x07,0x53,0x86,0x2C,0xF9
};

uint8_t crsf_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0u;
    for (uint16_t i = 0u; i < len; ++i) {
        crc = crsf_crc_table[crc ^ data[i]];
    }
    return crc;
}

/* ------------------------------------------------------------------------- */
/* Helpers internos.                                                          */
/* ------------------------------------------------------------------------- */
static HAL_StatusTypeDef crsf_read_byte(uint8_t *b, uint32_t timeout_ms)
{
    return HAL_UART_Receive(&huart4, b, 1u, timeout_ms);
}

/* ------------------------------------------------------------------------- */
/* crsf_init                                                                  */
/* ------------------------------------------------------------------------- */
void crsf_init(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart4);
    __HAL_UART_CLEAR_NEFLAG(&huart4);
    __HAL_UART_CLEAR_FEFLAG(&huart4);
    __HAL_UART_CLEAR_PEFLAG(&huart4);
}

/* ------------------------------------------------------------------------- */
/* crsf_read_frame                                                            */
/*                                                                            */
/* Estados:                                                                   */
/*  0 - esperando byte de direccion 0xC8                                      */
/*  1 - leyendo longitud (debe estar en 2..62)                                */
/*  2 - leyendo type + payload + crc                                          */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_read_frame(crsf_frame_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return CRSF_ERR_UART;
    }
    memset(out, 0, sizeof(*out));

    uint32_t t0 = HAL_GetTick();
    uint8_t  state = 0u;
    uint8_t  remaining = 0u;
    uint8_t  buf[64];        /* [type + payload + crc] */
    uint8_t  buf_idx = 0u;

    while ((HAL_GetTick() - t0) < timeout_ms) {
        uint8_t b;
        HAL_StatusTypeDef st = crsf_read_byte(&b, 50u);
        if (st == HAL_TIMEOUT) {
            continue;
        }
        if (st != HAL_OK) {
            __HAL_UART_CLEAR_OREFLAG(&huart4);
            __HAL_UART_CLEAR_NEFLAG(&huart4);
            __HAL_UART_CLEAR_FEFLAG(&huart4);
            __HAL_UART_CLEAR_PEFLAG(&huart4);
            state = 0u;
            buf_idx = 0u;
            continue;
        }

        switch (state) {
        case 0:
            if (b == CRSF_ADDR_FC) {
                out->device_addr = b;
                state = 1u;
            }
            break;

        case 1:
            if (b < 2u || b > (CRSF_MAX_PAYLOAD + 2u)) {
                state = 0u;
                break;
            }
            out->len = b;
            remaining = b;       /* type + payload + crc */
            buf_idx = 0u;
            state = 2u;
            break;

        case 2:
            buf[buf_idx++] = b;
            if (--remaining == 0u) {
                /* Frame completo. */
                out->type = buf[0];
                out->payload_len = (uint8_t)(out->len - 2u);
                if (out->payload_len > CRSF_MAX_PAYLOAD) {
                    return CRSF_ERR_BAD_LEN;
                }
                memcpy(out->payload, &buf[1], out->payload_len);
                out->crc_rx   = buf[buf_idx - 1u];
                /* CRC calculado sobre [type + payload]. */
                out->crc_calc = crsf_crc8(buf, (uint16_t)(out->len - 1u));
                out->valid = (out->crc_rx == out->crc_calc);
                return out->valid ? CRSF_OK : CRSF_ERR_CRC;
            }
            break;

        default:
            state = 0u;
            break;
        }
    }

    return CRSF_ERR_TIMEOUT;
}

/* ------------------------------------------------------------------------- */
/* crsf_decode_channels                                                       */
/* Empaquetado: 16 canales x 11 bits = 176 bits = 22 bytes, little-endian.    */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_decode_channels(const crsf_frame_t *frame, crsf_channels_t *out)
{
    if (frame == NULL || out == NULL) {
        return CRSF_ERR_UART;
    }
    if (frame->type != CRSF_TYPE_RC_CHANNELS) {
        return CRSF_ERR_BAD_LEN;
    }
    if (frame->payload_len != 22u) {
        return CRSF_ERR_BAD_LEN;
    }

    uint32_t bit_buf = 0u;
    uint32_t bits = 0u;
    uint8_t  ch_idx = 0u;
    const uint8_t *p = frame->payload;

    for (uint8_t i = 0u; i < 22u; ++i) {
        bit_buf |= ((uint32_t)p[i]) << bits;
        bits += 8u;
        while (bits >= 11u && ch_idx < CRSF_CHANNELS_COUNT) {
            out->ch[ch_idx++] = (uint16_t)(bit_buf & 0x7FFu);
            bit_buf >>= 11u;
            bits   -= 11u;
        }
    }
    return CRSF_OK;
}
