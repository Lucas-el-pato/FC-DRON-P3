/**
 ******************************************************************************
 * @file    driver_crsf.c
 * @brief   Driver CRSF (ELRS / Crossfire) por UART4 @ 420000.
 *
 *          RX: DMA circular DMA1_Stream2 Canal 4 (UART4_RX), sin IRQ.
 *              El hardware llena el ring; crsf_poll_frame() consume bytes.
 *              Asi el TX bloqueante no pierde canales RC.
 *
 *          TX: HAL_UART_Transmit bloqueante. Un attitude (10 bytes) tarda
 *              ~240 us @ 420000 bps; el DMA RX sigue capturando mientras.
 ******************************************************************************
 */

#include "driver_crsf.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

#include <math.h>
#include <string.h>

extern UART_HandleTypeDef huart4;

/* Ring DMA: 256 bytes ~= 6 ms de linea a 420000 bps. */
#define CRSF_RING_LEN       256u
#define CRSF_TX_TIMEOUT_MS  5u

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

volatile uint32_t g_crsfByteCount = 0u;
volatile uint32_t g_crsfValidCount = 0u;
volatile uint32_t g_crsfCrcErrCount = 0u;
volatile uint32_t g_crsfTxCount = 0u;

static DMA_HandleTypeDef hdma_uart4_rx;
static uint8_t  crsf_ring[CRSF_RING_LEN];
static uint16_t crsf_rd = 0u;
static bool     crsf_dma_running = false;

/* Estado del parser entre llamadas a poll. */
static uint8_t  parse_state = 0u;   /* 0=addr, 1=len, 2=body */
static uint8_t  parse_addr = 0u;
static uint8_t  parse_len = 0u;
static uint8_t  parse_remaining = 0u;
static uint8_t  parse_buf[64];
static uint8_t  parse_buf_idx = 0u;

uint8_t crsf_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0u;
    if (data == NULL) {
        return 0u;
    }
    for (uint16_t i = 0u; i < len; ++i) {
        crc = crsf_crc_table[crc ^ data[i]];
    }
    return crc;
}

uint16_t crsf_pack_altitude(int32_t altitude_dm)
{
    /* MSB=0: dm + offset 10000. MSB=1: metros | 0x8000. */
    if (altitude_dm < -10000) {
        return 0u;
    }
    if ((altitude_dm + 10000) < 0x8000) {
        return (uint16_t)(altitude_dm + 10000);
    }
    {
        int32_t meters = (altitude_dm + 5) / 10;
        if (meters > 0x7FFE) {
            meters = 0x7FFE;
        }
        return (uint16_t)(((uint16_t)meters & 0x7FFFu) | 0x8000u);
    }
}

static void crsf_write_be16(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)((value >> 8) & 0xFFu);
    dst[1] = (uint8_t)(value & 0xFFu);
}

static void crsf_check_uart_errors(void)
{
    const uint32_t sr = huart4.Instance->SR;
    if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0u) {
        (void)huart4.Instance->DR;
    }
}

static void crsf_parser_reset(void)
{
    parse_state = 0u;
    parse_buf_idx = 0u;
    parse_remaining = 0u;
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

    /* DMA1 ya habilitado por MX_DMA_Init(). UART4_RX = Stream2 Canal4. */
    CLEAR_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
    if (crsf_dma_running) {
        (void)HAL_DMA_Abort(&hdma_uart4_rx);
        (void)HAL_DMA_DeInit(&hdma_uart4_rx);
        crsf_dma_running = false;
    }

    hdma_uart4_rx.Instance = DMA1_Stream2;
    hdma_uart4_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK) {
        crsf_dma_running = false;
        return;
    }

    memset(crsf_ring, 0, sizeof(crsf_ring));
    crsf_rd = 0u;
    crsf_parser_reset();

    if (HAL_DMA_Start(&hdma_uart4_rx,
                      (uint32_t)&huart4.Instance->DR,
                      (uint32_t)crsf_ring,
                      CRSF_RING_LEN) != HAL_OK) {
        crsf_dma_running = false;
        return;
    }
    crsf_dma_running = true;

    (void)huart4.Instance->SR;
    (void)huart4.Instance->DR;
    SET_BIT(huart4.Instance->CR3, USART_CR3_DMAR);
}

/* ------------------------------------------------------------------------- */
/* crsf_poll_frame                                                            */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_poll_frame(crsf_frame_t *out)
{
    if (out == NULL) {
        return CRSF_ERR_UART;
    }
    if (!crsf_dma_running) {
        return CRSF_ERR_UART;
    }

    crsf_check_uart_errors();

    uint16_t wr = (uint16_t)(CRSF_RING_LEN -
                             __HAL_DMA_GET_COUNTER(&hdma_uart4_rx));
    if (wr >= CRSF_RING_LEN) {
        wr = 0u;
    }

    while (crsf_rd != wr) {
        const uint8_t b = crsf_ring[crsf_rd];
        crsf_rd = (uint16_t)((crsf_rd + 1u) % CRSF_RING_LEN);
        g_crsfByteCount++;

        switch (parse_state) {
        case 0:
            if (b == CRSF_ADDR_FC) {
                parse_addr = b;
                parse_state = 1u;
            }
            break;

        case 1:
            if (b < 2u || b > (CRSF_MAX_PAYLOAD + 2u)) {
                crsf_parser_reset();
                break;
            }
            parse_len = b;
            parse_remaining = b;
            parse_buf_idx = 0u;
            parse_state = 2u;
            break;

        case 2:
            parse_buf[parse_buf_idx++] = b;
            if (--parse_remaining == 0u) {
                memset(out, 0, sizeof(*out));
                out->device_addr = parse_addr;
                out->len = parse_len;
                out->type = parse_buf[0];
                out->payload_len = (uint8_t)(parse_len - 2u);
                if (out->payload_len > CRSF_MAX_PAYLOAD) {
                    crsf_parser_reset();
                    return CRSF_ERR_BAD_LEN;
                }
                memcpy(out->payload, &parse_buf[1], out->payload_len);
                out->crc_rx = parse_buf[parse_buf_idx - 1u];
                out->crc_calc = crsf_crc8(parse_buf, (uint16_t)(parse_len - 1u));
                out->valid = (out->crc_rx == out->crc_calc);
                crsf_parser_reset();
                if (out->valid) {
                    g_crsfValidCount++;
                    return CRSF_OK;
                }
                g_crsfCrcErrCount++;
                return CRSF_ERR_CRC;
            }
            break;

        default:
            crsf_parser_reset();
            break;
        }
    }

    return CRSF_ERR_NONE;
}

/* ------------------------------------------------------------------------- */
/* crsf_read_frame                                                            */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_read_frame(crsf_frame_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return CRSF_ERR_UART;
    }

    uint32_t t0 = HAL_GetTick();
    for (;;) {
        crsf_status_t st = crsf_poll_frame(out);
        if (st == CRSF_OK || st == CRSF_ERR_CRC || st == CRSF_ERR_BAD_LEN ||
            st == CRSF_ERR_UART) {
            if (st == CRSF_ERR_CRC) {
                /* Seguir buscando hasta timeout si el CRC fallo. */
                if ((HAL_GetTick() - t0) >= timeout_ms) {
                    return CRSF_ERR_CRC;
                }
                continue;
            }
            return st;
        }
        if ((HAL_GetTick() - t0) >= timeout_ms) {
            return CRSF_ERR_TIMEOUT;
        }
    }
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

/* ------------------------------------------------------------------------- */
/* crsf_send_frame                                                            */
/* ------------------------------------------------------------------------- */
crsf_status_t crsf_send_frame(uint8_t type, const uint8_t *payload, uint8_t payload_len)
{
    if (payload_len > CRSF_MAX_PAYLOAD) {
        return CRSF_ERR_BAD_LEN;
    }
    if (payload_len > 0u && payload == NULL) {
        return CRSF_ERR_UART;
    }

    uint8_t frame[64];
    const uint8_t len = (uint8_t)(payload_len + 2u); /* type + crc */
    uint8_t idx = 0u;

    frame[idx++] = CRSF_ADDR_FC;
    frame[idx++] = len;
    frame[idx++] = type;
    if (payload_len > 0u) {
        memcpy(&frame[idx], payload, payload_len);
        idx = (uint8_t)(idx + payload_len);
    }
    /* CRC sobre [type + payload] = frame[2 .. idx-1] */
    frame[idx++] = crsf_crc8(&frame[2], (uint16_t)(payload_len + 1u));

    if (HAL_UART_Transmit(&huart4, frame, idx, CRSF_TX_TIMEOUT_MS) != HAL_OK) {
        return CRSF_ERR_UART;
    }
    g_crsfTxCount++;
    return CRSF_OK;
}

/* ------------------------------------------------------------------------- */
/* Helpers de telemetria                                                      */
/* ------------------------------------------------------------------------- */
static int16_t crsf_rad_to_centirad(float rad)
{
    /* Clamp a -pi..+pi y escala * 10000. */
    const float pi = 3.14159265f;
    while (rad > pi) {
        rad -= 2.0f * pi;
    }
    while (rad < -pi) {
        rad += 2.0f * pi;
    }
    float scaled = rad * 10000.0f;
    if (scaled > 32767.0f) {
        scaled = 32767.0f;
    }
    if (scaled < -32768.0f) {
        scaled = -32768.0f;
    }
    return (int16_t)lroundf(scaled);
}

crsf_status_t crsf_send_attitude(float pitch_rad, float roll_rad, float yaw_rad)
{
    uint8_t payload[6];
    crsf_write_be16(&payload[0], (uint16_t)crsf_rad_to_centirad(pitch_rad));
    crsf_write_be16(&payload[2], (uint16_t)crsf_rad_to_centirad(roll_rad));
    crsf_write_be16(&payload[4], (uint16_t)crsf_rad_to_centirad(yaw_rad));
    return crsf_send_frame(CRSF_TYPE_ATTITUDE, payload, 6u);
}

crsf_status_t crsf_send_baro_altitude(int32_t altitude_dm)
{
    uint8_t payload[2];
    crsf_write_be16(payload, crsf_pack_altitude(altitude_dm));
    return crsf_send_frame(CRSF_TYPE_BARO_ALTITUDE, payload, 2u);
}

crsf_status_t crsf_send_vario(int16_t vspeed_cm_s)
{
    uint8_t payload[2];
    crsf_write_be16(payload, (uint16_t)vspeed_cm_s);
    return crsf_send_frame(CRSF_TYPE_VARIO, payload, 2u);
}
