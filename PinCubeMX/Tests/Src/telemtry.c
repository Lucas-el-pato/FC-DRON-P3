/**
 ******************************************************************************
 * @file    telemtry.c
 * @brief   Receptor KISS/AM32 ESC serial telemetry por USART6 RX (PC7).
 ******************************************************************************
 */

#include "telemtry.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

#include <string.h>

extern UART_HandleTypeDef huart6;

/* Watch these in CubeIDE Expressions: g_telemRxBuf, g_telemRxLen, g_telemLast */
uint8_t         g_telemRxBuf[TELEMTRY_FRAME_LEN];
uint8_t         g_telemRxLen = 0u;
telemtry_data_t g_telemLast = {0};
int32_t         g_telemLastStatus = (int32_t)TELEMTRY_ERR_NONE;
volatile uint32_t g_telemDebugMagic = 0x54454C4Du; /* ASCII "TELM" */
volatile uint32_t g_telemPollCount = 0u;
volatile uint32_t g_telemByteCount = 0u;
volatile uint32_t g_telemValidCount = 0u;

static void telemtry_clear_uart_errors(void)
{
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_CLEAR_NEFLAG(&huart6);
    __HAL_UART_CLEAR_FEFLAG(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);
}

static void telemtry_flush_rx(void)
{
    uint8_t discard;
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET) {
        discard = (uint8_t)(huart6.Instance->DR & 0xFFu);
        (void)discard;
    }
    g_telemRxLen = 0u;
}

static uint8_t update_crc8(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u = (uint8_t)(crc ^ crc_seed);
    for (uint8_t i = 0u; i < 8u; ++i) {
        crc_u = (crc_u & 0x80u) ? (uint8_t)(0x07u ^ (uint8_t)(crc_u << 1))
                                : (uint8_t)(crc_u << 1);
    }
    return crc_u;
}

uint8_t telemtry_crc8(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0u;
    if (buf == NULL) {
        return 0u;
    }
    for (uint8_t i = 0u; i < len; ++i) {
        crc = update_crc8(buf[i], crc);
    }
    return crc;
}

static void telemtry_decode(const uint8_t *frame, telemtry_data_t *out)
{
    out->temperature_c = (int8_t)frame[0];
    out->voltage_cv = (uint16_t)(((uint16_t)frame[1] << 8) | frame[2]);
    out->current_ca = (uint16_t)(((uint16_t)frame[3] << 8) | frame[4]);
    out->consumption_mah = (uint16_t)(((uint16_t)frame[5] << 8) | frame[6]);
    out->erpm_div100 = (uint16_t)(((uint16_t)frame[7] << 8) | frame[8]);
    out->erpm = (uint32_t)out->erpm_div100 * 100u;
    if (TELEMTRY_MOTOR_POLES >= 2u) {
        out->rpm = out->erpm / (TELEMTRY_MOTOR_POLES / 2u);
    } else {
        out->rpm = out->erpm;
    }
    out->valid = true;
}

void telemtry_init(void)
{
    telemtry_clear_uart_errors();
    telemtry_flush_rx();
}

telemtry_status_t telemtry_poll(telemtry_data_t *out)
{
    g_telemPollCount++;

    if (out == NULL) {
        g_telemLastStatus = (int32_t)TELEMTRY_ERR_PARAM;
        return TELEMTRY_ERR_PARAM;
    }

    out->valid = false;

    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET) {
        if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE) ||
            __HAL_UART_GET_FLAG(&huart6, UART_FLAG_NE) ||
            __HAL_UART_GET_FLAG(&huart6, UART_FLAG_FE) ||
            __HAL_UART_GET_FLAG(&huart6, UART_FLAG_PE)) {
            telemtry_clear_uart_errors();
            g_telemRxLen = 0u;
        }

        uint8_t b = (uint8_t)(huart6.Instance->DR & 0xFFu);
        g_telemByteCount++;

        if (g_telemRxLen < TELEMTRY_FRAME_LEN) {
            g_telemRxBuf[g_telemRxLen++] = b;
        }

        if (g_telemRxLen < TELEMTRY_FRAME_LEN) {
            continue;
        }

        /* Frame completo: validar CRC. */
        if (telemtry_crc8(g_telemRxBuf, (uint8_t)(TELEMTRY_FRAME_LEN - 1u)) ==
            g_telemRxBuf[TELEMTRY_FRAME_LEN - 1u]) {
            telemtry_decode(g_telemRxBuf, out);
            g_telemLast = *out;
            g_telemRxLen = 0u;
            g_telemValidCount++;
            g_telemLastStatus = (int32_t)TELEMTRY_OK;
            return TELEMTRY_OK;
        }

        /* Desync: desplazar un byte e intentar de nuevo. */
        memmove(&g_telemRxBuf[0], &g_telemRxBuf[1], TELEMTRY_FRAME_LEN - 1u);
        g_telemRxLen = (uint8_t)(TELEMTRY_FRAME_LEN - 1u);
    }

    g_telemLastStatus = (int32_t)TELEMTRY_ERR_NONE;
    return TELEMTRY_ERR_NONE;
}

telemtry_status_t telemtry_read(telemtry_data_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        g_telemLastStatus = (int32_t)TELEMTRY_ERR_PARAM;
        return TELEMTRY_ERR_PARAM;
    }

    uint32_t start = HAL_GetTick();
    for (;;) {
        telemtry_status_t st = telemtry_poll(out);
        if (st == TELEMTRY_OK) {
            return TELEMTRY_OK;
        }
        if (st == TELEMTRY_ERR_PARAM || st == TELEMTRY_ERR_UART) {
            g_telemLastStatus = (int32_t)st;
            return st;
        }
        if ((HAL_GetTick() - start) >= timeout_ms) {
            g_telemLastStatus = (int32_t)TELEMTRY_ERR_TIMEOUT;
            return TELEMTRY_ERR_TIMEOUT;
        }
    }
}
