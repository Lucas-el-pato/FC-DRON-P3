/**
 ******************************************************************************
 * @file    telemtry.c
 * @brief   Receptor KISS/AM32 ESC serial telemetry por USART6 RX (PC7).
 *
 *          La recepcion es por DMA circular (DMA2_Stream1_CH5 = USART6_RX).
 *          El USART6 del F4 no tiene FIFO: un frame KISS son 10 bytes seguidos
 *          (~870 us @115200) y cualquier bloqueo del CPU mayor a 87 us
 *          (HAL_Delay, console_printf por USB CDC, envio DShot) perderia bytes
 *          si se leyera DR por polling. Con DMA circular el hardware llena el
 *          ring y telemtry_poll() solo consume lo acumulado.
 ******************************************************************************
 */

#include "telemtry.h"
#include "usart.h"
#include "stm32f4xx_hal.h"

#include <string.h>

extern UART_HandleTypeDef huart6;

/* Ring de DMA: 128 bytes = ~11 ms de linea a 115200. */
#define TELEM_RING_LEN 128u

/* Un frame KISS son 10 bytes contiguos; un hueco mayor marca fin de frame. */
#define TELEM_GAP_MS 3u

/* Watch these in CubeIDE Expressions: g_telemRxBuf, g_telemRxLen, g_telemLast */
uint8_t         g_telemRxBuf[TELEMTRY_FRAME_LEN];
uint8_t         g_telemRxLen = 0u;
telemtry_data_t g_telemLast = {0};
int32_t         g_telemLastStatus = (int32_t)TELEMTRY_ERR_NONE;
volatile uint32_t g_telemDebugMagic = 0x54454C4Du; /* ASCII "TELM" */
volatile uint32_t g_telemPollCount = 0u;
volatile uint32_t g_telemByteCount = 0u;
volatile uint32_t g_telemValidCount = 0u;
volatile uint32_t g_telemCrcErrCount = 0u;
volatile uint32_t g_telemUartErrCount = 0u;
volatile uint16_t g_telemDmaWr = 0u;

static DMA_HandleTypeDef hdma_usart6_rx;
static uint8_t  telem_ring[TELEM_RING_LEN];
static uint16_t telem_rd = 0u;
static uint32_t telem_last_byte_ms = 0u;
static bool     telem_dma_running = false;

static void telemtry_check_uart_errors(void)
{
    /* En el F4 los flags de error solo se borran leyendo SR y luego DR, y esa
       lectura le roba el byte al DMA. Solo se hace si hay error real (en ese
       caso el dato ya se perdio). */
    const uint32_t sr = huart6.Instance->SR;
    if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0u) {
        (void)huart6.Instance->DR;
        g_telemUartErrCount++;
    }
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

telemtry_status_t telemtry_init(void)
{
    __HAL_RCC_DMA2_CLK_ENABLE();

    CLEAR_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    if (telem_dma_running) {
        (void)HAL_DMA_Abort(&hdma_usart6_rx);
        (void)HAL_DMA_DeInit(&hdma_usart6_rx);
        telem_dma_running = false;
    }

    hdma_usart6_rx.Instance = DMA2_Stream1;
    hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart6_rx) != HAL_OK) {
        g_telemLastStatus = (int32_t)TELEMTRY_ERR_UART;
        return TELEMTRY_ERR_UART;
    }

    memset(telem_ring, 0, sizeof(telem_ring));
    telem_rd = 0u;
    g_telemRxLen = 0u;
    telem_last_byte_ms = HAL_GetTick();

    if (HAL_DMA_Start(&hdma_usart6_rx,
                      (uint32_t)&huart6.Instance->DR,
                      (uint32_t)telem_ring,
                      TELEM_RING_LEN) != HAL_OK) {
        g_telemLastStatus = (int32_t)TELEMTRY_ERR_UART;
        return TELEMTRY_ERR_UART;
    }
    telem_dma_running = true;

    /* Vaciar DR y flags pendientes antes de conectar el DMA al USART. */
    (void)huart6.Instance->SR;
    (void)huart6.Instance->DR;

    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);

    g_telemLastStatus = (int32_t)TELEMTRY_ERR_NONE;
    return TELEMTRY_OK;
}

uint16_t telemtry_rx_pending(void)
{
    if (!telem_dma_running) {
        return 0u;
    }
    const uint16_t wr = (uint16_t)(TELEM_RING_LEN -
                                   __HAL_DMA_GET_COUNTER(&hdma_usart6_rx));
    return (uint16_t)((wr + TELEM_RING_LEN - telem_rd) % TELEM_RING_LEN);
}

telemtry_status_t telemtry_poll(telemtry_data_t *out)
{
    g_telemPollCount++;

    if (out == NULL) {
        g_telemLastStatus = (int32_t)TELEMTRY_ERR_PARAM;
        return TELEMTRY_ERR_PARAM;
    }

    out->valid = false;

    if (!telem_dma_running) {
        g_telemLastStatus = (int32_t)TELEMTRY_ERR_UART;
        return TELEMTRY_ERR_UART;
    }

    telemtry_check_uart_errors();

    uint16_t wr = (uint16_t)(TELEM_RING_LEN -
                             __HAL_DMA_GET_COUNTER(&hdma_usart6_rx));
    if (wr >= TELEM_RING_LEN) {
        wr = 0u;
    }
    g_telemDmaWr = wr;

    /* Frame parcial viejo: la linea quedo idle, no hay continuidad posible. */
    if ((telem_rd == wr) && (g_telemRxLen != 0u) &&
        ((HAL_GetTick() - telem_last_byte_ms) >= TELEM_GAP_MS)) {
        g_telemRxLen = 0u;
    }

    while (telem_rd != wr) {
        const uint8_t b = telem_ring[telem_rd];
        telem_rd = (uint16_t)((telem_rd + 1u) % TELEM_RING_LEN);
        telem_last_byte_ms = HAL_GetTick();
        g_telemByteCount++;

        if (g_telemRxLen < TELEMTRY_FRAME_LEN) {
            g_telemRxBuf[g_telemRxLen++] = b;
        }

        if (g_telemRxLen < TELEMTRY_FRAME_LEN) {
            continue;
        }

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
        g_telemCrcErrCount++;
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
