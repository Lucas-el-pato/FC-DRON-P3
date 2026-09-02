/**
 ******************************************************************************
 * @file    driver_motors.c
 * @brief   Driver DShot300/600 a un ESC seleccionado con TIM2 + DMA burst.
 *
 *          Estrategia DMA burst:
 *            El TIM2 expone DMAR; con DCR.DBA = CCR1 y DCR.DBL = 4, cada
 *            evento TIM_UP escribe CCR1..CCR4 desde RAM.
 *            Solo el canal del ESC seleccionado lleva CCR bit0/bit1; el resto
 *            queda en 0 (linea baja continua).
 ******************************************************************************
 */

#include "driver_motors.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

#include <string.h>

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_up_ch3;

/* Buffer DMA: 17 muestras x 4 canales (palabras de 32 bits). */
static uint32_t dshot_buf[MOTORS_DMA_LEN][MOTORS_COUNT];

static bool dshot_inited = false;
/* true = modo 4 motores (motors_init_all); false = un solo ESC. */
static bool dshot_all_mode = false;
static volatile bool dshot_busy = false;
static volatile uint32_t dshot_dma_errors = 0u;
static motors_protocol_t dshot_protocol = MOTORS_PROTO_DSHOT300;
static uint8_t dshot_select_esc = 1u; /* 1..4 */
static uint16_t dshot_arr = MOTORS_DSHOT300_ARR;
static uint16_t dshot_bit0 = MOTORS_DSHOT300_BIT0_CCR;
static uint16_t dshot_bit1 = MOTORS_DSHOT300_BIT1_CCR;

static void dshot_apply_protocol(motors_protocol_t protocol)
{
    if (protocol == MOTORS_PROTO_DSHOT600) {
        dshot_protocol = MOTORS_PROTO_DSHOT600;
        dshot_arr = MOTORS_DSHOT600_ARR;
        dshot_bit0 = MOTORS_DSHOT600_BIT0_CCR;
        dshot_bit1 = MOTORS_DSHOT600_BIT1_CCR;
    } else {
        dshot_protocol = MOTORS_PROTO_DSHOT300;
        dshot_arr = MOTORS_DSHOT300_ARR;
        dshot_bit0 = MOTORS_DSHOT300_BIT0_CCR;
        dshot_bit1 = MOTORS_DSHOT300_BIT1_CCR;
    }
}

static uint16_t dshot_make_packet(uint16_t value, bool telemetry)
{
    if (value > 2047u) {
        value = 2047u;
    }

    uint16_t packet = (uint16_t)(value << 1) | (telemetry ? 1u : 0u);
    uint16_t crc = (uint16_t)((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0Fu);
    return (uint16_t)((packet << 4) | crc);
}

/* Escribe los 16 bits de un frame en la columna de un canal (0..3). */
static void dshot_fill_channel(uint16_t frame, uint8_t ch)
{
    for (uint8_t bit = 0u; bit < MOTORS_DSHOT_FRAME_BITS; ++bit) {
        const uint8_t shift = (uint8_t)(15u - bit);
        const uint16_t bitval = (uint16_t)((frame >> shift) & 0x1u);
        dshot_buf[bit][ch] = (bitval != 0u) ? dshot_bit1 : dshot_bit0;
    }
}

/* Deja la columna de un canal en bajo continuo. */
static void dshot_clear_channel(uint8_t ch)
{
    for (uint8_t bit = 0u; bit < MOTORS_DSHOT_FRAME_BITS; ++bit) {
        dshot_buf[bit][ch] = 0u;
    }
}

/* Ultima fila: idle bajo comun a los 4 canales. */
static void dshot_fill_idle_row(void)
{
    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        dshot_buf[MOTORS_DMA_LEN - 1u][ch] = 0u;
    }
}

/* selected_ch: 0..3. Solo ese canal lleva el frame; los demas CCR=0. */
static void dshot_fill_buffer_selected(uint16_t frame, uint8_t selected_ch)
{
    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        if (ch == selected_ch) {
            dshot_fill_channel(frame, ch);
        } else {
            dshot_clear_channel(ch);
        }
    }
    dshot_fill_idle_row();
}

/* Clamp comun: 0 = parado, 1..47 son comandos DShot, tope 2047. */
static uint16_t dshot_clamp_throttle(uint16_t throttle)
{
    if (throttle == 0u) {
        return 0u;
    }
    if (throttle < 48u) {
        return 48u;
    }
    if (throttle > 2047u) {
        return 2047u;
    }
    return throttle;
}

static void dshot_ccr_zero(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);
}

/* Fin de transferencia del frame de 4 motores (contexto DMA1_Stream1 IRQ). */
static void dshot_dma_cplt(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
    dshot_ccr_zero();
    dshot_busy = false;
}

static void dshot_dma_error(DMA_HandleTypeDef *hdma)
{
    (void)hdma;
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
    dshot_ccr_zero();
    dshot_busy = false;
    dshot_dma_errors++;
}

static mot_status_t dshot_setup_timer(void)
{
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

    __HAL_TIM_SET_PRESCALER(&htim2, MOTORS_DSHOT_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim2, dshot_arr);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) return MOT_ERR_INIT;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) return MOT_ERR_INIT;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) return MOT_ERR_INIT;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK) return MOT_ERR_INIT;

    return MOT_OK;
}

static mot_status_t dshot_send_frame_blocking(void)
{
    HAL_DMA_Abort(&hdma_tim2_up_ch3);

    MODIFY_REG(htim2.Instance->DCR,
               (TIM_DCR_DBA | TIM_DCR_DBL),
               (TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS));

    HAL_StatusTypeDef st = HAL_DMA_Start(&hdma_tim2_up_ch3,
                                         (uint32_t)dshot_buf,
                                         (uint32_t)&htim2.Instance->DMAR,
                                         MOTORS_DMA_LEN * MOTORS_COUNT);
    if (st != HAL_OK) {
        return MOT_ERR_DMA;
    }

    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);

    /* Frame ~57 us (DShot300) / ~28 us (DShot600); 5 ms es margen amplio. */
    st = HAL_DMA_PollForTransfer(&hdma_tim2_up_ch3, HAL_DMA_FULL_TRANSFER, 5u);

    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
    HAL_DMA_Abort(&hdma_tim2_up_ch3);
    dshot_ccr_zero();

    return (st == HAL_OK) ? MOT_OK : MOT_ERR_DMA;
}

static uint8_t dshot_selected_channel(void)
{
    return (uint8_t)(dshot_select_esc - 1u);
}

mot_status_t motors_init(motors_protocol_t protocol, uint8_t select_esc)
{
    if (select_esc < 1u || select_esc > MOTORS_COUNT) {
        return MOT_ERR_PARAM;
    }
    if (protocol != MOTORS_PROTO_DSHOT300 && protocol != MOTORS_PROTO_DSHOT600) {
        return MOT_ERR_PARAM;
    }

    dshot_apply_protocol(protocol);
    dshot_select_esc = select_esc;
    memset(dshot_buf, 0, sizeof(dshot_buf));

    HAL_DMA_Abort(&hdma_tim2_up_ch3);
    dshot_busy = false;
    dshot_all_mode = false;
    /* Vuelta al modo bloqueante: no deben quedar callbacks del modo 4 motores. */
    (void)HAL_DMA_UnRegisterCallback(&hdma_tim2_up_ch3, HAL_DMA_XFER_CPLT_CB_ID);
    (void)HAL_DMA_UnRegisterCallback(&hdma_tim2_up_ch3, HAL_DMA_XFER_ERROR_CB_ID);

    mot_status_t st = dshot_setup_timer();
    if (st != MOT_OK) {
        return st;
    }

    dshot_inited = true;
    return MOT_OK;
}

mot_status_t motors_init_all(motors_protocol_t protocol)
{
    if (protocol != MOTORS_PROTO_DSHOT300 && protocol != MOTORS_PROTO_DSHOT600) {
        return MOT_ERR_PARAM;
    }

    dshot_apply_protocol(protocol);
    dshot_select_esc = 0u;              /* 0 = los 4 canales */
    memset(dshot_buf, 0, sizeof(dshot_buf));

    HAL_DMA_Abort(&hdma_tim2_up_ch3);
    dshot_busy = false;
    dshot_dma_errors = 0u;

    /* El IRQ de DMA1_Stream1 ya lo habilita MX_DMA_Init(); solo falta decirle
     * a HAL que llame a nuestros callbacks al terminar cada frame.          */
    if (HAL_DMA_RegisterCallback(&hdma_tim2_up_ch3, HAL_DMA_XFER_CPLT_CB_ID,
                                 dshot_dma_cplt) != HAL_OK) {
        return MOT_ERR_DMA;
    }
    if (HAL_DMA_RegisterCallback(&hdma_tim2_up_ch3, HAL_DMA_XFER_ERROR_CB_ID,
                                 dshot_dma_error) != HAL_OK) {
        return MOT_ERR_DMA;
    }

    mot_status_t st = dshot_setup_timer();
    if (st != MOT_OK) {
        return st;
    }

    dshot_all_mode = true;
    dshot_inited = true;
    return MOT_OK;
}

mot_status_t motors_write4(const uint16_t thr[MOTORS_COUNT], uint8_t telem_mask)
{
    if (!dshot_inited || !dshot_all_mode) {
        return MOT_ERR_INIT;
    }
    if (thr == NULL) {
        return MOT_ERR_PARAM;
    }
    if (dshot_busy) {
        /* El frame anterior sigue en vuelo: se descarta este. El ESC mantiene
         * el ultimo valor recibido hasta el proximo frame valido.           */
        return MOT_ERR_DMA;
    }

    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        const uint16_t value = dshot_clamp_throttle(thr[ch]);
        const bool telem = ((telem_mask >> ch) & 0x1u) != 0u;
        dshot_fill_channel(dshot_make_packet(value, telem), ch);
    }
    dshot_fill_idle_row();

    MODIFY_REG(htim2.Instance->DCR,
               (TIM_DCR_DBA | TIM_DCR_DBL),
               (TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS));

    dshot_busy = true;
    if (HAL_DMA_Start_IT(&hdma_tim2_up_ch3,
                         (uint32_t)dshot_buf,
                         (uint32_t)&htim2.Instance->DMAR,
                         MOTORS_DMA_LEN * MOTORS_COUNT) != HAL_OK) {
        dshot_busy = false;
        return MOT_ERR_DMA;
    }

    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);
    return MOT_OK;
}

uint32_t motors_dma_error_count(void)
{
    return dshot_dma_errors;
}

bool motors_output_busy(void)
{
    return dshot_busy;
}

mot_status_t motors_disarm_all(void)
{
    static const uint16_t zeros[MOTORS_COUNT] = { 0u, 0u, 0u, 0u };

    if (!dshot_inited || !dshot_all_mode) {
        return MOT_ERR_INIT;
    }

    /* Si hay un frame en vuelo se aborta: parar tiene prioridad. */
    if (dshot_busy) {
        HAL_DMA_Abort(&hdma_tim2_up_ch3);
        __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
        dshot_ccr_zero();
        dshot_busy = false;
    }
    return motors_write4(zeros, 0u);
}

mot_status_t motors_set_throttle_telem(uint16_t throttle, bool request_telem)
{
    if (!dshot_inited || dshot_all_mode) {
        return MOT_ERR_INIT;
    }

    /* Clamp: 1..47 son comandos especiales; forzar a 0 o subir a 48. */
    const uint16_t value = dshot_clamp_throttle(throttle);
    uint16_t frame = dshot_make_packet(value, request_telem);
    dshot_fill_buffer_selected(frame, dshot_selected_channel());
    return dshot_send_frame_blocking();
}

mot_status_t motors_set_throttle(uint16_t throttle)
{
    return motors_set_throttle_telem(throttle, false);
}

mot_status_t motors_send_command(uint16_t cmd)
{
    if (!dshot_inited || dshot_all_mode) {
        return MOT_ERR_INIT;
    }
    if (cmd > 47u) {
        return MOT_ERR_PARAM;
    }

    uint16_t frame = dshot_make_packet(cmd, false);
    dshot_fill_buffer_selected(frame, dshot_selected_channel());
    return dshot_send_frame_blocking();
}

mot_status_t motors_stop(void)
{
    HAL_DMA_Abort(&hdma_tim2_up_ch3);
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
    dshot_busy = false;
    dshot_ccr_zero();

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

    dshot_inited = false;
    dshot_all_mode = false;
    return MOT_OK;
}

motors_protocol_t motors_get_protocol(void)
{
    return dshot_protocol;
}

uint8_t motors_get_select_esc(void)
{
    return dshot_select_esc;
}

const char *motors_protocol_name(motors_protocol_t protocol)
{
    return (protocol == MOTORS_PROTO_DSHOT600) ? "DShot600" : "DShot300";
}

const char *motors_esc_pin_name(uint8_t select_esc)
{
    switch (select_esc) {
    case 1: return "M1/PA15";
    case 2: return "M2/PB3";
    case 3: return "M3/PA2";
    case 4: return "M4/PA3";
    default: return "invalid";
    }
}
