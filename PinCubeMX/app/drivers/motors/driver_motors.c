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

/* selected_ch: 0..3. Solo ese canal lleva el frame; los demas CCR=0. */
static void dshot_fill_buffer_selected(uint16_t frame, uint8_t selected_ch)
{
    for (uint8_t bit = 0u; bit < MOTORS_DSHOT_FRAME_BITS; ++bit) {
        uint8_t shift = (uint8_t)(15u - bit);
        for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
            if (ch != selected_ch) {
                dshot_buf[bit][ch] = 0u;
                continue;
            }
            uint16_t bitval = (uint16_t)((frame >> shift) & 0x1u);
            dshot_buf[bit][ch] = (bitval != 0u) ? dshot_bit1 : dshot_bit0;
        }
    }
    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        dshot_buf[MOTORS_DMA_LEN - 1u][ch] = 0u;
    }
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

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);

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

    mot_status_t st = dshot_setup_timer();
    if (st != MOT_OK) {
        return st;
    }

    dshot_inited = true;
    return MOT_OK;
}

mot_status_t motors_set_throttle_telem(uint16_t throttle, bool request_telem)
{
    if (!dshot_inited) {
        return MOT_ERR_INIT;
    }

    /* Clamp: 1..47 son comandos especiales; forzar a 0 o subir a 48. */
    uint16_t value;
    if (throttle == 0u) {
        value = 0u;
    } else if (throttle < 48u) {
        value = 48u;
    } else if (throttle > 2047u) {
        value = 2047u;
    } else {
        value = throttle;
    }

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
    if (!dshot_inited) {
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

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);

    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

    dshot_inited = false;
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
