/**
 ******************************************************************************
 * @file    driver_motors.c
 * @brief   Driver DShot600 a 4 motores con TIM2 + DMA burst.
 *
 *          Estrategia DMA burst:
 *            El TIM2 expone un registro "DMAR" (DMA Address for full transfer)
 *            que cuando es escrito reenvia los datos al bloque CCR1..CCR4
 *            (con DCR.DBA = TIM_DMABase_CCR1 y DCR.DBL = 4 transferencias).
 *            Por cada evento de TIM_UP, el DMA escribe 4 palabras de 32 bits
 *            consecutivas desde RAM al rango CCR1..CCR4.
 *            Asi un solo stream de DMA alimenta los 4 canales simultaneamente.
 *
 *          Sequencia tipica de una llamada motors_set_throttle():
 *            1. Para el timer (HAL_TIM_PWM_Stop / DMA_Stop).
 *            2. Calcula los 16 bits DShot por motor (con CRC).
 *            3. Llena dshot_buf[17][4] con CCR=50 o 100 segun cada bit.
 *            4. Habilita el DMA en burst y arranca el timer.
 *            5. Cuando el DMA termina el frame, el timer sigue corriendo
 *               con CCR=0 (linea baja), respetando el gap inter-frame.
 ******************************************************************************
 */

#include "driver_motors.h"
#include "tim.h"
#include "stm32f4xx_hal.h"

#include <string.h>

extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_up_ch3;

/* Buffer DMA: 17 muestras x 4 canales (palabras de 32 bits).                */
/* Posicion [i][ch]: valor del CCR para el bit i del motor ch.               */
/* Memoria alineada por el compilador (uint32_t).                            */
static uint32_t dshot_buf[MOTORS_DMA_LEN][MOTORS_COUNT];

/* Flag interno: indica si motors_init() fue invocado.                       */
static bool dshot_inited = false;

/* ------------------------------------------------------------------------- */
/* Construye el frame DShot de 16 bits a partir del valor (0..2047) y el     */
/* flag de telemetria.                                                       */
/* ------------------------------------------------------------------------- */
static uint16_t dshot_make_packet(uint16_t value, bool telemetry)
{
    if (value > 2047u) value = 2047u;

    uint16_t packet = (uint16_t)(value << 1) | (telemetry ? 1u : 0u);
    uint16_t crc = (uint16_t)((packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0Fu);
    return (uint16_t)((packet << 4) | crc);
}

/* ------------------------------------------------------------------------- */
/* Llena dshot_buf con los CCR de cada bit para los 4 motores.               */
/* MSB primero: el bit 15 del frame se transmite primero.                    */
/* ------------------------------------------------------------------------- */
static void dshot_fill_buffer(uint16_t f1, uint16_t f2, uint16_t f3, uint16_t f4)
{
    const uint16_t frames[MOTORS_COUNT] = { f1, f2, f3, f4 };

    for (uint8_t bit = 0u; bit < MOTORS_DSHOT_FRAME_BITS; ++bit) {
        /* En posicion [bit] del buffer: el bit MSB (frame >> 15) primero.    */
        uint8_t shift = (uint8_t)(15u - bit);
        for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
            uint16_t bitval = (uint16_t)((frames[ch] >> shift) & 0x1u);
            dshot_buf[bit][ch] = (bitval != 0u) ? MOTORS_DSHOT_BIT1_CCR
                                                : MOTORS_DSHOT_BIT0_CCR;
        }
    }
    /* Posicion 16: gap final con linea baja. */
    for (uint8_t ch = 0u; ch < MOTORS_COUNT; ++ch) {
        dshot_buf[MOTORS_DMA_LEN - 1u][ch] = 0u;
    }
}

/* ------------------------------------------------------------------------- */
/* Reconfigura TIM2 para DShot (PSC=0, ARR=139) sin re-init completo.        */
/* ------------------------------------------------------------------------- */
static mot_status_t dshot_setup_timer(void)
{
    /* Para por las dudas. */
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);

    /* Reconfiguracion del periodo. */
    __HAL_TIM_SET_PRESCALER(&htim2, MOTORS_DSHOT_PSC);
    __HAL_TIM_SET_AUTORELOAD(&htim2, MOTORS_DSHOT_ARR);

    /* Pulso inicial = 0 (linea baja). */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);

    /* Arranca los 4 canales en PWM (sin DMA todavia). */
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK) return MOT_ERR_INIT;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK) return MOT_ERR_INIT;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) return MOT_ERR_INIT;
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK) return MOT_ERR_INIT;

    return MOT_OK;
}

/* ------------------------------------------------------------------------- */
/* Envia un frame DShot completo bloqueante.                                  */
/* ------------------------------------------------------------------------- */
static mot_status_t dshot_send_frame_blocking(void)
{
    /* Detiene cualquier DMA previo. */
    HAL_DMA_Abort(&hdma_tim2_up_ch3);

    /* Configura DMA burst del TIM2:                                          */
    /*   DBA = CCR1, DBL = 4 transferencias (CCR1..CCR4).                     */
    /* En HAL, la llamada que setea esto es HAL_TIM_DMABurst_WriteStart, pero */
    /* aca lo armamos a mano para tener control fino.                         */
    MODIFY_REG(htim2.Instance->DCR,
               (TIM_DCR_DBA | TIM_DCR_DBL),
               (TIM_DMABASE_CCR1 | TIM_DMABURSTLENGTH_4TRANSFERS));

    /* Lanza el DMA: orig = dshot_buf, dest = &TIMx->DMAR, len = 17*4 = 68
     * palabras de 32 bits (porque el DMA esta configurado MINC + Word).      */
    HAL_StatusTypeDef st = HAL_DMA_Start(&hdma_tim2_up_ch3,
                                         (uint32_t)dshot_buf,
                                         (uint32_t)&htim2.Instance->DMAR,
                                         MOTORS_DMA_LEN * MOTORS_COUNT);
    if (st != HAL_OK) {
        return MOT_ERR_DMA;
    }

    /* Habilita el evento de update -> DMA en el TIM2. */
    __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_UPDATE);

    /* Espera a que el DMA termine (frame ~28 us a 600 kbit/s).               */
    st = HAL_DMA_PollForTransfer(&hdma_tim2_up_ch3, HAL_DMA_FULL_TRANSFER, 5u);

    /* Deshabilita el request DMA del timer. */
    __HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_UPDATE);
    HAL_DMA_Abort(&hdma_tim2_up_ch3);

    /* Aseguramos linea baja al salir. */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0u);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0u);

    return (st == HAL_OK) ? MOT_OK : MOT_ERR_DMA;
}

/* ------------------------------------------------------------------------- */
/* motors_init                                                                */
/* ------------------------------------------------------------------------- */
mot_status_t motors_init(void)
{
    memset(dshot_buf, 0, sizeof(dshot_buf));

    mot_status_t st = dshot_setup_timer();
    if (st != MOT_OK) {
        return st;
    }

    dshot_inited = true;

    /* Secuencia de armado: enviar throttle=0 (comando "motor stop") durante
     * ~1 segundo. La frecuencia de refresco DShot conviene >= 1 kHz; aca
     * mandamos un frame cada 1 ms.                                            */
    for (uint16_t i = 0u; i < 1000u; ++i) {
        st = motors_send_command(0u);
        if (st != MOT_OK) {
            return st;
        }
        HAL_Delay(1u);
    }
    return MOT_OK;
}

/* ------------------------------------------------------------------------- */
/* motors_set_throttle                                                        */
/* ------------------------------------------------------------------------- */
mot_status_t motors_set_throttle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
    if (!dshot_inited) {
        return MOT_ERR_INIT;
    }

    /* Clamp: si esta entre 1 y 47, DShot lo interpreta como comando especial.
     * Para evitar movimiento accidental por valores chicos, lo forzamos a 0
     * o lo subimos a 48.                                                     */
    const uint16_t clamp = (uint16_t)((m1 == 0u) ? 0u : (m1 < 48u ? 48u : (m1 > 2047u ? 2047u : m1)));
    const uint16_t v1 = clamp;
    const uint16_t v2 = (m2 == 0u) ? 0u : (m2 < 48u ? 48u : (m2 > 2047u ? 2047u : m2));
    const uint16_t v3 = (m3 == 0u) ? 0u : (m3 < 48u ? 48u : (m3 > 2047u ? 2047u : m3));
    const uint16_t v4 = (m4 == 0u) ? 0u : (m4 < 48u ? 48u : (m4 > 2047u ? 2047u : m4));

    uint16_t f1 = dshot_make_packet(v1, false);
    uint16_t f2 = dshot_make_packet(v2, false);
    uint16_t f3 = dshot_make_packet(v3, false);
    uint16_t f4 = dshot_make_packet(v4, false);

    dshot_fill_buffer(f1, f2, f3, f4);
    return dshot_send_frame_blocking();
}

/* ------------------------------------------------------------------------- */
/* motors_send_command                                                        */
/* ------------------------------------------------------------------------- */
mot_status_t motors_send_command(uint16_t cmd)
{
    if (cmd > 47u) {
        return MOT_ERR_INIT;
    }

    uint16_t f = dshot_make_packet(cmd, false);
    dshot_fill_buffer(f, f, f, f);
    return dshot_send_frame_blocking();
}

/* ------------------------------------------------------------------------- */
/* motors_stop                                                                */
/* ------------------------------------------------------------------------- */
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
