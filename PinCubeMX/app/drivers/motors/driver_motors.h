/**
 ******************************************************************************
 * @file    driver_motors.h
 * @brief   Driver de motores ESC en protocolo DShot300/600 usando TIM2 + DMA.
 *
 *          Pines:
 *            M1 = PA15 (TIM2_CH1)
 *            M2 = PB3  (TIM2_CH2)
 *            M3 = PA2  (TIM2_CH3)
 *            M4 = PA3  (TIM2_CH4)
 *
 *          Reloj TIM2 = 84 MHz (APB1 timer clock), PSC = 0.
 *
 *          DShot300:
 *            - Bit period = 3.333 us -> ARR = 279 (280 ticks)
 *            - Bit "0": CCR = 100 (~36 %)
 *            - Bit "1": CCR = 200 (~71 %)
 *
 *          DShot600:
 *            - Bit period = 1.667 us -> ARR = 139 (140 ticks)
 *            - Bit "0": CCR = 50  (~36 %)
 *            - Bit "1": CCR = 100 (~71 %)
 *
 *          Frame DShot (16 bits):
 *            [ 11 bits throttle | 1 bit telemetry req | 4 bits CRC ]
 *            CRC4 = ( (val) ^ (val >> 4) ^ (val >> 8) ) & 0xF
 *              donde val = (throttle << 1) | telemetry
 *
 *          Solo el ESC seleccionado (SelectEsc 1..4) emite el frame DShot;
 *          los demas canales quedan con CCR = 0 (linea baja).
 *
 *          Transferencia DMA burst (modo NORMAL):
 *            - 17 palabras x 4 canales (16 bits de datos + 1 idle bajo)
 *            - DMABase = CCR1, BurstLength = 4 (CCR1..CCR4 por update)
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_MOTORS_H_
#define TESTS_INC_DRIVER_MOTORS_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Geometria comun. */
#define MOTORS_DSHOT_PSC        0u
#define MOTORS_DSHOT_FRAME_BITS 16u
#define MOTORS_DMA_LEN          17u    /* 16 bits + 1 idle bajo */
#define MOTORS_COUNT            4u

/* Perfiles de timing (TIM2 @ 84 MHz). */
#define MOTORS_DSHOT300_ARR      279u
#define MOTORS_DSHOT300_BIT0_CCR 100u
#define MOTORS_DSHOT300_BIT1_CCR 200u

#define MOTORS_DSHOT600_ARR      139u
#define MOTORS_DSHOT600_BIT0_CCR 50u
#define MOTORS_DSHOT600_BIT1_CCR 100u

typedef enum {
    MOTORS_PROTO_DSHOT300 = 0,
    MOTORS_PROTO_DSHOT600 = 1
} motors_protocol_t;

typedef enum {
    MOT_OK = 0,
    MOT_ERR_INIT,
    MOT_ERR_DMA,
    MOT_ERR_PARAM
} mot_status_t;

/* ------------------------------------------------------------------------- */
/* Inicializa TIM2 + DMA burst para el protocolo y ESC indicados.             */
/*  select_esc: 1=M1 .. 4=M4. Solo ese canal emite DShot; el resto queda en 0.*/
/*  No arma el ESC: el test debe enviar throttle=0 el tiempo necesario.       */
/* ------------------------------------------------------------------------- */
mot_status_t motors_init(motors_protocol_t protocol, uint8_t select_esc);

/* ------------------------------------------------------------------------- */
/* Setea throttle [0..2047] solo en el ESC seleccionado.                      */
/* DShot reserva 1..47 para comandos; valores 1..47 se clampean a 48.         */
/* Sin bit de telemetria (request_telem = false).                             */
/* ------------------------------------------------------------------------- */
mot_status_t motors_set_throttle(uint16_t throttle);

/* ------------------------------------------------------------------------- */
/* Igual que motors_set_throttle, con bit de telemetria DShot opcional.       */
/* request_telem=true pide un frame KISS en el pad T del ESC (USART6 RX).     */
/* ------------------------------------------------------------------------- */
mot_status_t motors_set_throttle_telem(uint16_t throttle, bool request_telem);

/* ------------------------------------------------------------------------- */
/* Envia un comando especial DShot (0..47) solo al ESC seleccionado.          */
/* ------------------------------------------------------------------------- */
mot_status_t motors_send_command(uint16_t cmd);

/* ------------------------------------------------------------------------- */
/* Detiene la generacion DShot y deja los pines en bajo.                      */
/* ------------------------------------------------------------------------- */
mot_status_t motors_stop(void);

/* ------------------------------------------------------------------------- */
/* MODO 4 MOTORES (usado por el mixer del FC).                                */
/*                                                                            */
/* motors_init_all() configura TIM2 + DMA burst para emitir DShot en los 4    */
/* canales a la vez. Invalida el modo de un solo ESC hasta un motors_init().  */
/* ------------------------------------------------------------------------- */
mot_status_t motors_init_all(motors_protocol_t protocol);

/* ------------------------------------------------------------------------- */
/* Escribe throttle [0..2047] en los 4 motores y arranca el DMA sin bloquear. */
/*  thr[i] : 0 = motor detenido; 1..47 se clampean a 48 (rango de comandos).  */
/*  telem_mask : bit i pide telemetria KISS al ESC i (0 = ninguno).           */
/* Devuelve MOT_ERR_DMA si el frame anterior sigue en vuelo (frame perdido).  */
/* ------------------------------------------------------------------------- */
mot_status_t motors_write4(const uint16_t thr[MOTORS_COUNT], uint8_t telem_mask);

/* true mientras el DMA del frame anterior no termino. */
bool motors_output_busy(void);

/* Manda un frame de throttle 0 a los 4 motores (parada inmediata). */
mot_status_t motors_disarm_all(void);

/* Errores de DMA acumulados en el modo 4 motores (diagnostico). */
uint32_t motors_dma_error_count(void);

/* Helpers de consulta (utiles para el log del test). */
motors_protocol_t motors_get_protocol(void);
uint8_t motors_get_select_esc(void);
const char *motors_protocol_name(motors_protocol_t protocol);
const char *motors_esc_pin_name(uint8_t select_esc);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_MOTORS_H_ */
