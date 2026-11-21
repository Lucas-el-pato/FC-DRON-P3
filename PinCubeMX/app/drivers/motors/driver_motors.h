/**
 ******************************************************************************
 * @file    driver_motors.h
 * @brief   Driver de motores ESC en protocolo DShot300/600 usando TIM2 + DMA.
 *          Con motors_init_all_bidir() usa en cambio el DShot bidireccional
 *          de driver_dshot_bb.c (TIM1 + DMA2), con la misma API de 4 motores.
 *
 *          Pines (indice/ESC = canal TIM2; Mx = etiqueta de la PCB):
 *            CH1 = M4 = PA15
 *            CH2 = M3 = PB3
 *            CH3 = M2 = PA2
 *            CH4 = M1 = PA3
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
/*  select_esc: 1=CH1 .. 4=CH4. Solo ese canal emite DShot; el resto en 0.    */
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

/* ------------------------------------------------------------------------- */
/* Envia un comando DShot (1..47) a los ESC de esc_mask (bit0=CH1 .. bit3=CH4)*/
/* en un solo frame; los demas reciben throttle 0. No bloquea, como write4.   */
/* Comandos de configuracion (7, 8, 12...) requieren ~10 frames seguidos con  */
/* el motor parado; repetirlos es responsabilidad del llamador.               */
/* ------------------------------------------------------------------------- */
#define MOTORS_DSHOT_CMD_SPIN_DIRECTION_NORMAL   7u
#define MOTORS_DSHOT_CMD_SPIN_DIRECTION_REVERSED 8u
#define MOTORS_DSHOT_CMD_SAVE_SETTINGS           12u

mot_status_t motors_send_command4(uint16_t cmd, uint8_t esc_mask);

/* true mientras el DMA del frame anterior no termino. */
bool motors_output_busy(void);

/* Manda un frame de throttle 0 a los 4 motores (parada inmediata). */
mot_status_t motors_disarm_all(void);

/* Errores de DMA acumulados en el modo 4 motores (diagnostico). */
uint32_t motors_dma_error_count(void);

/* ------------------------------------------------------------------------- */
/* DShot BIDIRECCIONAL (driver_dshot_bb.c): TIM1 + DMA2 escriben y leen los   */
/* puertos GPIO; cada ESC contesta su eRPM despues de cada frame.            */
/* ------------------------------------------------------------------------- */

typedef struct {
    uint32_t frames;                    /* ciclos envio + lectura decodificados */
    uint32_t ok[MOTORS_COUNT];          /* respuestas validas                   */
    uint32_t bad[MOTORS_COUNT];         /* hubo respuesta pero GCR/CRC/largo mal */
    uint32_t no_reply[MOTORS_COUNT];    /* ningun flanco en la ventana          */
    uint32_t erpm[MOTORS_COUNT];        /* ultima eRPM valida (0 = parado)      */
    uint8_t  last_ok_mask;              /* bit i: valida en el ultimo ciclo     */
    uint32_t dma_errors;
    uint32_t stuck;                     /* ciclos trabados que se cortaron      */
} motors_bidir_stats_t;

/* Como motors_init_all, pero con DShot bidireccional (solo DShot300). Deja
 * de usar TIM2: los pines pasan a GPIO. write4 / send_command4 / busy /
 * disarm_all funcionan igual. motors_init_all() vuelve al modo TIM2.
 * Arranca RETENIDO (lineas en bajo, sin frames): hay que llamar a
 * motors_bidir_hold(false) cuando los ESC ya arrancaron (ver
 * dshot_bb_hold: el bootloader de AM32 no arranca con la linea en alto). */
mot_status_t motors_init_all_bidir(motors_protocol_t protocol);

/* Decodifica la respuesta del ultimo frame si ya llego. Llamarla en una
 * muestra de gyro sin PID (si no, la decodifica el proximo write4).     */
void motors_bidir_service(void);

/* true = lineas en bajo y sin frames (write4 devuelve MOT_ERR_INIT). */
void motors_bidir_hold(bool hold);
bool motors_bidir_held(void);

bool motors_bidir_active(void);

/* NULL si el bidireccional no esta activo. */
const motors_bidir_stats_t *motors_bidir_stats(void);

/* Helpers de consulta (utiles para el log del test). */
motors_protocol_t motors_get_protocol(void);
uint8_t motors_get_select_esc(void);
const char *motors_protocol_name(motors_protocol_t protocol);
const char *motors_esc_pin_name(uint8_t select_esc);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_MOTORS_H_ */
