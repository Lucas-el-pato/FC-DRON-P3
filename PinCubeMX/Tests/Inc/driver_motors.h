/**
 ******************************************************************************
 * @file    driver_motors.h
 * @brief   Driver de motores ESC en protocolo DShot600 usando TIM2 + DMA burst.
 *
 *          Pines:
 *            M1 = PA15 (TIM2_CH1)
 *            M2 = PB3  (TIM2_CH2)
 *            M3 = PA2  (TIM2_CH3)
 *            M4 = PA3  (TIM2_CH4)
 *
 *          Configuracion DShot600:
 *            - Bit rate: 600 kbit/s -> bit period = 1.667 us
 *            - APB1 timer clock = 84 MHz
 *            - PSC = 0, ARR = 139  =>  bit period = 140 / 84 MHz = 1.667 us
 *            - Bit "0": CCR = 50  (~36 % de ARR)
 *            - Bit "1": CCR = 100 (~71 % de ARR)
 *
 *          Frame DShot (16 bits):
 *            [ 11 bits throttle | 1 bit telemetry req | 4 bits CRC ]
 *            CRC4 = ( (val) ^ (val >> 4) ^ (val >> 8) ) & 0xF
 *              donde val = (throttle << 1) | telemetry
 *
 *          Transferencia DMA burst:
 *            - Una matriz de 17 palabras de 32 bits (17 = 16 bits de datos +
 *              1 bit "low" final para garantizar el pulso bajo de reset).
 *            - Cada palabra contiene los CCR de los 4 canales en formato
 *              packed: en realidad la cabecera DMA del TIM2 (DMABase=CCR1,
 *              DMABurstLength=4) escribe 4 registros consecutivos (CCR1..CCR4)
 *              por cada evento de update.
 *            - Esto significa que para cada uno de los 17 bits del frame,
 *              el DMA escribe 4 valores de 32 bits, cada uno conteniendo el
 *              CCR (50 o 100) que corresponde al bit que toca enviar a cada
 *              motor en ese instante.
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

/* Geometria DShot600 sobre 84 MHz. */
#define MOTORS_DSHOT_ARR        139u   /* TIM2 auto-reload */
#define MOTORS_DSHOT_PSC        0u
#define MOTORS_DSHOT_BIT0_CCR   50u    /* duty ~36% */
#define MOTORS_DSHOT_BIT1_CCR   100u   /* duty ~71% */
#define MOTORS_DSHOT_FRAME_BITS 16u
#define MOTORS_DMA_LEN          17u    /* 16 bits + 1 idle bajo */

/* Cantidad de motores. */
#define MOTORS_COUNT            4u

typedef enum {
    MOT_OK = 0,
    MOT_ERR_INIT,
    MOT_ERR_DMA
} mot_status_t;

/* ------------------------------------------------------------------------- */
/* Inicializa TIM2 para DShot600:                                             */
/*  - Reconfigura PSC = 0, ARR = 139                                          */
/*  - Arranca los 4 canales PWM en modo PWM1                                  */
/*  - Configura el DMA en modo burst (DMABase = CCR1, BurstLen = 4)           */
/*  - Envia frames "0" (throttle = 0) durante 1 segundo para que los ESCs     */
/*    se inicialicen y armen.                                                 */
/* ------------------------------------------------------------------------- */
mot_status_t motors_init(void);

/* ------------------------------------------------------------------------- */
/* Setea el throttle de los 4 motores [0..2047].                              */
/* DShot reserva 0..47 para comandos especiales; el throttle real arranca     */
/* en 48 y termina en 2047. La funcion clampea automaticamente.               */
/* ------------------------------------------------------------------------- */
mot_status_t motors_set_throttle(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);

/* ------------------------------------------------------------------------- */
/* Envia un comando especial DShot (0..47) a los 4 motores.                   */
/* ------------------------------------------------------------------------- */
mot_status_t motors_send_command(uint16_t cmd);

/* ------------------------------------------------------------------------- */
/* Detiene la generacion DShot y deja los pines en bajo.                      */
/* ------------------------------------------------------------------------- */
mot_status_t motors_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_MOTORS_H_ */
