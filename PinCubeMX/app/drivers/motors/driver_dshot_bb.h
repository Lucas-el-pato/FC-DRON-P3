/**
 ******************************************************************************
 * @file    driver_dshot_bb.h
 * @brief   DShot300 bidireccional por "bitbang" (TIM1 + DMA2 sobre GPIO),
 *          el mismo metodo que dshot_bitbang de Betaflight, escrito de cero.
 *
 *          Lo usa driver_motors.c cuando se inicializa con
 *          motors_init_all_bidir(); el resto del FC sigue llamando a la API
 *          de driver_motors.h (motors_write4, motors_send_command4, ...).
 *
 *          Hardware (CubeMX):
 *            TIM1 CH2 / CH4 "Output Compare No Output": solo disparan DMA.
 *            DMA2 Stream2 (TIM1_CH2, canal 6) -> puerto GPIOA: M4, M2, M1
 *            DMA2 Stream4 (TIM1_CH4, canal 6) -> puerto GPIOB: M3
 *            IRQ de los dos streams en prioridad 0.
 *            Solo el DMA2 llega a los registros GPIO (el DMA1 solo ve APB1).
 *
 *          Reloj comun: TIM1 a 1.5 MHz (0.667 us por paso), sin cambiar el
 *          periodo entre envio y recepcion:
 *            envio      bit DShot300 = 5 pasos (3.333 us); 80 escrituras a
 *                       BSRR por puerto. Senal invertida (reposo en alto):
 *                       bit 0 = 2 pasos en bajo (40 %), bit 1 = 4 (80 %).
 *            recepcion  bit de respuesta (5/4 x 300 kbit) = 4 muestras de
 *                       IDR por bit; ventana de BB_RX_SAMPLES muestras.
 *
 *          Lineas en BAJO mientras el ESC arranca (dshot_bb_hold): si el
 *          bootloader de AM32 ve la linea en alto al encender, no arranca.
 *
 *          Ciclo (cada llamada a dshot_bb_write, 2 kHz):
 *            1. decodifica la respuesta del ciclo anterior si nadie lo hizo
 *               (normalmente ya la decodifico dshot_bb_service)
 *            2. arma el frame y lanza los 2 streams (pines en salida)
 *            3. IRQ fin de envio: pines a entrada con pull-up, mismos
 *               streams a lectura de IDR
 *            4. IRQ fin de lectura: pines a salida en alto, listo
 *            Envio + respuesta = ~180 us, entra en el periodo de 500 us.
 *
 *          Respuesta del ESC (eRPM): 21 bits con cambio de nivel = 1 (el
 *          primero es el de inicio), 20 bits GCR -> 16 bits
 *          [eee mmmmmmmmm cccc]: periodo electrico = m << e us, CRC
 *          invertido de 4 bits. 0x0FFF = motor parado.
 ******************************************************************************
 */

#ifndef DRIVERS_MOTORS_DRIVER_DSHOT_BB_H_
#define DRIVERS_MOTORS_DRIVER_DSHOT_BB_H_

#include "driver_motors.h"

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Pasos de TIM1 por bit enviado y muestras por bit recibido. */
#define DSHOT_BB_STEP_HZ         1500000u
#define DSHOT_BB_STEPS_PER_BIT   5u
#define DSHOT_BB_TX_LEN          (16u * DSHOT_BB_STEPS_PER_BIT)
#define DSHOT_BB_RX_PER_BIT      4u
/* Ventana de lectura: 180 x 0.667 us = 120 us. El ESC empieza a responder
 * ~30 us despues del frame y la respuesta dura 21 x 2.667 = 56 us.       */
#define DSHOT_BB_RX_SAMPLES      180u
/* Muestras que se saltean al principio (la respuesta nunca llega antes). */
#define DSHOT_BB_RX_SKIP         12u

/* Configura TIM1, los 2 streams y los pines. Solo DShot300. Arranca
 * RETENIDO: pines en salida, en BAJO, sin frames (ver dshot_bb_hold).     */
mot_status_t dshot_bb_init(void);

/* hold = true : corta el ciclo en curso y deja las lineas en BAJO, sin
 *               frames (dshot_bb_write devuelve MOT_ERR_INIT).
 * hold = false: lineas en alto (reposo del invertido) y vuelven los frames.
 * Por que: el bootloader de AM32 mira la linea los primeros ~40 ms despues
 * de encenderse el ESC; si la ve en alto se queda en el bootloader
 * esperando un programador y el firmware no arranca (sin pitidos). La
 * linea tiene que estar en bajo mientras el ESC arranca: el FC suelta la
 * retencion cuando ve la bateria conectada desde hace un rato.          */
void dshot_bb_hold(bool hold);
bool dshot_bb_held(void);

/* Decodifica la respuesta anterior y manda un frame a los 4 motores.
 *  value[i]   : 0..2047 ya validado (0 = parado, 1..47 comando, 48.. gas)
 *  telem_mask : bit i = bit de telemetria del frame del motor i
 * MOT_ERR_DMA si el ciclo anterior sigue en curso (frame descartado).     */
mot_status_t dshot_bb_write(const uint16_t value[MOTORS_COUNT], uint8_t telem_mask);

/* Decodifica la ultima respuesta si ya llego (contexto tarea, ~50 us en
 * -O2). Llamarla en una muestra de gyro sin PID saca ese tiempo del ciclo
 * del PID; si no se llamo, dshot_bb_write la decodifica igual.           */
void dshot_bb_service(void);

/* true mientras hay un envio o una lectura en curso. */
bool dshot_bb_busy(void);

/* Corta el ciclo en curso y deja los pines en salida, en alto. */
void dshot_bb_abort(void);

const motors_bidir_stats_t *dshot_bb_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_MOTORS_DRIVER_DSHOT_BB_H_ */
