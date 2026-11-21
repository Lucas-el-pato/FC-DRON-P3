/**
 ******************************************************************************
 * @file    driver_mag.h
 * @brief   Driver del magnetometro MMC5983MA por I2C1.
 *
 *          Pines:
 *            SCL = PB6 (I2C1_SCL)
 *            SDA = PB7 (I2C1_SDA)
 *            Direccion 7-bit en bus = 0x30  -> en HAL se usa shifted: 0x60
 *
 *          Registros principales (datasheet MMC5983MA Rev D):
 *            Xout[0..2], Yout[0..2], Zout[0..2]:   0x00..0x06
 *            Xout LSB / Yout LSB / Zout LSB:       0x07
 *            T_out:                                0x07 (compartido en var)
 *            STATUS:                               0x08
 *            INTERNAL_CONTROL_0:                   0x09 (TM_M, SET, RESET)
 *            INTERNAL_CONTROL_1:                   0x0A (BW)
 *            INTERNAL_CONTROL_2:                   0x0B (CMM)
 *            INTERNAL_CONTROL_3:                   0x0C
 *            PRODUCT_ID:                           0x2F (esperado = 0x30)
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_MAG_H_
#define TESTS_INC_DRIVER_MAG_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MAG_I2C_ADDR_7BIT       0x30u
#define MAG_I2C_ADDR_HAL        (MAG_I2C_ADDR_7BIT << 1)

/* Registros. */
#define MAG_REG_XOUT_0          0x00u
#define MAG_REG_STATUS          0x08u
#define MAG_REG_INT_CTRL_0      0x09u
#define MAG_REG_INT_CTRL_1      0x0Au
#define MAG_REG_INT_CTRL_2      0x0Bu
#define MAG_REG_INT_CTRL_3      0x0Cu
#define MAG_REG_PRODUCT_ID      0x2Fu
#define MAG_PRODUCT_ID_VALUE    0x30u

/* Bits utiles en STATUS / INTERNAL_CONTROL_0. */
#define MAG_STATUS_MEAS_M_DONE  (1u << 0)
#define MAG_CTRL0_TM_M          (1u << 0)
#define MAG_CTRL0_SET           (1u << 3)
#define MAG_CTRL0_RESET         (1u << 4)

typedef enum {
    MAG_OK = 0,
    MAG_ERR_I2C,
    MAG_ERR_PRODUCT_ID,
    MAG_ERR_TIMEOUT
} mag_status_t;

/* Muestra cruda del magnetometro (18 bits por eje, ya expandido a int32).    */
typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} mag_sample_t;

/* ------------------------------------------------------------------------- */
/* Inicializa el magnetometro:                                                */
/*  - Verifica PRODUCT_ID = 0x30                                              */
/*  - Aplica secuencia SET -> RESET para descondicionar el sensor             */
/*  - Configura BW = 100 Hz (INT_CTRL_1 = 0x00)                               */
/* ------------------------------------------------------------------------- */
mag_status_t mag_init(void);

/* Lee un registro de 8 bits. */
mag_status_t mag_read_reg(uint8_t reg, uint8_t *value);

/* Escribe un registro de 8 bits. */
mag_status_t mag_write_reg(uint8_t reg, uint8_t value);

/* Lee multiples bytes a partir de reg. */
mag_status_t mag_read_burst(uint8_t reg, uint8_t *buf, uint16_t len);

/* Verifica el PRODUCT_ID. */
mag_status_t mag_check_product_id(uint8_t *id);

/* Dispara una medicion en modo TM_M (Take Measurement, magnetometer).        */
mag_status_t mag_trigger_measurement(void);

/* Espera hasta MEAS_M_DONE (con timeout_ms) y lee XYZ.                       */
mag_status_t mag_read_sample(mag_sample_t *out, uint32_t timeout_ms);

/* Convierte los 7 bytes de 0x00..0x06 (Xout0..Zout1 + XYZ LSB) a XYZ con
 * signo (offset de centro 2^17 ya restado).                                 */
void mag_parse_raw(const uint8_t raw[7], mag_sample_t *out);

/* ========================================================================= */
/* Modo no bloqueante (vuelo)                                                 */
/*                                                                            */
/* Maquina de estados por interrupcion de I2C1: dispara TM_M, espera la       */
/* medicion (8 ms a BW 100 Hz) sin bloquear, lee STATUS y, si MEAS_M_DONE,   */
/* los 7 bytes de XYZ. Al completar una lectura se re-dispara solo.           */
/*                                                                            */
/* I2C1_EV_IRQHandler / I2C1_ER_IRQHandler viven en driver_mag.c (el IRQ de   */
/* I2C1 no esta habilitado en el .ioc). Si algun dia se habilita en CubeMX,   */
/* borrar los handlers de driver_mag.c.                                       */
/*                                                                            */
/* Despues de mag_async_start() no usar las funciones bloqueantes de arriba:  */
/* compiten por el bus con la maquina de estados.                             */
/* ========================================================================= */

/* Prioridad NVIC de I2C1 (EV y ER). Debajo de EXTI2 (gyro, 5) y USART3
 * (GPS, 6): el mag nunca demora al IMU ni al ring del GPS.               */
#ifndef MAG_NVIC_PRIO
#define MAG_NVIC_PRIO           7u
#endif

/* Tiempo minimo entre disparo y lectura (medicion de 8 ms a BW 100 Hz). */
#ifndef MAG_ASYNC_MEAS_MS
#define MAG_ASYNC_MEAS_MS       10u
#endif

/* Una transferencia colgada mas que esto -> se reinicia el I2C1. */
#ifndef MAG_ASYNC_STUCK_MS
#define MAG_ASYNC_STUCK_MS      50u
#endif

typedef struct {
    uint32_t samples;      /* mediciones completas leidas               */
    uint32_t not_ready;    /* lecturas con MEAS_M_DONE todavia en 0      */
    uint32_t errors;       /* errores de bus / NACK / HAL ocupado        */
    uint32_t recoveries;   /* reinicios del periferico I2C1              */
} mag_async_stats_t;

/* Habilita los IRQ de I2C1 y arranca la maquina de estados. Llamar una vez
 * despues de mag_init() OK.                                               */
void mag_async_start(void);

/* Avanza la maquina de estados (dispara / lee / recupera). No bloquea.
 * Llamarla cada ~20 ms desde una tarea lenta.                             */
void mag_async_poll(void);

/* true si hay una medicion nueva desde la ultima llamada; la copia a out. */
bool mag_async_take(mag_sample_t *out);

void mag_async_stats(mag_async_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_MAG_H_ */
