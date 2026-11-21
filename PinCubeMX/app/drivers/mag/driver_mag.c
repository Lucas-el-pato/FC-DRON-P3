/**
 ******************************************************************************
 * @file    driver_mag.c
 * @brief   Driver del magnetometro MMC5983MA por I2C1, addr 0x30.
 ******************************************************************************
 */

#include "driver_mag.h"
#include "i2c.h"
#include "stm32f4xx_hal.h"

#define MAG_I2C_TIMEOUT     100u

extern I2C_HandleTypeDef hi2c1;

/* ------------------------------------------------------------------------- */
/* mag_read_reg                                                               */
/* ------------------------------------------------------------------------- */
mag_status_t mag_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return MAG_ERR_I2C;
    }
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDR_HAL, reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            value, 1u, MAG_I2C_TIMEOUT);
    return (st == HAL_OK) ? MAG_OK : MAG_ERR_I2C;
}

/* ------------------------------------------------------------------------- */
/* mag_write_reg                                                              */
/* ------------------------------------------------------------------------- */
mag_status_t mag_write_reg(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef st = HAL_I2C_Mem_Write(&hi2c1, MAG_I2C_ADDR_HAL, reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &value, 1u, MAG_I2C_TIMEOUT);
    return (st == HAL_OK) ? MAG_OK : MAG_ERR_I2C;
}

/* ------------------------------------------------------------------------- */
/* mag_read_burst                                                             */
/* ------------------------------------------------------------------------- */
mag_status_t mag_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u) {
        return MAG_ERR_I2C;
    }
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(&hi2c1, MAG_I2C_ADDR_HAL, reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf, len, MAG_I2C_TIMEOUT);
    return (st == HAL_OK) ? MAG_OK : MAG_ERR_I2C;
}

/* ------------------------------------------------------------------------- */
/* mag_check_product_id                                                       */
/* ------------------------------------------------------------------------- */
mag_status_t mag_check_product_id(uint8_t *id)
{
    uint8_t val = 0u;
    mag_status_t st = mag_read_reg(MAG_REG_PRODUCT_ID, &val);
    if (id != NULL) {
        *id = val;
    }
    if (st != MAG_OK) {
        return st;
    }
    return (val == MAG_PRODUCT_ID_VALUE) ? MAG_OK : MAG_ERR_PRODUCT_ID;
}

/* ------------------------------------------------------------------------- */
/* mag_init                                                                   */
/* ------------------------------------------------------------------------- */
mag_status_t mag_init(void)
{
    /* Pequena espera tras power-on. */
    HAL_Delay(10u);

    uint8_t id = 0u;
    mag_status_t st = mag_check_product_id(&id);
    if (st != MAG_OK) {
        return st;
    }

    /* Secuencia SET -> RESET para condicionar al sensor (degauss).            */
    st = mag_write_reg(MAG_REG_INT_CTRL_0, MAG_CTRL0_SET);
    if (st != MAG_OK) {
        return st;
    }
    HAL_Delay(2u);
    st = mag_write_reg(MAG_REG_INT_CTRL_0, MAG_CTRL0_RESET);
    if (st != MAG_OK) {
        return st;
    }
    HAL_Delay(2u);

    /* BW = 100 Hz (default), modo single-shot. */
    st = mag_write_reg(MAG_REG_INT_CTRL_1, 0x00u);
    if (st != MAG_OK) {
        return st;
    }

    /* CMM apagado: cada medicion se dispara con TM_M.                         */
    st = mag_write_reg(MAG_REG_INT_CTRL_2, 0x00u);
    return st;
}

/* ------------------------------------------------------------------------- */
/* mag_trigger_measurement                                                    */
/* ------------------------------------------------------------------------- */
mag_status_t mag_trigger_measurement(void)
{
    return mag_write_reg(MAG_REG_INT_CTRL_0, MAG_CTRL0_TM_M);
}

/* ------------------------------------------------------------------------- */
/* mag_read_sample                                                            */
/* Espera MEAS_M_DONE, luego lee 7 bytes (Xout0..2, Yout0..2, Zout0..2 + LSB).*/
/* Cada eje es 18 bits unsigned con offset 2^17.                              */
/* ------------------------------------------------------------------------- */
mag_status_t mag_read_sample(mag_sample_t *out, uint32_t timeout_ms)
{
    if (out == NULL) {
        return MAG_ERR_I2C;
    }

    /* Dispara una medicion. */
    mag_status_t st = mag_trigger_measurement();
    if (st != MAG_OK) {
        return st;
    }

    /* Polling de MEAS_M_DONE. */
    uint32_t t0 = HAL_GetTick();
    uint8_t status = 0u;
    while (1) {
        st = mag_read_reg(MAG_REG_STATUS, &status);
        if (st != MAG_OK) {
            return st;
        }
        if ((status & MAG_STATUS_MEAS_M_DONE) != 0u) {
            break;
        }
        if ((HAL_GetTick() - t0) > timeout_ms) {
            return MAG_ERR_TIMEOUT;
        }
    }

    /* Lectura de 7 bytes (Xout MSB/LSB, Yout MSB/LSB, Zout MSB/LSB, XYZ LSB). */
    uint8_t raw[7] = { 0u };
    st = mag_read_burst(MAG_REG_XOUT_0, raw, sizeof(raw));
    if (st != MAG_OK) {
        return st;
    }

    mag_parse_raw(raw, out);
    return MAG_OK;
}

/* ------------------------------------------------------------------------- */
/* mag_parse_raw                                                              */
/* ------------------------------------------------------------------------- */
void mag_parse_raw(const uint8_t raw[7], mag_sample_t *out)
{
    if (raw == NULL || out == NULL) {
        return;
    }

    /* Cada eje: 16 bits altos en raw[i,i+1] + 2 bits adicionales en raw[6].   */
    /* X: bits 17..2 en raw[0,1], bits 1..0 en raw[6] bits 7..6                */
    /* Y: bits 17..2 en raw[2,3], bits 1..0 en raw[6] bits 5..4                */
    /* Z: bits 17..2 en raw[4,5], bits 1..0 en raw[6] bits 3..2                */
    uint32_t x_u = ((uint32_t)raw[0] << 10) | ((uint32_t)raw[1] << 2) | ((raw[6] >> 6) & 0x3u);
    uint32_t y_u = ((uint32_t)raw[2] << 10) | ((uint32_t)raw[3] << 2) | ((raw[6] >> 4) & 0x3u);
    uint32_t z_u = ((uint32_t)raw[4] << 10) | ((uint32_t)raw[5] << 2) | ((raw[6] >> 2) & 0x3u);

    /* Offset: el sensor centra en 2^17 = 131072 (campo cero).                 */
    out->x = (int32_t)x_u - 131072;
    out->y = (int32_t)y_u - 131072;
    out->z = (int32_t)z_u - 131072;
}

/* ========================================================================= */
/* Modo no bloqueante                                                         */
/* ========================================================================= */

/* Dos lecturas por medicion: primero STATUS (0x08) y, si MEAS_M_DONE = 1,
 * los 7 bytes de 0x00..0x06. No se puede leer todo en un burst 0x00..0x08:
 * leer cualquier registro de datos borra MEAS_M_DONE, asi que al llegar a
 * STATUS el bit ya vale 0 (por eso el primer intento nunca veia muestras).  */
#define MAG_ASYNC_DATA_LEN      7u

typedef enum {
    MAG_AS_OFF = 0,
    MAG_AS_IDLE,          /* hay que disparar una medicion             */
    MAG_AS_TRIGGERING,    /* escribiendo TM_M por IT                    */
    MAG_AS_MEASURING,     /* el sensor mide; esperar MAG_ASYNC_MEAS_MS  */
    MAG_AS_READING_STATUS,/* leyendo STATUS por IT                      */
    MAG_AS_READING        /* leyendo XYZ por IT                         */
} mag_async_state_t;

static volatile mag_async_state_t s_as_state = MAG_AS_OFF;
static volatile uint32_t s_as_t_trigger = 0u;   /* tick del fin del disparo   */
static volatile uint32_t s_as_t_busy = 0u;      /* tick de inicio de la transf. */
static volatile uint32_t s_as_seq = 0u;         /* sube con cada muestra      */
static uint32_t s_as_seq_taken = 0u;
static uint8_t s_as_tx = MAG_CTRL0_TM_M;        /* buffer del IT (static)     */
static uint8_t s_as_status = 0u;
static uint8_t s_as_rx[MAG_ASYNC_DATA_LEN];
static uint8_t s_as_sample[7];                  /* ultima muestra completa    */
static volatile mag_async_stats_t s_as_stats;
static volatile uint8_t s_as_start_fail = 0u;

static void mag_async_irq_enable(bool on)
{
    if (on) {
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    } else {
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    }
}

/* Dispara TM_M. Se llama desde la tarea o desde el callback de RX (ISR). */
static void mag_async_trigger(void)
{
    s_as_tx = MAG_CTRL0_TM_M;
    s_as_state = MAG_AS_TRIGGERING;
    s_as_t_busy = HAL_GetTick();
    if (HAL_I2C_Mem_Write_IT(&hi2c1, MAG_I2C_ADDR_HAL, MAG_REG_INT_CTRL_0,
                             I2C_MEMADD_SIZE_8BIT, &s_as_tx, 1u) != HAL_OK) {
        s_as_stats.errors++;
        s_as_start_fail++;
        s_as_state = MAG_AS_IDLE;
    } else {
        s_as_start_fail = 0u;
    }
}

static void mag_async_read_status(void)
{
    s_as_state = MAG_AS_READING_STATUS;
    s_as_t_busy = HAL_GetTick();
    if (HAL_I2C_Mem_Read_IT(&hi2c1, MAG_I2C_ADDR_HAL, MAG_REG_STATUS,
                            I2C_MEMADD_SIZE_8BIT, &s_as_status, 1u) != HAL_OK) {
        s_as_stats.errors++;
        s_as_start_fail++;
        s_as_state = MAG_AS_IDLE;
    } else {
        s_as_start_fail = 0u;
    }
}

/* Se llama desde el callback de STATUS (ISR) con MEAS_M_DONE = 1. */
static void mag_async_read_data(void)
{
    s_as_state = MAG_AS_READING;
    s_as_t_busy = HAL_GetTick();
    if (HAL_I2C_Mem_Read_IT(&hi2c1, MAG_I2C_ADDR_HAL, MAG_REG_XOUT_0,
                            I2C_MEMADD_SIZE_8BIT, s_as_rx,
                            (uint16_t)MAG_ASYNC_DATA_LEN) != HAL_OK) {
        s_as_stats.errors++;
        s_as_start_fail++;
        s_as_state = MAG_AS_IDLE;
    } else {
        s_as_start_fail = 0u;
    }
}

/* Reinicio del periferico (transferencia colgada o HAL siempre ocupado). */
static void mag_async_recover(void)
{
    mag_async_irq_enable(false);
    (void)HAL_I2C_DeInit(&hi2c1);
    (void)HAL_I2C_Init(&hi2c1);
    s_as_stats.recoveries++;
    s_as_start_fail = 0u;
    s_as_state = MAG_AS_IDLE;
    mag_async_irq_enable(true);
}

void mag_async_start(void)
{
    s_as_seq = 0u;
    s_as_seq_taken = 0u;
    s_as_start_fail = 0u;
    s_as_stats.samples = 0u;
    s_as_stats.not_ready = 0u;
    s_as_stats.errors = 0u;
    s_as_stats.recoveries = 0u;

    HAL_NVIC_SetPriority(I2C1_EV_IRQn, MAG_NVIC_PRIO, 0u);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, MAG_NVIC_PRIO, 0u);
    mag_async_irq_enable(true);

    s_as_state = MAG_AS_IDLE;
    mag_async_trigger();
}

void mag_async_poll(void)
{
    const uint32_t now = HAL_GetTick();

    switch (s_as_state) {
    case MAG_AS_OFF:
        return;

    case MAG_AS_IDLE:
        /* Tres arranques seguidos rechazados (HAL ocupado, BUSY pegado):
         * se reinicia el periferico antes de reintentar.               */
        if (s_as_start_fail >= 3u) {
            mag_async_recover();
        }
        mag_async_trigger();
        break;

    case MAG_AS_MEASURING:
        if ((now - s_as_t_trigger) >= MAG_ASYNC_MEAS_MS) {
            mag_async_read_status();
        }
        break;

    case MAG_AS_TRIGGERING:
    case MAG_AS_READING_STATUS:
    case MAG_AS_READING:
    default:
        if ((now - s_as_t_busy) > MAG_ASYNC_STUCK_MS) {
            mag_async_recover();
        }
        break;
    }
}

bool mag_async_take(mag_sample_t *out)
{
    if (out == NULL) {
        return false;
    }

    /* La muestra la escribe el callback de RX (IRQ EV): copia atomica. */
    uint8_t raw[7];
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    const uint32_t seq = s_as_seq;
    for (uint8_t i = 0u; i < 7u; ++i) {
        raw[i] = s_as_sample[i];
    }
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);

    if (seq == s_as_seq_taken) {
        return false;
    }
    s_as_seq_taken = seq;
    mag_parse_raw(raw, out);
    return true;
}

void mag_async_stats(mag_async_stats_t *out)
{
    if (out == NULL) {
        return;
    }
    out->samples = s_as_stats.samples;
    out->not_ready = s_as_stats.not_ready;
    out->errors = s_as_stats.errors;
    out->recoveries = s_as_stats.recoveries;
}

/* ---------------------------- Callbacks HAL ------------------------------ */
/* Solo I2C1 se usa en este FC; igual se filtra por instancia.               */

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1 || s_as_state != MAG_AS_TRIGGERING) {
        return;
    }
    s_as_t_trigger = HAL_GetTick();
    s_as_state = MAG_AS_MEASURING;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }
    if (s_as_state == MAG_AS_READING_STATUS) {
        if ((s_as_status & MAG_STATUS_MEAS_M_DONE) == 0u) {
            /* Todavia midiendo: se reintenta en la proxima pasada. Si no
             * termina nunca (disparo perdido), se vuelve a disparar.    */
            s_as_stats.not_ready++;
            if ((HAL_GetTick() - s_as_t_trigger) > MAG_ASYNC_STUCK_MS) {
                s_as_state = MAG_AS_IDLE;
            } else {
                s_as_state = MAG_AS_MEASURING;
            }
            return;
        }
        mag_async_read_data();
        return;
    }
    if (s_as_state != MAG_AS_READING) {
        return;
    }
    for (uint8_t i = 0u; i < 7u; ++i) {
        s_as_sample[i] = s_as_rx[i];
    }
    s_as_seq++;
    s_as_stats.samples++;

    /* Proxima medicion enseguida (el bus ya esta libre en el callback). */
    mag_async_trigger();
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1 || s_as_state == MAG_AS_OFF) {
        return;
    }
    s_as_stats.errors++;
    s_as_state = MAG_AS_IDLE;
}

void I2C1_EV_IRQHandler(void)
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}
