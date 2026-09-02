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

    return MAG_OK;
}
