/**
 ******************************************************************************
 * @file    driver_baro.c
 * @brief   Driver del BMP388 por SPI2, CS=PB12 (bar_cs).
 ******************************************************************************
 */

#include "driver_baro.h"
#include "spi.h"
#include "stm32f4xx_hal.h"

#define BARO_SPI_READ       0x80u
#define BARO_SPI_TIMEOUT    100u

extern SPI_HandleTypeDef hspi2;

/* ------------------------------------------------------------------------- */
/* Helpers internos.                                                          */
/* ------------------------------------------------------------------------- */
static inline void baro_cs_low(void)
{
    HAL_GPIO_WritePin(bar_cs_GPIO_Port, bar_cs_Pin, GPIO_PIN_RESET);
}

static inline void baro_cs_high(void)
{
    HAL_GPIO_WritePin(bar_cs_GPIO_Port, bar_cs_Pin, GPIO_PIN_SET);
}

/* ------------------------------------------------------------------------- */
/* baro_read_reg                                                              */
/* BMP388 SPI read: tx = [addr|0x80, dummy, X], rx = [X, dummy, dato].        */
/* ------------------------------------------------------------------------- */
baro_status_t baro_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return BARO_ERR_SPI;
    }

    uint8_t tx[3] = { (uint8_t)(reg | BARO_SPI_READ), 0x00u, 0x00u };
    uint8_t rx[3] = { 0u, 0u, 0u };

    baro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3u, BARO_SPI_TIMEOUT);
    baro_cs_high();

    if (st != HAL_OK) {
        return BARO_ERR_SPI;
    }
    *value = rx[2];
    return BARO_OK;
}

/* ------------------------------------------------------------------------- */
/* baro_read_burst                                                            */
/* Tras enviar [addr|0x80, dummy] el sensor manda los bytes a partir de reg. */
/* ------------------------------------------------------------------------- */
baro_status_t baro_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u) {
        return BARO_ERR_SPI;
    }

    uint8_t header[2] = { (uint8_t)(reg | BARO_SPI_READ), 0x00u };
    HAL_StatusTypeDef st;

    baro_cs_low();
    st = HAL_SPI_Transmit(&hspi2, header, 2u, BARO_SPI_TIMEOUT);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(&hspi2, buf, len, BARO_SPI_TIMEOUT);
    }
    baro_cs_high();

    return (st == HAL_OK) ? BARO_OK : BARO_ERR_SPI;
}

/* ------------------------------------------------------------------------- */
/* baro_write_reg                                                             */
/* Para escribir: tx = [addr & 0x7F, value]. Sin dummy byte.                   */
/* ------------------------------------------------------------------------- */
baro_status_t baro_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };

    baro_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi2, tx, 2u, BARO_SPI_TIMEOUT);
    baro_cs_high();

    return (st == HAL_OK) ? BARO_OK : BARO_ERR_SPI;
}

/* ------------------------------------------------------------------------- */
/* baro_check_chip_id                                                         */
/* ------------------------------------------------------------------------- */
baro_status_t baro_check_chip_id(uint8_t *chip_id)
{
    uint8_t val = 0u;
    baro_status_t st = baro_read_reg(BARO_REG_CHIP_ID, &val);
    if (chip_id != NULL) {
        *chip_id = val;
    }
    if (st != BARO_OK) {
        return st;
    }
    return (val == BARO_CHIP_ID_VALUE) ? BARO_OK : BARO_ERR_CHIP_ID;
}

/* ------------------------------------------------------------------------- */
/* baro_init                                                                  */
/* ------------------------------------------------------------------------- */
baro_status_t baro_init(void)
{
    baro_cs_high();
    HAL_Delay(10u);

    /* 1. Verificar identidad. */
    uint8_t chip = 0u;
    baro_status_t st = baro_check_chip_id(&chip);
    if (st != BARO_OK) {
        return st;
    }

    /* 2. Soft reset. */
    st = baro_write_reg(BARO_REG_CMD, BARO_CMD_SOFT_RESET);
    if (st != BARO_OK) {
        return st;
    }
    HAL_Delay(10u);

    /* 3. PWR_CTRL = press_en(1) | temp_en(1<<1) | mode normal (0x3 << 4). */
    /*    Bit map: [press_en | temp_en | --- | mode<1:0>00]                  */
    /*    Valor estandar para press+temp + modo normal = 0x33.               */
    st = baro_write_reg(BARO_REG_PWR_CTRL, 0x33u);
    if (st != BARO_OK) {
        return st;
    }

    /* 4. OSR = press x4 (0x02), temp x1 (0x00 << 3) -> 0x0B (press x8 + t x1)*/
    /*    osr_p = 010 -> x4, osr_t = 000 -> x1  => OSR reg = 0x02            */
    st = baro_write_reg(BARO_REG_OSR, 0x02u);
    if (st != BARO_OK) {
        return st;
    }

    /* 5. ODR = 50 Hz (odr_sel = 0x02). */
    st = baro_write_reg(BARO_REG_ODR, 0x02u);
    if (st != BARO_OK) {
        return st;
    }

    /* 6. CONFIG = IIR off (iir_filter = 0). */
    st = baro_write_reg(BARO_REG_CONFIG, 0x00u);
    if (st != BARO_OK) {
        return st;
    }

    HAL_Delay(10u);
    return BARO_OK;
}

/* ------------------------------------------------------------------------- */
/* baro_read_sample                                                           */
/* Lee 6 bytes: press_xlsb, press_lsb, press_msb, temp_xlsb, temp_lsb,        */
/*              temp_msb. Cada uno es 24-bit unsigned little-endian.          */
/* ------------------------------------------------------------------------- */
baro_status_t baro_read_sample(baro_sample_t *out)
{
    if (out == NULL) {
        return BARO_ERR_SPI;
    }

    uint8_t raw[6] = { 0u };
    baro_status_t st = baro_read_burst(BARO_REG_DATA_0, raw, sizeof(raw));
    if (st != BARO_OK) {
        return st;
    }

    out->press_raw = ((uint32_t)raw[0])
                   | ((uint32_t)raw[1] << 8)
                   | ((uint32_t)raw[2] << 16);
    out->temp_raw  = ((uint32_t)raw[3])
                   | ((uint32_t)raw[4] << 8)
                   | ((uint32_t)raw[5] << 16);
    return BARO_OK;
}
