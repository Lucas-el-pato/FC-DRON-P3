/**
 ******************************************************************************
 * @file    driver_imu.c
 * @brief   Driver del IMU LSM6DSV16X por SPI1 (hardware).
 *
 *          Pines:
 *            PA5 (SPI1_SCK)   -> SCK  del IMU
 *            PA6 (SPI1_MISO)  -> SDO  del IMU
 *            PA7 (SPI1_MOSI)  -> SDI  del IMU
 *            PC5 (GPIO out)   -> CS   del IMU (active low)
 *
 *          SPI Mode 0, 5.25 MBits/s (prescaler 16, APB2 = 84 MHz).
 *          ODR gyro/accel: 7.68 kHz (high-performance mode).
 ******************************************************************************
 */

#include "driver_imu.h"
#include "spi.h"
#include "stm32f4xx_hal.h"

extern SPI_HandleTypeDef hspi1;

#define IMU_SPI_READ     0x80u
#define IMU_SPI_TIMEOUT  100u
#define IMU_CS_PORT      GPIOC
#define IMU_CS_PIN       GPIO_PIN_5

/* ------------------------------------------------------------------------- */
/* Helpers internos.                                                          */
/* ------------------------------------------------------------------------- */
static inline void imu_cs_low(void)
{
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_RESET);
}

static inline void imu_cs_high(void)
{
    HAL_GPIO_WritePin(IMU_CS_PORT, IMU_CS_PIN, GPIO_PIN_SET);
}

static imu_status_t imu_hal_to_status(HAL_StatusTypeDef st)
{
    if (st == HAL_OK) {
        return IMU_OK;
    }
    if (st == HAL_TIMEOUT) {
        return IMU_ERR_TIMEOUT;
    }
    return IMU_ERR_SPI;
}

/* ------------------------------------------------------------------------- */
/* imu_read_reg                                                               */
/* LSM6DSV16X SPI read: tx = [addr|0x80, dummy], rx = [X, data]. Sin dummy    */
/* intermedio extra (distinto del BMP388).                                    */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return IMU_ERR_SPI;
    }

    uint8_t tx[2] = { (uint8_t)(reg | IMU_SPI_READ), 0x00u };
    uint8_t rx[2] = { 0u, 0u };

    imu_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2u, IMU_SPI_TIMEOUT);
    imu_cs_high();

    if (st != HAL_OK) {
        return imu_hal_to_status(st);
    }
    *value = rx[1];
    return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* imu_write_reg                                                              */
/* ------------------------------------------------------------------------- */
imu_status_t imu_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7Fu), value };

    imu_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, 2u, IMU_SPI_TIMEOUT);
    imu_cs_high();

    return imu_hal_to_status(st);
}

/* ------------------------------------------------------------------------- */
/* imu_read_burst                                                             */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u) {
        return IMU_ERR_SPI;
    }

    uint8_t addr = (uint8_t)(reg | IMU_SPI_READ);

    imu_cs_low();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &addr, 1u, IMU_SPI_TIMEOUT);
    if (st == HAL_OK) {
        st = HAL_SPI_Receive(&hspi1, buf, len, IMU_SPI_TIMEOUT);
    }
    imu_cs_high();

    return imu_hal_to_status(st);
}

/* ------------------------------------------------------------------------- */
/* imu_check_who_am_i                                                         */
/* ------------------------------------------------------------------------- */
imu_status_t imu_check_who_am_i(uint8_t *who)
{
    uint8_t val = 0u;
    imu_status_t st = imu_read_reg(IMU_REG_WHO_AM_I, &val);
    if (who != NULL) {
        *who = val;
    }
    if (st != IMU_OK) {
        return st;
    }
    return (val == IMU_WHO_AM_I_VALUE) ? IMU_OK : IMU_ERR_WHO_AM_I;
}

/* ------------------------------------------------------------------------- */
/* imu_init                                                                   */
/* ------------------------------------------------------------------------- */
imu_status_t imu_init(void)
{
    /* 1. CS en alto (deseleccionado). Esperamos T_BOOT del LSM6DSV16X
     * (datasheet: ~35 ms desde POR hasta que el chip responde SPI).      */
    imu_cs_high();
    HAL_Delay(50u);

    /* 2. Verificar identidad con un par de reintentos por si el chip aun
     * esta saliendo del boot la primera vez.                              */
    uint8_t who = 0u;
    imu_status_t st = IMU_ERR_WHO_AM_I;
    for (uint8_t attempt = 0u; attempt < 5u; ++attempt) {
        st = imu_check_who_am_i(&who);
        if (st == IMU_OK) {
            break;
        }
        HAL_Delay(10u);
    }
    if (st != IMU_OK) {
        return st;
    }

    /* 3. Soft reset (CTRL3.SW_RESET = bit 0). */
    st = imu_write_reg(IMU_REG_CTRL3, 0x01u);
    if (st != IMU_OK) {
        return st;
    }
    HAL_Delay(20u);

    /* 4. CTRL3 = BDU(1<<6) | IF_INC(1<<2) = 0x44. */
    st = imu_write_reg(IMU_REG_CTRL3, 0x44u);
    if (st != IMU_OK) {
        return st;
    }

    /* 5. CTRL1 (XL): OP_MODE_XL=0000 (high-performance) | ODR_XL=1100 (7.68 kHz)
     *    -> 0x0C. FS_XL va en CTRL8.                                         */
    st = imu_write_reg(IMU_REG_CTRL1, 0x0Cu);
    if (st != IMU_OK) {
        return st;
    }

    /* 6. CTRL2 (G): OP_MODE_G=0000 (high-performance) | ODR_G=1100 (7.68 kHz)
     *    -> 0x0C. FS_G va en CTRL6.                                          */
    st = imu_write_reg(IMU_REG_CTRL2, 0x0Cu);
    if (st != IMU_OK) {
        return st;
    }

    /* 7. CTRL6 (FS_G): +/-500 dps -> bits [3:0] = 0010 = 0x02. */
    st = imu_write_reg(IMU_REG_CTRL6, 0x02u);
    if (st != IMU_OK) {
        return st;
    }

    /* 8. CTRL8 (FS_XL): +/-4 g -> bits [1:0] = 01 = 0x01. */
    st = imu_write_reg(IMU_REG_CTRL8, 0x01u);
    if (st != IMU_OK) {
        return st;
    }

    /* Esperar a que XL y G salgan del power-up tras habilitarlos. */
    HAL_Delay(50u);
    return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* imu_read_sample                                                            */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_sample(imu_sample_t *out)
{
    if (out == NULL) {
        return IMU_ERR_SPI;
    }

    /* OUTX_L_G..OUTZ_H_A son 12 bytes consecutivos a partir de 0x22. */
    uint8_t raw[12] = { 0u };
    imu_status_t st = imu_read_burst(IMU_REG_OUTX_L_G, raw, sizeof(raw));
    if (st != IMU_OK) {
        return st;
    }

    out->gx = (int16_t)((uint16_t)raw[0]  | ((uint16_t)raw[1]  << 8));
    out->gy = (int16_t)((uint16_t)raw[2]  | ((uint16_t)raw[3]  << 8));
    out->gz = (int16_t)((uint16_t)raw[4]  | ((uint16_t)raw[5]  << 8));
    out->ax = (int16_t)((uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8));
    out->ay = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
    out->az = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));

    return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* imu_read_sample_avg                                                        */
/*                                                                            */
/* Promedio de n muestras independientes. Para garantizar independencia       */
/* (cada muestra del IMU se entrega cada 1/ODR), antes de cada lectura       */
/* pollea el STATUS_REG hasta que ambos XLDA y GDA esten en alto.            */
/* Esto descarta ruido blanco por un factor ~sqrt(n).                        */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_sample_avg(imu_sample_t *out, uint8_t n_samples)
{
    if (out == NULL || n_samples == 0u) {
        return IMU_ERR_SPI;
    }

    /* Acumuladores int32 para no overflow: cada eje cabe en int16 (+/-32k),
     * con n_samples <= 255 el sumatorio queda <= ~8.4M, muy lejos de int32. */
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    for (uint8_t i = 0u; i < n_samples; ++i) {
        /* Esperar a que el IMU tenga muestra fresca de XL y G.
         * Guard para no colgarse si algo malo pasa con el bus.              */
        uint8_t  status = 0u;
        uint32_t guard  = 0u;
        const uint32_t kGuardMax = 5000u;  /* ~5000 reads del STATUS_REG */

        do {
            imu_status_t st = imu_read_reg(IMU_REG_STATUS, &status);
            if (st != IMU_OK) {
                return st;
            }
            if (++guard > kGuardMax) {
                return IMU_ERR_TIMEOUT;
            }
        } while ((status & (IMU_STATUS_XLDA | IMU_STATUS_GDA))
                 != (IMU_STATUS_XLDA | IMU_STATUS_GDA));

        imu_sample_t s = { 0 };
        imu_status_t st = imu_read_sample(&s);
        if (st != IMU_OK) {
            return st;
        }

        sum_gx += s.gx; sum_gy += s.gy; sum_gz += s.gz;
        sum_ax += s.ax; sum_ay += s.ay; sum_az += s.az;
    }

    out->gx = (int16_t)(sum_gx / (int32_t)n_samples);
    out->gy = (int16_t)(sum_gy / (int32_t)n_samples);
    out->gz = (int16_t)(sum_gz / (int32_t)n_samples);
    out->ax = (int16_t)(sum_ax / (int32_t)n_samples);
    out->ay = (int16_t)(sum_ay / (int32_t)n_samples);
    out->az = (int16_t)(sum_az / (int32_t)n_samples);

    return IMU_OK;
}
