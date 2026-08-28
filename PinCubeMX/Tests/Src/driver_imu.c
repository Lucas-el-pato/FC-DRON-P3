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
 *            PC2 (INT1)       -> gyro data-ready (EXTI2)
 *
 *          ODR gyro/accel: 8 kHz (HAODR, HAODR_SEL=01, ODR code 1100).
 *          INT1: INT1_CTRL.INT1_DRDY_G, latched, active high.
 ******************************************************************************
 */

#include "driver_imu.h"
#include "timebase.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

#define IMU_SPI_READ     0x80u
#define IMU_SPI_TIMEOUT  100u
#define IMU_CS_PORT      GPIOC
#define IMU_CS_PIN       GPIO_PIN_5

#define IMU_DRDY_NOMINAL_CYC   (168u * 125u)                 /* 21000 @ 8 kHz */
#define IMU_DRDY_LATE_CYC      (IMU_DRDY_NOMINAL_CYC * 3u / 2u)

static volatile bool     s_gyro_drdy = false;
static volatile uint32_t s_gyro_drdy_count = 0u;

static volatile bool     s_drdy_have_prev = false;
static volatile uint32_t s_drdy_prev_cyc = 0u;
static volatile uint32_t s_drdy_last_cyc = 0u;
static volatile uint32_t s_drdy_sum_cyc = 0u;
static volatile uint32_t s_drdy_n = 0u;
static volatile uint32_t s_drdy_min_cyc = UINT32_MAX;
static volatile uint32_t s_drdy_max_cyc = 0u;
static volatile uint32_t s_drdy_late_events = 0u;
static volatile uint32_t s_drdy_late_cyc = 0u;
static volatile uint32_t s_drdy_edges_window = 0u;
static volatile uint32_t s_drdy_window_start_cyc = 0u;

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
    /* EXTI2 apagado mientras se programan registros: un INT1 a 8 kHz
     * con prioridad 0 deja al USB CDC sin tiempo de CPU.                  */
    HAL_NVIC_DisableIRQ(EXTI2_IRQn);

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

    /* 5. IF_CFG: INT1/INT2 active-high, push-pull (defaults) y deshabilita
     * I2C/I3C porque el bus es SPI 4-wire. Datasheet §9.3.                 */
    st = imu_write_reg(IMU_REG_IF_CFG, IMU_IF_CFG_I2C_I3C_DISABLE);
    if (st != IMU_OK) {
        return st;
    }

    /* 6. CTRL4: DRDY_MASK=1 (no disparar INT hasta que el filtro asiente).
     * DRDY_PULSED=0 -> latched: INT1 baja al leer OUTZ_H_G. §9.17.         */
    st = imu_write_reg(IMU_REG_CTRL4, IMU_CTRL4_DRDY_MASK);
    if (st != IMU_OK) {
        return st;
    }

    /* 7. Full-scale mientras XL/G siguen en power-down (ODR=0000). */
    /* CTRL6 (FS_G): +/-500 dps -> bits [3:0] = 0010 = 0x02. */
    st = imu_write_reg(IMU_REG_CTRL6, 0x02u);
    if (st != IMU_OK) {
        return st;
    }

    /* CTRL8 (FS_XL): +/-4 g -> bits [1:0] = 01 = 0x01. */
    st = imu_write_reg(IMU_REG_CTRL8, 0x01u);
    if (st != IMU_OK) {
        return st;
    }

    /* 8. HAODR_CFG: SEL=01 (tabla 8 kHz). Datasheet §6.5: habilitar HAODR
     * solo en power-down. Si un sensor va a HAODR, el otro tambien.        */
    st = imu_write_reg(IMU_REG_HAODR_CFG, IMU_HAODR_SEL_8KHZ);
    if (st != IMU_OK) {
        return st;
    }

    /* 9. CTRL1 (XL): OP_MODE=001 (HAODR) | ODR=1100 -> 8 kHz. §9.14 / Table 20. */
    st = imu_write_reg(IMU_REG_CTRL1, IMU_CTRL_HAODR_8KHZ);
    if (st != IMU_OK) {
        return st;
    }

    /* 10. CTRL2 (G): igual, HAODR 8 kHz. FS_G ya esta en CTRL6. */
    st = imu_write_reg(IMU_REG_CTRL2, IMU_CTRL_HAODR_8KHZ);
    if (st != IMU_OK) {
        return st;
    }

    /* 11. INT1_CTRL: solo gyro data-ready en INT1 (PC2 / EXTI2).
     * Bit1 INT1_DRDY_G. Bits 2 y 7 deben quedar en 0. Datasheet §9.11.     */
    st = imu_write_reg(IMU_REG_INT1_CTRL, IMU_INT1_DRDY_G);
    if (st != IMU_OK) {
        return st;
    }

    /* Esperar a que XL y G salgan del power-up tras habilitarlos. */
    HAL_Delay(50u);

    /* Lectura dummy: en modo latched INT1 queda alto hasta leer el MSB del
     * gyro. Esto baja el pin y deja EXTI listo para el proximo flanco.     */
    imu_sample_t dummy = { 0 };
    (void)imu_read_sample(&dummy);
    __HAL_GPIO_EXTI_CLEAR_IT(Gyro_Data_Pin);
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
    s_gyro_drdy = false;
    s_gyro_drdy_count = 0u;
    s_drdy_have_prev = false;
    s_drdy_prev_cyc = 0u;
    s_drdy_last_cyc = 0u;
    s_drdy_sum_cyc = 0u;
    s_drdy_n = 0u;
    s_drdy_min_cyc = UINT32_MAX;
    s_drdy_max_cyc = 0u;
    s_drdy_late_events = 0u;
    s_drdy_late_cyc = 0u;
    s_drdy_edges_window = 0u;
    s_drdy_window_start_cyc = timebase_now();

    /* Prioridad 5: USB OTG_FS (0) puede preemptar el gyro DRDY. */
    HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

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
/* imu_read_gyro                                                              */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_gyro(imu_sample_t *out)
{
    if (out == NULL) {
        return IMU_ERR_SPI;
    }

    uint8_t raw[6] = { 0u };
    imu_status_t st = imu_read_burst(IMU_REG_OUTX_L_G, raw, sizeof(raw));
    if (st != IMU_OK) {
        return st;
    }

    out->gx = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    out->gy = (int16_t)((uint16_t)raw[2] | ((uint16_t)raw[3] << 8));
    out->gz = (int16_t)((uint16_t)raw[4] | ((uint16_t)raw[5] << 8));

    return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* imu_read_sample_avg                                                        */
/*                                                                            */
/* Promedio de n muestras independientes. Espera INT1 gyro DRDY (EXTI2)
 * antes de cada lectura. Reduce ruido por ~sqrt(n).                         */
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
        imu_status_t st = imu_wait_gyro_drdy(5u);
        if (st != IMU_OK) {
            return st;
        }

        imu_sample_t s = { 0 };
        st = imu_read_sample(&s);
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

/* ------------------------------------------------------------------------- */
/* INT1 gyro data-ready (EXTI2). El ISR no toca SPI.                          */
/* ------------------------------------------------------------------------- */
bool imu_gyro_drdy_take(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool pending = s_gyro_drdy;
    s_gyro_drdy = false;
    __set_PRIMASK(primask);
    return pending;
}

uint32_t imu_gyro_drdy_count(void)
{
    return s_gyro_drdy_count;
}

imu_status_t imu_wait_gyro_drdy(uint32_t timeout_ms)
{
    const uint32_t start = HAL_GetTick();
    while (!imu_gyro_drdy_take()) {
        if ((HAL_GetTick() - start) > timeout_ms) {
            return IMU_ERR_TIMEOUT;
        }
    }
    return IMU_OK;
}

void imu_gyro_drdy_stats_take(imu_drdy_stats_t *out)
{
    if (out == NULL) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    const uint32_t now = timebase_now();
    out->intervals    = s_drdy_n;
    out->edges        = s_drdy_edges_window;
    out->late_events  = s_drdy_late_events;
    out->late_cyc     = s_drdy_late_cyc;
    out->window_cyc   = now - s_drdy_window_start_cyc;
    out->dt_min_cyc   = (s_drdy_n > 0u) ? s_drdy_min_cyc : 0u;
    out->dt_max_cyc   = s_drdy_max_cyc;
    out->dt_avg_cyc   = (s_drdy_n > 0u) ? (s_drdy_sum_cyc / s_drdy_n) : 0u;

    s_drdy_sum_cyc = 0u;
    s_drdy_n = 0u;
    s_drdy_min_cyc = UINT32_MAX;
    s_drdy_max_cyc = 0u;
    s_drdy_late_events = 0u;
    s_drdy_late_cyc = 0u;
    s_drdy_edges_window = 0u;
    s_drdy_window_start_cyc = now;

    __set_PRIMASK(primask);
}

uint32_t imu_gyro_drdy_interval_us(void)
{
    return timebase_cycles_to_us(s_drdy_last_cyc);
}

uint32_t imu_gyro_drdy_missed_estimate(uint32_t late_cyc, uint32_t late_events)
{
    if (late_cyc < IMU_DRDY_NOMINAL_CYC) {
        return 0u;
    }
    const uint32_t slots = late_cyc / IMU_DRDY_NOMINAL_CYC;
    if (slots <= late_events) {
        return 0u;
    }
    return slots - late_events;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != Gyro_Data_Pin) {
        return;
    }

    const uint32_t now = timebase_now();

    if (s_drdy_have_prev) {
        const uint32_t d = now - s_drdy_prev_cyc;
        s_drdy_last_cyc = d;
        if (d > IMU_DRDY_LATE_CYC) {
            s_drdy_late_events++;
            s_drdy_late_cyc += d;
        } else {
            s_drdy_sum_cyc += d;
            s_drdy_n++;
            if (d < s_drdy_min_cyc) {
                s_drdy_min_cyc = d;
            }
            if (d > s_drdy_max_cyc) {
                s_drdy_max_cyc = d;
            }
        }
    } else {
        s_drdy_have_prev = true;
    }
    s_drdy_prev_cyc = now;

    s_gyro_drdy = true;
    s_gyro_drdy_count++;
    s_drdy_edges_window++;
}
