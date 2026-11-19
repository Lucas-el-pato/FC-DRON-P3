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
#include "pinout.h"
#include "timebase.h"
#include "spi.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

extern SPI_HandleTypeDef hspi1;

#define IMU_SPI_READ     0x80u
#define IMU_SPI_TIMEOUT  100u

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
/* Stream del lazo (ver imu_stream_*).                                        */
/* ------------------------------------------------------------------------- */
/* Burst: 1 byte de direccion + 12 de datos (OUTX_L_G .. OUTZ_H_A). */
#define IMU_STREAM_DATA_LEN    12u
#define IMU_STREAM_XFER_LEN    (1u + IMU_STREAM_DATA_LEN)

/* INT1 en alto sin lectura durante 4 periodos = flanco perdido (latched:
 * no vuelve a subir hasta leer OUTZ_H_G). Se relanza la lectura.        */
#define IMU_STUCK_CYC          (IMU_DRDY_NOMINAL_CYC * 4u)

static volatile bool     s_stream_active = false;
static volatile uint32_t s_last_kick_cyc = 0u;

static volatile uint32_t s_st_samples = 0u;
static volatile uint32_t s_st_overruns = 0u;
static volatile uint32_t s_st_skipped = 0u;
static volatile uint32_t s_st_spi_errors = 0u;
static volatile uint32_t s_st_recoveries = 0u;

#if IMU_USE_DMA
/* Buffers DMA en RAM normal (.bss): CCMRAM no es accesible por DMA. */
static uint8_t s_dma_tx[IMU_STREAM_XFER_LEN];
static uint8_t s_dma_rx[IMU_STREAM_XFER_LEN];

static volatile bool s_dma_busy = false;
static volatile bool s_sw_kick_pending = false;   /* EXTI por software */
static volatile bool s_unread = false;
static imu_sample_t  s_latest;                     /* escrito en ISR DMA */
#endif

/* ------------------------------------------------------------------------- */
/* Helpers internos.                                                          */
/* ------------------------------------------------------------------------- */
static inline void imu_cs_low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_RESET);
}

static inline void imu_cs_high(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
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

/* Con el stream DMA activo el SPI1 lo maneja el ISR: nada bloqueante. */
static inline bool imu_bus_locked(void)
{
#if IMU_USE_DMA
    return s_stream_active;
#else
    return false;
#endif
}

/* 12 bytes little-endian: gyro X,Y,Z y accel X,Y,Z. */
static void imu_decode12(const uint8_t *raw, imu_sample_t *out)
{
    out->gx = (int16_t)((uint16_t)raw[0]  | ((uint16_t)raw[1]  << 8));
    out->gy = (int16_t)((uint16_t)raw[2]  | ((uint16_t)raw[3]  << 8));
    out->gz = (int16_t)((uint16_t)raw[4]  | ((uint16_t)raw[5]  << 8));
    out->ax = (int16_t)((uint16_t)raw[6]  | ((uint16_t)raw[7]  << 8));
    out->ay = (int16_t)((uint16_t)raw[8]  | ((uint16_t)raw[9]  << 8));
    out->az = (int16_t)((uint16_t)raw[10] | ((uint16_t)raw[11] << 8));
}

/* ------------------------------------------------------------------------- */
/* imu_read_reg                                                             */
/* LSM6DSV16X SPI read: tx = [addr|0x80, dummy], rx = [X, data]. Sin dummy    */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL || imu_bus_locked()) {
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
    if (imu_bus_locked()) {
        return IMU_ERR_SPI;
    }

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
    if (buf == NULL || len == 0u || imu_bus_locked()) {
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

    /* Re-init con el stream corriendo: se suelta el bus y se espera a que
     * termine el burst en vuelo (~25 us) antes de usar SPI bloqueante.    */
    s_stream_active = false;
#if IMU_USE_DMA
    const uint32_t wait_t0 = HAL_GetTick();
    while (s_dma_busy && ((HAL_GetTick() - wait_t0) < 2u)) {
    }
    if (s_dma_busy) {
        (void)HAL_SPI_Abort(&hspi1);
        imu_cs_high();
        s_dma_busy = false;
    }
    s_sw_kick_pending = false;
    s_unread = false;
#endif

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

    /* CTRL8: FS_XL +/-4 g (bits [1:0] = 01) + LPF2 del accel en ODR/800. */
    st = imu_write_reg(IMU_REG_CTRL8,
                       (uint8_t)(IMU_CTRL8_FS_XL_4G | IMU_CTRL8_XL_LPF2_ODR_800));
    if (st != IMU_OK) {
        return st;
    }

    /* CTRL9: salida del accel desde el LPF2 (~10 Hz). */
    st = imu_write_reg(IMU_REG_CTRL9, IMU_CTRL9_LPF2_XL_EN);
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

    imu_decode12(raw, out);
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

/* ------------------------------------------------------------------------- */
/* Stream por DMA                                                             */
/* ------------------------------------------------------------------------- */
#if IMU_USE_DMA
/* Arranca el burst de 13 bytes. Solo desde el ISR de EXTI2: asi nunca hay
 * dos llamadores compitiendo por hspi1 (el lazo pide via EXTI por software). */
static void imu_dma_kick(void)
{
    if (!s_stream_active) {
        return;
    }
    if (s_dma_busy) {
        s_st_overruns++;
        return;
    }

    s_dma_busy = true;
    s_last_kick_cyc = timebase_now();

    imu_cs_low();
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, s_dma_tx, s_dma_rx,
                                    (uint16_t)IMU_STREAM_XFER_LEN) != HAL_OK) {
        imu_cs_high();
        s_dma_busy = false;
        s_st_spi_errors++;
    }
}

/* Pide una lectura desde el lazo: dispara EXTI2 por software. */
static void imu_request_sw_kick(void)
{
    s_sw_kick_pending = true;
    s_last_kick_cyc = timebase_now();
    EXTI->SWIER = Gyro_Data_Pin;
}

/* Fin del burst (contexto IRQ de DMA2_Stream0, SPI1_RX). */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) {
        return;
    }
    imu_cs_high();

    if (s_unread) {
        s_st_skipped++;   /* el lazo no tomo la muestra anterior */
    }
    imu_decode12(&s_dma_rx[1], &s_latest);
    s_unread = true;
    s_st_samples++;
    s_dma_busy = false;
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) {
        return;
    }
    imu_cs_high();
    s_st_spi_errors++;
    s_dma_busy = false;
}
#endif /* IMU_USE_DMA */

/* INT1 en alto, nada en curso y sin lectura hace > 4 periodos: el flanco se
 * perdio y con DRDY latched no va a llegar otro. Se fuerza una lectura.   */
static bool imu_int1_stuck(void)
{
    if (HAL_GPIO_ReadPin(Gyro_Data_GPIO_Port, Gyro_Data_Pin) != GPIO_PIN_SET) {
        return false;
    }
#if IMU_USE_DMA
    if (s_dma_busy || s_sw_kick_pending) {
        return false;
    }
#endif
    return (timebase_now() - s_last_kick_cyc) > IMU_STUCK_CYC;
}

imu_status_t imu_stream_start(void)
{
    s_st_samples = 0u;
    s_st_overruns = 0u;
    s_st_skipped = 0u;
    s_st_spi_errors = 0u;
    s_st_recoveries = 0u;
    s_last_kick_cyc = timebase_now();

#if IMU_USE_DMA
    if ((hspi1.hdmarx == NULL) || (hspi1.hdmatx == NULL)) {
        return IMU_ERR_SPI;   /* CubeMX sin DMA en SPI1 */
    }
    /* Direccion de lectura fija; el resto del TX son dummies en 0. */
    for (uint32_t i = 0u; i < IMU_STREAM_XFER_LEN; ++i) {
        s_dma_tx[i] = 0u;
    }
    s_dma_tx[0] = (uint8_t)(IMU_REG_OUTX_L_G | IMU_SPI_READ);
    s_dma_busy = false;
    s_unread = false;

    s_stream_active = true;
    /* INT1 puede haber quedado en alto desde imu_init() (latched, nadie
     * leyo): sin esta primera lectura no habria nunca otro flanco.      */
    imu_request_sw_kick();
#else
    s_stream_active = true;
#endif
    return IMU_OK;
}

bool imu_stream_active(void)
{
    return s_stream_active;
}

bool imu_stream_take(imu_sample_t *out)
{
    if (!s_stream_active || out == NULL) {
        return false;
    }

#if IMU_USE_DMA
    bool got = false;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_unread) {
        *out = s_latest;
        s_unread = false;
        got = true;
    }
    __set_PRIMASK(primask);

    if (got) {
        return true;
    }

    /* Burst que nunca termino (~25 us normal): se aborta y se relanza.   */
    if (s_dma_busy && ((timebase_now() - s_last_kick_cyc) > IMU_STUCK_CYC)) {
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
        if (s_dma_busy) {
            (void)HAL_SPI_Abort(&hspi1);
            imu_cs_high();
            s_dma_busy = false;
            s_st_spi_errors++;
        }
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    }

    if (imu_int1_stuck()) {
        s_st_recoveries++;
        imu_request_sw_kick();
    }
    return false;
#else
    bool want = imu_gyro_drdy_take();
    if (!want && imu_int1_stuck()) {
        s_st_recoveries++;
        want = true;
    }
    if (!want) {
        return false;
    }
    s_last_kick_cyc = timebase_now();
    if (imu_read_sample(out) != IMU_OK) {
        s_st_spi_errors++;
        return false;
    }
    s_st_samples++;
    return true;
#endif
}

void imu_stream_stats_take(imu_stream_stats_t *out)
{
    if (out == NULL) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    out->samples    = s_st_samples;
    out->overruns   = s_st_overruns;
    out->skipped    = s_st_skipped;
    out->spi_errors = s_st_spi_errors;
    out->recoveries = s_st_recoveries;
    s_st_samples = 0u;
    s_st_overruns = 0u;
    s_st_skipped = 0u;
    s_st_spi_errors = 0u;
    s_st_recoveries = 0u;
    __set_PRIMASK(primask);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != Gyro_Data_Pin) {
        return;
    }

#if IMU_USE_DMA
    /* Disparo por software (inicio del stream o recuperacion): no es un
     * flanco real de INT1, no entra en las estadisticas.                 */
    if (s_sw_kick_pending) {
        s_sw_kick_pending = false;
        imu_dma_kick();
        return;
    }
#endif

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

#if IMU_USE_DMA
    imu_dma_kick();
#endif
}
