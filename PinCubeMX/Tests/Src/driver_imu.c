/**
 ******************************************************************************
 * @file    driver_imu.c
 * @brief   Driver del IMU LSM6DSV16X por SPI bit-bang (GPIO).
 *
 *          IMPORTANTE: la PCB tiene MISO y MOSI cruzados a nivel de pistas
 *          respecto a la asignacion AF de CubeMX. Para evitar rework de
 *          hardware se emula SPI Mode 0 (CPOL=0, CPHA=0) en software, lo que
 *          permite reasignar los roles logicos de los pines:
 *
 *            CubeMX label   Pin    Rol logico (bit-bang)   Conecta a
 *            ------------   ----   --------------------   ----------
 *            SPI1_SCK       PA5    SCK (output)           SCK del IMU
 *            SPI1_MISO      PA6    MOSI (output)          SDI del IMU
 *            SPI1_MOSI      PA7    MISO (input)           SDO del IMU
 *            (GPIO out)     PC5    CS  (output, active L) CS  del IMU
 *
 *          El periferico SPI1 sigue inicializandose por CubeMX pero no se
 *          usa; los pines se sobrescriben a GPIO en imu_init().
 *
 *          Velocidad efectiva de SCK: ~1 MHz (suficiente para LSM6DSV16X
 *          a 104 Hz y muy por debajo de su maximo de 10 MHz).
 ******************************************************************************
 */

#include "driver_imu.h"
#include "stm32f4xx_hal.h"

#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* Mapeo de pines fisicos.                                                    */
/* ------------------------------------------------------------------------- */
#define IMU_SCK_PORT     GPIOA
#define IMU_SCK_PIN      GPIO_PIN_5

/* PA6: en CubeMX figura como SPI1_MISO, pero la pista llega al SDI del IMU.
 * En bit-bang lo usamos como MOSI (output desde STM32 hacia IMU).            */
#define IMU_MOSI_PORT    GPIOA
#define IMU_MOSI_PIN     GPIO_PIN_6

/* PA7: en CubeMX figura como SPI1_MOSI, pero la pista llega al SDO del IMU.
 * En bit-bang lo usamos como MISO (input desde IMU hacia STM32).             */
#define IMU_MISO_PORT    GPIOA
#define IMU_MISO_PIN     GPIO_PIN_7

/* CS del IMU en PC5 (igual que antes). */
#define IMU_CS_PORT      GPIOC
#define IMU_CS_PIN       GPIO_PIN_5

/* Bit de lectura SPI del LSM6DSV16X: addr | 0x80. */
#define IMU_SPI_READ     0x80u

/* ------------------------------------------------------------------------- */
/* Macros rapidos via BSRR/IDR (evitan overhead de HAL_GPIO_WritePin).        */
/* ------------------------------------------------------------------------- */
#define SCK_HIGH()   (IMU_SCK_PORT->BSRR  = IMU_SCK_PIN)
#define SCK_LOW()    (IMU_SCK_PORT->BSRR  = ((uint32_t)IMU_SCK_PIN  << 16))
#define MOSI_HIGH()  (IMU_MOSI_PORT->BSRR = IMU_MOSI_PIN)
#define MOSI_LOW()   (IMU_MOSI_PORT->BSRR = ((uint32_t)IMU_MOSI_PIN << 16))
#define MISO_READ()  ((IMU_MISO_PORT->IDR & IMU_MISO_PIN) ? 1u : 0u)
#define CS_HIGH()    (IMU_CS_PORT->BSRR   = IMU_CS_PIN)
#define CS_LOW()     (IMU_CS_PORT->BSRR   = ((uint32_t)IMU_CS_PIN   << 16))

/* Pequeno delay para fijar SCK en ~1 MHz a SYSCLK 168 MHz.
 * Ajustable: subir el limite del for ralentiza el clock.                    */
static inline void spi_bb_delay(void)
{
    for (volatile uint32_t i = 0u; i < 8u; ++i) {
        __NOP();
    }
}

/* ------------------------------------------------------------------------- */
/* Reconfigura PA5/PA6/PA7 de AF (SPI1) a GPIO normal.                        */
/* Idempotente: la primera llamada hace el trabajo real, las siguientes son   */
/* no-op. Lo importante es que CUALQUIER funcion publica del driver llama a   */
/* imu_bb_ensure_pins() en su entrada, asi no hace falta llamar a imu_init()  */
/* primero para que el bus quede operativo.                                   */
/* ------------------------------------------------------------------------- */
static bool s_pins_ready = false;

static void imu_bb_ensure_pins(void)
{
    if (s_pins_ready) {
        return;
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gi = { 0 };

    /* SCK = PA5 output push-pull. */
    gi.Pin   = IMU_SCK_PIN;
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_SCK_PORT, &gi);

    /* MOSI logico = PA6 output push-pull. */
    gi.Pin = IMU_MOSI_PIN;
    HAL_GPIO_Init(IMU_MOSI_PORT, &gi);

    /* MISO logico = PA7 input. Pull-up suave para que la linea no flote si
     * el IMU esta deseleccionado (CS alto) y deja el SDO en alta impedancia. */
    gi.Pin  = IMU_MISO_PIN;
    gi.Mode = GPIO_MODE_INPUT;
    gi.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IMU_MISO_PORT, &gi);

    /* Estado de reposo SPI mode 0: SCK low, MOSI low, CS alto. */
    SCK_LOW();
    MOSI_LOW();
    CS_HIGH();

    /* Pequena espera para que el IMU se asiente despues de los toggles
     * que produjo la salida de modo AF.                                      */
    HAL_Delay(2u);

    s_pins_ready = true;
}

/* ------------------------------------------------------------------------- */
/* spi_bb_xfer_byte                                                           */
/* Transfiere 1 byte (MSB primero) en SPI Mode 0:                             */
/*   - CPOL=0: SCK idle low                                                   */
/*   - CPHA=0: dato se setea con SCK low, se samplea en flanco de subida      */
/* Devuelve el byte recibido por MISO durante la misma transferencia.         */
/* ------------------------------------------------------------------------- */
static uint8_t spi_bb_xfer_byte(uint8_t out)
{
    uint8_t in = 0u;

    for (int8_t bit = 7; bit >= 0; --bit) {
        /* 1. Setup MOSI con el bit actual mientras SCK sigue en low. */
        if ((out >> bit) & 0x01u) {
            MOSI_HIGH();
        } else {
            MOSI_LOW();
        }
        spi_bb_delay();

        /* 2. Flanco de subida: el slave samplea MOSI, nosotros MISO. */
        SCK_HIGH();
        spi_bb_delay();

        if (MISO_READ() != 0u) {
            in |= (uint8_t)(1u << bit);
        }

        /* 3. Flanco de bajada: el slave actualiza SDO para el proximo bit. */
        SCK_LOW();
    }

    return in;
}

/* ------------------------------------------------------------------------- */
/* CS helpers (con pequenos margenes de setup/hold).                          */
/* ------------------------------------------------------------------------- */
static inline void imu_cs_low(void)
{
    CS_LOW();
    spi_bb_delay();  /* tsu(CS) */
}

static inline void imu_cs_high(void)
{
    spi_bb_delay();  /* th(CS) */
    CS_HIGH();
}

/* ------------------------------------------------------------------------- */
/* imu_read_reg                                                               */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_reg(uint8_t reg, uint8_t *value)
{
    if (value == NULL) {
        return IMU_ERR_SPI;
    }

    imu_bb_ensure_pins();
    imu_cs_low();
    (void)spi_bb_xfer_byte((uint8_t)(reg | IMU_SPI_READ));
    *value = spi_bb_xfer_byte(0x00u);
    imu_cs_high();

    return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* imu_write_reg                                                              */
/* ------------------------------------------------------------------------- */
imu_status_t imu_write_reg(uint8_t reg, uint8_t value)
{
    imu_bb_ensure_pins();
    imu_cs_low();
    (void)spi_bb_xfer_byte((uint8_t)(reg & 0x7Fu));
    (void)spi_bb_xfer_byte(value);
    imu_cs_high();

    return IMU_OK;
}

/* ------------------------------------------------------------------------- */
/* imu_read_burst                                                             */
/* ------------------------------------------------------------------------- */
imu_status_t imu_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0u) {
        return IMU_ERR_SPI;
    }

    imu_bb_ensure_pins();
    imu_cs_low();
    (void)spi_bb_xfer_byte((uint8_t)(reg | IMU_SPI_READ));
    for (uint16_t i = 0u; i < len; ++i) {
        buf[i] = spi_bb_xfer_byte(0x00u);
    }
    imu_cs_high();

    return IMU_OK;
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
    /* 0. Asegurar que los pines bit-bang estan listos (idempotente).
     *    Cualquier call previa a imu_read_reg / imu_check_who_am_i ya lo
     *    habria hecho, pero es seguro re-llamar.                          */
    imu_bb_ensure_pins();

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

    /* 5. CTRL1 (XL): OP_MODE_XL=0000 (high-performance) | ODR_XL=0110 (120 Hz)
     *    -> 0x06. En LSM6DSV16X las FS NO van aca (ver CTRL8).
     *    OJO: el chip viejo LSM6DSO usaba [7:4]=ODR y [3:2]=FS aca, pero
     *    el DSV16X cambio el layout y poner 0x42 ponia al XL en low-power.   */
    st = imu_write_reg(IMU_REG_CTRL1, 0x06u);
    if (st != IMU_OK) {
        return st;
    }

    /* 6. CTRL2 (G): OP_MODE_G=0000 (high-performance) | ODR_G=0110 (120 Hz)
     *    -> 0x06. Ponerlo en 0x41 dejaba OP_MODE_G=0100 que es SLEEP MODE,
     *    por eso el gyro devolvia 0,0,0 antes de este fix.                   */
    st = imu_write_reg(IMU_REG_CTRL2, 0x06u);
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
