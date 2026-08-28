/**
 ******************************************************************************
 * @file    driver_imu.h
 * @brief   Driver de la IMU LSM6DSV16X (acelerometro + giroscopo).
 *
 *          Bus SPI1 (hardware, 5.25 MBits/s, Mode 0):
 *            PA5 (SPI1_SCK)   -> SCK  del IMU
 *            PA6 (SPI1_MISO)  -> SDO  del IMU
 *            PA7 (SPI1_MOSI)  -> SDI  del IMU
 *            PC5 (GPIO output)-> CS   del IMU (active low)
 *            PC2 (INT1 / Gyro_Data, interrupt) -> data-ready del gyro
 *      
 *          Registros utilizados (datasheet LSM6DSV16X, Rev 4):
 *            WHO_AM_I       = 0x0F  -> debe leer 0x70
 *            IF_CFG         = 0x03  -> INT active-high, push-pull, SPI-only
 *            INT1_CTRL      = 0x0D  -> INT1_DRDY_G (bit 1)
 *            CTRL1 (XL ODR) = 0x10
 *            CTRL2 (G  ODR) = 0x11
 *            CTRL3 (BDU/IF) = 0x12
 *            CTRL4          = 0x13  -> DRDY_MASK (latched, no pulsed)
 *            HAODR_CFG      = 0x62  -> HAODR_SEL=01 (tabla 8 kHz)
 *            OUTX_L_G       = 0x22..0x27 (giro X,Y,Z LSB/MSB)
 *            OUTX_L_A       = 0x28..0x2D (accel X,Y,Z LSB/MSB)
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_IMU_H_
#define TESTS_INC_DRIVER_IMU_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Identidad. */
#define IMU_REG_WHO_AM_I      0x0Fu
#define IMU_WHO_AM_I_VALUE    0x70u

/* Control registers (LSM6DSV16X layout, distinto del DSO/DSL):
 *   CTRL1 [7:4]=OP_MODE_XL [3:0]=ODR_XL  (FS_XL NO esta aca, va en CTRL8)
 *   CTRL2 [7:4]=OP_MODE_G  [3:0]=ODR_G   (FS_G  NO esta aca, va en CTRL6)
 *   CTRL3 [7]=BOOT [6]=BDU [2]=IF_INC [0]=SW_RESET
 *   CTRL6 [3:0]=FS_G    (gyro full-scale)
 *   CTRL8 [1:0]=FS_XL   (accel full-scale)
 */
#define IMU_REG_IF_CFG        0x03u  /* INT polarity / PP-OD / I2C disable */
#define IMU_REG_INT1_CTRL     0x0Du  /* ruteo de IRQs a INT1 */
#define IMU_REG_CTRL1         0x10u  /* OP_MODE_XL + ODR_XL */
#define IMU_REG_CTRL2         0x11u  /* OP_MODE_G  + ODR_G  */
#define IMU_REG_CTRL3         0x12u  /* BDU, IF_INC, SW_RESET, BOOT */
#define IMU_REG_CTRL4         0x13u  /* DRDY_MASK / DRDY_PULSED */
#define IMU_REG_CTRL6         0x15u  /* FS_G (gyro full-scale) */
#define IMU_REG_CTRL8         0x17u  /* FS_XL (accel full-scale) */
#define IMU_REG_HAODR_CFG     0x62u  /* HAODR_SEL[1:0] Table 20 */

/* HAODR_CFG (62h) bits [1:0]. 01 = set 15.625..8000 Hz. Datasheet §9.67. */
#define IMU_HAODR_SEL_8KHZ    (1u << 0)

/* CTRL1/CTRL2: OP_MODE_[2:0]=001 (HAODR) | ODR_[3:0]=1100 (8 kHz con SEL=01).
 * Bit7 debe quedar 0. 0b0001_1100 = 0x1C. Datasheet §6.5 / Table 20.        */
#define IMU_OP_MODE_HAODR     (1u << 4)
#define IMU_ODR_CODE_MAX      0x0Cu
#define IMU_CTRL_HAODR_8KHZ   (uint8_t)(IMU_OP_MODE_HAODR | IMU_ODR_CODE_MAX)
#define IMU_ODR_HZ            8000u

/* INT1_CTRL (0x0D) bit1 = INT1_DRDY_G. Bit2 y bit7 deben quedar en 0. */
#define IMU_INT1_DRDY_G       (1u << 1)

/* IF_CFG (0x03): H_LACTIVE=0 (active high), PP_OD=0 (push-pull),
 * I2C_I3C_disable=1. Default de H_LACTIVE/PP_OD ya coincide con EXTI rising. */
#define IMU_IF_CFG_I2C_I3C_DISABLE  (1u << 0)

/* CTRL4 (0x13): enmascara DRDY hasta que asiente el filtro. Latched
 * (DRDY_PULSED=0): INT1 vuelve a 0 al leer el MSB del gyro. */
#define IMU_CTRL4_DRDY_MASK   (1u << 3)

/* Status (data-ready flags). */
#define IMU_REG_STATUS        0x1Eu  /* bit0=XLDA bit1=GDA */
#define IMU_STATUS_XLDA       (1u << 0)
#define IMU_STATUS_GDA        (1u << 1)

/* Output registers. */
#define IMU_REG_OUTX_L_G      0x22u  /* gyro X LSB */
#define IMU_REG_OUTX_L_A      0x28u  /* accel X LSB */

/* Resultado de operaciones. */
typedef enum {
    IMU_OK = 0,
    IMU_ERR_SPI,
    IMU_ERR_WHO_AM_I,
    IMU_ERR_TIMEOUT
} imu_status_t;

/* Lectura de los 6 ejes (raw, signed 16-bit). */
typedef struct {
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;
} imu_sample_t;

/* ------------------------------------------------------------------------- */
/* Inicializa el IMU:                                                         */
/*  - Pone CS en alto                                                         */
/*  - Verifica WHO_AM_I                                                       */
/*  - Soft reset + espera                                                     */
/*  - Habilita IF_INC y BDU                                                   */
/*  - HAODR 8 kHz (HAODR_SEL=01, ODR code 1100), FS XL +/-4 g, G +/-500 dps   */
/*  - INT1 = gyro data-ready (INT1_CTRL.INT1_DRDY_G), latched, active high    */
/* Devuelve IMU_OK o codigo de error.                                         */
/* ------------------------------------------------------------------------- */
imu_status_t imu_init(void);

/* Lee un registro de 8 bits. */
imu_status_t imu_read_reg(uint8_t reg, uint8_t *value);

/* Escribe un registro de 8 bits. */
imu_status_t imu_write_reg(uint8_t reg, uint8_t value);

/* Lee multiples bytes a partir de reg (con auto-incremento del IMU). */
imu_status_t imu_read_burst(uint8_t reg, uint8_t *buf, uint16_t len);

/* Lee el byte WHO_AM_I y compara con el valor esperado. */
imu_status_t imu_check_who_am_i(uint8_t *who);

/* Lee gyro+accel en un solo burst. */
imu_status_t imu_read_sample(imu_sample_t *out);

/* Lee n muestras independientes (esperando INT1 gyro DRDY) y devuelve el
 * promedio. Reduce ruido por sqrt(n). Bloqueante: tarda aproximadamente
 * n * (1/ODR) ms (al ODR de 8 kHz, n=32 -> 4 ms).                        */
imu_status_t imu_read_sample_avg(imu_sample_t *out, uint8_t n_samples);

/* Estadisticas de intervalo entre flancos INT1 gyro DRDY (medidas en ISR). */
typedef struct {
    uint32_t intervals;      /* intervalos buenos medidos           */
    uint32_t edges;          /* flancos INT1 totales en la ventana  */
    uint32_t dt_avg_cyc;
    uint32_t dt_min_cyc;
    uint32_t dt_max_cyc;
    uint32_t late_events;    /* intervalos > 1.5x nominal           */
    uint32_t late_cyc;       /* suma de esos intervalos             */
    uint32_t window_cyc;     /* span real de la ventana             */
} imu_drdy_stats_t;

/* INT1 / gyro data-ready (EXTI2 sobre Gyro_Data = PC2).
 * El ISR solo setea un flag; la lectura SPI se hace en el loop.          */
bool imu_gyro_drdy_take(void);
uint32_t imu_gyro_drdy_count(void);
imu_status_t imu_wait_gyro_drdy(uint32_t timeout_ms);

/* Snapshot atomico + reset de acumuladores. No llamar desde ISR. */
void imu_gyro_drdy_stats_take(imu_drdy_stats_t *out);

/* Ultimo intervalo medido, en microsegundos. */
uint32_t imu_gyro_drdy_interval_us(void);

/* Muestras perdidas estimadas a partir de late_cyc (fuera del ISR). */
uint32_t imu_gyro_drdy_missed_estimate(uint32_t late_cyc, uint32_t late_events);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_IMU_H_ */
