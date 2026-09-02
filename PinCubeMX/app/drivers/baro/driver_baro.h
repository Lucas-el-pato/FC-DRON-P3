/**
 ******************************************************************************
 * @file    driver_baro.h
 * @brief   Driver del barometro BMP388 por SPI2, CS=PB12 (label "bar_cs").
 *
 *          Pines (segun .ioc + esquematico):
 *            SCK   = PB13 (SPI2_SCK)
 *            MISO  = PB14 (SPI2_MISO)
 *            MOSI  = PB15 (SPI2_MOSI)
 *            CS    = PB12 (bar_cs_Pin / bar_cs_GPIO_Port)
 *            INT   = PC1  (no usado: polling)
 *
 *          Lectura SPI: addr | 0x80, despues el sensor manda 1 byte dummy y
 *          luego el dato (caracteristica especifica del BMP388).
 *          Escritura SPI: addr (sin set bit 7) seguido del dato.
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_BARO_H_
#define TESTS_INC_DRIVER_BARO_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Identidad. */
#define BARO_REG_CHIP_ID       0x00u
#define BARO_CHIP_ID_VALUE     0x50u

/* Registros principales (datasheet BMP388 Rev 1.5). */
#define BARO_REG_ERR_REG       0x02u
#define BARO_REG_STATUS        0x03u
#define BARO_REG_DATA_0        0x04u  /* press_xlsb */
#define BARO_REG_DATA_3        0x07u  /* temp_xlsb  */
#define BARO_REG_PWR_CTRL      0x1Bu
#define BARO_REG_OSR           0x1Cu
#define BARO_REG_ODR           0x1Du
#define BARO_REG_CONFIG        0x1Fu  /* IIR */
#define BARO_REG_CMD           0x7Eu  /* soft reset = 0xB6 */

#define BARO_CMD_SOFT_RESET    0xB6u

typedef enum {
    BARO_OK = 0,
    BARO_ERR_SPI,
    BARO_ERR_CHIP_ID,
    BARO_ERR_TIMEOUT
} baro_status_t;

/* Muestra cruda del BMP388 (datos sin compensar, 24 bits unsigned). */
typedef struct {
    uint32_t press_raw;
    uint32_t temp_raw;
} baro_sample_t;

/* ------------------------------------------------------------------------- */
/* Inicializa el barometro:                                                   */
/*  - CS en alto                                                              */
/*  - Verifica CHIP_ID = 0x50                                                 */
/*  - Soft reset + espera                                                     */
/*  - PWR_CTRL = 0x33 (press_en + temp_en + mode normal)                      */
/*  - OSR     = 0x0B (press x4, temp x1)                                      */
/*  - ODR     = 0x02 (50 Hz)                                                  */
/*  - CONFIG  = 0x00 (IIR off)                                                */
/* ------------------------------------------------------------------------- */
baro_status_t baro_init(void);

/* Lee un registro (1 byte). */
baro_status_t baro_read_reg(uint8_t reg, uint8_t *value);

/* Lectura burst (longitud len) a partir de reg. */
baro_status_t baro_read_burst(uint8_t reg, uint8_t *buf, uint16_t len);

/* Escribe un registro. */
baro_status_t baro_write_reg(uint8_t reg, uint8_t value);

/* Lee el byte CHIP_ID y compara con el esperado. */
baro_status_t baro_check_chip_id(uint8_t *chip_id);

/* Lee 6 bytes a partir de DATA_0: presion (24 bits) + temperatura (24 bits). */
baro_status_t baro_read_sample(baro_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_BARO_H_ */
