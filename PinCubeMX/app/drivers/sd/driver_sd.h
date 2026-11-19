/**
 ******************************************************************************
 * @file    driver_sd.h
 * @brief   SD card via SDIO 4-bit + FatFs.
 *
 *          Pines (CubeMX):
 *            PC8  SDIO_D0
 *            PC9  SDIO_D1
 *            PC10 SDIO_D2
 *            PC11 SDIO_D3
 *            PC12 SDIO_CK
 *            PD2  SDIO_CMD
 *            PB4  detect (label CubeMX: SD_CS). NO es chip-select: SDIO
 *                 no usa CS. Switch a GND con tarjeta; pull-up en la PCB.
 *
 *          MX_SDIO_SD_Init() + MX_FATFS_Init() solo configuran el handle y
 *          enlazan el driver. El enumerado de la tarjeta es sd_mount(),
 *          despues de console_init(): sin tarjeta HAL_SD_Init() bloquea
 *          varios segundos y no debe correr en el boot.
 ******************************************************************************
 */

#ifndef TESTS_INC_DRIVER_SD_H_
#define TESTS_INC_DRIVER_SD_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDIO_CK = 48 MHz / (div + 2). CubeMX deja 0 (24 MHz), pero con polling y
 * otras interrupciones activas (gyro a 8 kHz, USB) la FIFO se desborda y la
 * tarjeta no monta. 22 -> 2 MHz, ~1 MB/s en 4 bits: sobra para el logging. */
#ifndef SD_SDIO_CLK_DIV
#define SD_SDIO_CLK_DIV   22u
#endif

typedef enum {
    SD_OK = 0,
    SD_ERR_INIT,
    SD_ERR_MOUNT,
    SD_ERR_NOFS,
    SD_ERR_IO
} sd_status_t;

/* true si PB4 esta en bajo (tarjeta detectada por el switch). */
bool sd_card_detected(void);

/* HAL_SD_Init + bus 4-bit + f_mount(SDPath). Idempotente; si fallo se
 * puede volver a llamar (reintenta el init de la tarjeta). */
sd_status_t sd_mount(void);

/* Ultimo FRESULT de FatFs (0 = FR_OK, 1 = DISK_ERR, 3 = NOT_READY,
 * 13 = NO_FILESYSTEM...) y estado HAL de la tarjeta, para diagnostico. */
int sd_last_fresult(void);
uint32_t sd_last_hal_error(void);

/* Escribe buf[len] en path (crea/sobrescribe). Requiere sd_mount() OK. */
sd_status_t sd_write_file(const char *path, const void *buf, uint32_t len);

/* Lee hasta max bytes de path. *out_len recibe los bytes leidos. */
sd_status_t sd_read_file(const char *path, void *buf, uint32_t max, uint32_t *out_len);

/* ------------------------------------------------------------------------- */
/* Escritura por partes (logging). Un solo archivo abierto a la vez.          */
/* Cada append escribe en la SD con polling: no llamar desde el lazo de      */
/* control, solo desde la cola lenta y con trozos chicos (~512 B).           */
/* ------------------------------------------------------------------------- */

/* Crea (o sobrescribe) path y lo deja abierto para sd_file_append(). */
sd_status_t sd_file_create(const char *path);
sd_status_t sd_file_append(const void *buf, uint32_t len);
sd_status_t sd_file_close(void);

/* Agrega una linea al final de path (lo crea si no existe). Abre y cierra. */
sd_status_t sd_append_line(const char *path, const char *line);

/* Crea el directorio; OK si ya existia. */
sd_status_t sd_mkdir(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_SD_H_ */
