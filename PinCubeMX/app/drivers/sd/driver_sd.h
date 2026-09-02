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

typedef enum {
    SD_OK = 0,
    SD_ERR_INIT,
    SD_ERR_MOUNT,
    SD_ERR_NOFS,
    SD_ERR_IO
} sd_status_t;

/* true si PB4 esta en bajo (tarjeta detectada por el switch). */
bool sd_card_detected(void);

/* HAL_SD_Init + bus 4-bit + f_mount(SDPath). Idempotente. */
sd_status_t sd_mount(void);

/* Escribe buf[len] en path (crea/sobrescribe). Requiere sd_mount() OK. */
sd_status_t sd_write_file(const char *path, const void *buf, uint32_t len);

/* Lee hasta max bytes de path. *out_len recibe los bytes leidos. */
sd_status_t sd_read_file(const char *path, void *buf, uint32_t max, uint32_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_SD_H_ */
