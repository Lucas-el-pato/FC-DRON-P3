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
#include "ff.h"
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

/* ------------------------------------------------------------------------- */
/* Archivo contiguo + escritura raw (blackbox).                               */
/*                                                                            */
/* FatFs espera a la tarjeta en cada escritura (ms, a veces cientos de ms):   */
/* no se puede usar con el lazo de vuelo corriendo. El blackbox reserva un    */
/* archivo contiguo con FatFs (desarmado) y en vuelo escribe sus sectores     */
/* directo por DMA, sin esperar nunca.                                        */
/* ------------------------------------------------------------------------- */

/* Crea path (no debe existir) de 'bytes' de largo en clusters contiguos y
 * devuelve el primer sector (LBA). fp queda abierto. Bloqueante.          */
sd_status_t sd_contig_create(FIL *fp, const char *path, uint32_t bytes,
                             uint32_t *first_lba);

/* Recorta fp a 'bytes' y lo cierra. Bloqueante. */
sd_status_t sd_contig_close(FIL *fp, uint32_t bytes);

/* Espacio libre en bytes (satura en 4 GB). */
uint32_t sd_free_bytes(void);

typedef enum {
    SD_RAW_IDLE = 0,   /* nada en curso                              */
    SD_RAW_BUSY,       /* DMA en curso o tarjeta programando         */
    SD_RAW_DONE,       /* la ultima escritura termino bien           */
    SD_RAW_ERROR       /* la ultima escritura fallo                  */
} sd_raw_state_t;

/* Arranca la escritura de nblocks (512 B) desde buf (alineado a 4, RAM
 * normal) en el sector lba. No bloquea.                                   */
sd_status_t sd_raw_write_start(const void *buf, uint32_t lba, uint32_t nblocks);

/* Estado de la escritura raw. DONE / ERROR se informan una sola vez.
 * Puede mandar un CMD13 (~20 us) cuando el DMA ya termino.               */
sd_raw_state_t sd_raw_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* TESTS_INC_DRIVER_SD_H_ */
