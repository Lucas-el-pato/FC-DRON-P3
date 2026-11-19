/**
 ******************************************************************************
 * @file    driver_sd.c
 * @brief   SDIO + FatFs: detect, mount, read/write. Ver driver_sd.h.
 ******************************************************************************
 */

#include "driver_sd.h"
#include "fatfs.h"
#include "sdio.h"
#include "bsp_driver_sd.h"

#include <string.h>

static bool s_mounted = false;

bool sd_card_detected(void)
{
    return BSP_PlatformIsDetected() == SD_PRESENT;
}

/* Reemplaza el BSP_SD_Init() weak de CubeMX: aquel aborta si el pin de
 * detect no esta en bajo, y en esta placa PB4 puede no tener switch
 * mecanico (el label es SD_CS). El detect queda como dato de consola;
 * el init intenta enumerar la tarjeta igual. */
uint8_t BSP_SD_Init(void)
{
    if (hsd.State == HAL_SD_STATE_READY) {
        return MSD_OK;
    }

    if (HAL_SD_Init(&hsd) != HAL_OK) {
        return MSD_ERROR;
    }
    if (HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B) != HAL_OK) {
        return MSD_ERROR;
    }
    return MSD_OK;
}

sd_status_t sd_mount(void)
{
    if (s_mounted) {
        return SD_OK;
    }

    /* opt=1: monta ya y llama SD_initialize() -> BSP_SD_Init(). */
    FRESULT fr = f_mount(&SDFatFS, SDPath, 1);
    if (fr == FR_NO_FILESYSTEM) {
        return SD_ERR_NOFS;
    }
    if (fr != FR_OK) {
        return (fr == FR_NOT_READY) ? SD_ERR_INIT : SD_ERR_MOUNT;
    }
    s_mounted = true;
    return SD_OK;
}

sd_status_t sd_write_file(const char *path, const void *buf, uint32_t len)
{
    FIL fp;
    UINT written = 0u;
    FRESULT fr;

    if ((path == NULL) || (buf == NULL)) {
        return SD_ERR_IO;
    }

    fr = f_open(&fp, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        return SD_ERR_IO;
    }
    fr = f_write(&fp, buf, (UINT)len, &written);
    (void)f_close(&fp);
    if ((fr != FR_OK) || (written != (UINT)len)) {
        return SD_ERR_IO;
    }
    return SD_OK;
}

sd_status_t sd_read_file(const char *path, void *buf, uint32_t max, uint32_t *out_len)
{
    FIL fp;
    UINT n = 0u;
    FRESULT fr;

    if ((path == NULL) || (buf == NULL) || (out_len == NULL)) {
        return SD_ERR_IO;
    }
    *out_len = 0u;

    fr = f_open(&fp, path, FA_READ);
    if (fr != FR_OK) {
        return SD_ERR_IO;
    }
    fr = f_read(&fp, buf, (UINT)max, &n);
    (void)f_close(&fp);
    if (fr != FR_OK) {
        return SD_ERR_IO;
    }
    *out_len = (uint32_t)n;
    return SD_OK;
}

/* ------------------------------------------------------------------------- */
/* Archivo contiguo                                                           */
/* ------------------------------------------------------------------------- */
sd_status_t sd_contig_create(FIL *fp, const char *path, uint32_t bytes,
                             uint32_t *first_lba)
{
    if ((fp == NULL) || (path == NULL) || (first_lba == NULL) || (bytes == 0u)) {
        return SD_ERR_IO;
    }
    if (!s_mounted) {
        return SD_ERR_MOUNT;
    }

    if (f_open(fp, path, FA_CREATE_NEW | FA_WRITE) != FR_OK) {
        return SD_ERR_IO;
    }

    /* f_lseek mas alla del final agranda el archivo sin escribir datos.
     * create_chain toma el siguiente cluster libre: en una tarjeta sin
     * fragmentar queda contiguo. Se verifica abajo.                      */
    FRESULT fr = f_lseek(fp, (FSIZE_t)bytes);
    if ((fr != FR_OK) || (f_tell(fp) != (FSIZE_t)bytes)) {
        (void)f_close(fp);
        (void)f_unlink(path);
        return SD_ERR_IO;
    }
    /* Entrada de directorio con el tamano completo ya en la tarjeta: si se
     * corta la bateria en vuelo el archivo existe (el decoder corta donde
     * terminan los registros validos).                                   */
    if (f_sync(fp) != FR_OK) {
        (void)f_close(fp);
        (void)f_unlink(path);
        return SD_ERR_IO;
    }

    /* Mapa de clusters (fast seek): con tabla de 4 DWORD entra un solo
     * fragmento -> contiguo. Mas fragmentos = FR_NOT_ENOUGH_CORE.        */
    DWORD clmt[4] = { 4u, 0u, 0u, 0u };
    fp->cltbl = clmt;
    fr = f_lseek(fp, CREATE_LINKMAP);
    fp->cltbl = NULL;
    if (fr != FR_OK) {
        (void)f_close(fp);
        (void)f_unlink(path);
        return SD_ERR_IO;   /* fragmentado: formatear la tarjeta */
    }

    const FATFS *fs = fp->obj.fs;
    *first_lba = (uint32_t)(fs->database + (clmt[2] - 2u) * (DWORD)fs->csize);
    return SD_OK;
}

sd_status_t sd_contig_close(FIL *fp, uint32_t bytes)
{
    if (fp == NULL) {
        return SD_ERR_IO;
    }
    fp->cltbl = NULL;

    /* Los datos se escribieron por fuera de FatFs: su buffer no esta sucio
     * (nunca se escribio con f_write), asi que no pisa nada.             */
    FRESULT fr = f_lseek(fp, (FSIZE_t)bytes);
    if (fr == FR_OK) {
        fr = f_truncate(fp);
    }
    const FRESULT fc = f_close(fp);
    return ((fr == FR_OK) && (fc == FR_OK)) ? SD_OK : SD_ERR_IO;
}

uint32_t sd_free_bytes(void)
{
    if (!s_mounted) {
        return 0u;
    }
    FATFS *fs = NULL;
    DWORD nclst = 0u;
    if (f_getfree(SDPath, &nclst, &fs) != FR_OK || fs == NULL) {
        return 0u;
    }
    const uint64_t b = (uint64_t)nclst * (uint64_t)fs->csize * 512u;
    return (b > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)b;
}

/* ------------------------------------------------------------------------- */
/* Escritura raw por DMA                                                      */
/* ------------------------------------------------------------------------- */
static bool s_raw_active = false;

sd_status_t sd_raw_write_start(const void *buf, uint32_t lba, uint32_t nblocks)
{
    if ((buf == NULL) || (nblocks == 0u) || (((uint32_t)buf & 3u) != 0u)) {
        return SD_ERR_IO;
    }
    if (!s_mounted || s_raw_active || (HAL_SD_GetState(&hsd) != HAL_SD_STATE_READY)) {
        return SD_ERR_IO;
    }
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t *)buf, lba, nblocks) != HAL_OK) {
        return SD_ERR_IO;
    }
    s_raw_active = true;
    return SD_OK;
}

sd_raw_state_t sd_raw_poll(void)
{
    if (!s_raw_active) {
        return SD_RAW_IDLE;
    }
    /* DMA + DATAEND (+ CMD12) en curso: lo cierra HAL_SD_IRQHandler. */
    if (HAL_SD_GetState(&hsd) == HAL_SD_STATE_BUSY) {
        return SD_RAW_BUSY;
    }
    if (hsd.ErrorCode != HAL_SD_ERROR_NONE) {
        s_raw_active = false;
        return SD_RAW_ERROR;
    }
    /* La tarjeta sigue grabando internamente despues del ultimo bloque. */
    const HAL_SD_CardStateTypeDef cs = HAL_SD_GetCardState(&hsd);
    if ((cs == HAL_SD_CARD_PROGRAMMING) || (cs == HAL_SD_CARD_RECEIVING)) {
        return SD_RAW_BUSY;
    }
    s_raw_active = false;
    return (cs == HAL_SD_CARD_TRANSFER) ? SD_RAW_DONE : SD_RAW_ERROR;
}
