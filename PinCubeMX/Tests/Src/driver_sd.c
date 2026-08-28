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
