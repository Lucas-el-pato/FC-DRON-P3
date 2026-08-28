/**
 ******************************************************************************
 * @file    test_sd.c
 * @brief   Test de la microSD: pin de detect, mount FatFs, write/read.
 ******************************************************************************
 */

#include "console.h"
#include "driver_sd.h"
#include "bsp_driver_sd.h"

#include <string.h>

void test_sd_run(void)
{
    console_banner("SDIO + FatFs");

    const GPIO_PinState det_pin = HAL_GPIO_ReadPin(SD_CS_GPIO_Port, SD_CS_Pin);
    console_printf("PB4 detect (SD_CS) = %s (LOW = presente)\r\n",
                   (det_pin == GPIO_PIN_RESET) ? "LOW" : "HIGH");
    console_printf("sd_card_detected() = %s\r\n",
                   sd_card_detected() ? "si" : "no");

    sd_status_t st = sd_mount();
    if (st != SD_OK) {
        console_printf("sd_mount fallo (codigo=%d). Sin tarjeta, cableado SDIO\r\n",
                       (int)st);
        console_printf("o filesystem. Formatear FAT32 si codigo=%d (NOFS).\r\n",
                       (int)SD_ERR_NOFS);
        console_result(false, "no se pudo montar la SD");
        while (1) {
            HAL_Delay(500u);
        }
    }

    HAL_SD_CardInfoTypeDef info = {0};
    BSP_SD_GetCardInfo(&info);
    console_printf("card blocks=%lu size=%lu bytes\r\n",
                   (unsigned long)info.LogBlockNbr,
                   (unsigned long)info.LogBlockSize);

    static const char kMsg[] = "FC-DRON-P3 SDIO OK\r\n";
    st = sd_write_file("FC_TEST.TXT", kMsg, (uint32_t)(sizeof(kMsg) - 1u));
    if (st != SD_OK) {
        console_result(false, "f_write FC_TEST.TXT fallo");
        while (1) {
            HAL_Delay(500u);
        }
    }

    char buf[32];
    uint32_t n = 0u;
    memset(buf, 0, sizeof(buf));
    st = sd_read_file("FC_TEST.TXT", buf, sizeof(buf) - 1u, &n);
    if (st != SD_OK) {
        console_result(false, "f_read FC_TEST.TXT fallo");
        while (1) {
            HAL_Delay(500u);
        }
    }

    const bool match = (n == (sizeof(kMsg) - 1u)) &&
                       (memcmp(buf, kMsg, (size_t)n) == 0);
    console_printf("leido %lu bytes: '%s'\r\n", (unsigned long)n, buf);
    console_result(match, match ? "write/read OK" : "contenido no coincide");

    while (1) {
        console_printf("detect=%s  mounted=OK\r\n",
                       sd_card_detected() ? "si" : "no");
        HAL_Delay(1000u);
    }
}
