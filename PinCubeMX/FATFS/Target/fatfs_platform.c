/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : fatfs_platform.c
  * @brief          : fatfs_platform source file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
*/
/* USER CODE END Header */
#include "fatfs_platform.h"

uint8_t	BSP_PlatformIsDetected(void) {
    uint8_t status = SD_PRESENT;
    /* Check SD card detect pin */
    if(HAL_GPIO_ReadPin(SD_DETECT_GPIO_PORT, SD_DETECT_PIN) != GPIO_PIN_RESET)
    {
        status = SD_NOT_PRESENT;
    }
    /* USER CODE BEGIN 1 */
    /* Polaridad CubeMX: LOW = tarjeta presente (switch a GND). PB4 usa el
     * pull-up de la placa (CubeMX GPIO_NOPULL). No es CS: el bus es SDIO. */
    /* USER CODE END 1 */
    return status;
}
