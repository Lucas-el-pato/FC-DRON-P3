/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    sdio.h
  * @brief   This file contains all the function prototypes for
  *          the sdio.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDIO_H__
#define __SDIO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern SD_HandleTypeDef hsd;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_SDIO_SD_Init(void);

/* USER CODE BEGIN Prototypes */
/* MX_SDIO_SD_Init() solo carga la config; no toca la tarjeta ni aborta el boot.
 * sdio_sd_start() hace el enumerado real (bloqueante, segundos si no hay
 * tarjeta) y devuelve false en vez de colgar el firmware. Llamarlo desde el
 * test/logger que necesite la SD, nunca antes de levantar la consola USB.   */
bool sdio_sd_start(void);
bool sdio_sd_is_ready(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SDIO_H__ */

