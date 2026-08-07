/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN Private defines */

/* CRT (ESC amp sense) -> PC4 / ADC1_IN14. Escala fabricante: 11.75 mV/A. */
#define ADC_CRT_CHANNEL          ADC_CHANNEL_14
#define ADC_CRT_MV_PER_A_X100    1175u  /* 11.75 mV/A * 100 */
#define ADC_VREF_MV              3300u
#define ADC_MAX_COUNTS           4095u

/* USER CODE END Private defines */

void MX_ADC1_Init(void);

/* USER CODE BEGIN Prototypes */

/* Lectura blocking del pin CRT (PC4). Devuelve counts 12-bit o 0xFFFF si falla. */
uint16_t adc_crt_read_raw(void);

/* Convierte counts ADC a centiamperios (100 = 1.00 A). raw 0xFFFF -> 0. */
uint16_t adc_crt_raw_to_ca(uint16_t raw);

/* Corriente CRT en centiamperios (100 = 1.00 A). */
uint16_t adc_crt_read_ca(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

