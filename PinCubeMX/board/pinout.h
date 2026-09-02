/**
 ******************************************************************************
 * @file    pinout.h
 * @brief   Custom PCB pin map — source of truth for aliases CubeMX omitted.
 *
 *          Labels generated in Core/Inc/main.h (M1..M4, bar_cs, Gyro_Data,
 *          leds, SD_CS) stay there. This header only adds names CubeMX did
 *          not label (IMU CS) and documents bus ownership.
 ******************************************************************************
 */

#ifndef BOARD_PINOUT_H_
#define BOARD_PINOUT_H_

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IMU LSM6DSV16X — SPI1. CS is a GPIO; CubeMX left it unlabeled (PC5). */
#define IMU_CS_Pin          GPIO_PIN_5
#define IMU_CS_GPIO_Port    GPIOC

#ifdef __cplusplus
}
#endif

#endif /* BOARD_PINOUT_H_ */
