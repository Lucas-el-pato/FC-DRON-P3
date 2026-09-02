/**
 ******************************************************************************
 * @file    fc.c
 * @brief   Flight controller — bucle principal (gyro DRDY + imu_read_gyro).
 ******************************************************************************
 */

#include "fc.h"
#include "console.h"
#include "driver_imu.h"
#include "timebase.h"
#include "stm32f4xx_hal.h"

#define FC_LOG_PERIOD_US  500000u   /* 2 Hz */

void fc_run(void)
{
    (void)timebase_init();

    
}
