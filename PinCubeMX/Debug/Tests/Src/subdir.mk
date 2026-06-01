################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Tests/Src/console.c \
../Tests/Src/driver_baro.c \
../Tests/Src/driver_crsf.c \
../Tests/Src/driver_gps.c \
../Tests/Src/driver_imu.c \
../Tests/Src/driver_mag.c \
../Tests/Src/driver_motors.c \
../Tests/Src/test_baro.c \
../Tests/Src/test_gps.c \
../Tests/Src/test_imu.c \
../Tests/Src/test_imu_diag.c \
../Tests/Src/test_mag.c \
../Tests/Src/test_motors.c \
../Tests/Src/test_rc.c \
../Tests/Src/test_runner.c 

OBJS += \
./Tests/Src/console.o \
./Tests/Src/driver_baro.o \
./Tests/Src/driver_crsf.o \
./Tests/Src/driver_gps.o \
./Tests/Src/driver_imu.o \
./Tests/Src/driver_mag.o \
./Tests/Src/driver_motors.o \
./Tests/Src/test_baro.o \
./Tests/Src/test_gps.o \
./Tests/Src/test_imu.o \
./Tests/Src/test_imu_diag.o \
./Tests/Src/test_mag.o \
./Tests/Src/test_motors.o \
./Tests/Src/test_rc.o \
./Tests/Src/test_runner.o 

C_DEPS += \
./Tests/Src/console.d \
./Tests/Src/driver_baro.d \
./Tests/Src/driver_crsf.d \
./Tests/Src/driver_gps.d \
./Tests/Src/driver_imu.d \
./Tests/Src/driver_mag.d \
./Tests/Src/driver_motors.d \
./Tests/Src/test_baro.d \
./Tests/Src/test_gps.d \
./Tests/Src/test_imu.d \
./Tests/Src/test_imu_diag.d \
./Tests/Src/test_mag.d \
./Tests/Src/test_motors.d \
./Tests/Src/test_rc.d \
./Tests/Src/test_runner.d 


# Each subdirectory must supply rules for building sources it contributes
Tests/Src/%.o Tests/Src/%.su Tests/Src/%.cyclo: ../Tests/Src/%.c Tests/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Core/Inc -I../Tests/Inc -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Device/ST/STM32F4xx/Include -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I/home/hernan/STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3/Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Tests-2f-Src

clean-Tests-2f-Src:
	-$(RM) ./Tests/Src/console.cyclo ./Tests/Src/console.d ./Tests/Src/console.o ./Tests/Src/console.su ./Tests/Src/driver_baro.cyclo ./Tests/Src/driver_baro.d ./Tests/Src/driver_baro.o ./Tests/Src/driver_baro.su ./Tests/Src/driver_crsf.cyclo ./Tests/Src/driver_crsf.d ./Tests/Src/driver_crsf.o ./Tests/Src/driver_crsf.su ./Tests/Src/driver_gps.cyclo ./Tests/Src/driver_gps.d ./Tests/Src/driver_gps.o ./Tests/Src/driver_gps.su ./Tests/Src/driver_imu.cyclo ./Tests/Src/driver_imu.d ./Tests/Src/driver_imu.o ./Tests/Src/driver_imu.su ./Tests/Src/driver_mag.cyclo ./Tests/Src/driver_mag.d ./Tests/Src/driver_mag.o ./Tests/Src/driver_mag.su ./Tests/Src/driver_motors.cyclo ./Tests/Src/driver_motors.d ./Tests/Src/driver_motors.o ./Tests/Src/driver_motors.su ./Tests/Src/test_baro.cyclo ./Tests/Src/test_baro.d ./Tests/Src/test_baro.o ./Tests/Src/test_baro.su ./Tests/Src/test_gps.cyclo ./Tests/Src/test_gps.d ./Tests/Src/test_gps.o ./Tests/Src/test_gps.su ./Tests/Src/test_imu.cyclo ./Tests/Src/test_imu.d ./Tests/Src/test_imu.o ./Tests/Src/test_imu.su ./Tests/Src/test_imu_diag.cyclo ./Tests/Src/test_imu_diag.d ./Tests/Src/test_imu_diag.o ./Tests/Src/test_imu_diag.su ./Tests/Src/test_mag.cyclo ./Tests/Src/test_mag.d ./Tests/Src/test_mag.o ./Tests/Src/test_mag.su ./Tests/Src/test_motors.cyclo ./Tests/Src/test_motors.d ./Tests/Src/test_motors.o ./Tests/Src/test_motors.su ./Tests/Src/test_rc.cyclo ./Tests/Src/test_rc.d ./Tests/Src/test_rc.o ./Tests/Src/test_rc.su ./Tests/Src/test_runner.cyclo ./Tests/Src/test_runner.d ./Tests/Src/test_runner.o ./Tests/Src/test_runner.su

.PHONY: clean-Tests-2f-Src

