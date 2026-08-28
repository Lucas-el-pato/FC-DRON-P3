cmake_minimum_required(VERSION 3.22)
# Enable CMake support for ASM and C languages
enable_language(C ASM)

# STM32Cube F4 firmware package path (override with STM32_CUBE_FW_F4_PATH env var).
if(DEFINED ENV{STM32_CUBE_FW_F4_PATH})
    set(STM32_CUBE_FW_F4_PATH "$ENV{STM32_CUBE_FW_F4_PATH}")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../../STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3")
    set(STM32_CUBE_FW_F4_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../../STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3")
else()
    message(FATAL_ERROR "STM32 Cube F4 FW not found. Set STM32_CUBE_FW_F4_PATH to your STM32Cube_FW_F4_V1.28.3 path.")
endif()

# STM32CubeMX generated symbols (macros)
set(MX_Defines_Syms 
	USE_HAL_DRIVER 
	STM32F405xx
    $<$<CONFIG:Debug>:DEBUG>
)

# STM32CubeMX generated include paths
set(MX_Include_Dirs
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${STM32_CUBE_FW_F4_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
    ${STM32_CUBE_FW_F4_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
    ${STM32_CUBE_FW_F4_PATH}/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${STM32_CUBE_FW_F4_PATH}/Drivers/CMSIS/Include
)

# STM32CubeMX generated application sources
set(MX_Application_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/Target/usbd_conf.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usb_device.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_desc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../USB_DEVICE/App/usbd_cdc_if.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/main.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/gpio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/adc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/dma.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/i2c.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sdio.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/spi.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/tim.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/usart.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_it.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/stm32f4xx_hal_msp.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/sysmem.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/syscalls.c
    ${CMAKE_CURRENT_SOURCE_DIR}/../../startup_stm32f405xx.s
)

# STM32 HAL/LL Drivers
set(STM32_Drivers_Src
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/system_stm32f4xx.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pcd_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_mmc.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ${STM32_CUBE_FW_F4_PATH}/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
)

# Drivers Midllewares


set(USB_Device_Library_Src
    ${STM32_CUBE_FW_F4_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_core.c
    ${STM32_CUBE_FW_F4_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    ${STM32_CUBE_FW_F4_PATH}/Middlewares/ST/STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
    ${STM32_CUBE_FW_F4_PATH}/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
)

# Link directories setup
set(MX_LINK_DIRS

)
# Project static libraries
set(MX_LINK_LIBS 
    STM32_Drivers
    ${TOOLCHAIN_LINK_LIBRARIES}
    USB_Device_Library
)
# Interface library for includes and symbols
add_library(stm32cubemx INTERFACE)
target_include_directories(stm32cubemx INTERFACE ${MX_Include_Dirs})
target_compile_definitions(stm32cubemx INTERFACE ${MX_Defines_Syms})

# Create STM32_Drivers static library
add_library(STM32_Drivers OBJECT)
target_sources(STM32_Drivers PRIVATE ${STM32_Drivers_Src})
target_link_libraries(STM32_Drivers PUBLIC stm32cubemx)


# Create USB_Device_Library static library
add_library(USB_Device_Library OBJECT)
target_sources(USB_Device_Library PRIVATE ${USB_Device_Library_Src})
target_link_libraries(USB_Device_Library PUBLIC stm32cubemx)

# Add STM32CubeMX generated application sources to the project
target_sources(${CMAKE_PROJECT_NAME} PRIVATE ${MX_Application_Src})

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ${MX_LINK_DIRS})

# Add libraries to the project
target_link_libraries(${CMAKE_PROJECT_NAME} ${MX_LINK_LIBS})

# Add the map file to the list of files to be removed with 'clean' target
set_target_properties(${CMAKE_PROJECT_NAME} PROPERTIES ADDITIONAL_CLEAN_FILES ${CMAKE_PROJECT_NAME}.map)

# Validate that STM32CubeMX code is compatible with C standard
if((CMAKE_C_STANDARD EQUAL 90) OR (CMAKE_C_STANDARD EQUAL 99))
    message(ERROR "Generated code requires C11 or higher")
endif()
