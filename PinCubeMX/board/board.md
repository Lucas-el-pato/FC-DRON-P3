# Board — STM32F405RGTx custom FC

MCU: STM32F405RGTx @ 168 MHz. Canonical CubeMX project: `PinCubeMX.ioc`.
`docs/NewPinCube-Drone.ioc` is an unused snapshot; do not regenerate from it.

HAL is **not** vendored. Build needs `STM32_CUBE_FW_F4_PATH` or the default
`STM32Cube/Repository/STM32Cube_FW_F4_V1.28.3` relative to the workspace.

## Buses

| Bus | Pins | Device |
|-----|------|--------|
| SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI, **PC5 CS**, PC2 INT1 | LSM6DSV16X IMU |
| SPI2 | PB13 SCK, PB14 MISO, PB15 MOSI, PB12 CS, PC1 INT | BMP388 baro |
| I2C1 | PB6 SCL, PB7 SDA | MMC5983MA mag |
| UART4 420000 | PA0 TX, PA1 RX | CRSF / ELRS SuperD |
| USART3 115200 | PB10 TX, PB11 RX | GPS MAX-M10M |
| USART6 115200 | PC6 TX, PC7 RX | ESC KISS telemetry |
| TIM2 DShot | PA15 M1, PB3 M2, PA2 M3, PA3 M4 | ESCs |
| SDIO 4-bit | PC8–PC12, PD2, PB4 detect | microSD |
| USB FS | PA11 DM, PA12 DP | CDC console |
| ADC | PB0 IN8, PC4 IN14 | battery / analog |
| LEDs | PB1 red, PB2 green | status |

Gyro ODR: 8 kHz HAODR. Baro ODR: 50 Hz. Mag: I2C 0x30.

Edit `app/` and `tests/`. Do not edit CubeMX-generated files outside `USER CODE` blocks.
