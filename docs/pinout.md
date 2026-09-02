# Pinout — custom FC (STM32F405RGTx)

Source of truth for unlabeled pins: `PinCubeMX/board/pinout.h`.
CubeMX labels: `PinCubeMX/Core/Inc/main.h`. Narrative: `PinCubeMX/board/board.md`.

| Pin | Function |
|-----|----------|
| PA0 / PA1 | UART4 TX/RX — CRSF 420000 |
| PA2 / PA3 / PA15 / PB3 | TIM2 CH3/CH4/CH1/CH2 — M3/M4/M1/M2 DShot |
| PA5 / PA6 / PA7 / **PC5** | SPI1 + IMU CS |
| PC2 | IMU gyro DRDY (EXTI2) |
| PB6 / PB7 | I2C1 mag |
| PB12–PB15 / PC1 | SPI2 baro CS/SCK/MISO/MOSI / INT |
| PB10 / PB11 | USART3 GPS |
| PC6 / PC7 | USART6 ESC telem |
| PC8–PC12, PD2, PB4 | SDIO + card detect |
| PA11 / PA12 | USB FS |
| PB1 / PB2 | LED red / green |
| PB0 / PC4 | ADC |
| PA13 / PA14 | SWD |
