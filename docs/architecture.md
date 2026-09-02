# Architecture

Custom firmware for the STM32F405RGTx flight controller PCB. Inspired by
Betaflight layering (see `Diagrama_logica_Betaflight.mmd`), not a fork.

## Runtime (target)

1. CubeMX `main()` inits clocks, GPIO, SPI, UART, TIM, USB, SDIO.
2. CS lines idle high; gyro EXTI disabled until `imu_init()`.
3. `fc_run()` (default) or a single bring-up test.
4. Intended inner loop: IMU DRDY → read gyro → filter → rate PID → mixer → DShot.
5. Cooperative slow tasks: RX/CRSF, failsafe, baro/mag, telemetry, SD.

`app/control/` and `app/scheduler/` are placeholders. Drivers and tests already exist.

## Source ownership

| Tree | Owner |
|------|--------|
| `PinCubeMX/Core`, `FATFS`, `USB_DEVICE` | STM32CubeMX (`USER CODE` is ours) |
| `PinCubeMX/app` | application |
| `PinCubeMX/tests` | hardware bring-up |
| `PinCubeMX/board` | pin/board contract |
| `hardware/` | KiCad |
| `tools/` | host Python (dashboard, CRSF RX) |

## Related diagrams

- `Diagrama_logica_Betaflight.mmd` — scheduler / PID task model
- `Diagrama de bloques FC Dron.drawio.svg` — board block diagram
