# PinCubeMX firmware

STM32F405RGTx custom FC. CubeMX-generated tree stays at this directory root
(`Core/`, `FATFS/`, `USB_DEVICE/`, `*.ioc`). Application code is `app/`.

## Edit here

| Path | Role |
|------|------|
| `board/` | Pin aliases + board notes |
| `app/platform/` | timebase, USB CDC console |
| `app/drivers/<dev>/` | IMU, baro, mag, motors, CRSF, GPS, SD |
| `app/sensors/` | raw → SI + light attitude |
| `app/control/` | PID, mixer, arming, failsafe (empty until implemented) |
| `app/scheduler/` | gyro loop + slow tasks (empty until implemented) |
| `app/telemetry/` | ESC KISS serial |
| `app/fc/` | `fc_run()` flight entry |
| `tests/` | bring-up only (`TEST_SELECT_*` in `test_runner.h`) |

## Do not edit by hand

Generated: `Core/`, `FATFS/`, `USB_DEVICE/`, `cmake/stm32cubemx/`, linker scripts,
startup, `.ioc` (except peripheral config in CubeMX). HAL is linked from
`STM32_CUBE_FW_F4_PATH`, not copied into this repo.

## Dispatch

`Core/Src/main.c` (USER CODE): CubeMX inits peripherals, then:

- if any `TEST_SELECT_*` → `test_runner_run()`
- else → `fc_run()`

One test macro at a time. Default is flight (`fc_run`).

## Loop target (not fully implemented)

Gyro DRDY (PC2 / EXTI2) → filter → PID → mixer → DShot.
Slower: CRSF, baro/mag, telem, failsafe.

## Pins

See `board/pinout.h` and `board/board.md`. IMU CS is **PC5** (not labeled in CubeMX).
