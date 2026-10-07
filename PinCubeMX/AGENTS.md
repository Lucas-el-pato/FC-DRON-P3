# PinCubeMX firmware

STM32F405RGTx custom FC. CubeMX-generated tree stays at this directory root
(`Core/`, `FATFS/`, `USB_DEVICE/`, `*.ioc`). Application code is `app/`.

## Edit here

| Path | Role |
|------|------|
| `board/` | Pin aliases + board notes |
| `app/platform/` | timebase, USB CDC console |
| `app/drivers/<dev>/` | IMU, baro, mag, motors, CRSF, GPS, SD |
| `app/sensors/` | raw → SI, boot IMU calibration, light attitude |
| `app/control/` | PID, mixer, arming, failsafe, PT1 filter |
| `app/scheduler/` | gyro loop + slow-task queue |
| `app/telemetry/` | ESC KISS serial |
| `app/fc/` | `fc_run()` flight entry, task table, RC mapping, state |
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

## Loop

Boot: `sensors_calib` (1.25 s in the gyro loop) takes gyro bias and the
power-on attitude as accel level zero; `ARMING_DISABLED_CALIBRATING` until done.
Keep the drone still while it boots.

Gyro DRDY (PC2 / EXTI2, 8 kHz) → PT1 filter → PID (ANGLE: level loop on
roll/pitch + Betaflight-scaled rate PID) → quad-X mixer →
`motors_write4()` (DShot300 on M1..M4) at 2 kHz.
Slow queue (one task per pass): CRSF RX, attitude, baro, ESC telem, CRSF telem,
log. Failsafe FSM every 10 ms.

Stage 1 default: `FC_ENABLE_PID = 0` in `app/fc/fc_tasks.h` (throttle passthrough).
Bench test before flight: `TEST_SELECT_FC_RC_MOTORS`, props off.

## Pins

See `board/pinout.h` and `board/board.md`. IMU CS is **PC5** (not labeled in CubeMX).
