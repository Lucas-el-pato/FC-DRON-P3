# Architecture

Custom firmware for the STM32F405RGTx flight controller PCB. Inspired by
Betaflight layering (see `Diagrama_logica_Betaflight.mmd`), not a fork.

## Runtime

1. CubeMX `main()` inits clocks, GPIO, SPI, UART, TIM, USB, SDIO.
2. CS lines idle high; gyro EXTI disabled until `imu_init()`.
3. `fc_run()` (default) or a single bring-up test.
4. Inner loop (gyro DRDY on PC2/EXTI2, 8 kHz): read gyro → PT1 filter →
   rate PID → quad-X mixer → `motors_write4()` DShot300 at 8 kHz / 4 = 2 kHz.
5. Cooperative slow queue, one task per pass by dynamic priority: RX/CRSF,
   attitude, baro, ESC telemetry, CRSF telemetry TX, console log.
   Failsafe FSM runs on its own 10 ms clock.

Stage 1 ships with `FC_ENABLE_PID = 0` (`app/fc/fc_tasks.h`): the throttle stick
drives all four motors, PID corrections are zero. Mag and GPS tasks are off by
default because their drivers block the loop (I2C polling / UART timeout).

## Module map vs Betaflight (`src/main`)

| Betaflight | Here | Contents |
|---|---|---|
| `fc/init.c`, `fc/core.c` | `app/fc/fc.c` | phased init + `fc_run()` |
| `fc/tasks.c` | `app/fc/fc_tasks.c` | realtime stages + slow task table |
| `fc/rc.c`, `fc/rc_controls.c` | `app/fc/fc_rc.c` | CRSF channels → setpoints |
| `fc/runtime_config.c` | `app/fc/fc_state.c` | flight state + counters |
| `scheduler/scheduler.c` | `app/scheduler/scheduler.c` | gyro loop + slow queue |
| `flight/pid.c` | `app/control/pid.c` | rate PID |
| `flight/mixer.c` | `app/control/mixer.c` | quad-X mix → DShot |
| `flight/failsafe.c` | `app/control/failsafe.c` | RX-loss FSM |
| `common/filter.c` | `app/control/filter.c` | PT1 |
| — | `app/control/arming.c` | arming + disable flags (BF: in `fc/core.c`) |
| `blackbox/`, `cms/`, `msp/`, `osd/`, `pg/`, `cli/` | — | out of scope |

No EEPROM config profiles, MSP/CLI, blackbox, dyn-notch or RPM filter: gains and
rates are compile-time constants in the module headers.

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
