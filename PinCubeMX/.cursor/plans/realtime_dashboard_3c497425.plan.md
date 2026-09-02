---
name: Realtime Dashboard
overview: Add a USB-serial telemetry mode to the STM32 firmware and a new Python web dashboard that reads those telemetry lines and plots IMU, barometer, and magnetometer data in real time.
todos:
  - id: firmware-telemetry
    content: Add telemetry test mode that initializes all three sensors and streams JSON lines over USB CDC serial.
    status: completed
  - id: runner-build
    content: Wire the telemetry mode into the test runner and CMake build without removing existing tests.
    status: completed
  - id: web-dashboard
    content: Create the WebPage Python Flask serial reader and live browser dashboard.
    status: completed
  - id: docs-verify
    content: Add run instructions and perform available syntax/build/lint checks.
    status: completed
isProject: false
---

# Realtime Flight Dashboard Plan

## Approach

Use the existing USB CDC console path as the serial transport. The firmware already initializes USB device support in [`PinCubeMX/Core/Src/main.c`](PinCubeMX/Core/Src/main.c), and the tests already transmit through [`PinCubeMX/Tests/Src/console.c`](PinCubeMX/Tests/Src/console.c), so this avoids new wiring assumptions and lets the Python app read the board as a normal `/dev/ttyACM*` serial device.

Current firmware entry point:

```c
/* Despacho al test seleccionado en Tests/Inc/test_runner.h. */
/* test_runner_run() inicializa la consola USB CDC y nunca retorna. */
test_runner_run();
```

Current test selection lives in [`PinCubeMX/Tests/Inc/test_runner.h`](PinCubeMX/Tests/Inc/test_runner.h). I will add one new selectable mode, for example `TEST_SELECT_TELEMETRY`, while preserving the existing IMU/baro/mag/RC/motor tests.

## Firmware Changes

Add a small telemetry module under the existing test area:

- [`PinCubeMX/Tests/Inc/test_telemetry.h`](PinCubeMX/Tests/Inc/test_telemetry.h): public `test_telemetry_run()` prototype.
- [`PinCubeMX/Tests/Src/test_telemetry.c`](PinCubeMX/Tests/Src/test_telemetry.c): initialize `imu_init()`, `baro_init()`, and `mag_init()`, then loop at a fixed rate and stream one machine-readable line per sample.
- [`PinCubeMX/Tests/Inc/test_runner.h`](PinCubeMX/Tests/Inc/test_runner.h): add `TEST_SELECT_TELEMETRY` and its prototype.
- [`PinCubeMX/Tests/Src/test_runner.c`](PinCubeMX/Tests/Src/test_runner.c): include telemetry in the single-test count and dispatch it.
- [`PinCubeMX/CMakeLists.txt`](PinCubeMX/CMakeLists.txt): add `Tests/Src/test_telemetry.c` to the build.

Telemetry line format will be newline-delimited JSON because it is easy to parse and robust for browser forwarding:

```json
{"t_ms":12345,"imu":{"gx":1,"gy":2,"gz":3,"ax":4,"ay":5,"az":6},"baro":{"press_raw":123,"temp_raw":456},"mag":{"x":7,"y":8,"z":9}}
```

If a sensor read fails, the line will include numeric status fields so the dashboard can keep running instead of blocking on one failed sensor.

## WebPage Folder

Create a new [`PinCubeMX/WebPage`](PinCubeMX/WebPage) folder with:

- [`PinCubeMX/WebPage/app.py`](PinCubeMX/WebPage/app.py): Python Flask app that opens a serial port with `pyserial`, reads JSON lines in a background thread, keeps the latest rolling samples, and serves real-time updates to the browser.
- [`PinCubeMX/WebPage/templates/index.html`](PinCubeMX/WebPage/templates/index.html): dashboard UI with live cards and simple browser-side plots for gyro, accel, barometer raw pressure/temp, and magnetometer XYZ.
- [`PinCubeMX/WebPage/requirements.txt`](PinCubeMX/WebPage/requirements.txt): minimal dependencies, likely `flask` and `pyserial`.
- [`PinCubeMX/WebPage/README.md`](PinCubeMX/WebPage/README.md): run instructions, example command, and serial port notes for Linux.

The dashboard will accept configurable serial settings, for example:

```bash
python3 app.py --port /dev/ttyACM0 --baud 115200
```

For USB CDC the baud value is mostly ignored by the device, but keeping it configurable also supports later UART use.

## Verification

After implementation I will:

- Check edited C files with lints where available.
- Run a CMake configure/build command if the local STM32 toolchain paths are usable.
- Run a Python syntax check for `WebPage/app.py`.
- If hardware is not connected, verify the web app can start and document that end-to-end serial testing still needs the flight controller connected.

## Notes

This plan keeps the existing sensor drivers unchanged unless implementation reveals a compile issue. Barometer values will initially be raw because the current BMP388 driver exposes only `press_raw` and `temp_raw`; adding compensated pressure/temperature can be a follow-up once calibration support is added to the driver.