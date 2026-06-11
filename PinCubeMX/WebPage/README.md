# Flight Controller Web Dashboard

Real-time dashboard for IMU, barometer, and magnetometer telemetry streamed from the STM32 flight controller over USB CDC serial.

## Prerequisites

- Python 3.10+
- Flight controller flashed with `TEST_SELECT_TELEMETRY` enabled in `Tests/Inc/test_runner.h`
- USB cable connected to the board's USB CDC port

## Setup

```bash
cd WebPage
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python3 app.py --port /dev/ttyACM0 --baud 115200
```

Open the dashboard in your browser:

```
http://127.0.0.1:8080/
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | `/dev/ttyACM0` | Serial device path |
| `--baud` | `115200` | Baud rate (ignored by USB CDC) |
| `--host` | `127.0.0.1` | Web server bind address |
| `--web-port` | `8080` | Web server port |
| `--no-auto-connect` | off | Start web UI without opening serial |

You can also connect from the web UI header after starting with `--no-auto-connect`.

## Firmware telemetry format

Each sample is one newline-delimited JSON object:

```json
{"t_ms":12345,"imu":{"gx":1,"gy":2,"gz":3,"ax":4,"ay":5,"az":6,"st":0},"baro":{"press_raw":123,"temp_raw":456,"st":0},"mag":{"x":7,"y":8,"z":9,"st":0}}
```

Human-readable init messages appear before the `TELEMETRY_JSON_START` marker.

## Linux serial port notes

List available ports:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

If permission is denied, add your user to the `dialout` group and log in again:

```bash
sudo usermod -aG dialout $USER
```

## Build and flash firmware

1. Ensure only `TEST_SELECT_TELEMETRY` is uncommented in `Tests/Inc/test_runner.h`.
2. Build and flash the firmware with your usual STM32 workflow.
3. Reset the board, then start this dashboard.
