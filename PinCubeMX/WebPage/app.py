#!/usr/bin/env python3
"""
Real-time flight controller dashboard.

Reads newline-delimited JSON telemetry from the STM32 USB CDC serial port
and serves live plots in the browser.
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from collections import deque
from typing import Any

from flask import Flask, Response, jsonify, render_template, request

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

app = Flask(__name__)

MAX_SAMPLES = 300
SAMPLE_FIELDS = {
    "imu_gx": deque(maxlen=MAX_SAMPLES),
    "imu_gy": deque(maxlen=MAX_SAMPLES),
    "imu_gz": deque(maxlen=MAX_SAMPLES),
    "imu_ax": deque(maxlen=MAX_SAMPLES),
    "imu_ay": deque(maxlen=MAX_SAMPLES),
    "imu_az": deque(maxlen=MAX_SAMPLES),
    "baro_press": deque(maxlen=MAX_SAMPLES),
    "baro_temp": deque(maxlen=MAX_SAMPLES),
    "mag_x": deque(maxlen=MAX_SAMPLES),
    "mag_y": deque(maxlen=MAX_SAMPLES),
    "mag_z": deque(maxlen=MAX_SAMPLES),
    "t_ms": deque(maxlen=MAX_SAMPLES),
}

latest_sample: dict[str, Any] = {}
serial_status = {
    "connected": False,
    "port": "",
    "error": "",
    "lines_parsed": 0,
    "last_line_at": 0.0,
}
state_lock = threading.Lock()
stop_event = threading.Event()
reader_thread: threading.Thread | None = None


def reset_buffers() -> None:
    with state_lock:
        for field in SAMPLE_FIELDS.values():
            field.clear()
        latest_sample.clear()
        serial_status["lines_parsed"] = 0
        serial_status["last_line_at"] = 0.0
        serial_status["error"] = ""


def append_sample(payload: dict[str, Any]) -> None:
    imu = payload.get("imu", {})
    baro = payload.get("baro", {})
    mag = payload.get("mag", {})

    with state_lock:
        latest_sample.clear()
        latest_sample.update(payload)

        SAMPLE_FIELDS["t_ms"].append(payload.get("t_ms", 0))
        SAMPLE_FIELDS["imu_gx"].append(imu.get("gx", 0))
        SAMPLE_FIELDS["imu_gy"].append(imu.get("gy", 0))
        SAMPLE_FIELDS["imu_gz"].append(imu.get("gz", 0))
        SAMPLE_FIELDS["imu_ax"].append(imu.get("ax", 0))
        SAMPLE_FIELDS["imu_ay"].append(imu.get("ay", 0))
        SAMPLE_FIELDS["imu_az"].append(imu.get("az", 0))
        SAMPLE_FIELDS["baro_press"].append(baro.get("press_raw", 0))
        SAMPLE_FIELDS["baro_temp"].append(baro.get("temp_raw", 0))
        SAMPLE_FIELDS["mag_x"].append(mag.get("x", 0))
        SAMPLE_FIELDS["mag_y"].append(mag.get("y", 0))
        SAMPLE_FIELDS["mag_z"].append(mag.get("z", 0))

        serial_status["lines_parsed"] += 1
        serial_status["last_line_at"] = time.time()


def parse_json_line(line: str) -> dict[str, Any] | None:
    line = line.strip()
    if not line.startswith("{"):
        return None
    try:
        return json.loads(line)
    except json.JSONDecodeError:
        return None


def serial_reader(port: str, baud: int) -> None:
    global serial_status

    reset_buffers()

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1.0,
        )
    except SerialException as exc:
        with state_lock:
            serial_status["connected"] = False
            serial_status["error"] = str(exc)
        return

    # DTR asserted so firmware console_init() sees an open host port.
    ser.dtr = True
    ser.rts = False

    with state_lock:
        serial_status["connected"] = True
        serial_status["port"] = port
        serial_status["error"] = ""

    buffer = ""

    try:
        while not stop_event.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue

            buffer += chunk.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                payload = parse_json_line(line)
                if payload is not None:
                    append_sample(payload)
    except SerialException as exc:
        with state_lock:
            serial_status["error"] = str(exc)
    finally:
        ser.close()
        with state_lock:
            serial_status["connected"] = False


def start_serial_thread(port: str, baud: int) -> None:
    global reader_thread

    stop_event.set()
    if reader_thread and reader_thread.is_alive():
        reader_thread.join(timeout=2.0)

    stop_event.clear()
    reader_thread = threading.Thread(
        target=serial_reader,
        args=(port, baud),
        daemon=True,
    )
    reader_thread.start()


def snapshot_state() -> dict[str, Any]:
    with state_lock:
        return {
            "latest": dict(latest_sample),
            "series": {key: list(values) for key, values in SAMPLE_FIELDS.items()},
            "status": dict(serial_status),
        }


@app.route("/")
def index() -> str:
    return render_template("index.html")


@app.route("/api/data")
def api_data() -> Response:
    return jsonify(snapshot_state())


@app.route("/api/stream")
def api_stream() -> Response:
    def event_stream():
        while True:
            payload = json.dumps(snapshot_state())
            yield f"data: {payload}\n\n"
            time.sleep(0.1)

    return Response(event_stream(), mimetype="text/event-stream")


@app.route("/api/connect", methods=["POST"])
def api_connect() -> Response:
    body = request.get_json(silent=True) or {}
    port = body.get("port") or app.config.get("SERIAL_PORT", "")
    baud = int(body.get("baud") or app.config.get("SERIAL_BAUD", 115200))

    if not port:
        return jsonify({"ok": False, "error": "Serial port is required"}), 400

    start_serial_thread(port, baud)
    time.sleep(0.2)
    return jsonify({"ok": True, "status": snapshot_state()["status"]})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flight controller web dashboard")
    parser.add_argument(
        "--port",
        default="/dev/ttyACM0",
        help="Serial device path (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate (ignored by USB CDC, default: 115200)",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="Web server bind address (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--web-port",
        type=int,
        default=8080,
        help="Web server port (default: 8080)",
    )
    parser.add_argument(
        "--no-auto-connect",
        action="store_true",
        help="Do not open the serial port automatically at startup",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app.config["SERIAL_PORT"] = args.port
    app.config["SERIAL_BAUD"] = args.baud

    if not args.no_auto_connect:
        start_serial_thread(args.port, args.baud)

    print(f"Dashboard: http://{args.host}:{args.web_port}/")
    if not args.no_auto_connect:
        print(f"Serial: {args.port} @ {args.baud}")

    app.run(host=args.host, port=args.web_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
