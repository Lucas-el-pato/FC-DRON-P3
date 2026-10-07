#!/usr/bin/env python3
"""
Real-time flight controller dashboard.

Two data sources:
  json  - newline-delimited JSON from the STM32 USB CDC port (bench tests),
          shown at /.
  crsf  - CRSF telemetry from the radio's USB port (EdgeTX Telemetry Mirror),
          i.e. the drone in flight over ELRS, shown at /gcs.
  sim   - synthetic CRSF stream (no hardware), shown at /gcs.
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
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

from crsf_link import CrsfParser, TelemetryStore
from crsf_sim import Simulator

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


crsf_store = TelemetryStore()
crsf_status = {
    "connected": False,
    "source": "",
    "port": "",
    "error": "",
}


def crsf_reader(port: str, baud: int) -> None:
    """Radio USB VCP (Telemetry Mirror) -> CRSF parser -> crsf_store."""
    crsf_store.reset()
    parser = CrsfParser()

    try:
        ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
    except SerialException as exc:
        with state_lock:
            crsf_status.update(connected=False, error=str(exc))
        return

    with state_lock:
        crsf_status.update(connected=True, source="crsf", port=port, error="")

    try:
        while not stop_event.is_set():
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            crsf_store.bytes_rx += len(chunk)
            for ftype, payload in parser.feed(chunk):
                crsf_store.add(ftype, payload)
            crsf_store.frames_ok = parser.ok
            crsf_store.crc_err = parser.crc_err
    except SerialException as exc:
        with state_lock:
            crsf_status["error"] = str(exc)
    finally:
        ser.close()
        with state_lock:
            crsf_status["connected"] = False


def sim_reader(_port: str, _baud: int) -> None:
    """Synthetic CRSF bytes through the same parser as the real link."""
    crsf_store.reset()
    parser = CrsfParser()
    sim = Simulator()
    t0 = time.time()

    with state_lock:
        crsf_status.update(connected=True, source="sim", port="simulator", error="")

    while not stop_event.is_set():
        chunk = sim.step(time.time() - t0)
        if chunk:
            crsf_store.bytes_rx += len(chunk)
            for ftype, payload in parser.feed(chunk):
                crsf_store.add(ftype, payload)
            crsf_store.frames_ok = parser.ok
            crsf_store.crc_err = parser.crc_err
        time.sleep(0.01)

    with state_lock:
        crsf_status["connected"] = False


READERS = {"json": serial_reader, "crsf": crsf_reader, "sim": sim_reader}


def start_serial_thread(port: str, baud: int, source: str = "json") -> None:
    global reader_thread

    stop_event.set()
    if reader_thread and reader_thread.is_alive():
        reader_thread.join(timeout=2.0)

    stop_event.clear()
    reader_thread = threading.Thread(
        target=READERS[source],
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


def gcs_snapshot() -> dict[str, Any]:
    snap = crsf_store.snapshot()
    with state_lock:
        snap["status"] = dict(crsf_status)
    return snap


@app.route("/gcs")
def gcs() -> str:
    return render_template("gcs.html")


@app.route("/api/gcs/data")
def api_gcs_data() -> Response:
    return jsonify(gcs_snapshot())


@app.route("/api/gcs/stream")
def api_gcs_stream() -> Response:
    def event_stream():
        while True:
            yield f"data: {json.dumps(gcs_snapshot())}\n\n"
            time.sleep(0.1)

    return Response(event_stream(), mimetype="text/event-stream")


@app.route("/api/ports")
def api_ports() -> Response:
    ports = [{"device": p.device, "description": p.description} for p in list_ports.comports()]
    return jsonify(ports)


@app.route("/api/gcs/connect", methods=["POST"])
def api_gcs_connect() -> Response:
    body = request.get_json(silent=True) or {}
    source = body.get("source") or "crsf"
    port = body.get("port") or app.config.get("SERIAL_PORT", "")
    baud = int(body.get("baud") or app.config.get("SERIAL_BAUD", 115200))

    if source not in ("crsf", "sim"):
        return jsonify({"ok": False, "error": "source must be crsf or sim"}), 400
    if source == "crsf" and not port:
        return jsonify({"ok": False, "error": "Serial port is required"}), 400

    start_serial_thread(port, baud, source)
    time.sleep(0.2)
    return jsonify({"ok": True, "status": gcs_snapshot()["status"]})


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
    parser.add_argument(
        "--source",
        choices=sorted(READERS),
        default="json",
        help="json = FC USB CDC (/), crsf = radio Telemetry Mirror (/gcs), "
             "sim = synthetic CRSF (/gcs). Default: json",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    app.config["SERIAL_PORT"] = args.port
    app.config["SERIAL_BAUD"] = args.baud

    if not args.no_auto_connect:
        start_serial_thread(args.port, args.baud, args.source)

    page = "/" if args.source == "json" else "/gcs"
    print(f"Dashboard: http://{args.host}:{args.web_port}{page}")
    if not args.no_auto_connect:
        print(f"Source: {args.source}  Serial: {args.port} @ {args.baud}")

    app.run(host=args.host, port=args.web_port, debug=False, threaded=True)


if __name__ == "__main__":
    main()
