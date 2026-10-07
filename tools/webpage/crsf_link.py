"""
CRSF telemetry decoding for the ground station.

Byte stream source: the radio's USB VCP in EdgeTX "Telemetry Mirror" mode,
which repeats every CRSF frame the TX module receives from the drone.

Frames decoded (firmware side: PinCubeMX/app/fc/fc_telem.c):
  0x07 Vario          0x08 Battery        0x09 Baro altitude
  0x0C RPM            0x0D Temperature    0x14 Link statistics (from ELRS)
  0x1E Attitude       0x21 Flight mode    0x7F FC debug (project-specific)

All multi-byte fields are big-endian, CRC8 poly 0xD5 over [type + payload].
"""

from __future__ import annotations

import struct
import threading
import time
from collections import deque
from typing import Any

SYNC_BYTES = (0xC8, 0xEA, 0xEE)

TYPE_VARIO = 0x07
TYPE_BATTERY = 0x08
TYPE_BARO_ALTITUDE = 0x09
TYPE_RPM = 0x0C
TYPE_TEMP = 0x0D
TYPE_LINK_STATS = 0x14
TYPE_RC_CHANNELS = 0x16
TYPE_ATTITUDE = 0x1E
TYPE_FLIGHT_MODE = 0x21
TYPE_EXT_FIRST = 0x28
TYPE_FC_DEBUG = 0x7F

FC_DEBUG_VERSION = 1
FC_DEBUG_LEN = 46

RAD2DEG = 57.29577951308232

# Must match arming.h / failsafe.h.
ARMING_DISABLE_NAMES = [
    "NO_GYRO", "NO_MOTORS", "RX_LOSS", "THROTTLE",
    "FAILSAFE", "BOOT_GRACE", "ARM_SWITCH", "CALIBRATING",
]
FAILSAFE_NAMES = ["IDLE", "RX_LOSS_DETECTED", "LANDING", "LANDED"]


def _crc_table() -> list[int]:
    table = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) if (crc & 0x80) else (crc << 1)
        table.append(crc & 0xFF)
    return table


_CRC_TABLE = _crc_table()


def crsf_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc


class CrsfParser:
    """Splits a raw byte stream into (type, payload) frames with valid CRC."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.ok = 0
        self.crc_err = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        frames: list[tuple[int, bytes]] = []

        while True:
            while self._buf and self._buf[0] not in SYNC_BYTES:
                del self._buf[0]
            if len(self._buf) < 3:
                break

            length = self._buf[1]
            if length < 2 or length > 62:
                del self._buf[0]
                continue

            total = 2 + length
            if len(self._buf) < total:
                break

            body = bytes(self._buf[2:total - 1])
            if self._buf[total - 1] != crsf_crc8(body):
                # A sync byte may have appeared inside a payload: drop only
                # this byte and rescan instead of discarding the whole span.
                self.crc_err += 1
                del self._buf[0]
                continue

            del self._buf[:total]
            self.ok += 1
            frames.append((body[0], body[1:]))

        return frames


def _unpack_altitude_m(packed: int) -> float:
    if packed & 0x8000:
        return float(packed & 0x7FFF)
    return (packed - 10000) * 0.1


def _decode_debug(payload: bytes) -> dict[str, Any] | None:
    # Extended frame: [dest][origin][data...]
    if len(payload) < 2 + FC_DEBUG_LEN:
        return None
    data = payload[2:2 + FC_DEBUG_LEN]
    version, flags, arm_flags, fs_state = struct.unpack_from(">BBBB", data, 0)
    if version != FC_DEBUG_VERSION:
        return {"version": version, "error": "unknown debug version"}

    stick = struct.unpack_from(">hhh", data, 4)
    (throttle,) = struct.unpack_from(">H", data, 10)
    setpoint = struct.unpack_from(">hhh", data, 12)
    gyro = struct.unpack_from(">hhh", data, 18)
    pid_out = struct.unpack_from(">hhh", data, 24)
    motors = struct.unpack_from(">HHHH", data, 30)
    pid_max_us, rc_age_ms, gyro_errors, motor_drops = struct.unpack_from(">HHHH", data, 38)

    return {
        "version": version,
        "armed": bool(flags & 0x01),
        "failsafe": bool(flags & 0x02),
        "link_ok": bool(flags & 0x04),
        "calibrated": bool(flags & 0x08),
        "pid_enabled": bool(flags & 0x10),
        "has_baro": bool(flags & 0x20),
        "has_mag": bool(flags & 0x40),
        "esc_telem": bool(flags & 0x80),
        "arming_disable": [n for i, n in enumerate(ARMING_DISABLE_NAMES) if arm_flags & (1 << i)],
        "failsafe_state": FAILSAFE_NAMES[fs_state] if fs_state < len(FAILSAFE_NAMES) else str(fs_state),
        "stick": [v / 1000.0 for v in stick],
        "throttle": throttle / 1000.0,
        "setpoint_dps": [v / 10.0 for v in setpoint],
        "gyro_dps": [v / 10.0 for v in gyro],
        "pid_out": [v / 1000.0 for v in pid_out],
        "motors": list(motors),
        "pid_max_us": pid_max_us,
        "rc_age_ms": rc_age_ms,
        "gyro_errors": gyro_errors,
        "motor_drops": motor_drops,
    }


def decode_frame(ftype: int, payload: bytes) -> tuple[str, dict[str, Any]] | None:
    """Returns (key, values) for known telemetry frames, None otherwise."""
    n = len(payload)

    if ftype == TYPE_ATTITUDE and n >= 6:
        pitch, roll, yaw = struct.unpack(">hhh", payload[:6])
        return "attitude", {
            "pitch": pitch / 10000.0 * RAD2DEG,
            "roll": roll / 10000.0 * RAD2DEG,
            "yaw": yaw / 10000.0 * RAD2DEG,
        }

    if ftype == TYPE_BARO_ALTITUDE and n >= 2:
        (packed,) = struct.unpack(">H", payload[:2])
        return "baro", {"alt_m": _unpack_altitude_m(packed)}

    if ftype == TYPE_VARIO and n >= 2:
        (vspd,) = struct.unpack(">h", payload[:2])
        return "vario", {"vspeed_ms": vspd / 100.0}

    if ftype == TYPE_BATTERY and n >= 8:
        volt, curr = struct.unpack(">HH", payload[:4])
        used = (payload[4] << 16) | (payload[5] << 8) | payload[6]
        return "battery", {
            "voltage": volt / 10.0,
            "current": curr / 10.0,
            "used_mah": used,
            "remaining_pct": payload[7],
        }

    if ftype == TYPE_RPM and n >= 4:
        values = []
        for i in range(1, n - 2, 3):
            raw = (payload[i] << 16) | (payload[i + 1] << 8) | payload[i + 2]
            if raw & 0x800000:
                raw -= 1 << 24
            values.append(raw)
        return "rpm", {"source": payload[0], "rpm": values}

    if ftype == TYPE_TEMP and n >= 3:
        count = (n - 1) // 2
        values = struct.unpack(f">{count}h", payload[1:1 + count * 2])
        return "temp", {"source": payload[0], "temp_c": [v / 10.0 for v in values]}

    if ftype == TYPE_FLIGHT_MODE and n >= 1:
        text = payload.split(b"\x00", 1)[0].decode("ascii", errors="replace")
        return "mode", {"text": text, "armed": not text.endswith("*")}

    if ftype == TYPE_LINK_STATS and n >= 10:
        (rssi1, rssi2, lq, snr, ant, rf_mode, tx_pwr,
         d_rssi, d_lq, d_snr) = struct.unpack(">BBBbBBBBBb", payload[:10])
        return "link", {
            "rssi1_dbm": -rssi1,
            "rssi2_dbm": -rssi2,
            "lq": lq,
            "snr": snr,
            "antenna": ant,
            "rf_mode": rf_mode,
            "tx_power_idx": tx_pwr,
            "down_rssi_dbm": -d_rssi,
            "down_lq": d_lq,
            "down_snr": d_snr,
        }

    if ftype == TYPE_FC_DEBUG:
        dbg = _decode_debug(payload)
        if dbg is not None:
            return "debug", dbg

    return None


class TelemetryStore:
    """Thread-safe latest values, per-frame rates and short history series."""

    SERIES_LEN = 300

    SERIES_KEYS = (
        "roll", "pitch", "yaw", "alt_m", "vspeed_ms", "voltage", "current",
        "sp_roll", "sp_pitch", "sp_yaw", "gyro_roll", "gyro_pitch", "gyro_yaw",
        "pid_roll", "pid_pitch", "pid_yaw", "m1", "m2", "m3", "m4", "lq",
    )

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with self._lock:
            self._t0 = time.time()
            self._latest: dict[str, dict[str, Any]] = {}
            self._updated: dict[str, float] = {}
            self._arrivals: dict[str, deque[float]] = {}
            self._unknown: dict[str, int] = {}
            self._series = {k: deque(maxlen=self.SERIES_LEN) for k in self.SERIES_KEYS}
            self._series_t = {k: deque(maxlen=self.SERIES_LEN) for k in self.SERIES_KEYS}
            self.frames_ok = 0
            self.crc_err = 0
            self.bytes_rx = 0

    def _push(self, key: str, t: float, value: float) -> None:
        self._series[key].append(round(value, 4))
        self._series_t[key].append(round(t, 3))

    def add(self, ftype: int, payload: bytes) -> None:
        decoded = decode_frame(ftype, payload)
        now = time.time()
        t = now - self._t0

        with self._lock:
            if decoded is None:
                if ftype != TYPE_RC_CHANNELS:
                    name = f"0x{ftype:02X}"
                    self._unknown[name] = self._unknown.get(name, 0) + 1
                return

            key, values = decoded
            self._latest[key] = values
            self._updated[key] = now
            self._arrivals.setdefault(key, deque(maxlen=64)).append(now)

            if key == "attitude":
                self._push("roll", t, values["roll"])
                self._push("pitch", t, values["pitch"])
                self._push("yaw", t, values["yaw"])
            elif key == "baro":
                self._push("alt_m", t, values["alt_m"])
            elif key == "vario":
                self._push("vspeed_ms", t, values["vspeed_ms"])
            elif key == "battery":
                self._push("voltage", t, values["voltage"])
                self._push("current", t, values["current"])
            elif key == "link":
                self._push("lq", t, values["lq"])
            elif key == "debug" and "stick" in values:
                for i, ax in enumerate(("roll", "pitch", "yaw")):
                    self._push(f"sp_{ax}", t, values["setpoint_dps"][i])
                    self._push(f"gyro_{ax}", t, values["gyro_dps"][i])
                    self._push(f"pid_{ax}", t, values["pid_out"][i])
                for i in range(4):
                    self._push(f"m{i + 1}", t, values["motors"][i])

    def _rate_hz(self, key: str, now: float) -> float:
        arr = self._arrivals.get(key)
        if not arr:
            return 0.0
        recent = [x for x in arr if now - x <= 3.0]
        if len(recent) < 2:
            return 0.0 if now - arr[-1] > 3.0 else 1.0 / 3.0
        return (len(recent) - 1) / max(recent[-1] - recent[0], 1e-3)

    def snapshot(self) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            return {
                "latest": {k: dict(v) for k, v in self._latest.items()},
                "age_s": {k: round(now - v, 2) for k, v in self._updated.items()},
                "rate_hz": {k: round(self._rate_hz(k, now), 1) for k in self._arrivals},
                "unknown": dict(self._unknown),
                "series": {k: list(v) for k, v in self._series.items()},
                "series_t": {k: list(v) for k, v in self._series_t.items()},
                "counters": {
                    "frames_ok": self.frames_ok,
                    "crc_err": self.crc_err,
                    "bytes_rx": self.bytes_rx,
                },
            }
