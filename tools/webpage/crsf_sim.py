"""
Synthetic CRSF telemetry with the same byte layout as the firmware
(driver_crsf.c + fc_telem.c). Used by `app.py --source sim` to try the
ground station without the drone, and as a cross-check of the decoder.
"""

from __future__ import annotations

import math
import struct

from crsf_link import (
    FC_DEBUG_VERSION,
    TYPE_ATTITUDE,
    TYPE_BARO_ALTITUDE,
    TYPE_BATTERY,
    TYPE_FC_DEBUG,
    TYPE_FLIGHT_MODE,
    TYPE_LINK_STATS,
    TYPE_RPM,
    TYPE_TEMP,
    TYPE_VARIO,
    crsf_crc8,
)

ADDR_FC = 0xC8
ADDR_RADIO = 0xEA


def frame(ftype: int, payload: bytes, sync: int = ADDR_FC) -> bytes:
    body = bytes([ftype]) + payload
    return bytes([sync, len(body) + 1]) + body + bytes([crsf_crc8(body)])


def ext_frame(ftype: int, payload: bytes, dest: int = ADDR_RADIO) -> bytes:
    return frame(ftype, bytes([dest, ADDR_FC]) + payload)


def _i16(v: float) -> int:
    return max(-32768, min(32767, int(round(v))))


def pack_altitude(alt_dm: int) -> int:
    if alt_dm < -10000:
        return 0
    if alt_dm + 10000 < 0x8000:
        return alt_dm + 10000
    return ((alt_dm + 5) // 10 & 0x7FFF) | 0x8000


class Simulator:
    """Emits frames at roughly the firmware task rates."""

    PERIODS = {
        "attitude": 0.05,
        "baro": 0.2,
        "battery": 0.5,
        "mode": 0.5,
        "esc": 0.5,
        "debug": 0.1,
        "link": 0.2,
    }

    def __init__(self) -> None:
        self._next = {k: 0.0 for k in self.PERIODS}
        self._mah = 0.0

    def step(self, t: float) -> bytes:
        out = bytearray()
        for key, period in self.PERIODS.items():
            if t >= self._next[key]:
                self._next[key] = t + period
                out += getattr(self, f"_{key}")(t)
        return bytes(out)

    @staticmethod
    def _angles(t: float) -> tuple[float, float, float]:
        roll = 25.0 * math.sin(t * 0.7)
        pitch = 15.0 * math.sin(t * 0.45 + 1.0)
        yaw = ((t * 20.0 + 180.0) % 360.0) - 180.0
        return roll, pitch, yaw

    def _attitude(self, t: float) -> bytes:
        roll, pitch, yaw = (math.radians(a) * 10000.0 for a in self._angles(t))
        return frame(TYPE_ATTITUDE, struct.pack(">hhh", _i16(pitch), _i16(roll), _i16(yaw)))

    def _baro(self, t: float) -> bytes:
        alt_dm = int(round(15.0 + 10.0 * math.sin(t * 0.2)) * 10)
        vspd_cm = _i16(100.0 * 2.0 * math.cos(t * 0.2))
        return (frame(TYPE_BARO_ALTITUDE, struct.pack(">H", pack_altitude(alt_dm)))
                + frame(TYPE_VARIO, struct.pack(">h", vspd_cm)))

    def _battery(self, t: float) -> bytes:
        current_a = 8.0 + 4.0 * math.sin(t * 0.3)
        self._mah += current_a * 1000.0 / 3600.0 * self.PERIODS["battery"]
        volt_dv = int(round((16.6 - t * 0.004) * 10))
        used = int(self._mah)
        return frame(TYPE_BATTERY, struct.pack(">HH", volt_dv, int(current_a * 10))
                     + bytes([(used >> 16) & 0xFF, (used >> 8) & 0xFF, used & 0xFF, 0]))

    def _mode(self, t: float) -> bytes:
        text = b"ANGL" if (t % 30.0) > 5.0 else b"ANGL*"
        return frame(TYPE_FLIGHT_MODE, text + b"\x00")

    def _esc(self, t: float) -> bytes:
        rpm = int(12000 + 3000 * math.sin(t * 0.8))
        temps = struct.pack(">hh", int((38 + 5 * math.sin(t * 0.05)) * 10), 315)
        return (frame(TYPE_RPM, bytes([0, (rpm >> 16) & 0xFF, (rpm >> 8) & 0xFF, rpm & 0xFF]))
                + frame(TYPE_TEMP, bytes([0]) + temps))

    def _link(self, t: float) -> bytes:
        lq = 100 if (t % 20.0) < 17.0 else 70
        return frame(TYPE_LINK_STATS, struct.pack(">BBBbBBBBBb", 62, 65, lq, 9, 0, 7, 3, 70, lq, 6),
                     sync=ADDR_RADIO)

    def _debug(self, t: float) -> bytes:
        roll, pitch, _ = self._angles(t)
        armed = (t % 30.0) > 5.0
        stick = (math.sin(t * 0.7), math.sin(t * 0.45 + 1.0), 0.2 * math.sin(t * 1.3))
        throttle = 0.45 + 0.1 * math.sin(t * 0.2) if armed else 0.0
        setpoint = (5.0 * (stick[0] * 60 - roll), 5.0 * (stick[1] * 60 - pitch), stick[2] * 300)
        gyro = (17.5 * math.cos(t * 0.7), 6.75 * math.cos(t * 0.45 + 1.0), 4.0 * math.sin(t * 2.0))
        pid = tuple(max(-0.25, min(0.25, 0.6 * (s - g) / 1000.0)) for s, g in zip(setpoint, gyro))
        if armed:
            base = 0.055 + throttle * 0.945
            mix = (-pid[0] + pid[1] - pid[2], -pid[0] - pid[1] + pid[2],
                   pid[0] + pid[1] + pid[2], pid[0] - pid[1] - pid[2])
            motors = [int(48 + max(0.0, min(1.0, base + m)) * 1999) for m in mix]
        else:
            motors = [0, 0, 0, 0]

        flags = (0x01 if armed else 0) | 0x04 | 0x08 | 0x10 | 0x20 | 0x80
        data = struct.pack(">BBBB", FC_DEBUG_VERSION, flags, 0 if armed else 0x40, 0)
        data += struct.pack(">hhh", *(_i16(s * 1000) for s in stick))
        data += struct.pack(">H", int(throttle * 1000))
        data += struct.pack(">hhh", *(_i16(s * 10) for s in setpoint))
        data += struct.pack(">hhh", *(_i16(g * 10) for g in gyro))
        data += struct.pack(">hhh", *(_i16(p * 1000) for p in pid))
        data += struct.pack(">HHHH", *motors)
        data += struct.pack(">HHHH", 92, 6, 0, 0)
        return ext_frame(TYPE_FC_DEBUG, data)
