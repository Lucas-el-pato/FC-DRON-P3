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
    TYPE_GPS,
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
        "gps": 1.0,
    }

    # Centro del circulo simulado: campus FIUNA, San Lorenzo (aprox.).
    GPS_HOME = (-25.3310, -57.5195)
    GPS_RADIUS_M = 60.0
    GPS_PERIOD_S = 60.0

    def __init__(self) -> None:
        self._next = {k: 0.0 for k in self.PERIODS}
        self._mah = 0.0
        self._debug_page = 0

    def step(self, t: float) -> bytes:
        out = bytearray()
        for key, period in self.PERIODS.items():
            if t >= self._next[key]:
                self._next[key] = t + period
                out += getattr(self, f"_{key}")(t)
        return bytes(out)

    # Cada 70 s: parado 25..35 s (velocidad 0, flecha GPS en gris) y sin fix
    # 40..50 s. El mag "anda" 30 s de cada 60 (flecha por brujula vs GPS).
    STOP_WINDOW = (25.0, 35.0)
    YAW_OFFSET_DEG = 30.0   # la nariz no apunta hacia donde se mueve

    @classmethod
    def _moving_t(cls, t: float) -> float:
        """Segundos de recorrido del circulo (el reloj se frena al estar parado)."""
        lo, hi = cls.STOP_WINDOW
        stopped = (t // 70.0) * (hi - lo) + min(max((t % 70.0) - lo, 0.0), hi - lo)
        return max(0.0, t - 8.0 - stopped)

    @staticmethod
    def _stopped(t: float) -> bool:
        lo, hi = Simulator.STOP_WINDOW
        return lo <= (t % 70.0) < hi

    @staticmethod
    def _mag_ok(t: float) -> bool:
        return (t % 60.0) < 30.0

    def _angles(self, t: float) -> tuple[float, float, float]:
        roll = 25.0 * math.sin(t * 0.7)
        pitch = 15.0 * math.sin(t * 0.45 + 1.0)
        # Yaw = rumbo del recorrido + desfase (sigue igual con el dron parado).
        a = 2.0 * math.pi / self.GPS_PERIOD_S * self._moving_t(t)
        yaw = ((math.degrees(a) + 90.0 + self.YAW_OFFSET_DEG + 180.0) % 360.0) - 180.0
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

    def _gps(self, t: float) -> bytes:
        # Sin fix los primeros 8 s (como un arranque en frio), despues un
        # circulo de GPS_RADIUS_M alrededor de home, en sentido horario.
        if t < 8.0:
            return frame(TYPE_GPS, struct.pack(">iiHHHB", 0, 0, 0, 0, 1000, int(t / 2)))
        # Cada 70 s, 10 s "bajo techo": el FC deja de dar fix (pocos sats).
        if 40.0 <= (t % 70.0) < 50.0:
            return frame(TYPE_GPS, struct.pack(">iiHHHB", 0, 0, 0, 0, 1000, 4))
        w = 2.0 * math.pi / self.GPS_PERIOD_S
        a = w * self._moving_t(t)
        north = self.GPS_RADIUS_M * math.cos(a)
        east = self.GPS_RADIUS_M * math.sin(a)
        lat = self.GPS_HOME[0] + north / 111_320.0
        lon = self.GPS_HOME[1] + east / (111_320.0 * math.cos(math.radians(self.GPS_HOME[0])))
        speed_kmh = 0.0 if self._stopped(t) else w * self.GPS_RADIUS_M * 3.6
        # Parado, el GPS da un rumbo de desplazamiento sin sentido.
        heading = (t * 37.0) % 360.0 if self._stopped(t) else (math.degrees(a) + 90.0) % 360.0
        alt_m = 125 + int(15 + 10 * math.sin(t * 0.2))
        return frame(TYPE_GPS, struct.pack(">iiHHHB", int(round(lat * 1e7)), int(round(lon * 1e7)),
                                           int(round(speed_kmh * 10)), int(round(heading * 100)),
                                           alt_m + 1000, 11))

    def _debug(self, t: float) -> bytes:
        # Like fc_telem_debug(): one frame per period, alternating pages.
        page = self._debug_page
        self._debug_page ^= 1
        return self._debug_state(t) if page == 0 else self._debug_loop(t)

    def _debug_loop(self, t: float) -> bytes:
        armed = (t % 30.0) > 5.0
        data = struct.pack(">BB", FC_DEBUG_VERSION, 1)
        data += struct.pack(">9H", 200, 1600, 400, 140, 95 if armed else 60,
                            520 + int(40 * abs(math.sin(t))), 380, 120, 3)
        data += struct.pack(">B", 7)
        data += struct.pack(">8H", 1600, 2, 2, 0, 0, 0, 0, 0)
        # Blackbox: LOGGING armado (crece ~72 KB/s), READY desarmado.
        kb = int(((t % 30.0) - 5.0) * 72) if armed else 0
        data += struct.pack(">BBHHHB", 2 if armed else 1, 0, 3, kb, 0, 12 if armed else 0)
        return ext_frame(TYPE_FC_DEBUG, data)

    def _debug_state(self, t: float) -> bytes:
        roll, pitch, _ = self._angles(t)
        armed = (t % 30.0) > 5.0
        stick = (math.sin(t * 0.7), math.sin(t * 0.45 + 1.0), 0.2 * math.sin(t * 1.3))
        throttle = 0.45 + 0.1 * math.sin(t * 0.2) if armed else 0.0
        setpoint = (5.0 * (stick[0] * 60 - roll), 5.0 * (stick[1] * 60 - pitch), stick[2] * 300)
        gyro = (17.5 * math.cos(t * 0.7), 6.75 * math.cos(t * 0.45 + 1.0), 4.0 * math.sin(t * 2.0))
        pid = tuple(max(-0.25, min(0.25, 0.6 * (s - g) / 1000.0)) for s, g in zip(setpoint, gyro))
        if armed:
            base = 0.055 + throttle * 0.945
            mix = (-pid[0] - pid[1] + pid[2], -pid[0] + pid[1] - pid[2],
                   pid[0] - pid[1] - pid[2], pid[0] + pid[1] + pid[2])
            motors = [int(48 + max(0.0, min(1.0, base + m)) * 1999) for m in mix]
        else:
            motors = [0, 0, 0, 0]

        flags = (0x01 if armed else 0) | 0x04 | 0x08 | 0x10 | 0x20 | 0x80
        flags |= 0x40 if self._mag_ok(t) else 0
        data = struct.pack(">BBBBB", FC_DEBUG_VERSION, 0, flags, 0 if armed else 0x40, 0)
        data += struct.pack(">hhh", *(_i16(s * 1000) for s in stick))
        data += struct.pack(">H", int(throttle * 1000))
        data += struct.pack(">hhh", *(_i16(s * 10) for s in setpoint))
        data += struct.pack(">hhh", *(_i16(g * 10) for g in gyro))
        data += struct.pack(">hhh", *(_i16(p * 1000) for p in pid))
        data += struct.pack(">HHHH", *motors)
        data += struct.pack(">HHHH", 92, 6, 0, 0)
        return ext_frame(TYPE_FC_DEBUG, data)
