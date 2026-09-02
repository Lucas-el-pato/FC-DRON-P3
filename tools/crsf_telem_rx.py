#!/usr/bin/env python3
"""
Decode CRSF telemetry mirrored from a RadioMaster Pocket (USB VCP Telem Mirror).

Parses Attitude (0x1E), Baro Altitude (0x09) and Vario (0x07) frames and
prints them live.

Usage:
  python3 crsf_telem_rx.py
  python3 crsf_telem_rx.py --port /dev/ttyACM1 --baud 115200

Sync bytes accepted: 0xC8 (FC) and 0xEA (handset / radio mirror).
CRC8 polynomial 0xD5 over [type + payload] (same as firmware driver_crsf).
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
    from serial import SerialException
except ImportError as exc:
    raise SystemExit("pyserial is required: pip install pyserial") from exc

CRSF_SYNC_FC = 0xC8
CRSF_SYNC_RADIO = 0xEA
CRSF_TYPE_VARIO = 0x07
CRSF_TYPE_BARO_ALTITUDE = 0x09
CRSF_TYPE_ATTITUDE = 0x1E
CRSF_TYPE_RC_CHANNELS = 0x16
CRSF_TYPE_LINK_STATS = 0x14

# Precomputed CRC8 table, poly 0xD5 (same as driver_crsf.c).
_CRC_TABLE = [
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9,
]


def crsf_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = _CRC_TABLE[crc ^ b]
    return crc


def unpack_baro_altitude_m(packed: int) -> float:
    """Decode CRSF 0x09 packed altitude to meters relative."""
    if packed & 0x8000:
        return float(packed & 0x7FFF)
    return (packed - 10000) * 0.1


def decode_frame(ftype: int, payload: bytes) -> str | None:
    if ftype == CRSF_TYPE_ATTITUDE and len(payload) >= 6:
        pitch, roll, yaw = struct.unpack(">hhh", payload[:6])
        return (
            f"ATT  pitch={pitch / 10000.0 * 57.2958:7.2f} deg  "
            f"roll={roll / 10000.0 * 57.2958:7.2f} deg  "
            f"yaw={yaw / 10000.0 * 57.2958:7.2f} deg"
        )

    if ftype == CRSF_TYPE_BARO_ALTITUDE and len(payload) >= 2:
        packed = struct.unpack(">H", payload[:2])[0]
        alt_m = unpack_baro_altitude_m(packed)
        extra = ""
        if len(payload) >= 4:
            # ELRS-style optional int16 cm/s (not sent by our firmware).
            vspd = struct.unpack(">h", payload[2:4])[0]
            extra = f"  vspd={vspd} cm/s (in-frame)"
        elif len(payload) >= 3:
            extra = f"  vspd_packed={payload[2]} (TBS log)"
        return f"BARO alt={alt_m:8.2f} m{extra}"

    if ftype == CRSF_TYPE_VARIO and len(payload) >= 2:
        vspd = struct.unpack(">h", payload[:2])[0]
        return f"VARIO vspd={vspd:6d} cm/s ({vspd / 100.0:6.2f} m/s)"

    if ftype == CRSF_TYPE_LINK_STATS:
        return f"LINK stats ({len(payload)} B)"

    if ftype == CRSF_TYPE_RC_CHANNELS:
        return None  # noisy; ignore on telem mirror unless debugging

    return f"TYPE 0x{ftype:02X} len={len(payload)}"


class CrsfParser:
    def __init__(self) -> None:
        self._buf = bytearray()
        self.ok = 0
        self.crc_err = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buf.extend(data)
        frames: list[tuple[int, bytes]] = []

        while True:
            # Seek sync.
            while self._buf and self._buf[0] not in (CRSF_SYNC_FC, CRSF_SYNC_RADIO):
                del self._buf[0]

            if len(self._buf) < 3:
                break

            length = self._buf[1]
            if length < 2 or length > 64:
                del self._buf[0]
                continue

            total = 2 + length  # sync + len + (type..crc)
            if len(self._buf) < total:
                break

            frame = bytes(self._buf[:total])
            del self._buf[:total]

            body = frame[2:-1]  # type + payload
            crc_rx = frame[-1]
            crc_calc = crsf_crc8(body)
            if crc_rx != crc_calc:
                self.crc_err += 1
                # Resync one byte: re-insert remainder after first byte of bad frame.
                # We already consumed the frame; next loop seeks a new sync.
                continue

            self.ok += 1
            ftype = body[0]
            payload = body[1:]
            frames.append((ftype, payload))

        return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="CRSF telem decoder (Pocket Telem Mirror)")
    parser.add_argument("--port", default="/dev/ttyACM1", help="Serial port of the radio USB VCP")
    parser.add_argument("--baud", type=int, default=115200, help="Baud (ignored by USB CDC)")
    parser.add_argument("--raw-types", action="store_true", help="Print unknown frame types too")
    args = parser.parse_args()

    try:
        ser = serial.Serial(port=args.port, baudrate=args.baud, timeout=0.1)
    except SerialException as exc:
        print(f"ERROR opening {args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Listening on {args.port} (Telem Mirror). Ctrl+C to stop.")
    crsf = CrsfParser()
    t0 = time.time()

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if not chunk:
                continue
            for ftype, payload in crsf.feed(chunk):
                line = decode_frame(ftype, payload)
                if line is None and not args.raw_types:
                    continue
                if line is None:
                    line = f"TYPE 0x{ftype:02X} len={len(payload)}"
                elapsed = time.time() - t0
                print(f"[{elapsed:8.2f}] {line}  (ok={crsf.ok} crc_err={crsf.crc_err})")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
