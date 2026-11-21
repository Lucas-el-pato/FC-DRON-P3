#!/usr/bin/env python3
"""
Decoder del blackbox propio del FC (archivos BBnnn.BBL de la microSD).

Formato (firmware: PinCubeMX/app/fc/fc_blackbox.c):
  - Cabecera de texto de header_len bytes: lineas "clave=valor", la primera
    es "FCBBL <version>", y "fields=" lista "nombre:tipo:escala" en el orden
    del registro. Termina en "end" + relleno con ceros.
  - Registros binarios little-endian de record_len bytes.

El final real se detecta por el contador seq (u8) y t_us creciente: si se
corto la bateria en vuelo el archivo queda con su tamano reservado y lo que
sigue son datos viejos de la tarjeta.

Uso:
  python3 bbl_decode.py BB001.BBL                # resumen
  python3 bbl_decode.py BB001.BBL --csv out.csv  # + CSV con unidades reales
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from pathlib import Path
from typing import Any

TYPE_FMT = {"u8": "B", "i8": "b", "u16": "H", "i16": "h", "u32": "I", "i32": "i"}


def parse_header(raw: bytes) -> tuple[dict[str, str], list[tuple[str, str, float]]]:
    text = raw.split(b"\x00", 1)[0].decode("ascii", errors="replace")
    lines = text.splitlines()
    if not lines or not lines[0].startswith("FCBBL"):
        raise ValueError("no es un archivo de blackbox (falta 'FCBBL')")

    meta: dict[str, str] = {"version": lines[0].split()[1] if len(lines[0].split()) > 1 else "?"}
    fields: list[tuple[str, str, float]] = []
    for line in lines[1:]:
        if line == "end":
            break
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key == "fields":
            for item in value.split():
                name, typ, scale = item.split(":")
                fields.append((name, typ, float(scale)))
        else:
            meta[key] = value
    if not fields:
        raise ValueError("cabecera sin 'fields='")
    return meta, fields


def decode(path: Path) -> tuple[dict[str, str], list[str], list[list[float]], dict[str, Any]]:
    data = path.read_bytes()
    if len(data) < 512:
        raise ValueError("archivo demasiado corto")

    # header_len esta en la propia cabecera; se lee primero con un tope.
    meta, fields = parse_header(data[:4096])
    header_len = int(meta.get("header_len", "2048"))
    meta, fields = parse_header(data[:header_len])

    fmt = "<" + "".join(TYPE_FMT[t] for _, t, _ in fields)
    rec_len = struct.calcsize(fmt)
    if rec_len != int(meta.get("record_len", rec_len)):
        raise ValueError(f"record_len {meta.get('record_len')} != campos ({rec_len})")

    names = [n for n, _, _ in fields]
    scales = [s for _, _, s in fields]
    i_seq = names.index("seq") if "seq" in names else None
    i_t = names.index("t_us")

    rows: list[list[float]] = []
    seq_gaps = 0
    stop_reason = "fin del archivo"
    prev_seq = None
    prev_t = -1
    off = header_len
    while off + rec_len <= len(data):
        vals = struct.unpack_from(fmt, data, off)
        off += rec_len
        t_us = vals[i_t]
        if t_us < prev_t or (prev_t >= 0 and t_us - prev_t > 1_000_000):
            stop_reason = "t_us dejo de crecer (fin de los datos validos)"
            break
        if i_seq is not None and prev_seq is not None:
            expected = (prev_seq + 1) & 0xFF
            if vals[i_seq] != expected:
                if t_us == 0:
                    stop_reason = "relleno"
                    break
                seq_gaps += 1
        prev_seq = vals[i_seq] if i_seq is not None else None
        prev_t = t_us
        rows.append([v * s for v, s in zip(vals, scales)])

    stats: dict[str, Any] = {"records": len(rows), "seq_gaps": seq_gaps, "stop": stop_reason}
    if rows:
        t = [r[i_t] for r in rows]
        dur = (t[-1] - t[0]) / 1e6
        stats["duration_s"] = dur
        stats["rate_hz"] = (len(rows) - 1) / dur if dur > 0 else 0.0
        if "loop_us" in names:
            stats["loop_us_max"] = max(r[names.index("loop_us")] for r in rows)
    return meta, names, rows, stats


def main() -> int:
    ap = argparse.ArgumentParser(description="Decoder del blackbox del FC (BBnnn.BBL)")
    ap.add_argument("file", type=Path)
    ap.add_argument("--csv", type=Path, help="exportar a CSV (unidades reales, t en s)")
    args = ap.parse_args()

    try:
        meta, names, rows, stats = decode(args.file)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"{args.file.name}: formato v{meta.get('version')}  log {meta.get('log_hz')} Hz  "
          f"PID {meta.get('pid_hz')} Hz  gyro {meta.get('gyro_hz')} Hz")
    for axis in ("roll", "pitch", "yaw"):
        print(f"  {axis:5s} kp={meta.get(axis + '_kp')} ki={meta.get(axis + '_ki')} "
              f"kd={meta.get(axis + '_kd')}")
    print(f"  gain_scale={meta.get('gain_scale')} throttle_limit={meta.get('throttle_limit')}")
    print(f"registros: {stats['records']}  huecos de seq: {stats['seq_gaps']}  ({stats['stop']})")
    if stats["records"]:
        print(f"duracion: {stats['duration_s']:.2f} s  tasa real: {stats['rate_hz']:.1f} Hz  "
              f"loop_us max: {stats.get('loop_us_max', 0):.0f}")

    if stats["records"] and "erpm_ch1" in names:
        i_ok, i_err = names.index("rpm_ok"), names.index("rpm_err")
        ok = [int(r[i_ok]) for r in rows]
        pct = [100.0 * sum((v >> m) & 1 for v in ok) / len(ok) for m in range(4)]
        print(f"DShot bidir={meta.get('dshot_bidir')}  polos={meta.get('motor_poles')}  "
              "respuesta valida: " + " ".join(f"CH{m + 1} {p:.1f}%" for m, p in enumerate(pct))
              + f"  malas/ausentes: {sum(r[i_err] for r in rows):.0f}")

    if args.csv:
        i_t = names.index("t_us")
        with args.csv.open("w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["time_s"] + [n for n in names if n != "t_us"])
            for r in rows:
                w.writerow([f"{r[i_t] / 1e6:.6f}"] + [f"{v:g}" for i, v in enumerate(r) if i != i_t])
        print(f"CSV: {args.csv}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
