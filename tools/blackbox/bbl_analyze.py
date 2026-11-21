#!/usr/bin/env python3
"""
Graficas de analisis PID del blackbox propio (BBnnn.BBL).

Por archivo (en <out>/<BBnnn>/):
  overview.png   throttle, motores, bateria y tiempo del lazo
  tracking.png   setpoint vs gyro por eje + angulo vs objetivo (ANGLE)
  pid_terms.png  P, I, D por eje
  spectrogram.png ruido del gyro vs tiempo, con el throttle encima (y la
                 RPM medida x1/x2/x3 si el log trae DShot bidireccional, y el
                 centro del notch dinamico si el log es formato 3)
  rpm.png        (formato 2) RPM de cada motor vs notch, respuestas DShot
                 validas y carga de CPU

Comparando todos los archivos (en <out>/):
  step_response.png  respuesta al escalon estimada (deconvolucion, metodo
                     de PID-Analyzer) por eje
  noise_psd.png      espectro del gyro sin filtrar / filtrado y del D

Requiere numpy, scipy, matplotlib.

  python3 bbl_analyze.py /media/$USER/SD/BB00*.BBL --out analisis/
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import signal  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bbl_decode import decode  # noqa: E402

AXES = ("roll", "pitch", "yaw")
SUM_LIMIT = {"roll": 250.0, "pitch": 250.0, "yaw": 200.0}
# Con throttle bajo el dron esta apoyado en el banco/suelo: el setpoint pide
# giro pero el dron no se puede mover (I apagado, mixer al ralenti). Esos
# tramos ensucian el seguimiento y la respuesta al escalon.
MIN_THROTTLE = 0.20
COLORS = ["#2563eb", "#dc2626", "#16a34a", "#9333ea", "#ea580c", "#0891b2"]


class Log:
    def __init__(self, path: Path) -> None:
        meta, names, rows, stats = decode(path)
        if not rows:
            raise ValueError(f"{path.name}: sin registros")
        a = np.asarray(rows, dtype=float)
        self.name = path.stem
        self.meta = meta
        self.stats = stats
        self.c = {n: a[:, i] for i, n in enumerate(names)}
        self.t = (self.c["t_us"] - self.c["t_us"][0]) / 1e6
        # Tasa REAL de registros: si el PID se atrasa, el log va mas lento que
        # log_hz y los espectros con la tasa nominal saldrian corridos.
        dur = self.t[-1] - self.t[0]
        nominal = float(meta.get("log_hz", 1000))
        self.fs = (len(self.t) - 1) / dur if dur > 0 else nominal
        if abs(self.fs - nominal) > 0.02 * nominal:
            print(f"{self.name}: aviso, registros a {self.fs:.0f} Hz (nominal {nominal:.0f} Hz); "
                  f"los espectros usan la tasa real")
        self.angle_limit = float(meta.get("angle_limit_deg", 30.0))
        self.poles = float(meta.get("motor_poles", 14))
        self.frames_per_record = float(meta.get("pid_hz", 2000)) / self.fs

    @property
    def has_rpm(self) -> bool:
        return "erpm_ch1" in self.c and bool(np.any(self.c["erpm_ch1"] > 0))

    def motor_hz(self, i: int) -> np.ndarray:
        """Frecuencia de giro del rotor (Hz) del motor CH(i+1)."""
        return self.c[f"erpm_ch{i + 1}"] / 60.0 / (self.poles / 2.0)

    def __getitem__(self, key: str) -> np.ndarray:
        return self.c[key]


# --------------------------------------------------------------------------
# Analisis
# --------------------------------------------------------------------------
def step_response(sp: np.ndarray, gy: np.ndarray, fs: float,
                  valid: np.ndarray | None = None,
                  win_s: float = 2.0, resp_s: float = 0.5,
                  min_sp: float = 20.0) -> tuple[np.ndarray, int]:
    """Respuesta al escalon por deconvolucion de Wiener en ventanas solapadas.

    Solo usa ventanas con setpoint suficiente. Devuelve la mediana y cuantas
    ventanas entraron. En modo ANGLE el setpoint de rate sale del lazo de
    nivel (depende del propio angulo), asi que es una estimacion.
    """
    n = int(win_s * fs)
    m = int(resp_s * fs)
    if len(sp) < n:
        return np.array([]), 0
    hann = np.hanning(n)
    out = []
    for s in range(0, len(sp) - n, n // 4):
        if valid is not None and not np.all(valid[s:s + n]):
            continue
        x = sp[s:s + n]
        if np.max(np.abs(x)) < min_sp:
            continue
        y = gy[s:s + n]
        X = np.fft.rfft((x - x.mean()) * hann)
        Y = np.fft.rfft((y - y.mean()) * hann)
        pxx = np.abs(X) ** 2
        H = (Y * np.conj(X)) / (pxx + 1e-4 * pxx.max())
        h = np.fft.irfft(H, n)[:m]
        out.append(np.cumsum(h))
    if not out:
        return np.array([]), 0
    return np.median(np.asarray(out), axis=0), len(out)


def step_metrics(r: np.ndarray, fs: float) -> dict[str, float]:
    if r.size == 0:
        return {}
    tt = np.arange(r.size) / fs * 1000.0
    final = float(np.mean(r[int(0.3 * fs):int(0.5 * fs)]))
    ref = final if final > 0.2 else 1.0
    above = np.where(r >= 0.5 * ref)[0]
    return {
        "peak": float(r.max()),
        "t_peak_ms": float(tt[int(np.argmax(r))]),
        "t50_ms": float(tt[above[0]]) if above.size else float("nan"),
        "final": final,
        "overshoot_pct": float((r.max() / ref - 1.0) * 100.0),
    }


def psd(x: np.ndarray, fs: float) -> tuple[np.ndarray, np.ndarray]:
    f, p = signal.welch(x - np.mean(x), fs, nperseg=min(1024, len(x)))
    return f, p


def lag_ms(sp: np.ndarray, gy: np.ndarray, fs: float) -> float:
    if np.std(sp) < 1.0:
        return float("nan")
    cc = signal.correlate(gy - gy.mean(), sp - sp.mean(), mode="full")
    lags = signal.correlation_lags(len(gy), len(sp))
    sel = (lags >= 0) & (lags < int(0.3 * fs))
    return float(lags[sel][np.argmax(cc[sel])] / fs * 1000.0)


# --------------------------------------------------------------------------
# Graficas por archivo
# --------------------------------------------------------------------------
def plot_overview(lg: Log, out: Path) -> None:
    fig, axs = plt.subplots(4, 1, figsize=(13, 9), sharex=True)
    axs[0].plot(lg.t, lg["throttle"] * 100, color="k", lw=0.8)
    axs[0].set_ylabel("throttle %")
    for i in range(4):
        axs[1].plot(lg.t, lg[f"motor_ch{i + 1}"], lw=0.6, color=COLORS[i],
                    label=f"CH{i + 1} (M{4 - i})")
    axs[1].set_ylabel("motor DShot")
    axs[1].legend(loc="upper right", ncol=4, fontsize=8)
    axs[2].plot(lg.t, lg["vbat"], color="#ea580c", lw=0.8)
    axs[2].set_ylabel("bateria V")
    axs[3].plot(lg.t, lg["loop_us"], color="#0891b2", lw=0.6)
    axs[3].axhline(500, color="k", ls="--", lw=0.6)
    axs[3].set_ylim(400, 800)
    axs[3].set_ylabel("intervalo PID us")
    axs[3].set_xlabel("tiempo s")
    fig.suptitle(f"{lg.name} - resumen (intervalo PID recortado a 800 us; "
                 f"max real {lg['loop_us'].max():.0f} us)")
    for a in axs:
        a.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out / "overview.png", dpi=110)
    plt.close(fig)


def plot_tracking(lg: Log, out: Path) -> None:
    fig, axs = plt.subplots(5, 1, figsize=(13, 12), sharex=True)
    for i, ax in enumerate(AXES):
        axs[i].plot(lg.t, lg["setpoint_" + ax], color="k", lw=0.8, label="setpoint")
        axs[i].plot(lg.t, lg["gyro_" + ax], color=COLORS[i], lw=0.7, label="gyro filtrado")
        axs[i].set_ylabel(f"{ax} deg/s")
        axs[i].legend(loc="upper right", fontsize=8)
    for j, ax in enumerate(("roll", "pitch")):
        a = axs[3 + j]
        a.plot(lg.t, lg["stick_" + ax] * lg.angle_limit, color="k", lw=0.8,
               label="objetivo (stick x limite)")
        a.plot(lg.t, lg["angle_" + ax], color=COLORS[j], lw=0.8, label="angulo")
        a.set_ylabel(f"angulo {ax} deg")
        a.legend(loc="upper right", fontsize=8)
    axs[-1].set_xlabel("tiempo s")
    for a in axs:
        a.grid(alpha=0.3)
    fig.suptitle(f"{lg.name} - seguimiento (lazo de rate arriba, lazo de nivel abajo)")
    fig.tight_layout()
    fig.savefig(out / "tracking.png", dpi=110)
    plt.close(fig)


def plot_terms(lg: Log, out: Path) -> None:
    fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for i, ax in enumerate(AXES):
        a = axs[i]
        a.plot(lg.t, lg["p_" + ax], lw=0.6, color="#2563eb", label="P")
        a.plot(lg.t, lg["i_" + ax], lw=1.0, color="#16a34a", label="I")
        a.plot(lg.t, lg["d_" + ax], lw=0.6, color="#dc2626", label="D", alpha=0.8)
        lim = SUM_LIMIT[ax]
        a.axhline(lim, color="k", ls=":", lw=0.6)
        a.axhline(-lim, color="k", ls=":", lw=0.6)
        a.set_ylabel(f"{ax} (unid. BF)")
        a.legend(loc="upper right", ncol=3, fontsize=8)
        a.grid(alpha=0.3)
    axs[-1].set_xlabel("tiempo s")
    fig.suptitle(f"{lg.name} - terminos PID (/1000 = fraccion de motor; punteado = limite)")
    fig.tight_layout()
    fig.savefig(out / "pid_terms.png", dpi=110)
    plt.close(fig)


def plot_spectrogram(lg: Log, out: Path) -> None:
    fig, axs = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    for i, ax in enumerate(AXES):
        x = lg["gyro_raw_" + ax]
        f, tt, sxx = signal.spectrogram(x - x.mean(), lg.fs, nperseg=256, noverlap=192)
        a = axs[i]
        a.pcolormesh(tt, f, 10 * np.log10(sxx + 1e-6), shading="auto", cmap="inferno")
        a.set_ylabel(f"{ax} Hz")
        if lg.has_rpm:
            hz = np.mean([lg.motor_hz(m) for m in range(4)], axis=0)
            for h, ls in ((1, "-"), (2, "--"), (3, ":")):
                a.plot(lg.t, hz * h, color="lime", lw=0.7, ls=ls,
                       label=f"RPM medida x{h}" if i == 0 else None)
            a.set_ylim(0, lg.fs / 2)
        dk = "dyn_hz_" + ax
        if dk in lg.c and np.any(lg.c[dk] > 0):
            d = np.where(lg.c[dk] > 0, lg.c[dk], np.nan)
            a.plot(lg.t, d, color="white", lw=1.2, ls="--", label="notch dinamico")
            a.set_ylim(0, lg.fs / 2)
        if (lg.has_rpm or dk in lg.c) and i == 0:
            a.legend(loc="upper left", fontsize=8)
        a2 = a.twinx()
        a2.plot(lg.t, lg["throttle"] * 100, color="cyan", lw=1.0)
        a2.set_ylim(0, 100)
        a2.set_ylabel("throttle %", color="cyan")
    axs[-1].set_xlabel("tiempo s")
    fig.suptitle(f"{lg.name} - ruido del gyro sin filtrar (lineas que suben con el throttle = "
                 f"motores; lineas fijas = resonancia del frame/banco)")
    fig.tight_layout()
    fig.savefig(out / "spectrogram.png", dpi=110)
    plt.close(fig)


def plot_rpm(lg: Log, out: Path) -> None:
    """DShot bidireccional: RPM medida, respuestas validas y carga de CPU."""
    c = lg.c
    fig, axs = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    for i in range(4):
        axs[0].plot(lg.t, lg.motor_hz(i), lw=0.6, color=COLORS[i], label=f"CH{i + 1} (M{4 - i})")
    if "notch_hz" in c:
        rpm_on = lg.meta.get("gyro_rpm_filter") == "1"
        axs[0].plot(lg.t, c["notch_hz"], color="k", lw=1.0, ls="--",
                    label="filtro RPM: fundamental media" if rpm_on else "notch actual (estimado)")
    axs[0].set_ylabel("giro del rotor Hz")
    axs[0].legend(loc="upper left", ncol=5, fontsize=8)

    # % de registros con respuesta valida por motor (ventana de 0.25 s).
    w = max(int(lg.fs * 0.25), 1)
    k = np.ones(w) / w
    ok = c["rpm_ok"].astype(int)
    for i in range(4):
        axs[1].plot(lg.t, np.convolve((ok >> i) & 1, k, "same") * 100, lw=0.8,
                    color=COLORS[i], label=f"CH{i + 1}")
    axs[1].set_ylim(-5, 105)
    axs[1].set_ylabel("respuesta valida %")
    axs[1].legend(loc="lower left", ncol=4, fontsize=8)

    err_rate = np.convolve(c["rpm_err"], k, "same") * lg.fs   # por segundo
    axs[2].plot(lg.t, err_rate, color="#dc2626", lw=0.8)
    axs[2].set_ylabel("respuestas malas /s")

    axs[3].plot(lg.t, c["cpu"], color="#0891b2", lw=0.6, label="CPU %")
    axs[3].set_ylabel("CPU %")
    a2 = axs[3].twinx()
    a2.plot(lg.t, c["rt_us"], color="#9333ea", lw=0.5, alpha=0.7, label="cadena realtime us")
    a2.plot(lg.t, c["motor_us"], color="#ea580c", lw=0.5, alpha=0.9, label="motors_write4 us")
    a2.set_ylabel("us")
    a2.legend(loc="upper right", fontsize=8)
    axs[3].set_xlabel("tiempo s")
    for a in axs:
        a.grid(alpha=0.3)
    fig.suptitle(f"{lg.name} - DShot bidireccional (polos {lg.poles:.0f}) y carga")
    fig.tight_layout()
    fig.savefig(out / "rpm.png", dpi=110)
    plt.close(fig)


def summarize_rpm(lg: Log) -> list[str]:
    c = lg.c
    n = len(c["rpm_ok"])
    ok = c["rpm_ok"].astype(int)
    frames = n * lg.frames_per_record * 4
    bad = float(np.sum(c["rpm_err"]))
    lines = [
        "  DShot bidir: respuesta valida en el ultimo frame "
        + " ".join(f"CH{i + 1} {np.mean((ok >> i) & 1) * 100:.1f}%" for i in range(4))
        + f"; malas o ausentes {bad:.0f} de ~{frames:.0f} ({bad / max(frames, 1) * 100:.2f}%)",
    ]
    hz = [lg.motor_hz(i) for i in range(4)]
    on = c["throttle"] >= MIN_THROTTLE
    if np.any(on):
        lines.append("  giro del rotor con throttle >= 20 %: "
                     + " ".join(f"CH{i + 1} {np.median(hz[i][on]):.0f} Hz" for i in range(4))
                     + (f"; {'filtro RPM (media)' if lg.meta.get('gyro_rpm_filter') == '1' else 'notch estimado'} "
                        f"{np.median(c['notch_hz'][on]):.0f} Hz" if "notch_hz" in c else ""))
    if "dyn_hz_roll" in c:
        parts = []
        for ax in AXES:
            d = c["dyn_hz_" + ax]
            act = d > 0
            parts.append(f"{ax} {np.mean(act) * 100:.0f}% activo"
                         + (f" ~{np.median(d[act]):.0f} Hz" if np.any(act) else ""))
        lines.append("  notch dinamico: " + ", ".join(parts))
    lines.append(f"  CPU: mediana {np.median(c['cpu']):.0f}% (max {c['cpu'].max():.0f}%), "
                 f"cadena realtime p99 {np.percentile(c['rt_us'], 99):.0f} us (max {c['rt_us'].max():.0f}), "
                 f"motors_write4 p99 {np.percentile(c['motor_us'], 99):.0f} us (max {c['motor_us'].max():.0f})")
    return lines


def plot_events(lg: Log, out: Path, n_events: int = 3) -> list[str]:
    """Las maniobras mas grandes de cada eje, sin modelo: setpoint vs gyro."""
    notes: list[str] = []
    fig, axs = plt.subplots(3, n_events, figsize=(15, 9), squeeze=False)
    valid = lg["throttle"] >= MIN_THROTTLE
    for i, ax in enumerate(AXES):
        sp = lg["setpoint_" + ax].copy()
        gy = lg["gyro_" + ax]
        score = np.where(valid, np.abs(sp), 0.0)
        picks: list[int] = []
        for _ in range(n_events):
            k = int(np.argmax(score))
            if score[k] < 25.0:
                break
            picks.append(k)
            lo, hi = max(0, k - int(2 * lg.fs)), min(len(score), k + int(2 * lg.fs))
            score[lo:hi] = 0.0
        for j in range(n_events):
            a = axs[i][j]
            if j >= len(picks):
                a.axis("off")
                continue
            k = picks[j]
            lo, hi = max(0, k - int(0.6 * lg.fs)), min(len(sp), k + int(0.6 * lg.fs))
            a.plot(lg.t[lo:hi], sp[lo:hi], color="k", lw=1.2, label="setpoint")
            a.plot(lg.t[lo:hi], gy[lo:hi], color=COLORS[i], lw=0.9, label="gyro")
            seg_sp, seg_gy = sp[lo:hi], gy[lo:hi]
            sgn = np.sign(sp[k])
            ovs = (np.max(sgn * seg_gy) / np.max(sgn * seg_sp) - 1.0) * 100.0
            a.set_title(f"{ax} t={lg.t[k]:.1f}s  sobrepaso {ovs:+.0f}%", fontsize=9)
            a.grid(alpha=0.3)
            if j == 0:
                a.set_ylabel(f"{ax} deg/s")
                a.legend(fontsize=7)
            notes.append(f"{ax} t={lg.t[k]:.1f}s sp {sp[k]:.0f} -> sobrepaso {ovs:+.0f}%")
    fig.suptitle(f"{lg.name} - maniobras mas grandes (throttle >= 20 %)")
    fig.tight_layout()
    fig.savefig(out / "events.png", dpi=110)
    plt.close(fig)
    return notes


# --------------------------------------------------------------------------
# Graficas comparando archivos
# --------------------------------------------------------------------------
def plot_step(logs: list[Log], out: Path) -> dict[str, dict[str, dict[str, float]]]:
    res: dict[str, dict[str, dict[str, float]]] = {}
    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5), sharey=True)
    for i, ax in enumerate(AXES):
        a = axs[i]
        for k, lg in enumerate(logs):
            r, n = step_response(lg["setpoint_" + ax], lg["gyro_" + ax], lg.fs,
                                 valid=lg["throttle"] >= MIN_THROTTLE)
            if r.size == 0:
                continue
            tt = np.arange(r.size) / lg.fs * 1000.0
            a.plot(tt, r, color=COLORS[k % len(COLORS)], label=f"{lg.name} (n={n})")
            res.setdefault(lg.name, {})[ax] = step_metrics(r, lg.fs) | {"windows": n}
        a.axhline(1.0, color="k", ls="--", lw=0.7)
        a.set_title(ax)
        a.set_xlabel("ms")
        a.set_xlim(0, 500)
        a.grid(alpha=0.3)
        a.legend(fontsize=8)
    axs[0].set_ylabel("gyro / setpoint")
    fig.suptitle("Respuesta al escalon estimada, throttle >= 20 % (ideal: sube rapido a 1.0, "
                 "poco sobrepaso). Modo ANGLE: orientativa")
    fig.tight_layout()
    fig.savefig(out / "step_response.png", dpi=110)
    plt.close(fig)
    return res


def plot_noise(logs: list[Log], out: Path) -> None:
    fig, axs = plt.subplots(3, 3, figsize=(15, 10), sharex=True)
    titles = ("gyro sin filtrar", "gyro filtrado (entra al PID)", "termino D")
    for i, ax in enumerate(AXES):
        for j, key in enumerate(("gyro_raw_" + ax, "gyro_" + ax, "d_" + ax)):
            a = axs[i][j]
            if key == "d_yaw" and all(np.std(lg[key]) < 1e-3 for lg in logs):
                a.text(0.5, 0.5, "yaw sin D (kd = 0)", ha="center", va="center",
                       transform=a.transAxes)
                continue
            for k, lg in enumerate(logs):
                x = lg[key]
                if np.std(x) < 1e-3:
                    continue
                f, p = psd(x, lg.fs)
                a.semilogy(f, p, color=COLORS[k % len(COLORS)], lw=0.9,
                           label=f"{lg.name} thr {np.median(lg['throttle']) * 100:.0f}%")
            if i == 0:
                a.set_title(titles[j])
            if j == 0:
                a.set_ylabel(f"{ax} PSD")
            a.grid(alpha=0.3, which="both")
            a.legend(fontsize=7)
    for a in axs[-1]:
        a.set_xlabel("Hz")
    fig.suptitle("Espectro de ruido (el log es a 1 kHz: arriba de 500 Hz no se ve; "
                 "el gyro sin filtrar trae aliasing de 8 kHz)")
    fig.tight_layout()
    fig.savefig(out / "noise_psd.png", dpi=110)
    plt.close(fig)


# --------------------------------------------------------------------------
def summarize(lg: Log) -> list[str]:
    c = lg.c
    lu = c["loop_us"]
    lines = [
        f"{lg.name}: {lg.t[-1]:.1f} s, throttle mediana {np.median(c['throttle']) * 100:.0f}% "
        f"(max {c['throttle'].max() * 100:.0f}%), bateria {c['vbat'].max():.2f}->{c['vbat'].min():.2f} V",
        f"  lazo: mediana {np.median(lu):.0f} us, p99.9 {np.percentile(lu, 99.9):.0f} us, "
        f">1 ms: {int(np.sum(lu > 1000))} veces (max {lu.max():.0f} us)",
    ]
    m = np.stack([c[f"motor_ch{i}"] for i in range(1, 5)], 1)
    lines.append(f"  motores media CH1..CH4 {np.round(m.mean(0)).astype(int).tolist()}, "
                 f"max {int(m.max())} de 2047")
    v = c["throttle"] >= MIN_THROTTLE
    lines.append(f"  (seguimiento y ruido con throttle >= {MIN_THROTTLE * 100:.0f}%: "
                 f"{v.mean() * 100:.0f}% del log)")
    for ax in AXES:
        sp, gy = c["setpoint_" + ax][v], c["gyro_" + ax][v]
        if sp.size < 1024:
            continue
        err = np.sqrt(np.mean((sp - gy) ** 2))
        f, p = psd(gy, lg.fs)
        hf = f > 30
        noise = np.sqrt(np.trapezoid(p[hf], f[hf]))
        pk = f[hf][int(np.argmax(p[hf]))]
        lines.append(
            f"  {ax:5s} error rms {err:5.1f} deg/s, retardo {lag_ms(sp, gy, lg.fs):5.0f} ms, "
            f"ruido filtrado >30 Hz {noise:4.1f} deg/s (pico {pk:.0f} Hz), "
            f"I media {c['i_' + ax][v].mean():6.1f}, D rms {np.sqrt(np.mean(c['d_' + ax][v] ** 2)):4.1f}")
    return lines


def main() -> int:
    ap = argparse.ArgumentParser(description="Graficas PID del blackbox (BBnnn.BBL)")
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("--out", type=Path, default=Path("analisis"))
    args = ap.parse_args()

    logs: list[Log] = []
    for p in args.files:
        try:
            logs.append(Log(p))
        except (OSError, ValueError) as exc:
            print(f"salteado {p.name}: {exc}")
    if not logs:
        print("no hay logs validos", file=sys.stderr)
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    for lg in logs:
        d = args.out / lg.name
        d.mkdir(exist_ok=True)
        plot_overview(lg, d)
        plot_tracking(lg, d)
        plot_terms(lg, d)
        plot_spectrogram(lg, d)
        print("\n".join(summarize(lg)))
        if "erpm_ch1" in lg.c:
            plot_rpm(lg, d)
            print("\n".join(summarize_rpm(lg)))
        for note in plot_events(lg, d):
            print("  maniobra " + note)

    steps = plot_step(logs, args.out)
    plot_noise(logs, args.out)

    print("\nrespuesta al escalon (pico / t50 / final):")
    for name, axes in steps.items():
        parts = [f"{ax} {m['peak']:.2f}/{m['t50_ms']:.0f}ms/{m['final']:.2f} (n={m['windows']})"
                 for ax, m in axes.items()]
        print(f"  {name}: " + "  ".join(parts))
    print(f"\ngraficas en {args.out.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
