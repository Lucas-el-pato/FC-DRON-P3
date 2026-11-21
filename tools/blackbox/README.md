# Blackbox decoder

The flight firmware (`PinCubeMX/app/fc/fc_blackbox.c`) writes one `BBnnn.BBL`
file per arm to the microSD: a text header (gains, rates, field list) and
records at 1 kHz (gyro raw/filtered, setpoint, P/I/D/sum, angle, sticks,
throttle, motors, vbat, loop time). Format 1 = 72-byte records. Format 2
(89 bytes) adds per motor eRPM from bidirectional DShot (`erpm_chN`, eRPM),
`notch_hz` (estimated motor notch, or the mean rotor fundamental when the
RPM filter is on: header `gyro_rpm_filter=1`), `rt_us` / `motor_us` / `cpu` (load) and
`rpm_ok` / `rpm_err` (valid replies in the last frame / bad or missing
replies since the previous record). The header has `dshot_bidir` and
`motor_poles`. Format 3 (95 bytes) adds `dyn_hz_roll/pitch/yaw`, the center of
the dynamic notch per axis in Hz (0 = off), and `gyro_dyn_notch` / `dyn_notch_*`
in the header.

```bash
python3 bbl_decode.py /media/$USER/SDCARD/BB001.BBL               # summary
python3 bbl_decode.py /media/$USER/SDCARD/BB001.BBL --csv bb1.csv  # CSV, real units
```

Units in the CSV: time in s, gyro/setpoint in deg/s, P/I/D/sum in Betaflight
units (/1000 = motor fraction), angle in deg, sticks/throttle -1..1 / 0..1,
motors as DShot values, vbat in V, loop_us in us.

If the battery was unplugged while armed, the file keeps its reserved size;
the decoder stops where `seq` / `t_us` stop being continuous.

## Analysis plots

```bash
python3 bbl_analyze.py /media/$USER/SDCARD/BB00*.BBL --out analisis
```

Needs numpy, scipy and matplotlib. Per log (`analisis/BBnnn/`): `overview`,
`tracking`, `pid_terms`, `events` (largest maneuvers, setpoint vs gyro) and
`spectrogram` (with the measured RPM x1/x2/x3 on top in format 2). Format 2
also writes `rpm.png`: rotor Hz of each motor vs the estimated notch, valid
DShot replies, bad replies per second and CPU load. Across logs (`analisis/`): `noise_psd` and `step_response`.
Segments with throttle below 20 % are skipped (drone resting on the stand).
In ANGLE mode the step response is only indicative: the rate setpoint comes
from the level loop, so prefer `events.png` to judge overshoot and delay.
