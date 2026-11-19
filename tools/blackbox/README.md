# Blackbox decoder

The flight firmware (`PinCubeMX/app/fc/fc_blackbox.c`) writes one `BBnnn.BBL`
file per arm to the microSD: a text header (gains, rates, field list) and
72-byte records at 1 kHz (gyro raw/filtered, setpoint, P/I/D/sum, angle,
sticks, throttle, motors, vbat, loop time).

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
`spectrogram`. Across logs (`analisis/`): `noise_psd` and `step_response`.
Segments with throttle below 20 % are skipped (drone resting on the stand).
In ANGLE mode the step response is only indicative: the rate setpoint comes
from the level loop, so prefer `events.png` to judge overshoot and delay.
