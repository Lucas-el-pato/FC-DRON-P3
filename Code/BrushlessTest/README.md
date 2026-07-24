# BrushlessTest — Hobbywing XRotor 45A DShot/PWM Test

ESP-IDF project for testing one Hobbywing XRotor 45A ESC. The default
firmware path is **DShot300**; the known-working PWM implementation remains
available by changing `ESC_TEST_USE_DSHOT` in `main/main.c` from `1` to `0`.

## Wiring

| Signal | ESP32-S3 |
|---|---|
| ESC S1 signal | GPIO4 |
| ESC ground | GND |

The ESC and ESP32-S3 must share ground. Power the motor from its battery
through the ESC, never from the ESP32-S3.

Do not connect the telemetry/CRT wire for this first DShot test.

## Safety

- Remove the propeller before connecting battery power.
- Secure the motor.
- The test is capped at 50% throttle.

## DShot test

- Protocol: DShot300
- RMT resolution: 40 MHz
- Signal: GPIO4
- Telemetry requests: disabled
- Arm/stop value: DShot `0`
- Arm time: 5 seconds
- Ramp step: 2 seconds

The DShot encoder and update sequence are based on Espressif's official
`examples/peripherals/rmt/dshot_esc` example. Each throttle update restarts
the infinite RMT loop, as required by that example. Telemetry is deliberately
disabled because this test sends an infinite loop of frames.

The XRotor ESC must detect DShot at power-up. Power-cycle the ESC after
flashing so it does not remain in PWM detection mode from an earlier test.

## Build and flash

```bash
cd Code/BrushlessTest
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Adjust the serial port as needed. Expected output includes:

```text
DShot300 armed at value 0 for 5000 ms
DShot throttle 5% -> value ...
DShot throttle 10% -> value ...
```

If the logs advance but the ESC only beeps, verify the exact XRotor model
supports DShot300, that GPIO4 is connected to S1, that grounds are common,
and that the ESC is power-cycled while the DShot firmware is already running.

## PWM fallback

To run the working PWM test instead:

1. Change `#define ESC_TEST_USE_DSHOT 1` to `0` in `main/main.c`.
2. Build and flash again.

PWM uses 50 Hz and 1000–2000 us pulses on GPIO4.
