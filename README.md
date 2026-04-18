# ESP32 Mecanum Yaw Hold Fix

Focused Arduino/ESP32 test sketch for debugging mecanum yaw hold with an MPU-style IMU and four wheel RPM PID loops.

The main file is:

```text
wheel-pid-web/wheel-pid-web.ino
```

For PlatformIO builds, this repo includes:

```text
platformio.ini
```

This repo is intentionally small so the patched yaw-control behavior can be shared and tested without mixing it with the final Pi/ROS firmware.

## What Changed

- Replaced the previous hold-at-stop behavior with a latched yaw-hold state machine.
- Added hysteresis between active hold and settled hold.
- Added a 250 ms settle confirmation so IMU noise does not repeatedly re-arm correction.
- Changed in-place hold to a simple damped controller:

```cpp
shapedError = signedDeadband(rawHeadingError, headingHoldExitDeg);
pTerm = headingHoldPid.kp * shapedError;
dTerm = -headingHoldPid.kd * imuGyroDps;
corrTarget = headingCorrSign * (pTerm + dTerm);
headingCorrRpm = slewToward(previousCorr, corrTarget, headingCorrSlewRpmPerSec, dt);
```

- Made `Hold Min` a stiction assist only for larger, slow yaw errors, not a near-zero bang-bang floor.
- Added telemetry for heading phase, shaped error, correction target, P term, and D term.
- Made position-step buttons apply the latest dashboard tuning before starting the move.

## Hardware Mapping

Motor order is preserved:

| Index | Driver | Wheel |
|---:|---|---|
| 0 | MOTA | BackRight |
| 1 | MOTB | FrontRight |
| 2 | MOTC | BackLeft |
| 3 | MOTD | FrontLeft |

Default RPM signs are preserved from hardware testing:

```cpp
rpmSign[4] = {1, 1, -1, -1}; // BR, FR, BL, FL
headingCorrSign = -1;
```

Do not change those signs unless the quick sign validation in `docs/yaw_hold_test_procedure.md` fails.

## Starting Values

| Parameter | Start |
|---|---:|
| Hold Kp | `0.45` |
| Hold Ki | `0.00` |
| Hold Kd | `0.10` |
| Hold Max | `6.0 RPM` |
| Hold Min | `0.0 RPM` |
| Hold Deadband / Exit | `1.2 deg` |
| Hold Enter | `4.0 deg` |
| Hold Rate DB | `1.5 deg/s` |
| Corr Slew | `25 RPM/s` |
| Head Sign | `-1` |

## Dashboard

The ESP32 starts a Wi-Fi AP and serves the embedded dashboard from the sketch.

Typical network from the original sketch:

```text
SSID: ESP32-PID-TUNE
Password: pid12345
URL: http://192.168.4.1/
```

Use the dashboard to:

- test single-wheel velocity PID in Mode 0
- run position + heading tests in Mode 1
- start yaw hold in place
- view yaw, target heading, heading error, correction RPM, phase, and P/D terms

## Build Check

From the repo root:

```powershell
pio run
```

## Docs

- `docs/yaw_hold_test_procedure.md`
- `docs/tuning_guide.md`
- `docs/rtos_porting_notes.md`

## Important Safety Note

Keep the robot on blocks until these are verified:

- STOP cuts motor output immediately
- each wheel spins in the expected direction
- encoder signs match measured RPM
- Mode 0 wheel velocity PID still behaves correctly
- yaw correction sign is validated

## Upstream Source

This focused repo is based on the `wheel-pid-web.ino` sketch from:

```text
https://github.com/HamzaAlabiad/SOFTWARE
```
