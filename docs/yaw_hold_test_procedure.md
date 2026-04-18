# Yaw Hold Hardware Test Procedure

Use this first. Do not tune while the robot is on the floor.

## Bench Setup

1. Put the robot on blocks.
2. Flash `wheel-pid-web/wheel-pid-web.ino`.
3. Connect to the ESP32 Wi-Fi AP.
4. Open the dashboard.
5. Press STOP.
6. Zero yaw.
7. Calibrate IMU bias while the robot is still.

## Starting Dashboard Values

| Parameter | Value |
|---|---:|
| Head Kp | `0.90` |
| Head Ki | `0.00` |
| Head Kd | `0.05` |
| Max Head Corr RPM | `10` |
| Hold Max Corr RPM | `6` |
| Hold Min Corr RPM | `0` |
| Move Deadband | `2.0 deg` |
| Hold Deadband | `1.2 deg` |
| Hold Kp | `0.45` |
| Hold Ki | `0.00` |
| Hold Kd | `0.10` |
| Hold Rate DB | `1.5 dps` |
| Corr Slew | `25 rpm/s` |
| Head Sign | `-1` |

## Quick Sign Validation

This takes under 2 minutes.

1. Zero yaw near `0 deg`.
2. Set target heading to `+10 deg`.
3. Press `START YAW HOLD (IN PLACE)`.
4. The robot should rotate toward positive yaw.

Pass:

- yaw moves toward `+10 deg`
- heading error shrinks

Fail:

- yaw moves away from `+10 deg`

If it fails, flip only `Head Sign`. Do not change the RPM signs during this test.

## Disturbance Test

1. Start yaw hold at the current heading.
2. Manually twist the robot about `10-20 deg`.
3. Release it.

Pass:

- yaw returns smoothly toward target
- correction RPM does not repeatedly flip sign near target
- `phase` eventually becomes `3`
- final heading error is about `+/-1.5 deg`
- correction RPM slews back to zero

Fail:

- wheels chatter after yaw is already near target
- correction RPM flips sign repeatedly
- yaw repeatedly crosses the target with visible hunting

## Wrap Test

1. Rotate or set yaw near `+179 deg`.
2. Set target near `-179 deg`.
3. Start yaw hold.

Pass:

- robot takes the short path across wrap
- heading error is near `2 deg`, not `358 deg`

## Telemetry Meaning

| Field | Meaning |
|---|---|
| `phase=0` | idle |
| `phase=1` | moving heading correction |
| `phase=2` | active in-place yaw hold |
| `phase=3` | settled |
| `corr` | final slew-limited correction RPM |
| `target` | raw correction target before slew |
| `shaped` | heading error after deadband shaping |
| `P` | proportional RPM contribution |
| `D` | gyro damping RPM contribution |
