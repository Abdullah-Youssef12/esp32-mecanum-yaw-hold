# 10-15 Minute Yaw-Hold Tuning Guide

Tune only after the sign validation passes.

## Starting Values

| Parameter | Start |
|---|---:|
| Hold Kp | `0.45` |
| Hold Ki | `0.00` |
| Hold Kd | `0.10` |
| Hold Max | `6.0 RPM` |
| Hold Min | `0.0 RPM` |
| Hold Deadband | `1.2 deg` |
| Hold Rate DB | `1.5 deg/s` |
| Corr Slew | `25 RPM/s` |

Keep Hold Ki at `0`. This yaw hold is deliberately PD plus gyro damping.

## Procedure

1. Put robot on blocks.
2. Confirm STOP works.
3. Zero and calibrate IMU.
4. Start yaw hold at current heading.
5. Twist the robot `10-20 deg`, release, and watch yaw/error/correction.
6. Repeat after each small parameter change.

## Adjustment Rules

| Symptom | Change |
|---|---|
| Does not rotate back | Increase Hold Max to `8`, then raise Hold Kp by `0.10` |
| Motors buzz but yaw does not move | Raise Hold Min by `0.2 RPM`; stop near `1.5 RPM` |
| Returns but overshoots once | Raise Hold Kd by `0.03`, or lower Hold Kp by `0.05` |
| Repeated hunting near target | Set Hold Min to `0`, increase Hold Deadband to `1.5`, lower Hold Kp |
| Correction is jerky | Lower Corr Slew to `15-20` |
| Correction is too slow/dead | Raise Corr Slew to `30-40` |
| Correction target is always clamped | Lower Hold Kp or raise Hold Max slightly |
| Final error is too large but no chatter | Reduce Hold Deadband to `1.0` |

## What Good Looks Like

- target correction rises smoothly after disturbance
- final correction returns to zero
- phase reaches settled
- there is no repeated sign-flip near target
- yaw stays within about `+/-1.5 deg`

## Mode 0 Regression Checklist

Retest after flashing:

1. Single wheel target `+60 RPM`.
2. Single wheel target `-60 RPM`.
3. All four wheels show correct measured sign.
4. STOP immediately kills output.
5. Active wheel graph still follows target.
6. Raw PWM test still works.
7. Position step still translates before hold begins.
