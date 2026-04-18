# RTOS Porting Notes

If the yaw hold works in this web sketch, the same controller is applicable to the ESP32 RTOS firmware.

Port the controller logic, not the sketch architecture.

## Copy These Ideas

- `shortestAngleErrorDeg`
- signed deadband shaping
- heading phases
- hysteresis enter/exit
- 250 ms settle confirmation
- P on yaw error
- D on gyro Z rate
- correction clamp
- correction slew limiter
- telemetry values

## Do Not Copy These Parts

- WebServer handlers
- embedded HTML/JavaScript dashboard
- Arduino `String` JSON building
- sketch-specific command routes
- blocking calibration inside the control task

## RTOS Integration Shape

Call the yaw controller from the fixed-period motion task.

Recommended period:

```text
20 ms / 50 Hz
```

Inputs:

```cpp
float headingTargetDeg;
float imuYawDeg;
float imuGyroZDegPerSec;
bool positionDone;
bool holdHeadingAtStop;
float dtSec;
```

Outputs:

```cpp
float headingCorrRpm;
uint8_t headingPhase;
float shapedErrorDeg;
float pTermRpm;
float dTermRpm;
```

Mixing remains the same:

```cpp
rightTarget = positionCmdRpm - headingCorrRpm;
leftTarget  = positionCmdRpm + headingCorrRpm;
```

## Thread Safety

Runtime tuning values should be copied into the control task as a snapshot.

Good options:

- mutex-protected config struct
- critical section around a small config copy
- versioned config copied at task boundary

Avoid reading many independently updated globals directly inside the control loop.

## Suggested Module Boundary

Create one small controller function:

```cpp
YawHoldOutput updateYawHold(const YawHoldInput& in, YawHoldState& state, const YawHoldConfig& cfg);
```

Keep wheel velocity PID untouched until hardware data proves it is the remaining source of chatter.
