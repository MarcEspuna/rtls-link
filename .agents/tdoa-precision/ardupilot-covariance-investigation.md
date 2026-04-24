# ArduPilot MAVLink Covariance Investigation

Date: 2026-04-16

This document is intentionally separate from the TDoA solver-correction
roadmap. Solver changes affect the position estimate itself; covariance changes
affect how much ArduPilot trusts that estimate. They should be implemented and
validated as separate workstreams.

ArduPilot upstream `master` was fetched and checked before this investigation.
The inspected commit was:

```text
655175995cd04115aca908c40d78fece2152b0b3
2026-04-16 14:43:41 +1000
hwdef: correct BATT2_MONIOTOR -> BATT2_MONITOR
```

The key finding is that ArduPilot does not consume the full
`VISION_POSITION_ESTIMATE.covariance[21]` matrix as a matrix. It converts the
diagonal entries into two scalar noise values before passing the measurement to
EKF3:

```cpp
posErr = sqrtf(covariance[0] + covariance[6] + covariance[11]);
angErr = sqrtf(covariance[15] + covariance[18] + covariance[20]);
```

This is important for RTLS-link because the current firmware only has position
covariance. It sends finite XYZ covariance but leaves roll/pitch/yaw covariance
as `NaN`.

## Confirmed ArduPilot Data Path

For `VISION_POSITION_ESTIMATE`, `GLOBAL_VISION_POSITION_ESTIMATE`,
`ATT_POS_MOCAP`, and `ODOMETRY`, ArduPilot checks only the first covariance
element to decide whether covariance is present.

If `covariance[0]` is finite, ArduPilot computes:

- `posErr = sqrt(var_x + var_y + var_z)`
- `angErr = sqrt(var_roll + var_pitch + var_yaw)`

The off-diagonal covariance terms are ignored by the external-navigation path.
The output of that step is sent to `AP_VisualOdom`.

`AP_VisualOdom_MAV` then constrains these values:

```cpp
posErr = constrain_float(posErr, VISO_POS_M_NSE, 100.0f);
angErr = constrain_float(angErr, VISO_YAW_M_NSE, 1.5f);
```

The default minimums are:

- `VISO_POS_M_NSE = 0.2 m`
- `VISO_YAW_M_NSE = 0.2 rad`

If `constrain_float()` receives `NaN`, ArduPilot raises an internal
`constraining_nan` error and returns the midpoint of the constraint range.
This matches the kind of "ArduPilot complained" behavior seen when covariance
was enabled.

EKF3 then stores external-navigation position if position and `posErr` are not
NaN. It also stores external-navigation yaw when the attitude quaternion and
`angErr` are not NaN. External-navigation yaw is fused only when the active
EKF source set uses `EK3_SRCx_YAW = 6` (`ExternalNav`).

## Likely RTLS-Link Failure Modes

### 1. NaN Angular Covariance

Current RTLS-link mapping:

- fills position covariance entries,
- sets position-attitude cross terms to zero,
- leaves roll/pitch/yaw variances as `NaN`.

Because `covariance[0]` is finite, ArduPilot treats the whole covariance as
present and tries to compute angular error from indices `15`, `18`, and `20`.
Those are `NaN`, so `angErr` becomes `NaN`.

Then `AP_VisualOdom_MAV` calls `constrain_float(angErr, ...)`, triggering an
ArduPilot internal error. After that, `constrain_float()` returns a finite
fallback value, so EKF3 may store the yaw value from the message.

Since RTLS-link sends roll/pitch/yaw as `0, 0, 0`, this can become a false
external yaw measurement if ArduPilot is configured with `EK3_SRCx_YAW = 6`.
Even if external yaw is not fused, the internal error is enough to explain the
bad observed behavior when covariance was turned on.

### 2. Current 2D Mode Exports Huge Z Variance

The current runtime solver is effectively 2D. When covariance is enabled in
2D mode, RTLS-link maps the fixed-height uncertainty as:

```text
var_z = 100.0
```

That is semantically reasonable for a local 3D covariance matrix, but it is a
bad fit for ArduPilot's scalar conversion:

```text
posErr = sqrt(var_x + var_y + 100.0) ~= 10 m
```

ArduPilot then uses this single scalar as the external-navigation position
noise. This means the large Z uncertainty also downweights XY position, even if
ArduPilot is only using external navigation for horizontal position.

This is a second strong explanation for "covariance enabled did not work
right" on the current hardware/firmware path.

### 3. False Yaw Can Exist Even Without Covariance

When covariance is disabled, RTLS-link fills the MAVLink covariance array with
`NaN`. ArduPilot sees `covariance[0]` as `NaN`, so it leaves `posErr = 0` and
`angErr = 0`; `AP_VisualOdom_MAV` then constrains both to the configured
minimums.

That avoids the NaN internal error, but RTLS-link is still sending
roll/pitch/yaw as zero. EKF3 will store a yaw sample if the attitude quaternion
is finite. This yaw is only fused if the EKF yaw source is `ExternalNav`, but
it is still a configuration hazard.

For our UWB-only position estimate, ArduPilot should normally use compass, GPS
yaw, or GSF yaw, not external-navigation yaw from RTLS-link.

## MAVLink Transport Detail

In the local MAVLink C headers, `VISION_POSITION_ESTIMATE` has:

- minimum payload length: `32`
- full payload length: `117`
- covariance field starts at byte offset `32`

That means covariance and reset counter are MAVLink 2 extension data. The
local generated headers use MAVLink 2 start byte `253` by default, so RTLS-link
should transmit the covariance extension unless the channel status is forced to
MAVLink 1. If a future integration forces MAVLink 1 output, covariance will not
arrive as intended.

## Recommendations For Future Implementation

1. Do not re-enable MAVLink covariance in the current 2D mode with
   `var_z = 100.0`.

   Either keep covariance disabled in 2D mode, or add an ArduPilot-specific
   scalar noise mapping that intentionally reports only the horizontal position
   trust expected by ArduPilot. The cleaner path is to wait for the 3D solver
   before sending covariance.

2. Never send finite position covariance with `NaN` roll/pitch/yaw variances to
   ArduPilot.

   If covariance is enabled, fill the attitude variances with finite, very
   large values. A practical value is to make
   `sqrt(var_roll + var_pitch + var_yaw) >= 1.5 rad`, because ArduPilot clamps
   `angErr` to a maximum of `1.5 rad` in the MAV backend.

3. Prefer sending unknown attitude as NaN instead of `0,0,0`, or verify that
   ArduPilot is never configured to use external-navigation yaw from RTLS-link.

   The safest ArduPilot-side parameter expectation is:

   ```text
   EK3_SRCx_POSXY = 6  # ExternalNav, if RTLS-link is the position source
   EK3_SRCx_POSZ  = chosen intentionally: ExternalNav only when 3D UWB Z is trusted
   EK3_SRCx_YAW   != 6 # Do not use RTLS-link as yaw source
   ```

4. Treat the covariance sent to ArduPilot as a scalar confidence input, not as a
   full anisotropic covariance matrix.

   ArduPilot's current external-navigation path ignores covariance
   off-diagonals and collapses the XYZ diagonal to one scalar. For a future 3D
   solver, this means the value that matters operationally is effectively:

   ```text
   reported_position_noise = sqrt(trace(position_covariance))
   ```

5. Keep Huber/robust weighting separate from temporal filtering.

   Robust per-solve weighting does not itself add delay, but if it suppresses
   outliers it must still report conservative RMSE/covariance. Otherwise
   ArduPilot receives a cleaner-looking position and an overconfident
   measurement noise value.

## Practical Test Checklist

Before enabling covariance on real hardware again:

1. Log the exact outgoing MAVLink payload length and the key covariance entries:
   `cov[0]`, `cov[6]`, `cov[11]`, `cov[15]`, `cov[18]`, `cov[20]`.
2. Confirm ArduPilot receives MAVLink 2, not MAVLink 1.
3. Confirm ArduPilot parameters:
   `VISO_TYPE`, `VISO_POS_M_NSE`, `VISO_YAW_M_NSE`,
   `EK3_SRCx_POSXY`, `EK3_SRCx_POSZ`, and `EK3_SRCx_YAW`.
4. In ArduPilot logs, check visual odometry position error and angular error.
5. Only compare covariance-on vs covariance-off after eliminating the NaN
   angular covariance and false-yaw hazards.

## Source References

Local ArduPilot checkout:

- `/tmp/ardupilot-cov-investigation/libraries/GCS_MAVLink/GCS_Common.cpp`
- `/tmp/ardupilot-cov-investigation/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp`
- `/tmp/ardupilot-cov-investigation/libraries/AP_Math/AP_Math.cpp`
- `/tmp/ardupilot-cov-investigation/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp`
- `/tmp/ardupilot-cov-investigation/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp`
- `/tmp/ardupilot-cov-investigation/libraries/AP_NavEKF/AP_NavEKF_Source.cpp`

Pinned upstream URLs:

- `https://github.com/ArduPilot/ardupilot/blob/655175995cd04115aca908c40d78fece2152b0b3/libraries/GCS_MAVLink/GCS_Common.cpp#L4092-L4119`
- `https://github.com/ArduPilot/ardupilot/blob/655175995cd04115aca908c40d78fece2152b0b3/libraries/AP_VisualOdom/AP_VisualOdom_MAV.cpp#L28-L52`
- `https://github.com/ArduPilot/ardupilot/blob/655175995cd04115aca908c40d78fece2152b0b3/libraries/AP_Math/AP_Math.cpp#L282-L293`
- `https://github.com/ArduPilot/ardupilot/blob/655175995cd04115aca908c40d78fece2152b0b3/libraries/AP_NavEKF3/AP_NavEKF3_Measurements.cpp#L1072-L1124`
- `https://github.com/ArduPilot/ardupilot/blob/655175995cd04115aca908c40d78fece2152b0b3/libraries/AP_NavEKF3/AP_NavEKF3_PosVelFusion.cpp#L780-L835`
- `https://github.com/ArduPilot/ardupilot/blob/655175995cd04115aca908c40d78fece2152b0b3/libraries/AP_NavEKF/AP_NavEKF_Source.h#L39-L46`

RTLS-link files:

- `src/app.cpp`
- `src/mavlink/local_position_sensor.cpp`
- `src/uwb/uwb_tdoa_tag.cpp`
- `lib/c_library_v2/common/mavlink_msg_vision_position_estimate.h`
