# TDoA Precision Research Notes

Date: 2026-04-16  
Last updated: 2026-04-17 (strategy-stack follow-up)

## Problem

The current RTLS-link TDoA pipeline estimates tag position and sends it to
ArduPilot through MAVLink. For small drones performing light-show maneuvers,
the desired behavior is not only low absolute error, but highly consistent
relative position estimates between tags/drones inside the surveyed volume.

The investigated idea was to exploit the fact that anchors are static. Their
positions and inter-anchor distances can be measured ahead of time with higher
precision than the live UWB packets. Since anchors transmit/receive in a round
robin schedule, anchor packets may carry enough information for the tag to
correct or override some inter-anchor timing errors.

## Code Findings

### Antenna Delay Path

The TDoA anchor wrapper sets the DW1000 hardware antenna delay to zero in
`src/uwb/uwb_tdoa_anchor.cpp`. The configured antenna delay is instead stored
and broadcast in the TDoA payload.

The TDoA tag stores the broadcast antenna delay, but the core TDoA path appears
to store packet inter-anchor `distances[]` directly as remote time-of-flight.
In other words, the configured anchor antenna delay is visible to the tag, but
it is not clearly consumed in the main TDoA estimator path where inter-anchor
ToF is loaded.

Relevant files:

- `src/uwb/uwb_tdoa_anchor.cpp`
- `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp`
- `lib/tdoa_algorithm/src/tag/tdoa_tag_algorithm.cpp`
- `lib/tdoa_algorithm/src/tag/tdoaEngine.cpp`
- `src/uwb/uwb_tdoa_tag.cpp`

### Existing Calibration

The CLI already supports a manual antenna-delay calibration flow. The desktop
app and firmware can use a known distance, solve for antenna delay, and save
that value. This means a simple "calibrate ADelay once" workflow already exists.

The main recommendation is not to dynamically rewrite antenna delay during
flight. Antenna delay should be treated as a static hardware calibration. A
dynamic loop that adjusts ADelay to force live distances toward static survey
distances risks injecting jumps and conflating hardware delay with multipath,
receive-quality variation, clock error, and geometry error.

### 2D And 3D Solver Status

The current runtime tag wrapper uses a 2D estimator with an assumed tag height.
A 3D Newton-Raphson solver exists in the codebase, but runtime support is still
effectively 2D.

The future 8-anchor layout is important: two anchors at each rectangle corner,
one lower and one upper. Simulations show this geometry can materially improve
3D consistency, especially if the solver receives enough live anchor-pair
diversity.

An additional height-aperture simulation compared 8-anchor layouts with upper
anchors at 2, 3, 4, and 5 meters. A 2 m vertical separation remains usable, but
it worsens Z observability. In the simulated 5 m x 5 m footprint, 2 m height
produced about 1.6-2.5x higher vertical sensitivity than 4-5 m, depending on
whether the drone volume stayed below the top plane or extended above it. The
effect became more important with sparse live pair diversity. This means 2 m is
not disqualifying, but it should be treated as a higher-risk 3D geometry,
especially for flight volumes near or above 2 m.

A finer sweep from 2.00 m to 4.00 m in 0.25 m steps showed the biggest practical
improvement between 2.00 m and about 2.75-3.00 m. Above roughly 3.25 m the Z
improvement continued but with diminishing returns in the compact-noise model.
For sparse 12-pair measurements in a high 1.6-2.8 m flight volume, simulated Z
p95 improved from about 5.15 cm at 2.00 m to 2.84 cm at 3.00 m and 2.41 cm at
4.00 m.

### 3D Solver Computational Feasibility

A native C++/Eigen benchmark was added under
`.agents/tdoa-precision/benchmarks/` to measure the current
`tdoa_newton_raphson` implementation with 2D and 3D solve shapes. It uses the
same solver source, the repository Eigen headers, and matrix sizes matching
4-anchor, 6-anchor, and future 8-anchor layouts.

Host results do not show a matrix-size explosion from moving to 3D. With 8
anchors and all 28 anchor-pair measurements, the host benchmark converged in
about 3.7 iterations in the synthetic geometry. Forced 10-iteration 8-anchor
3D solves were still small on the host machine.

The important ESP32S3-specific risk is not the number of anchors by itself. It
is that the current solver uses `double`. The ESP32S3 build was checked with
the PlatformIO Xtensa toolchain and the solver object referenced software
double helpers such as `__adddf3`, `__muldf3`, `__divdf3`, and `sqrt`. This
confirms that the target build is using software-emulated double arithmetic for
the solver.

Conclusion:

- 3D solving is likely feasible on ESP32S3, but should be measured on hardware
  before being enabled permanently at the current 200 Hz estimator task rate.
- If target timing is comfortably below 1 ms typical and below about 2 ms p99,
  the current solver is probably acceptable.
- If timing is several milliseconds or jittery, the next step should be a
  `float` solver variant and an option to skip covariance computation when it
  is not needed.
- The 8-anchor cases did not show heap allocation inside each solve in the
  native allocation-tracking benchmark, but this should still be verified on
  target.

Detailed timings and build commands are recorded in
`.agents/tdoa-precision/benchmarks/benchmark-results.md`.

### Diagnostics Gap

The firmware currently does not export the full-rate TDoA diagnostic stream
needed for rigorous error-distribution analysis.

Existing observations:

- `estimatorCallback()` receives per-pair TDoA measurements.
- `estimatorProcess()` computes the position and RMSE.
- Current position logging is periodic debug output, not a full-rate data log.
- The existing TCP debug position sample path is TWR-oriented.
- The DW1000 driver already exposes useful receive diagnostics:
  - receive quality,
  - first-path power,
  - receive power.

A feature-flagged diagnostic stream should be added before relying on field
claims about distribution, consistency, or correction gains.

## UWB/DW1000 Error Distribution Findings

The likely distribution is not a single clean Gaussian centered on the true
range or true position.

For DW1000/DWM1000-class UWB systems, a more realistic measurement model is:

```text
measurement_error =
  static pair/location/orientation bias
  + short-term random jitter
  + receive-quality-dependent variance
  + occasional heavy-tail/NLOS outliers
  + slower drift from temperature, clocks, and environment changes
```

At the solved position level, these errors are transformed by nonlinear TDoA
geometry. The resulting position distribution is usually:

- anisotropic,
- location-dependent,
- worse near weak geometry zones,
- worse near edges/corners/outside the anchor hull,
- often heavy-tailed,
- potentially spatially biased by repeatable reflections.

This is why consistency must be measured separately from absolute accuracy.

## External References

Primary and high-relevance sources used during the investigation:

- Bitcraze TDoA2 protocol documentation  
  https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/protocols/tdoa2_protocol/

- Bitcraze TDoA principles  
  https://www.bitcraze.io/documentation/repository/lps-node-firmware/master/functional-areas/tdoa_principles/

- Qorvo DW1000 product/documentation page  
  https://www.qorvo.com/products/p/https/www.qorvo.com/products/p/DW1000

- Qorvo UWB FAQ  
  https://www.qorvo.com/innovation/ultra-wideband/resources/faqs

- Qorvo DW1000 NLOS measurements and range/bias characterization paper  
  https://www.qorvo.com/products/d/da008364

- UTIL: An Ultra-wideband Time-difference-of-arrival Indoor Localization
  Dataset, Zhao et al.  
  https://arxiv.org/abs/2203.14471

- Comprehensive outdoor UWB dataset: static and dynamic measurements in
  LOS/NLOS environments  
  https://www.nature.com/articles/s41597-025-05887-9

## Main Technical Conclusions

### Do Not Chase Sub-Tick Antenna Delay First

The DW1000 timestamp tick converted to distance is already on the order of a
few millimeters. The quantization simulation showed that integer tick rounding
is much smaller than the observed centimeter-level tails expected from
multipath, receive quality, anchor-pair timing, solver geometry, and survey
error.

More antenna-delay resolution may be useful later, but it is unlikely to be the
dominant improvement path.

### Correct The Actual TDoA Path First

The most direct firmware improvement is to make the tag-side handling of
inter-anchor ToF explicit and configurable:

```text
measured_prop_tof = raw_packet_tof - antenna_delay[from] - antenna_delay[to]
expected_prop_tof = distance(anchor[from], anchor[to]) / DW1000_TIME_TO_METERS
tof_used = mode(measured_prop_tof, expected_prop_tof)
```

Recommended modes:

- `OFF`: current behavior.
- `MONITOR`: compute corrected/expected values but do not use them.
- `ADELAY_CORRECTED`: subtract configured endpoint delays.
- `BLEND`: blend measured and static expected ToF.
- `LOCKED`: replace inter-anchor ToF with static geometry.

The first deployable step should be `MONITOR`, because it gives real evidence
without changing flight behavior.

### Prefer Robust Weighting Over Dynamic ADelay

Dynamic ADelay adaptation was not promising in simulation. It can reduce a
particular residual in calm conditions but reacts badly to outliers and
multipath because it changes a hardware calibration parameter in response to
environmental measurement error.

A better runtime approach is:

- keep ADelay static,
- correct or lock inter-anchor ToF,
- reject/downweight pair measurements with high residuals,
- use receive-quality indicators to reduce trust in likely NLOS samples,
- expose realistic MAVLink covariance.

Huber weighting should not be treated as a temporal filter. It does not smooth
positions across time and therefore does not inherently add delay. It is a
robust per-solve loss function: measurements that are internally inconsistent
with the current solve receive less influence in that same solve. The
implementation still needs care because aggressive robust weighting can make
the output look cleaner than the covariance reports. If Huber is used, the
reported covariance/RMSE should be computed in a way that still reflects
downweighted or rejected measurements, otherwise ArduPilot may be told the
measurement is more reliable than it really is.

The MAVLink covariance calculation itself requires recomputing a final
Jacobian. Native benchmarks showed this is smaller than the iterative solve but
not negligible, around 15-20 percent of the 3D solve cost in host 8-anchor
tests. On ESP32S3 it should be optional because the current solver uses
software-emulated double arithmetic.

### ArduPilot Covariance Handling Caveat

ArduPilot upstream `master` was inspected at commit
`655175995cd04115aca908c40d78fece2152b0b3` on 2026-04-16. Its external
navigation path does not consume the full MAVLink 6x6 covariance matrix. For
`VISION_POSITION_ESTIMATE`, it converts covariance into:

```text
posErr = sqrt(var_x + var_y + var_z)
angErr = sqrt(var_roll + var_pitch + var_yaw)
```

and passes those scalar values through `AP_VisualOdom` to EKF3.

This creates two important firmware implications:

- RTLS-link must not send finite position covariance while leaving orientation
  covariance variances as `NaN`. ArduPilot computes `angErr` from those fields;
  `NaN` angular error then triggers an ArduPilot internal `constraining_nan`
  path.
- The current 2D covariance mapping uses `var_z = 100.0` because Z is fixed and
  uncertain. ArduPilot collapses XYZ into one scalar, so that turns into
  roughly `10 m` position noise and downweights XY as well. Do not enable this
  covariance path in current 2D mode without an ArduPilot-specific mapping.

RTLS-link also sends roll/pitch/yaw as zero because UWB provides no attitude.
That is acceptable only if ArduPilot is not configured to use external
navigation yaw (`EK3_SRCx_YAW != 6`). For a robust future implementation, we
should either send unknown attitude as NaN or explicitly document/enforce that
RTLS-link is a position source only.

Detailed notes are in `ardupilot-covariance-investigation.md`.

### Online Anchor Correction Refines Geometry Lock

The second-pass simulations separated two static error families that the
earlier `geom_locked` model partially conflated:

```text
anchor_anchor_meas[i,j] = D_ij + endpoint_bias[i,j] + pair_path_bias[i,j] + noise
```

Geometry lock remains valuable because it removes noisy or biased
inter-anchor timing from the remote-anchor timing reconstruction. It does not,
by itself, guarantee removal of endpoint-delay leakage that appears as an
additive per-anchor term in the tag-side distance-difference.

The promising extension is to estimate a slow per-anchor correction from the
round-robin anchor-anchor stream and subtract the corresponding pair
difference before the TDoA solve:

```text
corrected_tdoa(A, B) =
  measured_tdoa(A, B) - (bias_hat[B] - bias_hat[A])
```

If the firmware exposes directional anchor-anchor information, this can be
modeled as a TX/RX decomposition. If only a symmetric DS-TWR-like distance is
available, the fallback is a per-anchor composite correction. The symmetric
fallback was weaker when simulated TX/RX endpoint biases were independent, but
nearly matched the directional correction when endpoint biases were correlated.

Latest stack results:

- current 4-anchor baseline profile: `rel_p95` improved from `4.94 cm` to
  `4.12 cm` with `geom+TX+Huber`;
- current 4-anchor hard-RF profile: `rel_p95` improved from `11.95 cm` to
  `7.43 cm` with `geom+TX+Huber`;
- future 8-anchor baseline profile: `rel_p95` improved from `3.78 cm` to
  `2.80 cm` with `geom+TX+Huber`;
- future 8-anchor hard-RF profile: `rel_p95` improved from `8.02 cm` to
  `4.72 cm` with `geom+TX+Huber`.

This does not make the correction flight-ready. It makes it the highest-value
new `MONITOR` candidate. The implementation should log directional fit,
symmetric fallback fit, pair residuals, and proposed per-pair correction before
applying anything to the flight path.

### Relative Consistency Requires Separate Metrics

For light shows, it is possible for absolute coordinates to be biased while
relative formations remain stable. Therefore, evaluation should include:

- raw absolute error,
- error after removing one constant offset,
- error after best rigid transform,
- error after similarity transform,
- two-tag relative vector error.

These metrics distinguish:

- pure global offset,
- survey frame rotation,
- scale error,
- spatial distortion,
- non-repeatable noise.

## Simplicity-To-Gain Ranking

1. Add TDoA diagnostic logging.
2. Add robust pair weighting (`Huber`) as the first low-risk apply change.
3. Apply or at least monitor antenna-delay correction in the actual TDoA path.
4. Add static-geometry inter-anchor ToF monitor/blend/lock modes.
5. Add online per-anchor correction in `MONITOR` mode from the anchor-anchor
   stream, then test it together with geometry lock/blend.
6. Move to 8-anchor 3D geometry with enough live pair diversity.
7. Add target-side solver timing instrumentation before enabling 3D by default.
8. Add low-latency smoothing tuned from measured noise and flight dynamics.
9. Add static-grid correction field only if experiments show repeatable spatial
   distortion.
10. Consider finer antenna-delay resolution only after the above sources have
   been measured and reduced.

## Suggested Feature Flags

Potential firmware feature flags:

- `USE_TDOA_DIAGNOSTIC_STREAM`
- `USE_TDOA_INTERANCHOR_TOF_CORRECTION`
- `USE_TDOA_STATIC_GEOMETRY_LOCK`
- `USE_TDOA_ONLINE_ANCHOR_TX_CAL`
- `USE_TDOA_ROBUST_SOLVER_WEIGHTS`
- `USE_TDOA_TRIANGLE_PAIR_REJECT`
- `USE_TDOA_ERROR_FIELD_CORRECTION`
- `USE_TDOA_SOLVER_TIMING_DIAGNOSTICS`

All new firmware features should follow the repository feature-flag convention
through `features.hpp`, `feature_validation.hpp`, and `user_defines.txt`.
