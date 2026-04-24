# Recommended TDoA Precision Roadmap And Test Plan

Date: 2026-04-17

This document turns the research findings into an implementation and validation
roadmap for the real RTLS-link UWB system.

The core principle is:

```text
Do not apply new correction logic first. Instrument it first.
```

The safest first implementation is to add the new functionality in a
`MONITOR` mode that computes what it would do, sends the diagnostic values
through telemetry/logging, and leaves the flight-path position estimate
unchanged. Once real logs show that the correction is stable and improves
offline replay metrics, then enable guarded `APPLY` modes one at a time.

## Current Recommendation

The best implementation order is:

1. Add Huber robust solver weighting.
2. Add diagnostic logging that can capture pair-level measurements and solver
   residuals.
3. Add static-geometry inter-anchor correction in `MONITOR`, then test
   guarded `BLEND` / `LOCKED` modes.
4. Add online per-anchor correction in `MONITOR` mode.
5. Run static-tag and two-tag consistency tests using telemetry logs.
6. Enable online correction in guarded `APPLY` mode only after the monitor logs
   pass acceptance criteria.
7. Test geometry lock/blend combined with online correction.
8. Keep ArduPilot covariance packing as a separate workstream.
9. Revalidate everything in the future 8-anchor 3D layout.

The recommended `APPLY` order is:

```text
Huber -> geometry lock/blend -> online per-anchor correction
```

The recommended observability order is slightly broader:

```text
diagnostics -> Huber -> geometry MONITOR -> online correction MONITOR
```

Online correction can be instrumented early because `MONITOR` is flight-path
safe. It should not be applied before the simpler geometry path has been
measured.

## Expected Gains

Set expectations conservatively for today's hardware.

The current 4-anchor layout is already tight geometrically. In the latest stack
simulation, the full `geom+TX+Huber` stack gave:

| Regime | Current rel_p95 | Full stack rel_p95 | Drop |
|---|---:|---:|---:|
| baseline realistic | 4.94 cm | 4.12 cm | 17% |
| low-bias HW | 3.47 cm | 3.15 cm | 9% |
| hard RF / outliers | 11.95 cm | 7.43 cm | 38% |

So on the current 4-anchor rig in nominal conditions, expect roughly
`10-20%` relative-consistency improvement, not `40%`. Larger gains are most
likely in hard RF / outlier-heavy conditions and in the future 8-anchor 3D
layout.

This matters for acceptance tests: a 10-15% repeatable `rel_p95` improvement
on today's hardware is a meaningful result. A lack of 40% improvement in a
well-calibrated nominal setup should not be treated as failure by itself.

## Why Monitor First

The online correction idea is physically plausible and simulation-supported,
but the real DW1000/TDoA2 timing path may not match the simplified simulation
exactly. Real measurements include:

- endpoint TX/RX bias,
- pair-specific multipath,
- distance-dependent bias,
- antenna orientation effects,
- temperature drift,
- receive-quality-dependent jitter,
- occasional NLOS outliers.

`MONITOR` mode lets the firmware compute the correction and publish the
diagnostics without changing the position sent to ArduPilot. This answers the
important reliability questions before the correction affects flight:

- Are the estimated per-anchor corrections stable?
- Are pair residuals small enough to trust the endpoint-bias model?
- Does the directional TX/RX model fit better than the symmetric fallback?
- Would corrected measurements reduce offline position and formation error?
- Does the correction drift with temperature or after a power cycle?
- Does it fail gracefully when a pair becomes noisy or blocked?

## Firmware Roadmap

### Phase 1: Huber Robust Solver Weighting

Feature flag:

```text
USE_TDOA_ROBUST_SOLVER_WEIGHTS
```

Behavior:

- Apply robust residual weights inside one solver call.
- Do not smooth over time.
- Do not introduce intentional delay.
- Report RMSE/covariance conservatively, including downweighted residuals.

Why first:

- It is the simplest useful improvement.
- It directly targets tag-side outliers.
- It does not depend on the online calibration model being correct.

Expected effort:

- 2-4 engineering days for implementation, review, and native tests.

### Phase 2: Diagnostic Stream

Feature flag:

```text
USE_TDOA_DIAGNOSTIC_STREAM
```

Minimum position record:

```text
timestamp_us
tag_id
estimated_x
estimated_y
estimated_z
solver_valid
solver_rmse
solver_mode
correction_mode
measurement_count
covariance_or_scalar_confidence_if_available
```

Minimum measurement record:

```text
timestamp_us
tag_id
anchor_a
anchor_b
raw_tdoa_distance_diff_m
used_tdoa_distance_diff_m
residual_after_solve_m
measurement_age_us
pair_weight
pair_rejected
clock_correction
```

Radio diagnostics where available:

```text
rx_power_dbm
first_path_power_dbm
rx_minus_fp_db
receive_quality
packet_sequence
packet_gap
```

Why this matters:

- It enables offline replay of candidate corrections.
- It prevents subjective field testing.
- It lets us compare absolute error, offset-corrected error, rigid error, and
  formation consistency from the same dataset.

Expected effort:

- 4-8 engineering days, depending on transport format and desktop tooling.

### Phase 3: Static Geometry Correction

Feature flags:

```text
USE_TDOA_STATIC_GEOMETRY_LOCK
USE_TDOA_INTERANCHOR_TOF_CORRECTION
```

Modes:

```text
OFF
MONITOR
BLEND
LOCKED
```

Behavior:

- Use configured/surveyed anchor positions to compute expected inter-anchor
  ToF.
- In `MONITOR`, publish the expected-vs-measured difference but do not modify
  the solve.
- In `BLEND`, blend measured and expected inter-anchor ToF conservatively.
- In `LOCKED`, replace the inter-anchor ToF with the static geometry value.

Why before online correction `APPLY`:

- It is mechanically simpler than runtime LS.
- It directly uses the fact that anchors are static.
- It is easier to reason about and to disable.

Main risk:

- It depends on anchor survey quality. A bad survey can create repeatable
  spatial distortion. This is why `MONITOR` and static-grid validation still
  come before normal use.

Expected effort:

- 3-6 engineering days if anchor positions are already available in the tag
  runtime path.

### Phase 4: Online Anchor Correction In MONITOR

Feature flag:

```text
USE_TDOA_ONLINE_ANCHOR_TX_CAL
```

Submodes:

```text
OFF
MONITOR
APPLY
```

`MONITOR` computes:

```text
bias_hat[i]
directional_tx_hat[i]       # if observable
directional_rx_hat[i]       # if observable
symmetric_bias_hat[i]       # fallback
pair_residual[i,j]
pair_sample_count[i,j]
proposed_pair_correction[A,B] = bias_hat[B] - bias_hat[A]
```

It also publishes:

```text
directional_fit_rms
symmetric_fit_rms
max_pair_residual
correction_age_ms
correction_valid
correction_reject_reason
```

Initial behavior:

- Collect running anchor-anchor means.
- Wait for a minimum sample count before marking correction valid.
- Solve at boot after warm-up, then update slowly.
- Do not apply correction to the solver in `MONITOR`.
- Log both directional and symmetric fallback estimates.
- Use a deliberately slow estimator. Initial EWMA/reference time constant:
  `1-10 s`, default `5 s`. Do not allow sub-second tuning such as `50 ms` for
  flight-path correction. For first `APPLY`, prefer a frozen correction after
  warm-up; only test slow online updates after static and motion logs are good.

Expected effort:

- 5-10 engineering days for firmware monitor mode.
- Additional time for desktop/log parsing if the manager needs UI support.

### Phase 5: Offline Replay And Static Validation

Before any `APPLY` mode, use logs to replay:

```text
current
Huber
Huber + online correction prediction
Huber + geometry lock prediction
Huber + geometry lock + online correction prediction
```

The replay tool should compute:

```text
raw_p95
off_p95
rigid_p95
rel_p95
p99
outlier_rate
pair_residual_rms
```

Why replay first:

- It lets us test correction logic on real data without risking flight
  behavior.
- It can compare several modes from one physical data collection.
- It exposes whether the simulated gains survive contact with real RF.

Expected effort:

- 3-6 days for first useful replay scripts if logs are clean.

### Phase 6: Guarded APPLY Mode

Only add `APPLY` after static logs show improvement.

Guardrails:

```text
minimum_sample_count_per_pair
maximum_allowed_fit_rms
maximum_pair_residual
maximum_pair_correction
maximum_correction_rate
maximum_correction_age
fallback_to_OFF_on_invalid_fit
telemetry_flag_when_correction_is_active
```

Suggested first behavior:

- Apply only Huber by default.
- Apply geometry `BLEND` / `LOCKED` before applying online correction, assuming
  static-grid validation supports it.
- Allow online correction apply only from a feature flag or runtime setting
  after `MONITOR` logs pass the decision gate.
- Freeze correction during flight unless slow update has already been proven.
- Never update correction frame-by-frame.
- Keep a hard fallback to current behavior.

Expected effort:

- About 1 week after monitor logs are good.

Test order:

1. Huber only.
2. Huber + geometry lock/blend.
3. Huber + online correction.
4. Huber + geometry lock + online correction.

Why not first:

- Geometry lock depends on anchor survey quality.
- A bad survey can create repeatable spatial distortion.
- The latest simulations show geometry lock and online correction are
  complementary, so they should be evaluated together after both are
  observable.

### Phase 7: Desktop Manager And Operator UX

Project policy requires matching firmware and desktop support for new features.
Create a parallel `tools/rtls-link-manager` branch and matching PR.

Minimum desktop scope:

- Runtime controls for correction mode:
  - `OFF`
  - `MONITOR`
  - `BLEND` / `LOCKED` where applicable
  - `APPLY` only when feature-enabled
- Visible status:
  - correction active/inactive,
  - correction valid/invalid,
  - reject reason,
  - sample counts,
  - fit RMS,
  - maximum pair residual,
  - correction age.
- Diagnostics panel or export view for:
  - per-pair anchor residuals,
  - `bias_hat`,
  - directional-vs-symmetric fit comparison,
  - pair rejection/weighting state.
- Log export suitable for offline replay.

Do not hide `APPLY` behind a silent firmware flag. Operators need to know when
the correction is influencing the position estimate.

### Phase 8: ArduPilot Covariance

Keep this separate from solver correction testing.

The detailed covariance investigation belongs in
`ardupilot-covariance-investigation.md`. This roadmap only keeps it as a
separate cross-linked workstream so it does not get mixed with solver
correction validation.

Required fixes:

- Do not send finite position covariance with NaN attitude covariance entries.
- Do not use literal `var_z = 100` for XY-only mode when ArduPilot consumes a
  scalar `posErr = sqrt(var_x + var_y + var_z)`.
- Send an ArduPilot-specific scalar confidence mapping for XY-only mode.
- Keep RTLS-link configured as a position source, not a yaw source.

Why separate:

- Solver changes affect measurement values.
- Covariance changes affect how much ArduPilot trusts those values.
- Testing both at once makes failures difficult to diagnose.

## First Hardware Test Plan

### Test 1: Anchor-Only Stability

Purpose:

- Verify that the anchor-anchor stream produces stable diagnostics before using
  a tag.

Setup:

- Anchors fixed in the normal 4-anchor layout.
- Known approximate anchor positions.
- No tag movement required.

Run:

- Log 10-20 minutes of anchor-anchor diagnostics.
- Repeat after power cycle.
- Repeat with one intentional RF stress condition if easy, such as a person
  standing near one path.

Pass criteria:

- Pair sample counts grow as expected.
- Directional and symmetric fit RMS are stable after warm-up.
- `bias_hat` does not jump unexpectedly.
- Bad pair residuals identify the stressed path.
- The monitor mode never affects reported position.

### Test 2: Single Static Tag, One Good Point

Purpose:

- Check basic logging, residuals, and correction prediction with a tag.

Setup:

- One tag fixed near the center of the anchor area.
- Known tag position measured as well as practical.
- Run current solver output unchanged.

Run:

- 5-10 minutes baseline with monitor diagnostics.
- 5-10 minutes with Huber enabled.
- No online correction apply yet.

Metrics:

```text
position_std
position_p95
solver_rmse
pair_residual_rms
outlier_rate
offline_replay_current_vs_huber_vs_predicted_correction
```

Pass criteria:

- Huber does not introduce visible latency because the tag is static.
- Huber reduces tails or at least does not worsen p95/p99.
- Predicted online correction improves offline replay or is clearly neutral.
- No correction estimate jumps.

### Test 3: Static Grid

Purpose:

- Measure whether improvements are consistent across the surveyed space.

Setup:

- 5 x 5 XY grid minimum.
- At least one nominal flight height.
- Add more heights if testing 3D readiness.

Run:

- 2-3 minutes per point.
- Discard first 5-10 seconds after moving the tag.
- Repeat selected points at different yaw orientations.
- Repeat a few points after power cycle.

Modes:

```text
current
Huber
Huber + online correction prediction
Huber + geometry lock prediction
Huber + geometry lock + online correction prediction
```

Metrics:

```text
raw_p95
off_p95
rigid_p95
rel_p95
per-zone_p95
pair_outlier_rate
receive_quality_correlation
```

Pass criteria to consider `APPLY`:

- `rel_p95` improves versus Huber-only by a meaningful margin, for example
  10-15 percent or more.
- `p99` does not get worse.
- Improvement appears across most of the grid, not only one point.
- Residuals identify bad pairs rather than hiding them.
- Correction remains stable after power cycle.

### Test 4: Two Static Tags

Purpose:

- Directly test the light-show-relevant relative consistency metric.

Setup:

- Two tags mounted at known fixed separation.
- Test several separations and orientations if possible.
- Keep both tags static for each run.

Run:

- 2-5 minutes per placement.
- Repeat center, edge, and near-corner placements.

Metrics:

```text
inter_tag_distance_error
inter_tag_vector_error
relative_p95
relative_p99
per_tag_absolute_error
```

Pass criteria:

- Relative vector error improves or remains stable.
- No tag-specific correction bias appears.
- Edge/corner cases do not degrade badly.

### Test 5: Slow Motion Replay

Purpose:

- Check that robust weighting and correction do not create jumps during motion.

Setup:

- Move the tag slowly along a measured line or rectangular path.
- Use a cart/rail/string-marked path if no motion capture is available.

Run:

- Current mode.
- Huber.
- Later, guarded geometry apply and then online correction apply if static
  tests passed.

Metrics:

```text
path_smoothness
step_jumps
solver_valid_rate
pair_drop_rate
estimated_latency
```

Pass criteria:

- No sudden correction-induced jumps.
- Solver valid rate does not fall.
- Huber does not create obvious lag because it is not a temporal smoother.

### Test 6: ArduPilot Bench Integration

Purpose:

- Verify MAVLink behavior before flight.

Setup:

- ArduPilot connected on bench.
- Drone not flying.
- RTLS-link sends position messages.

Run:

- Current covariance disabled.
- Corrected covariance mapping enabled later as a separate test.

Check:

```text
VISION_POSITION_ESTIMATE accepted
posErr finite
angErr finite
no constraining_nan internal error
EKF innovations reasonable
RTLS-link not used as yaw source
```

Pass criteria:

- No ArduPilot internal NaN/constrain warnings.
- EKF behavior does not change unexpectedly when covariance mode changes.

### Test 7: First Flight-Like Test

Purpose:

- Confirm no operational surprises before relying on correction for formation
  accuracy.

Setup:

- Start with Huber only.
- Geometry and online correction still in monitor unless all static tests
  passed.
- Conservative flight envelope.

Run:

- Hover or slow manual movement.
- Log all diagnostics.

Pass criteria:

- No solver jumps.
- No ArduPilot EKF instability.
- Monitor correction prediction remains stable during flight-like conditions.

## APPLY Decision Gate

Do not enable online correction apply mode for normal use until:

1. Anchor-only logs show stable endpoint or symmetric structure.
2. Static tag logs show offline improvement.
3. Two-tag logs show relative consistency improvement or neutrality.
4. Motion logs show no jumps.
5. ArduPilot bench test is clean.
6. Fallback-to-current behavior is tested.
7. Operators can see when correction is active, invalid, or disabled.

## Recommended First Branch Scope

The first implementation branch should include:

- `USE_TDOA_ROBUST_SOLVER_WEIGHTS`
- `USE_TDOA_DIAGNOSTIC_STREAM`
- `USE_TDOA_STATIC_GEOMETRY_LOCK` / `USE_TDOA_INTERANCHOR_TOF_CORRECTION` with
  `OFF` and `MONITOR`
- `USE_TDOA_ONLINE_ANCHOR_TX_CAL` with `OFF` and `MONITOR`
- offline log parser/replay script
- matching `rtls-link-manager` branch for controls, status, and log export
- no online correction `APPLY` by default
- no ArduPilot covariance behavior change unless guarded separately

This branch gives useful data quickly while keeping flight behavior controlled.

## Bottom Line

Yes, the first real implementation should compute the new correction logic and
send telemetry/logging metrics before applying it. The best early test is not a
flight test. It is:

1. anchor-only monitor stability,
2. one static tag at one point,
3. static grid replay,
4. two-tag relative consistency,
5. only then guarded apply and flight-like validation.

This path turns the idea from a plausible simulation result into something we
can trust on the actual UWB system.
