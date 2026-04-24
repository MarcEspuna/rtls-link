# Static Tag Experiment Plan

## Goal

Measure the real error distribution of the current DW1000/TDoA system across
the usable show volume.

The main questions:

1. Is the error mostly a constant offset?
2. Is there a repeatable spatial distortion field?
3. Are tails caused by specific anchor pairs or locations?
4. Does receive quality explain bad samples?
5. Do static geometry corrections improve relative consistency?
6. Does online per-anchor correction from the anchor-anchor stream improve
   consistency?
7. What covariance should be sent to ArduPilot?

## Why This Is Worth Doing

The simulations are useful for ranking ideas, but UWB error is environment
dependent. Multipath, floor reflections, body shadowing, antenna orientation,
anchor height, and surface materials can change both the bias and tail behavior.

A static-tag grid test is the cheapest way to find out whether the real system
has:

- Gaussian-like jitter that can be averaged,
- heavy-tail outliers that need robust rejection,
- a stable offset that can be removed,
- a stable distortion field that can be mapped,
- anchor-pair-specific problems,
- tag-orientation-specific bias.

## Physical Setup

Use the real anchor layout, real tag hardware, and real drone mounting
orientation whenever possible.

Recommended first pass:

- 5 x 5 grid minimum in XY.
- 7 x 7 grid if setup time allows.
- At least three heights for future 3D analysis:
  - low,
  - nominal flight height,
  - high.
- 2-3 minutes per point.
- Discard the first 5-10 seconds after moving the tag.
- Repeat selected points at yaw orientations:
  - 0 degrees,
  - 90 degrees,
  - 180 degrees,
  - 270 degrees.
- Repeat the full baseline after a power cycle.

Use a non-metal stand/tripod. Avoid placing the tag directly on metal, dense
electronics, or a person's hand during logging.

## Conditions To Test

Baseline:

- clear line of sight,
- no moving people,
- stable anchor mounts,
- known anchor configuration.

Stress passes:

- near floor,
- near walls,
- near corners,
- tag at different yaw orientations,
- one person moving around outside the direct setup,
- one person intentionally blocking selected links,
- drone body mounted versus bare tag if the mechanical installation differs.

## Data To Log

Position-level data:

```text
timestamp_us
estimated_x
estimated_y
estimated_z
solver_rmse
solver_valid
measurement_count
solver_mode
correction_mode
covariance_if_available
```

Measurement-level data:

```text
timestamp_us
anchor_a
anchor_b
raw_tdoa_distance_diff_m
used_tdoa_distance_diff_m
residual_after_solve_m
measurement_age_us
packet_sequence
packet_gap
```

Radio diagnostics:

```text
rx_power_dbm
first_path_power_dbm
rx_minus_fp_db
receive_quality
noise_or_std_noise_if_available
clock_correction_if_available
```

Configuration metadata:

```text
firmware_commit
desktop_manager_commit
anchor_positions
anchor_antenna_delays
online_anchor_bias_hat
online_anchor_bias_mode
online_anchor_pair_residuals
radio_channel
data_rate
preamble
correction_feature_flags
tag_id
anchor_ids
```

Ground-truth metadata:

```text
true_x
true_y
true_z
tag_yaw_deg
mounting_notes
surface_notes
visibility_notes
temperature_if_available
operator_notes
```

## Firmware Support Needed

Add a feature-flagged diagnostic stream, for example:

```text
USE_TDOA_DIAGNOSTIC_STREAM
```

The first implementation can be simple:

- JSONL or CSV over UDP/TCP.
- One position record per solver output.
- Optional measurement records per TDoA pair.
- Optional periodic anchor-distance diagnostics.
- Optional online anchor-correction diagnostics:
  - directional TX/RX fit if available,
  - symmetric composite fallback,
  - pair residuals,
  - proposed per-pair correction.

The important point is to record raw enough data to analyze later, not just the
filtered position sent to ArduPilot.

## Analysis Steps

For each static point:

1. Remove warmup samples.
2. Compute mean, median, standard deviation, MAD.
3. Compute p50, p90, p95, p99, p99.9 error.
4. Fit or inspect Gaussian, Laplace, Student-t, and mixture behavior.
5. Plot histograms and Q-Q plots.
6. Compute autocorrelation and Allan deviation to see if averaging helps.
7. Compute covariance ellipses.
8. Split errors by anchor pair and receive quality.

Across all points:

1. Plot absolute error heatmaps.
2. Plot mean bias vectors.
3. Remove one global offset and recompute residuals.
4. Fit best rigid transform and recompute residuals.
5. Fit best similarity transform and recompute residuals.
6. Estimate relative formation error between point pairs.
7. Identify bad anchor pairs by residual and outlier rate.
8. Compare modes with paired runs at the same physical points.
9. Compare directional online-correction residuals against symmetric fallback
   residuals to decide what the real firmware can identify.

## Decision Criteria

If one global offset removes most error:

- keep setup simple,
- correct frame offset in configuration or manager tooling.

If rigid/similarity transform helps but offset alone does not:

- improve anchor survey,
- consider self-survey calibration,
- verify dimensions and orientation.

If a smooth spatial bias remains:

- consider a small calibration field / lookup table,
- validate on withheld grid points before deploying.

If tails dominate:

- add robust residual weighting,
- add pair rejection,
- include receive-quality gating.

If error strongly depends on tag yaw:

- standardize drone mounting orientation,
- consider tag-specific calibration,
- test with the full drone body.

If error strongly depends on location:

- improve anchor geometry,
- add top anchors,
- avoid weak geometry zones,
- use per-zone covariance.

## Recommended First Experimental Sequence

1. Current firmware baseline, no correction changes.
2. Same grid with diagnostic receive-quality logging.
3. Firmware `MONITOR` mode for antenna-delay, static-geometry correction, and
   online per-anchor correction.
4. Firmware Huber robust weighting enabled. This is the first candidate apply
   change because it is per-solve, not temporal smoothing.
5. Firmware correction modes tested with Huber already enabled:
   - antenna-delay corrected,
   - geometry blend,
   - geometry lock,
   - online per-anchor correction apply mode,
   - geometry lock plus online per-anchor correction.
6. Repeat selected points with two tags to measure formation consistency.

Do not use this sequence to tune dynamic ADelay in flight. ADelay remains a
static hardware calibration; the online correction being tested is a separate
tag-side measurement correction.
