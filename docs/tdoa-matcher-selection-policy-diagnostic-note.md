# TDoA Matcher Selection Policy Diagnostic Note

Date: 2026-04-25

GitHub issue: https://github.com/MarcEspuna/rtls-link/issues/30

## Context

During stationary hardware validation of the TDoA anchor-model feature, firmware-side estimator diagnostics showed that the final TDoA estimator input stream was dominated by four anchor pairs:

- `01`
- `03`
- `12`
- `23`

The diagonal/non-neighbor pairs `02` and `13` were almost never present in the estimator input stream, even though the anchor model collects and locks all six four-anchor inter-anchor pairs.

This initially looked like it might explain why locking all six inter-anchor ToF values only produced a small stationary jitter improvement. We investigated far enough to identify the cause, then intentionally stopped to avoid pulling the current branch away from its main goal.

## Hardware Evidence

A temporary firmware diagnostic was added to count matcher candidate availability and final selection. In a 30-second `OFF` baseline on the live four-anchor setup, the matcher counters showed:

| Pair | Eligible | Selected |
|---|---:|---:|
| `01` | 8580 | 3063 |
| `02` | 5224 | 15 |
| `03` | 7718 | 2451 |
| `12` | 8573 | 2752 |
| `13` | 5898 | 11 |
| `23` | 8389 | 2821 |

The important result is that `02` and `13` were eligible thousands of times. They were not missing because of absent inter-anchor ToF, missing anchor context, or sequence-number mismatch. They were eligible but were almost never selected.

## Code Path

The TDoA tag algorithm initializes the engine with `TdoaEngineMatchingAlgorithmYoungest`:

```cpp
tdoaEngineInit(&tdoaEngineState,
               now_ms,
               sendTdoaToEstimatorCallback,
               LOCODECK_TS_FREQ,
               TdoaEngineMatchingAlgorithmYoungest);
```

The `Youngest` matcher scans the remote anchors listed in the current packet and picks the eligible candidate with the newest stored anchor update time.

For a cyclic four-anchor TDMA schedule, the newest eligible candidate is usually a temporal neighbor of the current packet source. That naturally favors:

- `01`
- `12`
- `23`
- `03` as the wraparound neighbor

The diagonal/non-neighbor pairs:

- `02`
- `13`

are often valid candidates, but they usually lose the selection contest because a newer temporal neighbor is available.

## Interpretation

This is a matcher selection-policy issue, not an anchor-model collection issue.

The anchor-model feature correctly observes and locks all six inter-anchor pairs. However, the current TDoA estimator path normally emits only one selected pair per received anchor packet. Because the selected pair is chosen by the `Youngest` policy, downstream estimator measurements are heavily biased toward temporal-neighbor pairs.

This means stabilizing all six inter-anchor pair ToFs can only partially affect the current estimator input stream. In the stationary tests, the locked model remained stable and slightly reduced full-window horizontal RMS, but the geometry contribution of `02` and `13` was limited because those pairs are rarely selected.

## Why This Should Move To A Separate Branch

Changing matcher behavior is a different investigation from the current anchor-model branch.

The current branch goal is to validate firmware-side collection, robust locking, health monitoring, and safe override/fallback behavior for inter-anchor ToF stabilization.

A matcher-policy branch should instead study questions such as:

- Should the engine use a round-robin or randomized eligible-pair selector instead of `Youngest`?
- Should it enqueue more than one eligible TDoA pair per received anchor packet?
- Would selecting `02` and `13` more often improve geometry enough to offset additional estimator load or measurement correlation?
- Does changing selection policy affect update rate, RMSE rejects, stale-measurement churn, or flight behavior?
- Should the matcher expose compact candidate/selected-pair telemetry as a normal diagnostic?

Those changes could alter estimator input rate, pair correlation, scheduling assumptions, and CPU load. They should be evaluated cleanly and independently.

## Recommendation

Do not expand the current anchor-model branch to solve matcher selection.

For this branch, keep focus on:

- TDoA anchor-model lock/override behavior,
- compact model status and health reporting,
- safe rollback to `OFF`,
- evidence of whether locked ToF improves stability without regressions.

Open a separate matcher-policy investigation branch for `02`/`13` selection behavior.
