# TDoA Strategy Stack Results

Date: 2026-04-17

This note captures the second-pass experiments run after reviewing the
TX/RX-decomposition caveats. The goal was to make the implementation path more
concrete by testing the candidate strategies as a stack:

- current solver behavior,
- Huber robust residual weighting,
- geometry-locked inter-anchor timing,
- per-anchor online TX-bias correction,
- symmetric per-anchor correction fallback,
- triangle-inequality pair rejection.

Companion script:

- `simulations/tdoa_strategy_stack_sim.py`

The script is intentionally a decision harness, not a field predictor. It
separates three error families:

1. static per-anchor endpoint bias (`TX_i`, `RX_i`),
2. static inter-anchor pair/path bias,
3. tag-side random noise/outliers and tag-path pair bias.

Metrics are p95 position errors in centimeters. `rel_p95` is the primary
light-show metric: random two-tag formation-vector error after both tags are
localized.

## Commands

Current 4-anchor rig:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 4_2d --model all --seeds 16 --point-repeats 16
```

Future 8-anchor cuboid:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 8_3d --model all --seeds 10 --point-repeats 10
```

Symmetric-only fallback:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 4_2d --model symmetric_good --seeds 16 --point-repeats 16

python3 .agents/tdoa-precision/simulations/tdoa_strategy_stack_sim.py \
  --layout 8_3d --model symmetric_good --seeds 8 --point-repeats 8
```

## 4-Anchor 2D Results

| Model | Current rel_p95 | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber | Geom+Sym+Huber |
|---|---:|---:|---:|---:|---:|---:|
| baseline realistic | 4.94 | 4.72 | 4.70 | 4.16 | 4.12 | 4.26 |
| hard RF / outliers | 11.95 | 8.80 | 8.53 | 8.39 | 7.43 | 7.78 |
| low-bias HW | 3.47 | 3.35 | 3.31 | 3.22 | 3.15 | 3.24 |

Relative drop versus current:

| Model | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
|---|---:|---:|---:|---:|
| baseline realistic | 5% | 5% | 16% | 17% |
| hard RF / outliers | 26% | 29% | 30% | 38% |
| low-bias HW | 4% | 5% | 7% | 9% |

Interpretation:

- On the current 4-anchor layout, Huber is the safest first improvement
  because it handles tag-side outliers and does not depend on the calibration
  model being correct.
- Per-anchor TX correction is the biggest additive gain in the baseline model.
- Geometry lock and TX correction are complementary in hard RF. The full stack
  gave the largest 4-anchor gain there: `11.95 cm -> 7.43 cm rel_p95`.
- In a well-calibrated, low-bias system, all gains shrink. That is expected:
  once static bias is small, the remaining limiter is random/tag-path error.
- Expectation setting: on today's 4-anchor hardware in nominal conditions,
  expect about `10-20%` `rel_p95` improvement from the full stack, not `40%`.
  The larger gain is a hard-RF/outlier result.

## 8-Anchor 3D Results

| Model | Current rel_p95 | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber | Geom+Sym+Huber |
|---|---:|---:|---:|---:|---:|---:|
| baseline realistic | 3.78 | 3.57 | 3.63 | 3.00 | 2.80 | 3.14 |
| hard RF / outliers | 8.02 | 5.94 | 5.70 | 5.67 | 4.72 | 5.12 |
| low-bias HW | 2.63 | 2.43 | 2.40 | 2.32 | 2.21 | 2.31 |

Relative drop versus current:

| Model | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
|---|---:|---:|---:|---:|
| baseline realistic | 6% | 4% | 21% | 26% |
| hard RF / outliers | 26% | 29% | 29% | 41% |
| low-bias HW | 8% | 9% | 12% | 16% |

Interpretation:

- The 8-anchor layout benefits more because it has more pair redundancy and
  better 3D observability.
- The full stack is the best simulated option: `3.78 cm -> 2.80 cm rel_p95`
  in baseline and `8.02 cm -> 4.72 cm rel_p95` in hard RF.
- The improvement is still primarily a consistency improvement, not a promise
  of absolute truth relative to the surveyed anchor frame.

## Symmetric-Only Fallback

The directional correction assumes the firmware exposes enough information to
distinguish `anchor_i -> anchor_j` from `anchor_j -> anchor_i`. If the real
stream only provides a symmetric DS-TWR average, the script can only estimate a
per-anchor composite term.

When TX/RX endpoint biases are mostly uncorrelated, this fallback is weaker
but still useful in the model:

| Layout | Model | Geom+TX+Huber rel_p95 | Geom+Sym+Huber rel_p95 |
|---|---|---:|---:|
| 4_2d | baseline | 4.12 | 4.26 |
| 4_2d | hard RF | 7.43 | 7.78 |
| 8_3d | baseline | 2.80 | 3.14 |
| 8_3d | hard RF | 4.72 | 5.12 |

When TX/RX endpoint biases are highly correlated (`symmetric_good` model), the
symmetric correction nearly matches directional correction:

| Layout | Current rel_p95 | Geom+TX+Huber | Geom+Sym+Huber |
|---|---:|---:|---:|
| 4_2d | 4.94 | 4.12 | 4.12 |
| 8_3d | 3.68 | 2.79 | 2.82 |

Practical conclusion: implement `MONITOR` so the device logs both the
directional model fit and the symmetric fallback prediction. Do not assume the
optimistic directional model until real anchor logs prove it.

## Calibration Averaging

The per-anchor estimate reached its floor after a small number of samples per
anchor pair. More averaging did not erase the remaining error because the
limiter was static pair/path residual, not random timestamp noise.

Directional TX RMS error:

| Layout | Model | 4 samples | 8 samples | 32 samples | 128 samples |
|---|---|---:|---:|---:|---:|
| 4_2d | baseline | 1.70 | 1.60 | 1.52 | 1.51 |
| 4_2d | hard RF | 3.45 | 3.28 | 3.18 | 3.17 |
| 8_3d | baseline | 1.13 | 1.05 | 0.98 | 0.96 |
| 8_3d | hard RF | 2.19 | 2.08 | 2.04 | 2.00 |

Practical conclusion: boot-time calibration does not need hundreds of samples
before producing a stable estimate. Use a short warm-up window for `MONITOR`,
then keep slow running statistics online.

## Pair-Residual Regularization

A sweep from `pair_reg = 1e-4` to `100` produced very similar directional TX
RMS in all tested regimes. Examples at 32 samples per pair:

| Layout | Model | Best RMS | RMS at pair_reg=1 |
|---|---|---:|---:|
| 4_2d | baseline | 1.51 | 1.56 |
| 4_2d | hard RF | 3.07 | 3.15 |
| 8_3d | baseline | 1.01 | 1.02 |
| 8_3d | hard RF | 2.00 | 2.03 |

Practical conclusion: the regularization value is not a fragile tuning knob in
this model. A conservative fixed value is acceptable for a first firmware
implementation, with residuals logged for validation.

## Triangle Rejection

Triangle-inequality rejection was mixed:

- 4-anchor hard RF: `geom+tx+huber` was `7.43 cm`, while
  `geom+tx+tri+huber` was `7.68 cm`.
- 8-anchor hard RF: `geom+tx+huber` was `4.72 cm`, while
  `geom+tx+tri+huber` was `4.70 cm`.

Practical conclusion: keep it as a logged guardrail or optional compile-time
switch, but do not make it part of the first default apply stack.

## Decision

Recommended implementation path:

1. Add diagnostics first:
   - per-pair anchor-anchor running mean and variance,
   - directional TX/RX fit if available,
   - symmetric composite fallback fit,
   - pair residuals after endpoint decomposition,
   - proposed per-tag-pair correction.
2. Enable `Huber` robust residual weighting as the first apply-mode change.
   It is not a temporal filter and should not add delay; it only reduces the
   influence of bad simultaneous pair residuals inside one solve.
3. Add geometry lock/blend in `MONITOR`, then test guarded `BLEND` / `LOCKED`
   before applying online correction. It is mechanically simpler than runtime
   LS, but still depends on survey quality.
4. Add per-anchor correction in `MONITOR`, then apply it only if real logs
   show stable endpoint or symmetric structure and improved static-tag-grid
   metrics.
5. Keep ArduPilot covariance changes separate from solver correction changes.
   Solver RMSE/covariance should remain conservative so ArduPilot's EKF is not
   fed overconfident measurements.

Bottom line: the clearest path is **Huber first, then monitored geometry, then
monitored per-anchor online correction, then combined geometry+online apply mode
after real logs confirm the model**. The simulated upside is modest in low-bias
conditions, meaningful on the current 4-anchor rig, and strongest on the future
8-anchor 3D layout.
