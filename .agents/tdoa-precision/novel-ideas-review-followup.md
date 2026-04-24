# Review Follow-Up: Novel TDoA Ideas And ArduPilot XY Covariance

Date: 2026-04-17

This is a review note for the new follow-up experiments in
`novel-ideas-followup.md`, `strategy-stack-results.md`, and their companion
simulation scripts. It is intended to be forwarded as a concise technical
assessment.

## Short Conclusion

The new feedback is legitimate and useful, but it should be treated as a strong
`MONITOR`-mode experiment, not as a proven flight-path correction yet.

The online per-anchor TX-delay calibration idea is the most valuable new
candidate. It targets a different error path than geometry lock:

- geometry lock replaces or corrects noisy inter-anchor ToF used to reconstruct
  anchor transmit timing;
- per-anchor TX correction tries to remove anchor TX-delay leakage from the
  tag-side TDoA measurement itself.

Those two approaches are complementary. The latest stack results also make the
implementation order clearer: apply Huber first, test geometry lock/blend
before online correction `APPLY`, keep online per-anchor correction in
`MONITOR`, and only apply the combined geometry+per-anchor stack after hardware
logs show the model is stable.

## ArduPilot Covariance Separation

The ArduPilot covariance work is intentionally kept separate from this solver
roadmap. The details live in `ardupilot-covariance-investigation.md`.

Reason: solver changes affect the position estimate itself, while covariance
changes affect how much ArduPilot trusts that estimate. Testing both at once
makes failures difficult to diagnose.

## Assessment Of The New TX/RX Decomposition Idea

The new simulation separates two error families that the earlier simulations
collapsed together:

```text
measured_anchor_to_anchor[i,j] =
    D_ij + TX_i + RX_j + pair_multipath_ij + noise
```

This distinction is useful. Per-anchor endpoint delay is decomposable across
many anchor-anchor measurements. Per-pair multipath is not decomposable and
should remain as a residual.

The proposed correction solves for per-anchor `TX_hat` and `RX_hat`, then
applies only the tag-relevant TX difference:

```text
corrected_tag_tdoa(A, B) =
    measured_tag_tdoa(A, B) - (TX_hat[B] - TX_hat[A])
```

This is a legitimate idea because the tag RX delay cancels in TDoA
subtraction, while anchor TX delay can remain as a difference between anchor
transmissions.

It is also importantly different from dynamic ADelay adaptation. Hardware
ADelay stays static. The correction is inferred from anchor-anchor data and
should update slowly, such as at boot and then on a minutes-scale if needed.

## What The Simulation Supports

I ran the companion script:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_novel_ideas_sim.py \
  --seed 1 --repeats 2
```

The published table values reproduce from the checked-in script.

The strongest simulated result is:

- `TX cal + Huber` materially reduces raw error in all tested regimes.
- It also improves the light-show-relevant relative error metric.
- Triangle rejection gives only marginal gain on top of Huber.
- Pair inverse-variance weighting and direct pair-residual cancellation do not
  justify their complexity in the current simulation.

This supports adding the TX/RX decomposition to the roadmap as a serious
candidate.

## Latest Strategy-Stack Results

The second-pass harness `simulations/tdoa_strategy_stack_sim.py` compared the
candidate corrections together instead of treating TX calibration alone as the
main decision.

Current 4-anchor 2D layout, `rel_p95` in centimeters:

| Model | Current | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
|---|---:|---:|---:|---:|---:|
| baseline realistic | 4.94 | 4.72 | 4.70 | 4.16 | 4.12 |
| hard RF / outliers | 11.95 | 8.80 | 8.53 | 8.39 | 7.43 |
| low-bias HW | 3.47 | 3.35 | 3.31 | 3.22 | 3.15 |

Future 8-anchor 3D layout, `rel_p95` in centimeters:

| Model | Current | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
|---|---:|---:|---:|---:|---:|
| baseline realistic | 3.78 | 3.57 | 3.63 | 3.00 | 2.80 |
| hard RF / outliers | 8.02 | 5.94 | 5.70 | 5.67 | 4.72 |
| low-bias HW | 2.63 | 2.43 | 2.40 | 2.32 | 2.21 |

The directional-vs-symmetric fallback test is important. If TX/RX endpoint
biases are correlated, the symmetric fallback nearly matches the directional
model:

| Layout | Current | Geom+TX+Huber | Geom+Sym+Huber |
|---|---:|---:|---:|
| 4-anchor | 4.94 | 4.12 | 4.12 |
| 8-anchor | 3.68 | 2.79 | 2.82 |

This means the idea may still be useful even if the firmware cannot expose a
clean directional TX/RX decomposition. That is useful, but it is also exactly
why `MONITOR` mode must log both model fits before flight-path application.

Expectation note: on today's 4-anchor hardware in nominal conditions, the
latest stack model predicts about `10-20%` `rel_p95` improvement, not `40%`.
The larger numbers belong to hard-RF/outlier conditions and the future
8-anchor layout.

## Important Reservations

### 1. The Real Firmware Timing Model Is More Complex

The simulation uses a simplified one-way expression:

```text
D_ij + TX_i + RX_j
```

The real anchor code computes inter-anchor distance through a double-sided
timing expression:

```text
distance = ((tround2 * tround1) - (treply1 * treply2))
           / (2 * (treply1 + tround2))
```

So the endpoint-delay decomposition may still be valid, but the exact
coefficients, signs, and observability need to be verified against the real
TDoA2 timing path. This is the main reason to start with `MONITOR`.

### 2. The Self-Survey Claim Is Not Yet Proven By The Script

The follow-up document says the zero-setup/self-survey alternating pass
converges in 2-3 iterations. That may be true, but the checked-in script uses
the true anchor positions when estimating delays. It does not yet fully test
the alternating loop:

```text
anchor distances -> self-survey positions -> TX/RX estimate -> updated positions
```

This should be simulated explicitly before relying on the zero-setup claim.

### 3. The LS Size Has Been Corrected

The first note incorrectly described the 8-anchor LS problem as `15 x 15` after
gauge fixing. The corrected model includes pair residual unknowns. For
8 anchors:

```text
TX unknowns:       7   # TX_0 fixed as gauge
RX unknowns:       8
pair residuals:   28
total unknowns:   43
observations:     56 directional measurements
```

This is still very small computationally, and the follow-up document has been
updated.

### 4. Residual Interpretation Needs Caution

A low LS residual would be strong evidence that the endpoint-delay model fits
the current site and hardware. A high residual means pair multipath,
orientation, temperature, distance-dependent bias, or bad survey geometry is
dominating.

That does not necessarily make TX correction harmful, but it reduces confidence
that `APPLY` will match the simulation gains.

## Does This Change Previous Findings?

It refines them; it does not overturn them.

Still valid:

- Dynamic ADelay adaptation should not be used as the main runtime correction.
- Geometry lock/blend remains valuable because anchor geometry is static.
- Huber is still the right robust per-solve layer, as long as covariance/RMSE
  reporting remains conservative.
- The latest stack results make Huber the first apply-mode change, not merely
  a layer to add after online calibration.
- Geometry lock/blend is mechanically simpler than online correction and should
  be tested as the next guarded apply candidate before online correction apply.
- Static-tag experiments are still necessary before trusting field claims.
- For ArduPilot, covariance must be packed for its scalar external-navigation
  interpretation, not as a generic 6x6 matrix.

New addition:

- Add online per-anchor TX-delay calibration as a high-priority monitorable
  correction candidate.

## Recommended Updated Roadmap

1. Keep current diagnostic-stream and static-tag experiment plan.
2. Add `USE_TDOA_ROBUST_SOLVER_WEIGHTS` as the first apply-mode solver change.
3. Add static geometry correction in `MONITOR`, then validate guarded
   `BLEND` / `LOCKED` before online correction apply.
4. Add `USE_TDOA_ONLINE_ANCHOR_TX_CAL` in `MONITOR` mode.
5. Use an initial online-correction EWMA/reference time constant of `1-10 s`
   (`5 s` default), with no sub-second flight-path updates.
6. Log `TX_hat`, `RX_hat`, symmetric fallback, pair residuals, and proposed
   per-measurement corrections without applying them.
7. Compare monitor residuals against real static-tag position error and
   formation consistency metrics.
8. Only move to online correction `APPLY` if the residuals show that the endpoint-delay
   decomposition is stable on real hardware.
9. Add matching desktop-manager support: OFF/MONITOR/APPLY controls,
   correction status, fit RMS, sample counts, pair residuals, and log export.
10. Keep ArduPilot covariance as a separate linked workstream.

## Final Recommendation

Incorporate the new feedback, but label it as:

```text
Promising, physically plausible, simulation-supported, requires hardware
MONITOR validation before flight-path application.
```

The most useful immediate implementation is Huber plus instrumentation. Huber
can be applied first because it is local to one solve and does not add temporal
delay. The online correction should first measure whether the real
anchor-anchor data decomposes into stable per-anchor directional or symmetric
terms with small residuals.
