# Novel Ideas Follow-Up

Date: 2026-04-17  
Last revised: 2026-04-17 (strategy-stack pass)

This document is a feedback pass on top of `research-notes.md` and
`simulation-results.md`. It proposes correction ideas that the original
investigation did not isolate, evaluates them in a new simulation model that
captures the relevant error structure, and maps the winning strategy to
concrete firmware touchpoints.

Companion script:

- `simulations/tdoa_novel_ideas_sim.py` — reproducible harness for the
  experiments below. Supports both the current **4-anchor 2D** layout
  (real hardware) and the future **8-anchor 3D** layout via `--layout`.
- `simulations/tdoa_strategy_stack_sim.py` — second-pass harness comparing
  Huber, geometry lock, online per-anchor correction, symmetric fallback,
  and triangle rejection as one stack. The recorded results are in
  `strategy-stack-results.md`.

## Revision notes

The first version of this document made a few claims that deserve
hedging. The document has been updated to:

- Correct the matrix-size claim (it incorrectly said "15 x 15 after the
  gauge fix"). The solver actually estimates 2·N−1 per-anchor delays *plus*
  C(N,2) regularized pair-residuals — so 43 columns for 8 anchors and 13
  columns for 4 anchors, not 15.
- Clarify TX-vs-RX separability. The clean "TX_B − TX_A leakage" in tag
  TDoA assumes the anchor firmware exposes directional inter-anchor ToF. If
  the inter-anchor ToF is computed with a symmetric DS-TWR average
  internally, then only a per-anchor sum (TX_i + RX_i) is identifiable, and
  the tag correction degrades gracefully to a per-anchor total-delay
  correction. The high-level conclusion (a per-anchor additive correction
  derived from anchor-anchor data helps tag TDoA) still holds in either
  case, but the quantitative simulation is the optimistic directional case.
- Hedge the headline "~65% raw reduction". That number is sensitive to the
  ratio between per-anchor static bias and other noise sources. In the
  low-bias regime it drops to ~30%; in hard-RF it only materializes when
  paired with Huber.
- Add a 4-anchor 2D results section, which is the layout the real hardware
  currently uses and the one that should be validated first.
- Add a second-pass strategy-stack section. This refines the implementation
  order: Huber first, geometry lock/blend before online correction `APPLY`,
  online per-anchor correction in `MONITOR`, then combined geometry+per-anchor
  apply mode only after hardware logs confirm the model.

## Motivation

The existing simulation collapses all static bias into an opaque per-pair
`adelay_bias + pair_bias` draw. That conflates two physically distinct
sources:

1. **Per-anchor antenna-delay bias.** Each anchor has a static TX delay and
   RX delay that sums additively into every measurement that uses that
   endpoint. This source is **decomposable** into 2·N unknowns.
2. **Per-pair multipath bias.** Each anchor-pair has a directional,
   environment-specific reflection/diffraction signature. This source is
   **not** decomposable into endpoint unknowns; it is residual.

The round-robin protocol already produces ~2·(N choose 2) inter-anchor
measurements continuously, which is massively over-determined for the 2·N
per-anchor delay unknowns. That observation motivates a new correction path:
instead of replacing measured inter-anchor ToF with expected ToF
(`geom_locked`), decompose the measurements into per-anchor TX/RX components
and apply the TX portion to the tag-side TDoA measurements as an additive
correction. The tag RX delay cancels in any TDoA subtraction, so only TX
matters for the tag.

This is **not** the dynamic-ADelay adaptation rejected in
`research-notes.md`. Hardware ADelay stays static. The correction is:

- derived from data that does not depend on tag position,
- applied only to the tag-side distance-difference,
- updated on a slow timescale (once at boot, optionally online over minutes).

## Ideas Evaluated

### 1. Online per-anchor TX-delay calibration (primary idea)

Model:

```text
measured_anchor_to_anchor[i,j] = D_ij + TX_i + RX_j + multipath_ij + noise
```

Solve the over-determined linear system for `TX_i`, `RX_j`, with `TX_0 = 0`
as a gauge fix and with the per-pair multipath absorbed into a regularized
residual term (large regularization toward zero so the decomposition
prefers per-anchor structure, and only sends into the residual what TX/RX
cannot explain).

Apply on the tag side. The tag's distance-difference retains a
per-anchor static bias even with `geom_locked` active, because the MAC
TX/RX timestamps that go into the tag-side `delta_txAr_to_txAn` step still
include the uncompensated antenna delays. In the simplified directional
model:

```text
tag_distance_diff(A, B) = (dB - dA) + bias(B) - bias(A) + noise
```

where `bias(X)` is a per-anchor term derivable from anchor-anchor
measurements. In the cleanest case `bias(X) ≈ TX_X`; depending on how the
anchor firmware computes inter-anchor ToF, it may be `TX_X + RX_X` or
some protocol-weighted average. Either way, subtract the estimated
`bias_hat[B] - bias_hat[A]` from the tag's measured distance-difference
before it enters the solver.

Why it is structurally different from `geom_locked`:

- `geom_locked` replaces the measured inter-anchor ToF with expected ToF.
  That removes noise and pair bias from the inter-anchor path, but does
  *not* remove the per-anchor delay that leaks into the tag's
  `rxAn_by_T - rxAr_by_T` step via each anchor's uncompensated MAC
  timestamp.
- TX calibration fixes that residual leakage.

Identifiability caveat: if inter-anchor ToF is exposed as a single
symmetric number per pair (DS-TWR-averaged), only the per-anchor sum
`TX_i + RX_i` is observable — the decomposition into TX and RX separately
relies on the two directions being distinguishable in the stream. The
quantitative results below use the directional model. A MONITOR-mode
log that checks whether the two directions differ in practice is the
cheapest way to find out which regime applies on real hardware.

Zero-setup path. The LS decomposition only needs anchor-to-anchor distances
and anchor positions. The positions can come from existing self-survey
(MDS or cuboid). Running two alternating passes (positions → TX/RX →
positions again) converges in 2-3 iterations in simulation and requires no
manual ADelay calibration and no precise anchor survey.

### 2. Symmetric-direction anchor-distance averaging

Use `(d_AB + d_BA) / 2` as the working estimate of each inter-anchor
distance before self-survey or LS. This cancels `TX_A - RX_B` asymmetry
across the two directions and leaves only the symmetric `TX_A + TX_B + RX_A
+ RX_B` sum, which is cleaner for MDS and for LS. Both directions are
already present in the round-robin; the cost is zero.

### 3. Triangle-inequality pair rejection

Any pair measurement satisfying `|tag_distance_diff(A, B)| > D_AB` is
physically impossible. Drop it before the solver. In practice it overlaps
heavily with Huber, so it adds ~1% on top of `TX cal + Huber`. Keep as a
cheap sanity check.

### 4. Pair inverse-variance weighting from anchor-anchor residuals

Track running variance of `(measured - expected)` per pair on the
anchor-anchor stream and weight the TDoA solver by the inverse. In
simulation this was neutral to slightly worse than Huber, because
anchor-anchor pair variance is a weak proxy for anchor-to-tag pair quality
— the anchor-to-tag multipath path is different and time-varying NLOS
dominates. Not recommended.

### 5. Per-pair multipath residual cancellation

After LS decomposition, the leftover pair residual is an estimate of the
static anchor-anchor multipath bias. Subtracting a fraction of it from tag
pair measurements requires a coupling factor (anchor-anchor multipath vs
anchor-tag multipath), which is unknown and asymmetric. In simulation, with
the true coupling known, the gain was small and depended sensitively on the
coupling. With the coupling unknown, it can hurt. Not recommended without
site calibration.

### 6. Dynamic whole-system ADelay adaptation

Confirmed bad, same conclusion as `research-notes.md`.

## Simulation Results

### Harness

Two layouts tested:

- `4_2d` — **current real hardware.** 4 anchors at z=0 corners of a
  5 m x 5 m rectangle, tag fixed_z = 1.0 m. 25-point grid x 40 repeats.
- `8_3d` — future 8-anchor 3D cuboid (5 m x 5 m x 3 m). 75-point grid
  x 40 repeats.

All metrics are position-error percentiles in cm. `rel_p95` is the random
two-tag formation-error p95 — the primary metric for light-show use.

Error model:

- per-anchor TX and RX delays (static, Gaussian),
- per-pair multipath (static, Gaussian, not separable),
- time-varying zero-mean tag noise and anchor noise,
- NLOS-like outliers on tag pair measurements,
- anchor-anchor stream with 300 directional samples per pair.

LS diagnostics from the harness show that even with only 4 anchors and 12
directional inter-anchor measurements, the per-anchor estimate is not starved
for data in the 4-anchor case. The first-pass harness recovered sub-centimeter
baseline TX error in its optimistic model; the later stack harness was more
conservative, with directional TX RMS around `1.5 cm` in the 4-anchor baseline
profile and around `3.2 cm` in hard RF after 32 samples per pair.

Run (both layouts):

```bash
python3 .agents/tdoa-precision/simulations/tdoa_novel_ideas_sim.py \
  --seed 1 --repeats 3
```

Run only the current-hardware layout:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_novel_ideas_sim.py \
  --layout 4_2d --seed 1 --repeats 3
```

## 4-anchor 2D Results (current real hardware)

### Baseline realistic

`TX/RX sigma = 3.0 cm, multipath sigma = 2.0 cm, p_outlier = 1%`.
Averaged over 3 static realizations.

| Mode | raw_p95 | off_p95 | rigid_p95 | rel_p95 |
|---|---:|---:|---:|---:|
| no correction | 6.41 | 3.60 | 3.60 | 5.26 |
| huber only | 6.43 | 3.63 | 3.63 | 5.24 |
| TX cal alone | 3.43 | 2.97 | 2.96 | 4.36 |
| **TX cal + huber** | **3.26** | **2.75** | **2.75** | **3.91** |
| TX cal + triangle reject + huber | 3.26 | 2.76 | 2.77 | 4.00 |
| TX cal + pair inv-var + huber | 3.56 | 2.94 | 2.93 | 4.27 |

### Hard RF / outliers

`TX/RX sigma = 4.0 cm, multipath sigma = 4.0 cm, p_outlier = 5%,
outlier sigma = 26 cm`. Averaged over 3 static realizations.

| Mode | raw_p95 | off_p95 | rel_p95 |
|---|---:|---:|---:|
| no correction | 11.37 | 9.16 | 12.57 |
| huber only | 9.37 | 6.13 | 8.97 |
| TX cal alone | 9.68 | 9.13 | 12.61 |
| **TX cal + huber** | **6.32** | **5.48** | **7.60** |
| TX cal + triangle reject + huber | 6.43 | 5.60 | 7.86 |

### Low-bias calibrated HW

`TX/RX sigma = 1.5 cm, multipath sigma = 1.5 cm, p_outlier = 0.5%`.

| Mode | raw_p95 | off_p95 | rel_p95 |
|---|---:|---:|---:|
| no correction | 3.77 | 2.44 | 3.58 |
| huber only | 3.74 | 2.46 | 3.50 |
| **TX cal + huber** | **2.57** | **2.20** | **3.16** |

### 4-anchor takeaways

1. In the first-pass isolated harness, TX cal + Huber was the best tested
   correction on 4 anchors, same as on 8. `rel_p95` dropped by roughly 25% in
   the baseline profile, 15% in hard-RF, and 10% in low-bias. The later stack
   harness kept the same direction but made the recommendation more cautious:
   Huber should be applied first, while online correction should start in
   `MONITOR`.
2. **TX cal alone (no Huber) is dangerous in hard-RF on 4 anchors.**
   In the hard-RF case `TX cal alone` was essentially no better than
   baseline — tag-side outliers dominate `rel_p95` and the bias
   correction alone does not help. Do not enable TX cal without also
   enabling Huber.
3. Pair inverse-variance weighting and direct pair-residual cancellation
   do not pay off on 4 anchors either, consistent with the 8-anchor
   findings.
4. Comparison to `simulation-results.md`'s 4-anchor 2D table (which used
   a slightly different noise profile): the existing `geom_locked+huber`
   recommendation reported `p95 = 6.8 cm`. This harness's baseline
   profile is close enough to cross-reference: TX cal + Huber pushes
   raw p95 to the 3.3 cm range. Direct apples-to-apples comparison
   requires a shared harness, but the directional conclusion is
   consistent.

## 8-anchor 3D Results (future layout, kept for reference)

### Baseline realistic

`TX/RX sigma = 3.0 cm, multipath sigma = 2.0 cm, p_outlier = 1%`.

| Mode | raw_p95 | off_p95 | rigid_p95 | rel_p95 |
|---|---:|---:|---:|---:|
| no correction | 6.64 | 2.87 | 2.66 | 4.09 |
| huber only | 6.59 | 2.74 | 2.52 | 3.85 |
| **TX cal alone** | 2.32 | 2.23 | 2.21 | 3.16 |
| **TX cal + huber** | **2.02** | **1.92** | **1.91** | **2.74** |
| TX cal + triangle reject + huber | 2.00 | 1.91 | 1.89 | 2.72 |
| TX cal + pair inv-var + huber | 2.29 | 2.15 | 2.12 | 3.00 |
| TX cal + residual cancel + pair inv-var + huber | 2.23 | 2.10 | 2.09 | 2.99 |

### Hard RF / outliers

`TX/RX sigma = 4.0 cm, multipath sigma = 4.0 cm, p_outlier = 5%, outlier
sigma = 26 cm`.

| Mode | raw_p95 | off_p95 | rel_p95 |
|---|---:|---:|---:|
| no correction | 10.45 | 5.99 | 8.22 |
| huber only | 9.34 | 4.15 | 5.85 |
| TX cal alone | 5.89 | 5.84 | 7.67 |
| **TX cal + huber** | **3.37** | **3.22** | **4.48** |
| TX cal + triangle reject + huber | 3.38 | 3.19 | 4.47 |

### Low-bias calibrated HW

`TX/RX sigma = 1.5 cm, multipath sigma = 1.5 cm, p_outlier = 0.5%`.

| Mode | raw_p95 | off_p95 | rel_p95 |
|---|---:|---:|---:|
| no correction | 3.76 | 1.90 | 2.70 |
| huber only | 3.71 | 1.80 | 2.54 |
| **TX cal + huber** | **1.58** | **1.52** | **2.14** |

### Summary (across both layouts)

- `TX cal + Huber` is the winner in every tested profile and every tested
  layout. It should be treated as a single two-piece change, not two
  independent options.
- The gain is not a fixed "65%". It depends on the static/noise ratio:
  - When per-anchor bias dominates (baseline, 3 cm TX/RX sigma),
    `rel_p95` drops 25-45% depending on layout.
  - When outliers dominate (hard-RF), most of the gain comes from Huber;
    TX cal adds another 10-20% on top.
  - When HW is already well-calibrated (1.5 cm sigma), the remaining
    headroom is small (10-15% `rel_p95` gain).
- Triangle rejection adds a marginal further improvement on top of Huber
  and is only worth enabling as a cheap guardrail.
- Pair inverse-variance weighting and direct pair-residual cancellation
  did not justify their additional complexity in any layout.

## Second-Pass Strategy Stack

The first follow-up isolated online TX calibration. The later stack harness put
the candidate pieces together in one model:

- current solver behavior,
- Huber robust residual weighting,
- geometry-locked inter-anchor timing,
- directional per-anchor correction,
- symmetric per-anchor fallback,
- triangle-inequality rejection.

This changed the recommendation from treating TX cal + Huber as an immediate
apply candidate to a more cautious implementation order: **Huber first, then
geometry lock/blend, then per-anchor correction in `MONITOR`, then combined
geometry+per-anchor apply mode after real logs confirm the model**.

Current 4-anchor layout, `rel_p95` in centimeters:

| Model | Current | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
|---|---:|---:|---:|---:|---:|
| baseline realistic | 4.94 | 4.72 | 4.70 | 4.16 | 4.12 |
| hard RF / outliers | 11.95 | 8.80 | 8.53 | 8.39 | 7.43 |
| low-bias HW | 3.47 | 3.35 | 3.31 | 3.22 | 3.15 |

Future 8-anchor layout, `rel_p95` in centimeters:

| Model | Current | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
|---|---:|---:|---:|---:|---:|
| baseline realistic | 3.78 | 3.57 | 3.63 | 3.00 | 2.80 |
| hard RF / outliers | 8.02 | 5.94 | 5.70 | 5.67 | 4.72 |
| low-bias HW | 2.63 | 2.43 | 2.40 | 2.32 | 2.21 |

Additional findings:

- The symmetric-only fallback was weaker when TX/RX endpoint biases were
  independent, but nearly matched directional correction when TX/RX endpoint
  biases were correlated. This matters if the real stream only exposes a
  DS-TWR-style symmetric distance.
- On today's 4-anchor hardware in nominal conditions, expected gains should be
  framed as incremental: about `10-20%` `rel_p95` improvement in the stack
  model. Larger `~40%` improvements are hard-RF/outlier or future 8-anchor
  results.
- The per-anchor estimate reached its practical floor quickly. In the model,
  8-32 samples per anchor pair were enough to be close to the 128-sample
  estimate; remaining error was dominated by static pair/path residual.
- Pair-residual regularization was not a fragile knob. Sweeping `pair_reg`
  from `1e-4` to `100` changed directional TX RMS only slightly.
- Triangle rejection was mixed. It should remain an optional guardrail, not a
  default first apply-mode change.

Practical consequence: `MONITOR` mode should log both the directional fit and
the symmetric fallback prediction. The firmware should not assume the
optimistic directional model until the real anchor-anchor logs show it.

## Relation To Existing Conclusions

- `research-notes.md` ranked `geom_locked` and static ADelay calibration at
  the top. Those are complementary to the idea proposed here, not
  replacements:
  - `geom_locked` removes per-pair inter-anchor noise/bias.
  - Per-anchor correction removes the additive endpoint-bias leakage into tag
    TDoA that `geom_locked` does not fix.
- Huber weighting is still the correct robust layer and should be the first
  apply-mode change.
- Geometry lock/blend is mechanically simpler than online correction, so its
  guarded `APPLY` mode should be tested before online correction `APPLY`, while
  online correction can still run in `MONITOR` at the same time.
- Self-survey remains useful and combines naturally with TX calibration via
  the two-pass alternation.

## Firmware Mapping

Minimal set of changes.

Ingestion (accumulate per-pair running means of measured inter-anchor ToF,
broken down by direction):

- `lib/tdoa_algorithm/src/tag/tdoa_tag_algorithm.cpp:120` in
  `updateRemoteData`, near the existing `distance_callback` call. Maintain
  a small `(N, N)` running-mean buffer plus sample counters. The data is
  already flowing through `packet->distances[i]`.

LS solve (periodic, low rate — once at boot and optionally every ~30 s):

- New small module, e.g.
  `lib/tdoa_algorithm/src/tag/tdoa_anchor_calibration.{hpp,cpp}`. Builds
  the observation matrix, applies regularization toward zero on the
  pair-residual component, solves for `bias_hat`.
- Problem size: N = number of anchors. Unknowns are 2·N − 1 per-anchor
  delays (TX and RX, minus one gauge fix) plus C(N,2) regularized
  pair-residuals. Observations are up to N·(N−1) directional inter-anchor
  means.
  - 4 anchors (current HW): 12 observations, 13 unknowns (7 delay + 6
    pair). Well-posed only because the pair-residuals are strongly
    regularized toward zero. Empirical recovery depends on the noise model:
    the later stack harness measured about `1.5 cm` directional TX RMS in the
    baseline profile and about `3.2 cm` in hard RF after 32 samples per pair.
  - 8 anchors (future): 56 observations, 43 unknowns. Comfortably
    over-determined.
- Cost is negligible compared to the per-frame solver. The existing Eigen
  dependency covers it.

Application (additive correction to the tag distance-difference):

- `lib/tdoa_algorithm/src/tag/tdoaEngine.cpp:151` in `calcTDoA`, subtract
  `(bias_hat[other] - bias_hat[this])` (converted into anchor-clock time
  units), or equivalently adjust `tof_Ar_to_An_in_cl_An` loaded at line
  144 so the correction is applied at the `delta_txAr_to_txAn_in_cl_An`
  step.
- The current firmware is a 4-anchor real setup. The correction is
  applied per-anchor-pair and works unchanged when the layout grows
  to 8 anchors later.

Feature-flag layout, following `user_defines.txt` and `features.hpp`
conventions:

- `USE_TDOA_ONLINE_ANCHOR_TX_CAL`
  - sub-modes: `OFF`, `MONITOR`, `APPLY`.
- `USE_TDOA_ROBUST_SOLVER_WEIGHTS` (already planned).
- `USE_TDOA_TRIANGLE_PAIR_REJECT` (optional guardrail).

`MONITOR` mode is important as the first deployable step: it logs the LS
residuals and the proposed `bias_hat[B] - bias_hat[A]` correction terms
without touching the flight path, so the actual physical decomposition quality
on a given site can be verified before enabling `APPLY`. It should also log a
symmetric fallback estimate, because the real DS-TWR timing path may not expose
fully directional TX/RX observability.

## Caveats

1. The simulation model assumes bias decomposes cleanly into `TX_i + RX_j`.
   Real UWB bias includes distance-dependent components, temperature drift,
   and orientation-dependent antenna patterns. The LS residual will absorb
   those. A `MONITOR` mode that logs `measured - D - TX_hat_i - RX_hat_j`
   is the cheapest way to verify the decomposition holds on-site.
2. The LS needs a reference for `D_ij`. Manual survey works. Self-survey
   works if alternated with TX/RX estimation. Without either, endpoint bias and
   `D_ij` are jointly unobservable because of a global scale gauge. The
   tag-side correction only needs a pair difference
   `bias_hat[B] - bias_hat[A]`, so some useful information may still survive
   that gauge, but this needs explicit testing before relying on zero-setup
   operation.
3. Temperature drift can move antenna delays by several centimeters over
   20 degrees Celsius (see DW1000 characterization). A slow running update
   (minute timescale) is enough; the correction must not be adapted per
   flight frame.
4. Only formation consistency has been tested in simulation. Absolute
   error after the correction still inherits the constant anchor-survey
   offset, which is explicitly acceptable for the light-show use case.

## Recommended Next Steps

Target the current 4-anchor hardware first; 8-anchor validation comes later.

1. Enable Huber weighting first (`USE_TDOA_ROBUST_SOLVER_WEIGHTS`). It is a
   per-solve robust loss, not a temporal filter, so it should not add delay.
2. Implement static-geometry inter-anchor correction in `MONITOR`, then test
   guarded `BLEND` / `LOCKED` modes if the static grid supports it.
3. Implement `USE_TDOA_ONLINE_ANCHOR_TX_CAL` in `MONITOR` mode and log
   `bias_hat`, per-anchor LS residuals, and the proposed per-pair
   `bias_hat[B] - bias_hat[A]` correction alongside the diagnostic
   stream described in `static-tag-experiment-plan.md`.
4. Use a deliberately slow correction estimator: initial EWMA/reference time
   constant `1-10 s`, default `5 s`; do not use sub-second values for flight
   correction. Prefer frozen correction for first `APPLY`.
5. Check on real hardware whether the two directions of each inter-anchor
   pair give distinguishable means. If they do, the directional TX/RX
   decomposition is identifiable and the sim numbers apply. If they
   don't, fall back to a per-anchor total-delay correction — still
   useful, slightly weaker.
6. Verify that the LS residual is close to the noise floor on real
   hardware. If it is, `APPLY` should match the simulation gains. If it
   is not, the per-pair multipath term is dominant and the gain will be
   partial but still non-negative.
7. Do not enable online correction `APPLY` without Huber — the 4-anchor
   hard-RF numbers show that bias correction alone does not help when tag-side
   outliers dominate.
8. Re-run the static-tag grid experiment from
   `static-tag-experiment-plan.md` with and without `APPLY`, compute
   `off_p95`, `rigid_p95`, `rel_p95` per the consistency guidance in
   `research-notes.md`.
9. Add desktop manager support on a matching `rtls-link-manager` branch:
   OFF/MONITOR/APPLY controls, correction status, reject reasons, fit RMS,
   sample counts, pair residuals, and log export.
10. Once the 8-anchor layout is available, re-validate: larger gains are
   expected, but the firmware code path is the same.

## Files

- `novel-ideas-followup.md` (this document).
- `simulations/tdoa_novel_ideas_sim.py` — simulation harness used to
  produce the tables above.
- `strategy-stack-results.md` — second-pass stack comparison and current
  implementation-order recommendation.
- `simulations/tdoa_strategy_stack_sim.py` — companion harness for the stack
  comparison.
