# Practical Geometry Search for Better Z Accuracy

Date: 2026-04-23

This note evaluates anchor layouts that could improve 3D Z accuracy for the
planned small-drone light-show volume without requiring very tall anchors.

The target setup remains simple:

- compact 5 m x 5 m show footprint,
- anchors mounted on tripod-like stations,
- lower anchor close to floor level,
- upper anchor around 3.0 m, with a 3.4 m modest-extension variant,
- no external ground-truth system assumed during normal operation.

The best practical result is a **fifth center tripod**, giving **10 anchors**:
four corner stations plus one center station, each with one low and one high
anchor.

## Simulation Harness

Script:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_geometry_search_sim.py --dop-only
python3 .agents/tdoa-precision/simulations/tdoa_geometry_search_sim.py --mc-finalists 6
```

The script evaluates:

- geometry DOP from the TDoA distance-difference Jacobian,
- all-pair availability,
- a sparse-but-plausible pair set using lower ring, upper ring, vertical pairs,
  and center-to-corner radial pairs when a center station exists,
- a Monte Carlo position solve with a `geom_locked + Huber`-like solver,
- compact RF and hard-RF noise/outlier profiles.

This is still a geometry/sensitivity model, not a full DW1000 physics model.
It does not include antenna radiation pattern, body shadowing, timestamp
freshness, clock-correction failures, or real NLOS maps. The value is in
ranking practical geometries and exposing weak cases before hardware work.

## Layouts Tested

| Layout | Anchors | Notes |
|---|---:|---|
| `4tripod_cube_0p25_3p0` | 8 | Baseline: four corner tripods, low 0.25 m, top 3.0 m. |
| `4tripod_cube_0p10_3p0` | 8 | Same, but low anchors at 0.10 m. |
| `4tripod_cube_0p25_3p4` | 8 | Same four tripods, top anchors at 3.4 m. |
| `4tripod_staggered_0p15_3p2` | 8 | Small low/top height staggering. |
| `4tripod_top_outset_0p5` | 8 | Top anchors pushed 0.5 m outward. |
| `4tripod_top_inset_0p5` | 8 | Top anchors pulled 0.5 m inward. |
| `5tripod_center_0p25_3p0` | 10 | Four corners plus a center tripod. |
| `5tripod_center_0p10_3p0` | 10 | Center tripod plus lower anchors at 0.10 m. |
| `5tripod_center_0p25_3p4` | 10 | Center tripod plus top anchors at 3.4 m. |
| `5tripod_center_staggered` | 10 | Center tripod plus modest height staggering. |
| `5tripod_mid_side_plus_center` | 10 | Side-mid stations plus center, no corners. |
| `6tripod_corners_plus_mid_x` | 12 | Corners plus two mid-side stations. |

## DOP Ranking

Lower score is better. The score prioritizes Z p95 in the show volume and near
the top plane, includes above-plane performance, includes sparse pair
availability, and adds a small setup-complexity penalty.

| Rank | Layout | Stations | Anchors | Score |
|---:|---|---:|---:|---:|
| 1 | `5tripod_center_0p25_3p4` | 5 | 10 | 1.53 cm |
| 2 | `5tripod_center_staggered` | 5 | 10 | 1.65 cm |
| 3 | `5tripod_center_0p10_3p0` | 5 | 10 | 1.69 cm |
| 4 | `5tripod_center_0p25_3p0` | 5 | 10 | 1.74 cm |
| 5 | `4tripod_cube_0p25_3p4` | 4 | 8 | 1.76 cm |
| 6 | `6tripod_corners_plus_mid_x` | 6 | 12 | 1.85 cm |
| 7 | `5tripod_mid_side_plus_center` | 5 | 10 | 1.89 cm |
| 8 | `4tripod_staggered_0p15_3p2` | 4 | 8 | 1.93 cm |
| 9 | `4tripod_cube_0p10_3p0` | 4 | 8 | 1.98 cm |
| 10 | `4tripod_top_outset_0p5` | 4 | 8 | 2.05 cm |
| 11 | `4tripod_cube_0p25_3p0` | 4 | 8 | 2.06 cm |
| 12 | `4tripod_top_inset_0p5` | 4 | 8 | 2.31 cm |

## Important DOP Comparisons

### All-pair geometry

This is the optimistic case where the solver has broad pair diversity.

| Layout | Show Z p95 | Near-top Z p95 | Above-top Z p95 |
|---|---:|---:|---:|
| Baseline 4 tripods, 0.25/3.0 m | 1.13 cm | 1.29 cm | 2.37 cm |
| 4 tripods, top 3.4 m | 1.05 cm | 1.07 cm | 1.84 cm |
| 5 tripods center, 0.25/3.0 m | 0.81 cm | 0.99 cm | 1.70 cm |
| 5 tripods center, 0.10/3.0 m | 0.77 cm | 0.97 cm | 1.68 cm |
| 5 tripods center, 0.25/3.4 m | 0.77 cm | 0.80 cm | 1.40 cm |

Relative to the baseline, the 5-center-tripod 3.0 m layout improves predicted
Z p95 by roughly:

- 28% in the main show volume,
- 23% near the top plane,
- 28% above the top plane.

The 5-center-tripod 3.4 m layout improves predicted Z p95 by roughly:

- 32% in the main show volume,
- 38% near the top plane,
- 41% above the top plane.

### Sparse-pair geometry

This is the more conservative case. It matters because the real firmware may
not always have all pairs equally fresh, equally trusted, or equally usable.

| Layout | Show Z p95 | Near-top Z p95 | Above-top Z p95 |
|---|---:|---:|---:|
| Baseline 4 tripods, 0.25/3.0 m | 2.23 cm | 2.55 cm | 4.70 cm |
| 4 tripods, top 3.4 m | 2.07 cm | 2.10 cm | 3.66 cm |
| 5 tripods center, 0.25/3.0 m | 1.70 cm | 2.06 cm | 3.21 cm |
| 5 tripods center, 0.10/3.0 m | 1.61 cm | 2.03 cm | 3.18 cm |
| 5 tripods center, 0.25/3.4 m | 1.62 cm | 1.67 cm | 2.80 cm |

The fifth center tripod still helps materially when pair availability is
sparse, but the exact benefit depends on whether the firmware keeps the radial
center-to-corner pairs fresh and usable.

## Monte Carlo Results

The Monte Carlo solve includes static pair bias, random tag-side noise,
occasional outliers, and Huber weighting.

Compact RF, all-pair case:

| Layout | Show Z p95 | Near-top Z p95 | Above-top Z p95 |
|---|---:|---:|---:|
| Baseline 4 tripods, 0.25/3.0 m | 1.74 cm | 2.15 cm | 2.86 cm |
| 5 tripods center, 0.25/3.0 m | 1.22 cm | 1.41 cm | 2.30 cm |
| 5 tripods center, 0.10/3.0 m | 1.22 cm | 1.96 cm | 2.27 cm |
| 5 tripods center, 0.25/3.4 m | 1.18 cm | 1.23 cm | 1.97 cm |

Hard RF, all-pair case:

| Layout | Show Z p95 | Near-top Z p95 | Above-top Z p95 |
|---|---:|---:|---:|
| Baseline 4 tripods, 0.25/3.0 m | 3.82 cm | 3.81 cm | 4.69 cm |
| 5 tripods center, 0.25/3.0 m | 1.81 cm | 2.53 cm | 4.03 cm |
| 5 tripods center, 0.25/3.4 m | 1.94 cm | 2.14 cm | 3.12 cm |

This is a strong result: with enough live pair diversity, the center tripod
does not merely improve ideal DOP; it also reduces the error tails in the
biased/outlier model.

However, the sparse-pair Monte Carlo is more mixed. In the compact sparse case,
the 5-center-tripod 3.0 m layout improves above-top Z p95 from 6.32 cm to
4.28 cm, but the 5-center-tripod 3.4 m layout has a worse show-volume Z p95
than the baseline in one run. That is not a geometry contradiction. It means
that with a small biased pair set, systematic pair errors can dominate the
geometric advantage.

Implementation implication: **the 10-anchor geometry only pays off reliably if
the firmware and diagnostics preserve enough fresh pair diversity**. The center
station should not be added while still treating pair freshness and pair quality
as an afterthought.

## Extra Search Results

A small broader search was run after the fixed candidate test:

- Keeping the four corners fixed and moving the fifth station over a grid,
  the optimum was at the center, `(2.5, 2.5)`.
- Nearby placements around the center were nearly identical, so the setup does
  not require millimeter-level center placement.
- Random five-station layouts inside or slightly outside the footprint did not
  beat the simple four-corners-plus-center layout once corner coverage and
  setup simplicity were considered.
- A single extra center anchor is less effective than a full center vertical
  pair, but if a 9-anchor compromise is ever needed, the center-low anchor was
  more useful than center-top in the tested geometry.

Reference all-pair score from the extra search:

| Variant | Score | Z p95 | Total p95 |
|---|---:|---:|---:|
| 4 corners, 0.25/3.0 m | 1.96 cm | 1.88 cm | 2.23 cm |
| 4 corners, 0.25/3.4 m | 1.55 cm | 1.46 cm | 1.81 cm |
| 4 corners + center, 0.25/3.0 m | 1.48 cm | 1.40 cm | 1.71 cm |
| 4 corners + center, 0.25/3.4 m | 1.21 cm | 1.12 cm | 1.45 cm |

These values use a smaller grid than the main DOP table, so they should be
read as confirmation of ranking, not as the canonical numbers.

## Layout Guidance

### Best practical geometry

Use **five tripods / ten anchors**:

- four corner tripods,
- one center tripod,
- one low anchor and one high anchor on every tripod,
- low anchor as low as safely practical,
- high anchor around 3.0 m, or 3.4 m if the mechanical setup remains easy and
  safe.

Why this works:

- the center station adds radial distance-difference directions,
- the center vertical pair improves Z observability without making the whole
  rig tall,
- it improves inside-volume, near-top, and above-top geometry,
- it keeps the setup mental model simple: four corners plus center.

### Best minimal change

If adding a fifth tripod is not acceptable yet, the next best simple option is:

- keep the four corner tripods,
- raise the top anchors modestly from 3.0 m to about 3.4 m.

This mainly helps near and above the upper plane. It does less for floor/takeoff
than adding the center station.

### Low-anchor placement

Lowering the bottom anchors from 0.25 m to 0.10 m helps floor and show-volume
Z geometry a little, but it is not enough by itself to solve above-plane
weakness. Treat it as a cheap improvement, not the main strategy.

### Avoid top inset

Pulling top anchors inward is bad for above-plane Z. It looks attractive
mechanically, but it narrows the useful aperture and worsens the Z tail above
the top plane.

### Be careful with top outset

Pushing top anchors outward helps above-plane DOP, but it hurts floor/show
geometry in the sparse-pair model and adds mechanical complexity. It is not
currently better than the center tripod.

### More than five tripods

Six tripods / twelve anchors can improve the all-pair ideal geometry, but it
does not clearly beat the five-tripod center layout after setup complexity and
sparse-pair behavior are considered. It is probably not the next best product
step.

## What This Means for Drones Near the Floor

The drone starts near `z = 0`, while the low anchors are slightly above the
floor. That puts takeoff near the lower edge of the UWB volume.

Geometry changes can help, but the safest control/integration approach remains:

- use UWB strongly for XY,
- use barometer/rangefinder/ArduPilot vertical estimation during takeoff and
  landing,
- treat UWB Z near the floor as useful but not the only vertical authority,
- log the UWB residuals and Z uncertainty separately near takeoff.

The center tripod helps floor Z DOP, but it does not remove the fundamental
edge-of-volume issue.

## AoA Note

AoA can help most where TDoA Z geometry is weak: near the floor and outside the
upper plane. But if the AoA unit only provides around 2-3 degrees angular
accuracy, it should be treated as a weak additional constraint, not as a
centimeter-grade Z source.

Approximate lateral error from angle alone is:

```text
error ~= range * tan(angle_error)
```

At 5 m range:

- 2 degrees is about 17 cm,
- 3 degrees is about 26 cm.

That is too coarse to replace TDoA, but it can still improve reliability as a
gating/prior term:

- reject mirror or wrong-height TDoA solutions,
- downweight impossible vertical jumps,
- improve observability above the top plane,
- detect when a tag is outside the reliable UWB volume.

Recommended use: fuse AoA as a low-weight bearing residual or diagnostic
gate, not as a hard correction.

## Firmware/Diagnostics Implications

Before committing to the 10-anchor geometry, the implementation should expose:

- `used_pair_count`,
- `fresh_pair_count`,
- per-pair age,
- per-pair residual,
- per-pair Huber weight,
- anchor-pair ID coverage,
- solver condition or approximate covariance,
- Z residual and Z jump metrics.

The simulations show that geometry alone is not enough. Pair diversity and pair
freshness decide whether the added anchors turn into real Z accuracy.

## Recommendation

For the next physical 3D rig, test this order:

1. Baseline 4 corner tripods, 8 anchors, 0.25/3.0 m.
2. Same layout with lower anchors as low as practical.
3. Add the center tripod, 10 anchors, 0.25/3.0 m.
4. If mechanically acceptable, test 10 anchors with top anchors around 3.4 m.

Do not spend engineering time first on top-inset, side-mid-only, or 12-anchor
layouts. The center tripod gives the best simplicity-to-Z-accuracy tradeoff in
the current simulations.

