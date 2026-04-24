# 3 m Anchor Height: Above-Plane Z Sweep

Date: 2026-04-23

This note records a focused sweep for the future 8-anchor layout with:

- footprint: 5 m x 5 m,
- lower anchors: z = 0 m,
- upper anchors: z = 3 m,
- solver mode: `geom_locked + huber`,
- tested tag heights: z = 2.0 m through z = 6.0 m.

The purpose is to understand how height error propagates when tags move near
and above the top-anchor plane.

## Interpretation

There is no sharp cliff exactly at z = 3 m in the idealized model. The error
growth is gradual near the plane, then becomes much worse as the tag moves
well above all anchors.

The important operational pattern:

- z <= 2.8 m: still close to the in-volume behavior.
- z = 3.0-3.5 m: caution zone; Z p95 grows but remains manageable with all
  pairs.
- z = 4.0 m: clearly degraded; condition number and Z error are about 1.6-1.8x
  worse than the 2.4 m reference.
- z >= 5.0 m: poor geometry for this 3 m anchor layout, especially with sparse
  pairs or hard RF.

This simulation does not include antenna radiation pattern, drone-body
shadowing, ceiling/truss reflections, or top-anchor pitch. Real hardware above
the top-anchor plane is likely worse than this idealized result.

## Compact RF Results

Compact RF is the cleaner profile used for nominal comparison.

| z_m | pairs | dop_z1_cm | cond95 | z_p95_cm | z_p99_cm | xy_p95_cm | total_p95_cm | z_p95_ratio_vs_2p4 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 2.0 | all28 | 0.98 | 2.9 | 1.53 | 2.01 | 1.32 | 1.82 | 0.92 |
| 2.4 | all28 | 1.02 | 3.2 | 1.66 | 2.28 | 1.35 | 1.89 | 1.00 |
| 2.8 | all28 | 1.10 | 3.9 | 1.79 | 2.33 | 1.36 | 2.01 | 1.08 |
| 3.0 | all28 | 1.16 | 4.4 | 1.93 | 2.48 | 1.35 | 2.15 | 1.16 |
| 3.2 | all28 | 1.25 | 5.0 | 2.11 | 2.65 | 1.36 | 2.27 | 1.27 |
| 3.5 | all28 | 1.49 | 6.5 | 2.33 | 3.02 | 1.41 | 2.53 | 1.40 |
| 4.0 | all28 | 1.95 | 10.2 | 2.75 | 3.65 | 1.61 | 3.01 | 1.65 |
| 4.5 | all28 | 2.46 | 14.3 | 3.54 | 4.67 | 1.75 | 3.76 | 2.13 |
| 5.0 | all28 | 3.04 | 18.8 | 4.28 | 5.70 | 2.09 | 4.66 | 2.58 |
| 5.5 | all28 | 3.70 | 24.0 | 5.40 | 6.95 | 2.32 | 5.75 | 3.25 |
| 6.0 | all28 | 4.47 | 30.1 | 6.48 | 8.86 | 2.59 | 6.80 | 3.90 |

Sparse 12-pair compact result:

| z_m | dop_z1_cm | cond95 | z_p95_cm | z_p99_cm | xy_p95_cm | total_p95_cm | z_p95_ratio_vs_2p4 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.4 | 2.04 | 3.3 | 3.15 | 4.36 | 2.53 | 3.66 | 1.00 |
| 3.0 | 2.31 | 4.7 | 3.66 | 4.92 | 2.61 | 4.08 | 1.16 |
| 3.5 | 2.91 | 7.2 | 4.65 | 5.99 | 2.81 | 5.09 | 1.48 |
| 4.0 | 3.85 | 11.6 | 5.61 | 7.55 | 3.00 | 6.04 | 1.78 |
| 5.0 | 5.90 | 20.0 | 8.93 | 11.81 | 4.05 | 9.61 | 2.83 |
| 6.0 | 8.54 | 29.8 | 12.87 | 17.73 | 5.15 | 13.47 | 4.08 |

## Hard RF Results

Hard RF uses larger noise, larger pair bias, and more outliers.

| z_m | pairs | dop_z1_cm | cond95 | z_p95_cm | z_p99_cm | xy_p95_cm | total_p95_cm | z_p95_ratio_vs_2p4 |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 2.0 | all28 | 0.98 | 2.9 | 2.14 | 3.12 | 1.96 | 2.60 | 0.85 |
| 2.4 | all28 | 1.02 | 3.2 | 2.52 | 3.26 | 1.97 | 2.88 | 1.00 |
| 2.8 | all28 | 1.10 | 3.9 | 2.56 | 3.33 | 1.99 | 2.92 | 1.01 |
| 3.0 | all28 | 1.16 | 4.4 | 2.70 | 3.61 | 1.91 | 3.00 | 1.07 |
| 3.2 | all28 | 1.25 | 5.0 | 3.00 | 4.23 | 2.04 | 3.35 | 1.19 |
| 3.5 | all28 | 1.49 | 6.5 | 3.38 | 4.58 | 2.09 | 3.68 | 1.34 |
| 4.0 | all28 | 1.95 | 10.2 | 4.13 | 5.56 | 2.31 | 4.47 | 1.64 |
| 4.5 | all28 | 2.46 | 14.3 | 5.39 | 6.81 | 2.57 | 5.66 | 2.14 |
| 5.0 | all28 | 3.04 | 18.8 | 6.29 | 8.10 | 2.97 | 6.66 | 2.49 |
| 5.5 | all28 | 3.70 | 24.0 | 7.98 | 10.28 | 3.37 | 8.47 | 3.16 |
| 6.0 | all28 | 4.47 | 30.1 | 9.90 | 12.83 | 3.79 | 10.56 | 3.92 |

Sparse 12-pair hard-RF result:

| z_m | dop_z1_cm | cond95 | z_p95_cm | z_p99_cm | xy_p95_cm | total_p95_cm | z_p95_ratio_vs_2p4 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2.4 | 2.04 | 3.3 | 4.78 | 6.45 | 4.00 | 5.53 | 1.00 |
| 3.0 | 2.31 | 4.7 | 5.51 | 7.56 | 3.99 | 6.25 | 1.15 |
| 3.5 | 2.91 | 7.2 | 6.79 | 9.13 | 4.25 | 7.41 | 1.42 |
| 4.0 | 3.85 | 11.6 | 8.37 | 11.52 | 4.56 | 9.02 | 1.75 |
| 5.0 | 5.90 | 20.0 | 12.72 | 18.35 | 5.78 | 13.54 | 2.66 |
| 6.0 | 8.54 | 29.8 | 18.98 | 26.26 | 7.72 | 20.08 | 3.97 |

## Practical Height Bands

For this 3 m upper-anchor layout:

| Tag height | Recommendation |
|---:|---|
| <= 2.4 m | Good planning ceiling for reliable operation. |
| 2.4-2.8 m | Usable if hardware logs confirm pair availability and residuals. |
| 2.8-3.2 m | Caution zone near/above the top-anchor plane. |
| 3.2-4.0 m | Degraded, especially with sparse pairs; avoid for precision formation work. |
| > 4.0 m | Geometry becomes increasingly poor; use higher top anchors instead. |

Bottom line: going slightly above the 3 m plane is not an immediate failure in
the idealized model, but error grows steadily. By 4 m, Z p95 is roughly
1.6-1.8x worse than at 2.4 m. By 5-6 m, the layout is no longer a good 3D
measurement geometry for precision drone work.
