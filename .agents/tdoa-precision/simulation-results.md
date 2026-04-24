# Simulation Results

These results are from the investigation simulations. They should be treated as
directional, not as hardware measurements. The purpose was to compare strategies
that are plausible on the current DW1000/ESP32 hardware.

The tables below preserve the key values observed during the interactive
investigation. The script in `simulations/tdoa_precision_sim.py` is a
consolidated reproducible harness for the same experiment families; because it
uses a compact stochastic model, its exact default numbers can differ from
these historical tables while preserving the same strategy ordering.

## Metrics

- `mean`, `median`, `p95`, `p99`: position error magnitudes.
- `off_p95`: p95 after removing one constant global offset.
- `rigid_p95`: p95 after best translation and rotation.
- `sim_p95`: p95 after translation, rotation, and scale.
- `rel_p95`: p95 random two-tag relative-position error.

## Latest Strategy-Stack Summary

The newest decision harness is
`simulations/tdoa_strategy_stack_sim.py`, with full notes in
`strategy-stack-results.md`. It is a second-pass model that compares Huber,
geometry lock, online per-anchor correction, symmetric-only fallback, and
triangle rejection in the same run.

Current 4-anchor 2D layout, `rel_p95` in centimeters:

| Model | Current | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
| --- | ---: | ---: | ---: | ---: | ---: |
| baseline realistic | 4.94 | 4.72 | 4.70 | 4.16 | 4.12 |
| hard RF / outliers | 11.95 | 8.80 | 8.53 | 8.39 | 7.43 |
| low-bias HW | 3.47 | 3.35 | 3.31 | 3.22 | 3.15 |

Future 8-anchor 3D layout, `rel_p95` in centimeters:

| Model | Current | Huber | Geom+Huber | TX+Huber | Geom+TX+Huber |
| --- | ---: | ---: | ---: | ---: | ---: |
| baseline realistic | 3.78 | 3.57 | 3.63 | 3.00 | 2.80 |
| hard RF / outliers | 8.02 | 5.94 | 5.70 | 5.67 | 4.72 |
| low-bias HW | 2.63 | 2.43 | 2.40 | 2.32 | 2.21 |

Takeaway: Huber is the safest first apply change. Online per-anchor correction
is the most promising new monitorable correction. Geometry lock and per-anchor
correction are complementary, especially in hard RF and in the future
8-anchor layout.

Expectation note: on the current 4-anchor rig in nominal conditions, the latest
stack model predicts incremental gains, not dramatic ones. The baseline
realistic profile improved `rel_p95` by 17%, and the low-bias profile improved
by 9%. The large `38-41%` gains appear in hard-RF/outlier-heavy conditions and
in the future 8-anchor layout.

Symmetric-only fallback:

| Layout | Current | Geom+TX+Huber | Geom+Sym+Huber |
| --- | ---: | ---: | ---: |
| 4-anchor, TX/RX correlated | 4.94 | 4.12 | 4.12 |
| 8-anchor, TX/RX correlated | 3.68 | 2.79 | 2.82 |

Takeaway: if the real firmware only exposes a symmetric DS-TWR-style
anchor-anchor distance, the fallback may still be useful. It must be evaluated
in `MONITOR` mode because the result depends on real endpoint-bias structure.

## Baseline 2D, 5 m x 5 m, 4 Anchors

| Mode | Mean | Median | P95 |
| --- | ---: | ---: | ---: |
| current_raw | 0.059 m | 0.049 m | 0.133 m |
| static_adelay | 0.053 m | 0.044 m | 0.131 m |
| geom_locked | 0.035 m | 0.029 m | 0.074 m |
| blend50 | 0.041 m | 0.034 m | 0.090 m |
| raw+huber | 0.056 m | n/a | 0.116 m |
| geom_locked+huber | 0.033 m | n/a | 0.068 m |

Takeaway: using static geometry for inter-anchor ToF was materially more useful
than static antenna-delay calibration alone.

## Hard RF / More Outliers

| Mode | Mean | P95 |
| --- | ---: | ---: |
| current_raw | 0.117 m | 0.301 m |
| static_adelay | 0.107 m | 0.300 m |
| geom_locked | 0.051 m | 0.135 m |
| blend50 | 0.071 m | 0.193 m |
| geom_locked+huber | 0.045 m | 0.097 m |

Takeaway: robust weighting helped most when heavy tails were present.

## Minimum Live Diversity, 4 Of 6 Pairs

| Mode | Mean | P95 |
| --- | ---: | ---: |
| current_raw | 0.090 m | 0.200 m |
| static_adelay | 0.065 m | 0.159 m |
| geom_locked | 0.042 m | 0.087 m |
| blend50 | 0.050 m | 0.113 m |

Takeaway: losing pair diversity hurts, but static geometry still helps.

## Survey Error, One Anchor Moved 5 cm

| Mode | Mean | P95 |
| --- | ---: | ---: |
| static_adelay | 0.061 m | 0.138 m |
| geom_locked | 0.059 m | 0.106 m |
| blend50 | 0.054 m | 0.110 m |
| geom_locked+huber | 0.059 m | 0.106 m |

Takeaway: static geometry lock still helped tails, but survey accuracy becomes
important. This motivated the self-survey and consistency analysis.

## Larger 10 m x 7 m Area

| Mode | Mean | P95 |
| --- | ---: | ---: |
| current_raw | 0.054 m | 0.126 m |
| geom_locked | 0.036 m | 0.075 m |
| geom_locked+huber | 0.034 m | 0.069 m |

## Tick Quantization

| Case | Mean | P95 | P99 |
| --- | ---: | ---: | ---: |
| integer tick ToF rounding | 0.65 mm | 1.23 mm | 1.50 mm |
| 0.1 tick residual | 0.065 mm | 0.123 mm | n/a |

Takeaway: antenna-delay sub-tick resolution is unlikely to be the dominant
centimeter-level error source.

## Additional Anchors, 2D

| Geometry | Mean | P95 | P99 |
| --- | ---: | ---: | ---: |
| 4 corners | 0.031 m | 0.062 m | 0.098 m |
| 5 including center | 0.025 m | 0.050 m | 0.065 m |
| 6 corners + mid top/bottom | 0.021 m | 0.041 m | 0.052 m |
| 6 corners + mid sides | 0.020 m | 0.040 m | 0.052 m |

Takeaway: anchor count and geometry improve both average error and tails.

## Low-Latency EMA Smoothing

Simulation assumptions:

- 50 Hz update rate,
- 5 cm measurement sigma,
- 1 percent outliers.

| EMA Alpha | Typical Mean Error | Typical P95 | Approx Delay |
| --- | ---: | ---: | ---: |
| 0.60 | 4.5 cm | 8.4-8.9 cm | 13 ms |
| 0.40 | 3.6-4.0 cm | 6.8-7.6 cm | 30 ms |
| 0.25 | lower noise, more lag | motion-dependent | 60 ms |

Takeaway: smoothing is useful, but should be tuned against motion. A too-low
alpha can hurt moving drones even if it looks good on static data.

## Dynamic ADelay Adaptation

Dynamic adaptation showed no clear residual improvement over static
calibration. With harsh outliers, aggressive adaptation created large jumps:

- residual p95 around 0.413 m,
- estimated-delay jump p95 around 0.242 m in the simplified model.

Takeaway: do not use dynamic ADelay as the primary runtime correction strategy.

## 2D Consistency, 5 m x 5 m

Values are centimeters.

| Configuration | raw_p95 | off_p95 | rel_p95 |
| --- | ---: | ---: | ---: |
| manual exact current_raw | 13.76 | 12.19 | 18.14 |
| manual exact static_adelay | 12.27 | 12.33 | 18.15 |
| manual exact geom_locked | 6.89 | 6.91 | 11.85 |
| manual exact geom_locked+huber | 6.31 | 6.33 | 9.29 |
| manual 5 cm error geom_locked+huber | 8.49 | 7.55 | 10.89 |
| self-survey rectangle + geom_locked+huber | 6.39 | 6.25 | 9.25 |
| self-survey MDS + geom_locked+huber | 6.84 | 6.45 | 9.62 |

Takeaway: if self-survey distances are antenna-delay-corrected, a simple static
setup can approach manually surveyed consistency in the simulated conditions.

## 3D Consistency, 8 Anchors, 5 m x 5 m x 4 m

Values are centimeters.

| Configuration | raw_p95 | off_p95 | rel_p95 |
| --- | ---: | ---: | ---: |
| manual exact current_raw | 10.66 | 9.27 | 12.63 |
| manual exact static_adelay | 9.34 | 9.15 | 12.51 |
| manual exact geom_locked | 6.29 | 6.29 | 9.25 |
| manual exact geom_locked+huber | 4.03 | 4.03 | 5.79 |
| manual 5 cm error geom_locked+huber | 9.36 | 6.45 | 9.10 |
| self-survey cuboid + geom_locked+huber | 4.37 | 4.24 | 5.94 |
| self-survey MDS + geom_locked+huber | 4.92 | 4.42 | 6.12 |

Sparse 12 live-pair case:

| Mode | raw_p95 | rel_p95 |
| --- | ---: | ---: |
| static_adelay | 16.41 | 22.10 |
| geom_locked | 9.28 | 16.01 |
| blend50 | 11.79 | n/a |
| geom_locked+huber | 7.23 | 10.24 |

Takeaway: the 8-anchor 3D layout is promising, but it needs enough live pair
diversity. Sparse pair availability can erase a large part of the benefit.

## 3D Anchor Height Separation Sensitivity

Scenario added after the main investigation:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py \
  --scenario height-separation-3d --seed 42
```

Geometry:

- 8 anchors on a 5 m x 5 m footprint.
- Four lower anchors at floor level.
- Four upper anchors at height `H`.
- Compared `H = 2, 3, 4, 5 m`.
- `DOP z1_p95` is the p95 predicted 1-sigma Z error across the sampled volume,
  assuming 3 cm independent TDoA distance-difference noise.
- `MC z_p95` is the simulated p95 absolute Z error using `geom_locked+huber`.

All 28 anchor pairs available:

| Flight Volume | H | DOP z1_p95 | MC z_p95 | MC z_p99 | Condition p95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| fixed 0.4-1.6 m | 2 m | 1.38 cm | 2.12 cm | 2.84 cm | 6.5 |
| fixed 0.4-1.6 m | 3 m | 1.02 cm | 1.53 cm | 2.11 cm | 3.2 |
| fixed 0.4-1.6 m | 4 m | 0.88 cm | 1.32 cm | 1.74 cm | 2.2 |
| fixed 0.4-1.6 m | 5 m | 0.81 cm | 1.27 cm | 1.58 cm | 1.9 |
| high 1.6-2.8 m | 2 m | 1.90 cm | 2.61 cm | 3.53 cm | 11.2 |
| high 1.6-2.8 m | 3 m | 1.07 cm | 1.58 cm | 2.19 cm | 3.5 |
| high 1.6-2.8 m | 4 m | 0.77 cm | 1.20 cm | 1.65 cm | 1.8 |
| high 1.6-2.8 m | 5 m | 0.66 cm | 1.04 cm | 1.38 cm | 1.9 |

Sparse 12-pair case:

| Flight Volume | H | DOP z1_p95 | MC z_p95 | MC z_p99 | Condition p95 |
| --- | ---: | ---: | ---: | ---: | ---: |
| fixed 0.4-1.6 m | 2 m | 2.74 cm | 4.06 cm | 5.80 cm | 6.5 |
| fixed 0.4-1.6 m | 3 m | 2.00 cm | 2.98 cm | 4.06 cm | 3.3 |
| fixed 0.4-1.6 m | 4 m | 1.74 cm | 2.65 cm | 3.42 cm | 2.3 |
| fixed 0.4-1.6 m | 5 m | 1.61 cm | 2.47 cm | 3.19 cm | 2.1 |
| high 1.6-2.8 m | 2 m | 3.71 cm | 5.34 cm | 7.16 cm | 11.7 |
| high 1.6-2.8 m | 3 m | 2.10 cm | 3.00 cm | 3.73 cm | 3.6 |
| high 1.6-2.8 m | 4 m | 1.53 cm | 2.44 cm | 3.10 cm | 1.9 |
| high 1.6-2.8 m | 5 m | 1.31 cm | 2.07 cm | 2.69 cm | 2.1 |

Takeaway: a 2 m vertical anchor separation is usable, but it is noticeably
worse for Z than 3-5 m. With all 28 pairs, 2 m height produced roughly 1.6-2.5x
the vertical sensitivity of 4-5 m, depending on flight volume. With sparse
12-pair diversity, the 2 m layout pushed simulated Z p95 to about 4-5.3 cm in
these compact-noise simulations. If real measurement tails are larger, this
scales up. The risk is highest when drones fly near or above the 2 m top-anchor
plane.

### Fine 2-4 m Sweep

Scenario:

```bash
python3 .agents/tdoa-precision/simulations/tdoa_precision_sim.py \
  --scenario height-separation-fine-3d --seed 42
```

Heights were swept from 2.00 m to 4.00 m in 0.25 m increments. The table below
keeps the key Z p95 values from the run.

All 28 anchor pairs:

| H | z=0.4-1.6 m Z p95 | z=1.6-2.8 m Z p95 | High-volume condition p95 |
| ---: | ---: | ---: | ---: |
| 2.00 m | 2.08 cm | 2.91 cm | 11.0 |
| 2.25 m | 1.91 cm | 2.23 cm | 7.8 |
| 2.50 m | 1.80 cm | 1.97 cm | 5.8 |
| 2.75 m | 1.67 cm | 1.68 cm | 4.5 |
| 3.00 m | 1.59 cm | 1.57 cm | 3.5 |
| 3.25 m | 1.48 cm | 1.49 cm | 2.8 |
| 3.50 m | 1.40 cm | 1.39 cm | 2.3 |
| 3.75 m | 1.32 cm | 1.25 cm | 2.0 |
| 4.00 m | 1.31 cm | 1.19 cm | 1.8 |

Sparse 12-pair case:

| H | z=0.4-1.6 m Z p95 | z=1.6-2.8 m Z p95 | High-volume condition p95 |
| ---: | ---: | ---: | ---: |
| 2.00 m | 4.16 cm | 5.15 cm | 11.6 |
| 2.25 m | 3.66 cm | 4.18 cm | 8.2 |
| 2.50 m | 3.43 cm | 3.83 cm | 6.1 |
| 2.75 m | 3.41 cm | 3.44 cm | 4.6 |
| 3.00 m | 3.14 cm | 2.84 cm | 3.6 |
| 3.25 m | 2.85 cm | 2.88 cm | 2.9 |
| 3.50 m | 2.75 cm | 2.68 cm | 2.4 |
| 3.75 m | 2.67 cm | 2.59 cm | 2.0 |
| 4.00 m | 2.56 cm | 2.41 cm | 1.9 |

Takeaway: the biggest practical gain appears between 2.00 m and about
2.75-3.00 m. Past about 3.25 m, Z continues improving but with diminishing
returns in this model. For a high flight volume and sparse pair diversity,
moving top anchors from 2.00 m to 3.00 m reduced simulated Z p95 from 5.15 cm
to 2.84 cm, which is a large enough improvement to matter operationally.

## Self-Survey Dimension Error

3D cuboid self-survey from averaged anchor distances after antenna-delay
correction:

| Samples Per Pair | P95 Dimension Error |
| ---: | ---: |
| 10 | 3.5-3.8 cm |
| 20 | 3.0-3.3 cm |
| 50 | 2.8-3.0 cm |
| 250 | about 2.8 cm |

Using raw uncorrected distances produced p95 dimension error around 10-11 cm.

Takeaway: average more than a few samples, but do not expect averaging alone to
remove static residual bias.

## Deterministic Geometry Error

2D geometry lock, no random noise:

| Error | raw_p95 | off_p95 |
| --- | ---: | ---: |
| global translation +10 cm / -5 cm | 11.18 cm | 0.00 cm |
| uniform scale +2 percent | 8.52 cm | 3.12 cm |
| anisotropic W +10 cm, H -10 cm | 10.44 cm | 6.97 cm |
| one anchor misplaced 10 cm | 5.94 cm | 3.58 cm |

3D:

| Error | raw_p95 | off_p95 |
| --- | ---: | ---: |
| top height +20 cm | 7.65 cm | 5.56 cm |
| uniform XYZ scale +2 percent | 10.89 cm | 2.40 cm |
| one top anchor misplaced 10 cm | 2.60 cm | 2.15 cm |

Takeaway: a bad survey is not always just a constant offset. Some survey errors
create spatial distortion, which matters for formations.
