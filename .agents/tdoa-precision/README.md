# TDoA Precision Research Bundle

This folder captures the TDoA precision investigation performed for the
RTLS-link firmware and its ArduPilot/MAVLink use case.

The focus is small-drone light-show positioning, where relative consistency
between tags/drones can matter more than absolute agreement with the surveyed
anchor coordinate frame.

## Contents

- `research-notes.md`  
  Consolidated findings from code inspection, upstream protocol behavior,
  DW1000/UWB error research, antenna-delay discussion, and recommended
  implementation direction.

- `simulation-results.md`  
  Recorded simulation outcomes from the investigation, ordered by expected
  simplicity-to-gain ratio.

- `static-tag-experiment-plan.md`  
  Proposed real-world logging experiment for characterizing error
  distributions across the surveyed volume.

- `simulations/tdoa_precision_sim.py`  
  Reproducible Python simulation script covering baseline 2D, 3D, consistency,
  self-survey, tick quantization, low-latency smoothing, and dynamic
  antenna-delay adaptation models.

- `simulations/README.md`  
  How to run the scripts and interpret the output.

- `benchmarks/`  
  Native C++/Eigen benchmark harness and recorded results for the current
  TDoA Newton-Raphson solver. Start with
  `benchmarks/benchmark-results.md` for the ESP32S3 feasibility summary.

- `ardupilot-covariance-investigation.md`  
  ArduPilot upstream trace for `VISION_POSITION_ESTIMATE` covariance handling,
  including why the current RTLS-link covariance packing can trigger ArduPilot
  internal errors and why the current 2D `var_z = 100` mapping is a bad fit for
  ArduPilot's scalar external-navigation noise path.

- `novel-ideas-followup.md`  
  Follow-up feedback pass on `research-notes.md` and `simulation-results.md`.
  Proposes and evaluates an online per-anchor TX-delay self-calibration derived
  from the round-robin anchor-anchor stream, plus symmetric-direction averaging
  and triangle-inequality pair rejection. Includes tables, firmware mapping,
  and caveats.

- `strategy-stack-results.md`  
  Second-pass experiment note comparing the candidate corrections as a stack:
  Huber, geometry lock, online per-anchor correction, symmetric-only fallback,
  and triangle rejection. Includes current 4-anchor and future 8-anchor
  relative-consistency tables plus implementation-order guidance.

- `height-ceiling-3m-above-plane-results.md`  
  Focused 3D height sweep for the 8-anchor layout with upper anchors at
  3 m. Quantifies how Z error grows as tags move near and above the top-anchor
  plane.

- `geometry-search-results.md`  
  Practical anchor-geometry search for improving Z accuracy without very tall
  anchors. Evaluates 8-, 9-, 10-, and 12-anchor ideas and recommends a
  five-tripod / ten-anchor layout with four corner stations plus one center
  station.

- `recommended-roadmap-and-test-plan.md`  
  Implementation roadmap and first hardware test plan. Explains why the new
  correction should be added in telemetry-only `MONITOR` mode first, then
  validated through anchor-only, static-tag, grid, two-tag, motion, and
  ArduPilot bench tests before guarded `APPLY`.

- `simulations/tdoa_novel_ideas_sim.py`  
  Companion simulation harness for `novel-ideas-followup.md`. Models
  per-anchor TX/RX delays plus non-separable per-pair multipath and compares
  the proposed strategies against the current `huber`/`geom_locked` baselines.

- `simulations/tdoa_strategy_stack_sim.py`  
  Decision harness used by `strategy-stack-results.md`. It evaluates whether
  the candidate corrections are substitutes or complementary and tests the
  symmetric-only fallback for DS-TWR-style anchor streams.

- `simulations/tdoa_geometry_search_sim.py`  
  Practical 3D anchor-layout search harness. Compares the baseline 8-anchor
  corner-tripod cube against modest-height, center-tripod, inset/outset,
  staggered, and 12-anchor alternatives.

## Important Context

These notes are based on the codebase state inspected on 2026-04-16. At the
time of the investigation, the firmware source tree was checked against
upstream `origin/main` and was content-equivalent for the relevant paths. The
desktop manager submodule had newer upstream changes related to antenna
calibration, but the submodule checkout itself was not changed.

The follow-up experiments were refreshed on 2026-04-17 after `git fetch
origin --prune`. Local `HEAD` was an ancestor of `origin/main`, and
`git diff HEAD..origin/main` was empty, so the investigated source tree was
content-equivalent to current upstream main for the relevant firmware files.

The simulation scripts are intentionally simple and hardware-constrained. They
model only strategies that can plausibly be implemented on the current
DW1000/ESP32 system:

- static antenna-delay calibration,
- static anchor geometry use,
- online per-anchor correction from the anchor-anchor stream,
- tag-side inter-anchor ToF correction/blending,
- robust measurement weighting,
- low-latency smoothing,
- static-grid calibration,
- self-survey from measured anchor distances.

They do not model unavailable hardware features or require external motion
capture during runtime.

The benchmark harness is also host-side only. It compares solver shapes and
checks for obvious risks, but the final CPU budget decision must come from
ESP32S3 target measurements around the real solver call.
