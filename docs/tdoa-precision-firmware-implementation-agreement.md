# TDoA Precision Firmware Implementation Agreement

Branch: `Feature/TDoA-Precision-Firmware-Validation`  
Worktree: `/home/singu/dev/firmware/repos/rtls-link-tdoa-precision-firmware-validation`

## Goal

Implement and validate a firmware-side TDoA anchor correction feature for a stable four-anchor setup.

The core idea is simple: once the anchors are powered, stable, and inter-anchor ranging is working, firmware collects a startup/static sample window, estimates a stable anchor model, locks it, and then uses that locked model instead of continuously feeding noisy per-packet inter-anchor distance values into the TDoA engine.

This is an investigation-backed feature. It should merge only if closed-loop hardware testing shows meaningful improvement over current firmware without unacceptable update-rate, telemetry, or estimator-health regressions.

## Research References

Research worktree:

`/home/singu/dev/firmware/repos/rtls-link-tdoa-precision-simulations-reports`

Key files:

- `/home/singu/dev/firmware/repos/rtls-link-tdoa-precision-simulations-reports/docs/tdoa-distance-constraint-correction/README.md`
- `/home/singu/dev/firmware/repos/rtls-link-tdoa-precision-simulations-reports/docs/antenna-delay-self-calibration.md`
- `/home/singu/dev/firmware/repos/rtls-link-tdoa-precision-simulations-reports/docs/tdoa-distance-constraint-correction/simulation/tdoa_sim.py`
- `/home/singu/dev/firmware/repos/rtls-link-tdoa-precision-simulations-reports/docs/tdoa-distance-constraint-correction/simulation/debug_check.py`

The simulations showed that current raw ToF consumption is consistently worse than methods that stabilize or correct the inter-anchor ToF term. The best candidates were geometry/static truth, gated geometry, EWMA/median-style stabilization, and antenna-delay-aware variants. Hardware will decide which minimal behavior survives.

## Design Decisions

- Model establishment is firmware-side. The host may start or observe the process, but collection, robust estimation, and model lock run on the device.
- The initial collection window should be configurable, with a starting value around `10s`.
- The anchors are assumed stable during and after the collection window.
- Do not stream raw high-rate ToF over the network. Firmware should aggregate locally and expose compact status/report data.
- The runtime product surface should stay small: `OFF`, `MONITOR`, and `LOCKED_ANCHOR_MODEL`.
- Extra correction methods are allowed internally for A/B testing, but unsuccessful modes should be hidden or removed before merge.

## ToF Domains

The first implementation should support both domains for testing:

- `RAW_EFFECTIVE`: lock the same raw/effective ToF domain currently consumed from `packet->distances[]`. This is the lower-risk default because it directly replaces a noisy input with its stable estimate.
- `PROPAGATION`: lock physical propagation ToF after subtracting both endpoint antenna delays.

```text
tof_propagation = raw_packet_distance_ticks - ADelay[from] - ADelay[to]
```

Hardware testing decides which domain performs better.

## Model Establishment

On startup or explicit command:

1. Wait until TDoA inter-anchor ranging is active.
2. Collect per-pair samples locally for the configured window.
3. Build statistics for the six four-anchor pairs: `01`, `02`, `03`, `12`, `13`, `23`.
4. Use median/MAD as batch statistics to identify the center and spread of the collection window.
5. Use Huber/IRLS weighting to reduce the influence of transient or NLOS-biased samples.
6. Lock the six-pair ToF table and derived anchor coordinates.
7. Expose compact model status and residuals.

Median/MAD and Huber are not runtime flight filters. They are used to establish one locked model from a deliberate collection window. Runtime should not add avoidable latency on top of ArduPilot’s filtering.

## Runtime Behavior

`OFF`:

- Current firmware behavior.

`MONITOR`:

- Build/use the locked model for diagnostics only.
- Do not change the value passed to `tdoaStorageSetRemoteTimeOfFlight()`.
- Report compact measured-vs-locked residuals and health metrics.

`LOCKED_ANCHOR_MODEL`:

- Replace incoming `packet->distances[]` with the locked pair ToF for known pairs.
- Use fixed/locked anchor coordinates in the estimator.
- Continue observing live packet distances only for health monitoring.
- Use pair-level health gates and visible fallback state.

## Firmware Parameters

Add scalar params using the current firmware parameter model where practical:

- `tdoaAnchorModelMode`: `OFF`, `MONITOR`, `LOCKED_ANCHOR_MODEL`.
- `tdoaAnchorModelStartupCollect`: enable automatic startup collection/lock.
- `tdoaAnchorModelCollectWindowMs`: default around `10000`.
- `tdoaAnchorModelMinSamplesPerPair`.
- `tdoaAnchorModelDomain`: `RAW_EFFECTIVE` or `PROPAGATION`.
- `tdoaAnchorModelHealthThresholdTicks`.
- `tdoaAnchorModelHealthWindow`.
- `tdoaAnchorModelHealthQuorum`.

The six-pair locked table can be stored as explicit pair params or in a compact LittleFS model file. Control flags should remain normal params.

## Commands And Telemetry

Expected command surface:

- `tdoa-anchor-model-reset`
- `tdoa-anchor-model-collect-start`
- `tdoa-anchor-model-collect-status`
- `tdoa-anchor-model-lock`
- `tdoa-anchor-model-status`
- `tdoa-anchor-model-export`

Command responses should be compact aggregate JSON. Useful metrics:

- per-pair sample count,
- locked value,
- current residual mean/RMS/max,
- robust spread/MAD or approximate variance,
- health/fallback state,
- estimator output rate,
- estimator reject counts,
- mode, domain, and model version.

## Implementation Phases

### Phase 0: Tooling Readiness

- Initialize `tools/rtls-link-manager` in this worktree.
- Decide whether to keep the pinned manager submodule commit or update it to upstream `main`.
- Reconcile root CLI and manager CLI/core support for discovery, commands, OTA, logs, `tdoa-distances`, and future model commands.
- Add an explicit `firmware-info` board/artifact compatibility gate before OTA.
- Keep raw WebSocket command fallback available for early bring-up.

### Phase 1: Firmware Foundation

- Add mode params and runtime plumbing.
- Add startup collection params and state machine.
- Add the overwrite site before `tdoaStorageSetRemoteTimeOfFlight()`.
- Implement internal/test correction candidates:
  - locked raw/effective table,
  - locked propagation table,
  - antenna-delay-corrected path,
  - EWMA,
  - rolling median,
  - blend/gated geometry.
- Add local aggregation and compact status/report commands.
- Ensure `MONITOR` is behaviorally neutral.
- Ensure `OFF` restores baseline behavior.
- Add tests for model validity, overwrite selection, fallback, and command JSON.

At the end of this phase, stop and report that the firmware foundation is ready for in-hardware testing. Hardware testing starts only after the user signals that the anchors and tag/drone are available.

### Phase 2: Firmware-Side Model Lock

- Collect the startup/static window on device.
- Run median/MAD and Huber/IRLS model establishment on firmware.
- Lock the six-pair table and derived coordinates.
- Persist or retain the model according to the chosen storage path.
- Expose model status, residuals, and report export.

### Phase 3: Stationary Closed-Loop Hardware Test

With the drone/tag sitting still near the middle of the anchor square:

- discover devices,
- verify boards with `firmware-info`,
- build and OTA correct artifacts,
- enable compact telemetry/logs,
- capture baseline with `OFF`,
- establish and lock the model,
- run `MONITOR`,
- run `LOCKED_ANCHOR_MODEL`.

Primary checks:

- stationary jitter,
- update rate,
- estimator rejects,
- pair residuals,
- fallback events,
- telemetry/log health.

This is not a perfect ground-truth test. It is a closed-loop check that the feature was implemented correctly and improves stability relative to baseline.

### Phase 4: Flight Monitor Test

If stationary results are promising:

- fly with monitor enabled and overwrite disabled,
- keep telemetry compact,
- check that the locked model remains healthy during motion,
- do not proceed if residuals, estimator health, or telemetry degrade.

### Phase 5: Flight Override Test

If monitor flight is healthy:

- enable the selected correction/override behavior,
- fly a comparable path,
- compare against baseline and monitor flight,
- roll back to `OFF` immediately if health gates or telemetry indicate degradation.

### Phase 6: Product Narrowing

- Keep the smallest behavior that improves hardware results.
- Hide or remove unsuccessful internal modes.
- Keep useful diagnostics/reporting.
- Update manager/CLI workflow to match final command names and mode surface.

## In-Hardware Testing Gate

Before live operations:

- Four anchors and one tag/drone must be powered and reachable on the same local network.
- Discovery on UDP `3333` must find them, or IPs must be supplied.
- Device board types must be confirmed with `firmware-info`.
- ESP32 anchors use `esp32_application`; ESP32-S3 tags use `esp32s3_application`.
- UDP logs should be enabled with `wifi.logUdpEnabled=1` and `wifi.gcsIp` set to this machine when logs are needed.
- No board-mismatched OTA is allowed.

## Merge Criteria

The branch can merge only if hardware shows:

- meaningful jitter or stability improvement over `OFF`,
- no unacceptable update-rate regression,
- no estimator reject increase that offsets the gain,
- stable pair residuals with the locked model,
- successful rollback to `OFF`,
- usable CLI/manager workflow for setup and verification.

If hardware does not show meaningful improvement, runtime correction should not merge. Diagnostics or model-capture tooling may still merge if useful independently.
