# TDoA Anchor Model Stationary Validation

Date: 2026-04-25

## Scope

This note records the stationary closed-loop hardware result for the firmware-side TDoA anchor-model feature on branch `Feature/TDoA-Precision-Firmware-Validation`.

The test validates the implementation-agreement Phase 3 gate: the tag was stationary near the middle of a four-anchor square, and firmware was exercised in baseline, locked raw/effective, and locked propagation domains.

Matcher selection-policy behavior is documented separately in `docs/tdoa-matcher-selection-policy-diagnostic-note.md` and is intentionally out of scope for this branch.

## Hardware

Discovery found the expected five-device setup:

| IP | Role | Board | Anchor ID |
|---|---|---|---|
| `192.168.0.100` | `anchor_tdoa` | `MAKERFABS_ESP32` | `2` |
| `192.168.0.101` | `anchor_tdoa` | `MAKERFABS_ESP32` | `1` |
| `192.168.0.102` | `anchor_tdoa` | `MAKERFABS_ESP32` | `0` |
| `192.168.0.103` | `anchor_tdoa` | `MAKERFABS_ESP32` | `3` |
| `192.168.0.104` | `tag_tdoa` | `ESP32S3_UWB` | tag |

The tag artifact was built from `esp32s3_application` and uploaded only after confirming the tag board type with `firmware-info`.

## Firmware Behavior Validated

- `OFF` preserved baseline behavior.
- `MONITOR` and collection plumbing produced compact model/status output without changing estimator input.
- `LOCKED_ANCHOR_MODEL` replaced known inter-anchor ToF values with the locked model.
- `RAW_EFFECTIVE` locked the same raw/effective ToF domain consumed by the existing TDoA engine.
- `PROPAGATION` subtracted antenna delays during model establishment and converted back to raw/effective ToF before storage.
- Rollback to `OFF` restored baseline operation.
- Estimator diagnostics reported accepted/rejected counts, RMSE distribution, stale removals, measurement-count distribution, position-window statistics, full-window statistics, and selected TDoA pair distributions.

## Stationary Results

Each capture used a full-window estimator reset before the run.

| Mode | Capture | Accepted | Rejected | Stale Removed | Horizontal RMS | RMSE Mean | RMSE Max | Fallback |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `OFF` | 120 s | 23945 | 0 | 9 | 5.38 cm | 74.08 mm | 288.29 mm | n/a |
| `LOCKED_ANCHOR_MODEL`, `RAW_EFFECTIVE` | 120 s | 23967 | 0 | 13 | 5.07 cm | 74.33 mm | 205.07 mm | false |
| `LOCKED_ANCHOR_MODEL`, `PROPAGATION` | 120 s | 23983 | 0 | 7 | 5.02 cm | 74.20 mm | 199.90 mm | false |

The locked model established all six four-anchor pairs and reported six healthy pairs in both domains.

Representative locked raw/effective ToF values:

| Pair | Locked ToF | MAD |
|---|---:|---:|
| `01` | 34099-34101 | 3-5 |
| `02` | 34209-34217 | 3 |
| `03` | 33529-33530 | 2 |
| `12` | 33494-33497 | 2-3 |
| `13` | 34211-34217 | 3-4 |
| `23` | 34092-34095 | 3-5 |

## Interpretation

The feature is working as an anchor-model stabilization layer:

- locked override remained healthy,
- estimator rejects did not increase,
- update rate remained usable,
- rollback to `OFF` worked,
- stationary horizontal RMS improved slightly.

The improvement is modest, not transformational. The likely reason is the existing TDoA matcher's `Youngest` selection policy, which strongly favors temporal-neighbor pairs and rarely emits diagonal pairs `02` and `13` into the estimator input stream. That issue is a separate selection-policy investigation, not a blocker for completing this branch through the flight-monitor gate.

## Decision

Proceed to Phase 4 only as a monitor-mode flight check:

- keep override disabled,
- collect/lock the model on the ground,
- fly with `tdoaAnchorModelMode=MONITOR`,
- watch pair residual health, estimator rejects, stale removals, update rate, and telemetry/log stability.

Do not proceed to flight override unless the monitor flight remains healthy.

## Pre-Flight Monitor Ground Check

After documenting the stationary override results, the tag was configured for a final ground check matching the intended flight-monitor setup:

- `tdoaAnchorModelMode=MONITOR`
- `tdoaAnchorModelDomain=PROPAGATION`
- model reset before collection
- estimator stats reset after the model locked
- override disabled for the capture

The model locked all six pairs and remained healthy:

| Pair | Locked Propagation ToF | MAD | Residual Max |
|---|---:|---:|---:|
| `01` | 1131 | 4 | 10 |
| `02` | 1259 | 3 | 8 |
| `03` | 523 | 2 | 9 |
| `12` | 528 | 2 | 8 |
| `13` | 1196 | 4 | 7 |
| `23` | 1089 | 4 | 11 |

Monitor-only estimator result after a 60-second post-lock capture:

| Metric | Value |
|---|---:|
| Accepted positions | 14536 |
| Rejected positions | 0 |
| Reject RMSE | 0 |
| Reject NaN | 0 |
| Reject insufficient | 0 |
| Stale removed | 17 |
| Measurement count | 4-6 |
| Full-window horizontal RMS | 5.12 cm |
| RMSE mean | 73.62 mm |
| RMSE max | 185.62 mm |
| Healthy pairs | 6 |
| Fallback active | false |

Discovery after the capture still found all five devices. The tag reported `anchorsSeen=4`, `sendingPos=true`, and `avgRateCHz=18856`.

This is sufficient to move to Phase 4: a flight with monitor enabled and override disabled.

## Phase 4 Monitor Flight

The first monitor-flight attempt was invalidated because the tag was rebooted after the ground lock. The normal parameters persisted, so the tag came back in `MONITOR` and `PROPAGATION`, but the locked six-pair model is currently RAM-only and was lost. Post-flight status showed `locked=false`, `version=0`, and no locked pair values. This was not a model-health failure during flight.

A second flight was run without rebooting after ground lock. This flight included both hover and motion. The tag was configured as:

- `tdoaAnchorModelMode=MONITOR`
- `tdoaAnchorModelDomain=PROPAGATION`
- locked model retained in RAM
- override disabled
- estimator stats reset before flight

Post-flight model status:

| Pair | Locked Propagation ToF | MAD | Residual Max | Healthy |
|---|---:|---:|---:|---|
| `01` | 1131 | 5 | 16 | true |
| `02` | 1257 | 3 | 8 | true |
| `03` | 524 | 2 | 6 | true |
| `12` | 528 | 2 | 6 | true |
| `13` | 1194 | 4 | 9 | true |
| `23` | 1090 | 5 | 10 | true |

Post-flight estimator and discovery result:

| Metric | Value |
|---|---:|
| Accepted positions | 24382 |
| Rejected positions | 0 |
| Reject RMSE | 0 |
| Reject NaN | 0 |
| Reject insufficient | 0 |
| Stale removed | 14 |
| Measurement count | 4-5 |
| RMSE mean | 110.16 mm |
| RMSE max | 391.28 mm |
| Healthy pairs | 6 |
| Fallback active | false |
| Discovery anchors seen | 4 |
| Discovery sending positions | true |
| Discovery average rate | 195.35 Hz |

The full-window position RMS is not interpreted as stationary jitter because this was not a hover-only flight; the drone intentionally moved. The monitor-specific gates passed: the locked model stayed present, residual health remained good, fallback did not activate, estimator rejects stayed at zero, stale removals remained low, and discovery/update-rate health stayed normal.

This satisfies Phase 4. The branch can proceed to a carefully bounded Phase 5 override flight, still using immediate rollback to `OFF` if health degrades.

## Phase 5 Override Flight

A bounded override flight was run after a fresh ground lock. The tag was configured as:

- `tdoaAnchorModelMode=LOCKED_ANCHOR_MODEL`
- `tdoaAnchorModelDomain=PROPAGATION`
- locked model retained in RAM
- estimator stats reset before flight

Post-flight model status:

| Pair | Locked Propagation ToF | MAD | Residual Max | Healthy |
|---|---:|---:|---:|---|
| `01` | 1131 | 3 | 12 | true |
| `02` | 1258 | 3 | 4 | true |
| `03` | 524 | 2 | 2 | true |
| `12` | 528 | 3 | 5 | true |
| `13` | 1193 | 4 | 15 | true |
| `23` | 1091 | 5 | 10 | true |

Post-flight estimator and discovery result:

| Metric | Value |
|---|---:|
| Accepted positions | 24223 |
| Rejected positions | 0 |
| Reject RMSE | 0 |
| Reject NaN | 0 |
| Reject insufficient | 0 |
| Stale removed | 16 |
| Measurement count | 4-5 |
| RMSE mean | 51.96 mm |
| RMSE max | 519.26 mm |
| Healthy pairs | 6 |
| Fallback active | false |
| Discovery anchors seen | 4 |
| Discovery sending positions | true |
| Discovery average rate | 184.88 Hz |

The pilot did not perceive a clear difference compared with the monitor flight. That is consistent with the stationary results: the feature is stable and does not regress estimator health, but the improvement is small in the current matcher-policy regime.

The override flight passed the safety/health gates:

- model remained locked,
- all six pairs stayed healthy,
- fallback did not activate,
- estimator rejects stayed at zero,
- stale removals stayed low,
- tag continued sending positions with all four anchors seen.

After the test, the tag was rolled back to `tdoaAnchorModelMode=OFF`. Rollback succeeded; status reported `mode=OFF`.

This completes Phase 5 as a bounded validation. The data supports keeping the model/diagnostic implementation, but it does not yet support claiming a large flight-visible improvement. Before productizing override as a default behavior, the branch should address RAM-only model lifetime and product narrowing, and the separate matcher-selection investigation should evaluate whether better pair selection unlocks larger gains.

## Phase 6 Product Narrowing

The retained runtime surface is intentionally small:

- `OFF`
- `MONITOR`
- `LOCKED_ANCHOR_MODEL`

The retained test domains are:

- `RAW_EFFECTIVE`
- `PROPAGATION`

`PROPAGATION` is the leading candidate from the stationary and flight checks. `RAW_EFFECTIVE` remains useful as the lower-risk A/B baseline because it locks the same domain already consumed by the TDoA engine.

The branch should not claim a large user-visible flight improvement from the current matcher policy. The validated product claim is narrower:

- firmware can collect and lock a stable six-pair anchor model,
- monitor mode can observe model health during flight without changing estimator input,
- override mode can run without update-rate, fallback, or estimator-reject regressions in this setup,
- rollback to `OFF` works.

The RAM-only model lifetime found during the first monitor-flight attempt is addressed by persisting the locked six-pair model to LittleFS. A successful lock writes a compact model file; reboot can reload the model when the persisted domain matches `tdoaAnchorModelDomain`; `tdoa-anchor-model-reset` clears the persisted model.

The persistence path was hardware-verified on the ESP32-S3 tag:

- OTA uploaded the persistence build to `192.168.0.104`,
- collected and locked a fresh `PROPAGATION` model,
- status reported `persisted=true`,
- issued `reboot`,
- post-reboot status reported `lastError="persisted model loaded"`, `locked=true`, `persisted=true`, `healthyPairs=6`,
- the tag was returned to `tdoaAnchorModelMode=OFF` after verification.

Remaining pre-merge hardening:

- add focused automated tests for model validity, domain conversion, fallback, and command JSON,
- keep override disabled by default,
- keep matcher-selection changes out of this branch.
