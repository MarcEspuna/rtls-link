# TDoA Newton-Raphson Estimator Optimization Plan

Branch: `feature/tdoa-estimator-optimization`
Companion `rtls-link-manager` PR: required only for Phase 1.5 (`USE_2D_ESTIMATOR` parameter wiring).

## Goals

1. Eliminate update-rate jitter (currently ~5 ms quantized + variable solver cost).
2. Reduce solver compute by switching to single-precision while preserving numerical stability near the anchor centroid and on hyperboloid asymptotes.
3. Decouple producer (UWB callbacks) from consumer (estimator) timing.
4. Preserve height (rangefinder) telemetry to the autopilot during UWB dropout.

---

## Phase 1 — Solver math & numerical work (`lib/tdoa_newton_raphson/`)

### 1.1 Type strategy

- `Scalar = float` for solver storage and iteration.
- `double` retained for **covariance computation only** (single shot at convergence; matrix inverse is sensitive).
- Type aliases:
  - `PosMatrix` (`Nx3` float, `MaxRows=kMaxCapacity`)
  - `DynVector` (`Nx1` float, `MaxRows=kMaxCapacity`)
  - `PosVector2D` (`2x1` float)
  - `CovMatrix3D` / `CovMatrix2D` stay `double`.
- Caller (`uwb_tdoa_tag.cpp`) fills float matrices from existing parameter doubles via cast.

### 1.2 Stability mechanisms (avoid the float instability previously observed)

- Solve via **HouseholderQR on `J` directly** (not normal equations). Avoids squared-condition penalty that broke earlier float attempts.
- Add **Levenberg–Marquardt damping**:
  - Init `λ = 1e-3 * trace(JᵀJ) / 3`.
  - On accepted step (residual decreases): `λ /= 2`.
  - On rejected step (residual increases): `λ *= 4`, redo from previous pos.
  - Step solves the augmented system `[J; sqrt(λ)·I] Δ = [r; 0]` in float QR — keeps QR path, gets damping.
- Near-zero distance floor raised slightly (`1e-4 m` instead of `1e-6 m`) — well below sensor noise, avoids float-underflow rows.

### 1.3 Compute optimizations

- `calculateResiduals` returns residuals **and** `dL`, `dR` per row via output references; NR step reuses them instead of recomputing norms.
- The **converged Jacobian is cached** in the result struct and reused for covariance — eliminates the third Jacobian rebuild.
- Convergence: **min(`step.norm() < 1e-3 m`, relative residual change `< 1e-3`)**, max iterations dropped from **10 → 5**.
- Hoist solver scratch (`gradF`, `residuals`, `step`, `distancesL/R`) into a `SolverContext` struct passed into the iteration — single construction per call instead of one per iteration.

### 1.4 Covariance path

- Reuse cached float Jacobian, cast row-by-row into a `double` `JᵀJ`.
- Keep existing Tikhonov regularization and LDLT logic.
- `measurementVariance` and downstream covariance stay `double` end to end.

### 1.5 `USE_2D_ESTIMATOR` parameter wiring

- Add `uwbParams.use2DEstimator` (uint8) to `UWBParams`.
- Replace hardcoded `static constexpr bool USE_2D_ESTIMATOR = true` in `estimatorProcess` with runtime read of the parameter.
- LittleFS schema bumped (with default = 1 to preserve current behavior).
- Companion change in `rtls-link-manager` to expose the parameter in the UI.

### 1.6 Tests

- New `test_tdoa_newton_raphson_numerical.cpp`:
  - Tag near anchor centroid (worst-case condition number)
  - Tag near a hyperboloid asymptote (`dL ≈ dR`)
  - Bad initial guess far from solution (LM divergence path)
  - Float-LM-QR vs. current double output, max residual error within 5 cm.
- Existing `test_tdoa_newton_raphson.cpp` updated to new types / signatures.

---

## Phase 2 — Threading & data flow (`src/uwb/uwb_tdoa_tag.cpp`)

### 2.1 Pair-indexed measurement storage

- Replace `etl::vector<TDoAMeasurement, 64>` + `std::find_if` + dual `remove_if` with:
  ```cpp
  static etl::array<TDoASample, 28> measurement_slots;   // 8C2 = 28 unique pairs
  static uint32_t fresh_pair_count;                      // slots with a valid timestamp
  ```
- `pair_index(a, b) = lookup_table[min(a,b)][max(a,b)]` (compile-time table).
- `estimatorCallback`: O(1) write to slot, increments `fresh_pair_count` only when a stale slot becomes fresh.
- `estimatorProcess`: iterate 28 slots, copy fresh ones into a local `etl::vector<…, 28>` for solver, mark stale slots in place.
- Sign-flip handling preserved (canonical pair stores `tdoa` with `anchor_a < anchor_b`).

### 2.2 Event-driven estimator task

- `pos_estimator_task` keeps its 200 Hz cadence as a **watchdog only**; primary trigger is `xTaskNotifyGive` from `estimatorCallback`.
- Notify condition: `fresh_pair_count >= MIN_MEASUREMENTS` AND time since last notify > 5 ms (debounce against bursty UWB rx).
- Task body: `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50))` — wakes immediately on data, falls through every 50 ms for the rangefinder watchdog (Phase 3).
- `vTaskDelayUntil` removed.

### 2.3 Bounded producer mutex

- `xSemaphoreTake(measurements_mtx, portMAX_DELAY)` → `pdMS_TO_TICKS(2)` in `estimatorCallback`.
- On timeout: drop sample, `stats_producer_dropped++` (gated on `TDOA_STATS_LOGGING`).
- Stats dump line extended with this counter.

### 2.4 Task priority

- Audit current priority map (WiFi / Console / App task creations).
- Raise `pos_estimator_task` priority to be **strictly above** the App / UWB-handler task that feeds it, below any hard-real-time task.
- Document chosen priority near the task-holder definition.

---

## Phase 3 — Rangefinder height fallback during UWB dropout (`src/app.cpp`)

### 3.1 Detect UWB dropout in App

- `last_sample_timestamp_ms_` already tracked.
- New constant `kUwbDropoutMs = 500`.
- Helper `bool IsUwbStale(now_ms)` returning `now_ms - last_sample_timestamp_ms_ > kUwbDropoutMs`.

### 3.2 Independent height-forward path (Option A — DISTANCE_SENSOR auto-forward)

- In `App::Update()` (10 Hz), when `IsUwbStale() && IsRangefinderHealthy()`:
  - Emit `DISTANCE_SENSOR` directly, **regardless of `rfForwardEnable`**, using cached `last_rangefinder_distance_cm_` and orientation from params.
  - Reuses existing `local_position_sensor_.send_distance_sensor()` path.
- Flag `rangefinder_dropout_forward_active_` prevents double-send when `rfForwardEnable=1` already covers it.

### 3.3 Logging

- Single `LOG_WARN` on entering dropout state, single `LOG_INFO` on exit. No per-iteration spam.

### 3.4 Watchdog tie-in

- Estimator task's 50 ms watchdog wake (Phase 2.2) is unrelated — App task's own 10 Hz schedule keeps running regardless of UWB silence. No extra wiring.

---

## Phase 4 — Validation

### 4.1 Build

- `pio run -e esp32_application` — clean.
- `pio run -e esp32s3_application` — clean (rangefinder path is S3-only).
- `pio run -e native` — runs new and existing solver tests.

### 4.2 Runtime smoke test (manual, post-merge)

- Position output cadence steady at UWB measurement rate.
- UWB blocked physically with rangefinder attached → DISTANCE_SENSOR continues flowing to ArduPilot.
- Good geometry: tag mid-anchors, no plot jitter in `rtls-link-manager`.
- Bad geometry: tag near centroid, solver produces sane positions (LM doing its job).

### 4.3 Stats sanity

With `TDOA_STATS_LOGGING == ENABLE`:

- `Sent` / `Rej` ratio similar to baseline.
- `Iterations` (new field) typical ~2–3, down from ~10.
- `producer_dropped` ≈ 0 in steady state.

---

## Commit / PR strategy

- Branch: `feature/tdoa-estimator-optimization`.
- Commits land in dependency order in one PR:
  1. Phase 1 — solver math (types, LM, QR, scratch hoisting, tests).
  2. Phase 1.5 — `USE_2D_ESTIMATOR` parameter wiring (firmware + manager submodule).
  3. Phase 2 — pair-indexed storage, event-driven task, bounded producer.
  4. Phase 3 — rangefinder dropout fallback.
- Companion `rtls-link-manager` branch with the same name for Phase 1.5; cross-link in PR descriptions.
- No new `user_defines.txt` flag needed — all changes improve existing paths. The `USE_2D_ESTIMATOR` knob moves from compile-time constant to runtime parameter.

---

## Confirmed decisions

- (1) Float strategy: float storage + float QR + LM damping; double covariance.
- (2) Solver math cleanup per Phase 1.
- (3) Threading: event-driven, bounded producer, pair-indexed storage, audited priority.
- (3.2) Rangefinder height: Option A (DISTANCE_SENSOR auto-forward on UWB dropout).
- Dropout threshold: 500 ms.
- `USE_2D_ESTIMATOR`: wired to `uwbParams`, with manager-side UI exposure.
- Branch name: `feature/tdoa-estimator-optimization`.
