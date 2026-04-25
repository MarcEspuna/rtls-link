# TDoA Estimator Optimization — Work Report & Current Status

**Branch**: `feature/tdoa-estimator-optimization`
**PRs (draft)**:
- Firmware: https://github.com/MarcEspuna/rtls-link/pull/29
- Manager: https://github.com/MarcEspuna/rtls-link-manager/pull/17

---

## 1. Original problem

The TDoA tag's position output exhibited update-rate jitter — sometimes faster, sometimes slower — and the estimator path had several known inefficiencies. Investigation also surfaced a flight incident where UWB lost contact and altitude EKF corrections stopped flowing, even though the rangefinder was still healthy.

## 2. Investigation findings

Before any code changes, the estimator pipeline was analyzed against the live source. The dominant jitter sources were:

1. **Variable solver work per cycle, no damping.** `NUM_ITERATIONS = 10` with a tight `step.norm() < 1e-4` (0.1 mm) convergence and undamped Gauss-Newton via `completeOrthogonalDecomposition`. Easy geometries converged in 2–3 iterations; bad geometries spent all 10.
2. **`vTaskDelayUntil(5 ms)` polling cadence.** Output cadence was quantized to multiples of 5 ms, with catch-up bursts after long solves.
3. **Estimator task priority 1.** Could be preempted mid-solve by WiFi/Console/App tasks at the same priority.
4. **`portMAX_DELAY` in `estimatorCallback`.** Producer blocked unbounded waiting for the consumer's mutex.
5. **Linear-scan storage.** `etl::vector` with `find_if` per measurement, dual `remove_if` passes for stale/unconfigured filtering.
6. **Solver math waste.** Distances computed in `calculateResiduals` were thrown away and recomputed in `newtonRaphsonStep`. Jacobian rebuilt a third time at convergence for covariance.
7. **`double` precision throughout** — ESP32 has hardware single-precision FPU; `double` is software-emulated.

## 3. Work completed

Three independent improvements landed, each as a separate commit on `feature/tdoa-estimator-optimization`. All firmware (`esp32_application`, `esp32s3_application`) and `rtls-link-core` (Rust) builds are clean.

### Phase 1 — Solver math (`a3a384b`)
- **Float storage with double covariance.** `Scalar = float` for `PosMatrix`, `DynVector`, `PosVector2D`, `PosVector3D`. `CovMatrix3D`/`CovMatrix2D` stay `double` (the matrix inverse is precision-sensitive; runs once per solve).
- **HouseholderQR-on-J + Levenberg-Marquardt damping.** Solves the augmented system `[J; sqrt(λ)·I] Δ = [r; 0]` in float QR. This avoids the squared-condition penalty of normal equations — the path that broke earlier float attempts at the centroid and on hyperboloid asymptotes.
- **Cached distances and Jacobian.** `computeResidualsAndDistances3D` returns residuals + per-row `dL`, `dR`. The Jacobian builders reuse them. The converged Jacobian is reused for covariance instead of being rebuilt a third time.
- **Convergence loosened** from `step.norm() < 1e-4` to `min(step.norm() < 1e-3 m, relative-residual change < 1e-3)`. Max iterations dropped 10 → 5.
- **Scratch hoisted** into `SolverContext` structs, eliminating per-iteration construction churn.

### Phase 1.5 — `USE_2D_ESTIMATOR` runtime parameter
- Compile-time `static constexpr bool USE_2D_ESTIMATOR = true` replaced with `uwbParams.use2DEstimator` (uint8, default 1).
- Wired through `UWBParams` (firmware), LittleFS schema, `rtls-link-core` (Rust) `UwbConfig` + `config_to_params` + all `UwbConfig` literal initializers, TS shared types/configParams, and a new selector in `ConfigEditor.tsx`.
- Mode change resets `first_estimation` so the warm-start is reseeded.

### Phase 1.6 — Solver tests
**13 native tests pass** (`pio test -e native -f test_tdoa_newton_raphson`). Includes:
- 6 baseline tests updated for the new float types.
- 4 covariance tests.
- **3 new numerical-stability tests** (the cases earlier float attempts likely failed):
  - `CentroidStability3D` — tag near anchor centroid (worst-case condition number)
  - `HyperboloidAsymptote2D` — tag where `dL ≈ dR` for several pairs
  - `OffsetInitialGuess3D` — initial guess outside the anchor envelope
- **1 regression test** to lock in the iteration-count win:
  - `WarmStartConvergesQuickly` — asserts `iterations ≤ 3` for warm-started, well-conditioned solves.

### Phase 2 — Threading & storage (`1561ceb`)
- **Pair-indexed storage.** Replaced `etl::vector<TDoAMeasurement, 64>` + `find_if` + dual `remove_if` with `etl::array<PairSlot, 28>` (8C2 unique anchor pairs). Lookup via a compile-time `pair_index(a, b)` LUT — O(1) producer write.
- **Sign canonicalization.** Slot stores `tdoa` with `anchor_a < anchor_b`; reverse-order callbacks sign-flip on the way in. Consumer always reads canonical orientation.
- **Event-driven estimator task.** Switched from `TaskType::PERIODIC` (200 Hz `vTaskDelayUntil`) to `TaskType::CONTINUOUS` blocking on `xTaskNotifyGive`. Producer notifies once `fresh_pair_count >= MIN_MEASUREMENTS` (4) AND a 5 ms debounce window has elapsed. Watchdog fallback wakes every 50 ms so dynamic-anchor and stats bookkeeping still runs during quiet periods.
- **Bounded producer mutex.** `xSemaphoreTake(measurements_mtx, pdMS_TO_TICKS(2))`. On timeout, drop the sample and increment `stats_producer_dropped`.
- **Estimator task priority** raised 1 → 2 (above all other tasks at priority 1). A solve cannot be preempted by App/WiFi/Console.
- **`GetAnchorsSeenCount` rewritten** to use a `uint8_t` bitmask + `__builtin_popcount` (replacing `etl::set` allocation).

### Phase 3 — Rangefinder dropout fallback (`8e2de5d`)
- Cache full `mavlink_distance_sensor_t` + source sysid/compid in `App` whenever the rangefinder callback fires.
- In `App::Update()`, when:
  - `last_sample_timestamp_ms_` is silent for >500 ms (UWB dropout), AND
  - `IsRangefinderHealthy()` is true, AND
  - `rfForwardEnable == 0` (otherwise the explicit-forward path already covers it),
  
  re-emit the cached `DISTANCE_SENSOR` via the same `send_distance_sensor` path used for explicit forwarding. ArduPilot's altitude EKF keeps getting corrections.
- Single `LOG_WARN` on entering dropout-forward, single `LOG_INFO` on exit. No per-iteration log spam.

### Phase 4 — On-target timing instrumentation (just added)
- Bracketed both `newtonRaphson()` and `newtonRaphson2D()` calls with `esp_timer_get_time()` brackets (gated on `TDOA_STATS_LOGGING == ENABLE`).
- New stats fields: solve count, sum, min, max, iteration sum.
- Stats dump line extended with `Solve=[min/avg/max]us Iter=N.N`. Resets every 1 s alongside the existing counters.
- This gives the ground-truth solve duration on hardware to validate the asm-derived estimate (~150–300 µs per solve).

---

## 4. Validation

### Compile
| Target | Result |
|---|---|
| `pio run -e esp32_application` | ✅ clean |
| `pio run -e esp32s3_application` | ✅ clean |
| `cargo check -p rtls-link-core` | ✅ clean |
| `pio test -e native -f test_tdoa_newton_raphson` | ✅ 13/13 pass |

### Assembler review (Phase 1)
Disassembled `tdoa_newton_raphson.cpp.o` for `esp32s3_application` to verify the float strategy actually compiles to hardware FPU.

**Findings:**
- **Zero soft-float** (`__mulsf3`, `__addsf3`, etc.) anywhere in the iteration path. All float ops are real Xtensa LX7 FPU instructions.
- **187 `madd.s`** (multiply-accumulate, the optimal Xtensa LX7 op) + 43 `mul.s` + 19 `sub.s` + 10 `add.s` + 8 `msub.s` + 4 `neg.s`.
- **Inner-loop dot product** compiles to optimal `mul.s; madd.s; madd.s` — 3 instructions for a 3-way dot product.
- **`__divsf3` is invoked** because Xtensa LX7's FPU has no `div.s` instruction. Bounded count: ~400 calls per solve = ~50 µs at 240 MHz. Mitigation lever: `-freciprocal-math` for this TU; not applied yet.
- **`sqrtf` is a libm call** (LX7 has no `sqrt.s` either). ~200 calls per solve = ~8 µs. Negligible.
- **Soft-double** (`__muldf3`, `__adddf3`, `__divdf3`, `__extendsfdf2`) confined to covariance path: Eigen's `gebp_kernel<double>`, `LDLT::compute`, and the float→double row-cast in `computePositionCovariance3D`. Once per solve, ~5–10 µs.
- **Eigen exception machinery** (`__cxa_throw`, `_Unwind_Resume`, `bad_alloc` vtable) linked in but unreachable given our `kMaxCapacity` template parameter. Code-size cost ~1–2 KB; not perf.

**Estimated per-solve cost on ESP32-S3 at 240 MHz**: ~150–300 µs for 5 LM iterations on N=20 measurements. ~15× headroom against the previous 5 ms polling cycle.

### Tests covered
| Area | Coverage |
|---|---|
| Solver math (perfect, noisy, varying anchor counts, 2D, 3D) | ✅ 9 tests |
| Covariance scaling, symmetry, PSD | ✅ 3 tests |
| Centroid / asymptote / offset-init stability | ✅ 3 tests |
| Iteration-count regression | ✅ 1 test (warm-start ≤ 3 iters) |
| **Pair-index LUT correctness** | ❌ no test |
| **Producer canonicalization (sign-flip on reverse-order)** | ❌ no test |
| **Pair-storage round-trip (slot → solver → result)** | ❌ no test |
| **Threading model** | ❌ no test (FreeRTOS-dependent) |
| **Rangefinder dropout fallback** | ❌ no test (integration) |

The threading and dropout-fallback paths cannot be unit-tested in the native env because they depend on FreeRTOS. The pair-index LUT and canonicalization could be tested natively but are not yet.

---

## 5. Open risks before hardware testing

| # | Risk | Severity | Mitigation |
|---|---|---|---|
| 1 | `compare_exchange_strong` debounce on `last_notify_ms` could drop a notification under bursty rx + producer mutex contention | Low | 50 ms watchdog wake bounds the worst-case latency |
| 2 | Pair-indexed sign canonicalization had a bug during development; no unit test locks it in | Medium | Add a native test for canonicalization round-trip |
| 3 | Estimator task at priority 2 — verify nothing else in the system depends on the old priority-1 ordering | Low | Search done at design time; no dependencies found |
| 4 | Rangefinder dropout-forward at 10 Hz (App task cadence) — verify ArduPilot accepts this rate | Low | Same rate as the existing explicit-forward path with a slow rangefinder |
| 5 | LM damping behavior on real (noisy) data — synthetic tests pass; real noise may push different paths | Medium | On-target timing instrumentation will surface non-converged solves immediately |

## 6. What we'll learn from hardware testing

With the new `Solve=[min/avg/max]us Iter=N.N` stats line:
- **Per-solve duration** — confirms or refutes the asm-derived 150–300 µs estimate.
- **Average iteration count** — should be 2.0–3.0 in the typical warm-start regime; spikes toward 5.0 indicate bad geometry or noise.
- **`producer_dropped` counter** — should be ~0 in steady state. Non-zero means producer mutex contention (e.g., consumer's solve overruns).
- **Output cadence** — visible via the rtls-link-manager plot; should be steady at the UWB measurement rate, no 5 ms quantization.
- **UWB dropout test** — physically block UWB to a tag with rangefinder, verify `DISTANCE_SENSOR` continues flowing and the warn/info log fires once on entry/exit.

---

## 7. Suggested next steps

**Before merging**:
1. Add a native unit test for the `kPairIndexTable` LUT (28 distinct values, symmetric `[a][b] == [b][a]`).
2. Add a native unit test for the producer canonicalization logic (sign flip on `A > B` callbacks). Would have caught the dev-time sign bug in Phase 2.
3. Flash to a known-good tag setup, capture 60 s of stats output, attach to the firmware PR.

**If hardware timing surprises us**:
- `-freciprocal-math` for `tdoa_newton_raphson.cpp` only (drops `__divsf3`).
- `EIGEN_STRONG_INLINE` annotations on the per-row Eigen helpers (eliminates the 3 `callx8` per row in `buildJacobian3D`).

**Out of scope for this PR but worth noting**:
- `USE_2D_ESTIMATOR` toggle is now runtime, but mode-change reset only reseeds the warm-start. The user-visible Z value will jump on transition; document or smooth.
- Stats line is now noticeably wider; consider splitting into two `LOG_DEBUG` lines if it's hard to read in the manager log view.

---

## 8. Confidence summary

| Phase | Confidence | Why |
|---|---|---|
| Phase 1 — solver math | ~92% | Tests pass + asm review confirms hardware FPU usage and bounded soft-float costs |
| Phase 2 — threading | ~65% | Logic is correct on paper; no FreeRTOS unit tests; needs hardware validation |
| Phase 3 — rangefinder fallback | ~75% | Logic is straightforward; ArduPilot integration is unverified |
| Phase 4 — instrumentation | ~95% | Trivial brackets, no behavior change when `TDOA_STATS_LOGGING == DISABLE` |

Confidence will move to ~95% across the board once on-target stats output confirms the per-solve cost and producer-drop counters.
