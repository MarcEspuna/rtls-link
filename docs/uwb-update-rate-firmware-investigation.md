# UWB Update-Rate Instability: Firmware-Side Investigation

Date: 2026-03-01  
Scope: firmware-only analysis before live hardware investigation.  
Targets: TDoA tag and TDoA anchor paths in `esp32_application` and `esp32s3_application`.

## 1) Purpose

This report complements `docs/esp32s3-dw1000-update-rate-instability-report.md`.

The earlier report focused mainly on ESP32-S3 and DW1000 hardware/SPI margin. This report focuses on firmware behaviors that can also reduce the observed update rate, either by:

- causing real UWB packet loss or anchor stalls, or
- receiving UWB data successfully but suppressing, delaying, or discarding samples before they are exposed as position updates.

## 2) Executive Summary

The strongest already-known cause remains the ESP32-S3 DW1000 SPI margin issue at 20 MHz.

In addition, this code audit found several firmware-side causes that can independently reduce update rate or make a hardware issue look worse:

1. The tag hot path can block on `measurements_mtx` while processing a fresh TDoA measurement.
2. The tag freshness gate can lose a wakeup because it uses `load()` + `store(0)` instead of an atomic `exchange(0)`.
3. The tag silently drops measurements from anchors that are not configured locally.
4. The parameter model supports only 6 configured anchors even though the TDoA path accepts anchor IDs `0..7`.
5. The output rate seen by the user can be lower than the internal estimator rate because MAVLink position output is heartbeat-gated.
6. Anchor-side delayed TX/RX scheduling remains a real source of intermittent stalls, and the wrapper already contains a stall watchdog that reinitializes the anchor.
7. Current LAN observability is good enough for coarse health checks and live UDP log capture, but weak for algorithm-internal state.
8. The earlier concern about "not being able to add logs in the UWB libraries cleanly" is substantially true with the current project structure.

## 3) Measurement Pipeline Map

### 3.1 Tag receive path

High-level flow:

1. DW1000 IRQ sets `isr_flag` in the ISR.
2. `UWBTagTDoA::Update()` runs in `loop()` and calls `dwHandleInterrupt()`.
3. libdw1000 callback sets `m_DwData.interrupt_flags`.
4. `TagTDoADispatcher` forwards RX/TX/timeout/fail events to `uwbTdoa2TagAlgorithm`.
5. The tag algorithm produces `tdoaMeasurement_t`.
6. `estimatorCallback()` inserts or updates the shared `measurements` vector.
7. `estimatorProcess()` runs in a periodic FreeRTOS task at 200 Hz, builds the solver matrices, runs Newton-Raphson, and calls `App::SendSample()`.

Relevant code:

- `src/main.cpp`
- `src/uwb/uwb_tdoa_tag.cpp`
- `lib/tdoa_algorithm/src/tag/tdoa_tag_algorithm.cpp`

### 3.2 Anchor transmit/receive path

High-level flow:

1. DW1000 IRQ sets `isr_flag` in the ISR.
2. `UWBAnchorTDoA::Update()` runs in `loop()` and calls `dwHandleInterrupt()`.
3. The wrapper dispatches the event into `uwbTdoa2Algorithm`.
4. The anchor algorithm uses delayed RX/TX scheduling to maintain TDMA slot timing.
5. If no interrupts are seen for too long, the wrapper reinitializes the algorithm.

Relevant code:

- `src/uwb/uwb_tdoa_anchor.cpp`
- `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp`

## 4) Findings

### 4.1 Tag hot path can block on a mutex while processing fresh measurements

`estimatorCallback()` acquires `measurements_mtx` with `portMAX_DELAY`.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:384-421`

At the same time, `estimatorProcess()` also locks the same mutex every 5 ms to:

- remove stale entries,
- remove invalid entries,
- resize solver matrices,
- copy all measurements into Eigen structures.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:424-540`

Why this matters:

- `estimatorCallback()` is not called from the hardware ISR, but it is still on the UWB event-processing path.
- If it blocks behind the estimator task, the radio/event handling thread is delayed exactly when new measurements are arriving.
- On a marginal system, this can compound packet loss and appear as intermittent sample starvation.

Assessment: high confidence contributor.

### 4.2 Freshness tracking has a lost-update race

The estimator freshness gate does:

- `new_count = new_measurement_count.load(...)`
- if nonzero, `new_measurement_count.store(0, ...)`

Code:

- `src/uwb/uwb_tdoa_tag.cpp:465-470`

If a new measurement arrives between the `load()` and `store(0)`, that increment is erased. The next estimator cycle may then skip even though fresh data did arrive.

This does not permanently break ranging, but it can reduce apparent update rate when the packet stream is already thin.

Assessment: medium confidence contributor.

### 4.3 Measurements from unconfigured anchors are silently dropped

Before inserting a measurement, the tag requires both anchor IDs to exist in `configured_anchor_ids`.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:390-395`

The estimator also removes stored measurements that reference anchors not configured locally.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:494-506`

This means a tag can be receiving anchor packets correctly but still discard the resulting TDoA measurements if:

- `uwb.anchorCount` is too low,
- local anchor IDs are wrong,
- local anchor ordering differs from the deployed anchor set,
- or the tag configuration is stale relative to the real installation.

Because the solver requires at least 4 measurements:

- `src/uwb/uwb_tdoa_tag.cpp:436`
- `src/uwb/uwb_tdoa_tag.cpp:516`

misconfiguration can present exactly as intermittent insufficient-sample behavior.

Assessment: high confidence contributor, especially for per-device differences.

### 4.4 The parameter model only supports 6 configured anchors while the TDoA path accepts anchor IDs 0..7

`UWBParams` exposes `devId1..devId6` and sets:

- `maxAnchorCount = 6`

Code:

- `src/uwb/uwb_params.hpp:48-114`

But the TDoA code:

- allocates arrays of size 8,
- accepts anchor IDs up to 7,
- and the underlying TDoA algorithm is built around 8 anchors.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:112-113`
- `src/uwb/uwb_tdoa_tag.cpp:98-103`
- `lib/tdoa_algorithm/src/tag/tdoa_tag_algorithm.cpp:50-60`

Implications:

- A deployment using 7 or 8 anchors cannot be fully represented in the current parameter model.
- Measurements involving anchors 6 or 7 will be dropped unless the local config happens to avoid them.
- Even in smaller deployments, this mismatch is a design smell that increases the chance of config/tooling drift.

Assessment: high-confidence design defect. Whether it is active in the field depends on the actual anchor count in the installation.

### 4.5 The observed "update rate" can be lower than the internal estimator rate because MAVLink output is heartbeat-gated

`App::SendSample()` only forwards positions when a recent MAVLink heartbeat has been received.

Code:

- `src/app.cpp:658-695`

If the user is watching the rate through:

- telemetry,
- the manager UI,
- or downstream ArduPilot-visible position messages,

then a heartbeat issue can look like a UWB update-rate issue even though the tag estimator is still producing valid solutions.

Assessment: medium confidence contributor. This depends on how the issue was observed.

### 4.6 Anchor TDMA scheduling can still create real UWB stalls

The anchor algorithm continuously schedules delayed RX and delayed TX events:

- `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:309-325`
- `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:375-391`

The wrapper has an explicit stall watchdog:

- `src/uwb/uwb_tdoa_anchor.hpp:37`
- `src/uwb/uwb_tdoa_anchor.cpp:192-205`

Interpretation:

- if the anchor misses timing margins,
- or a delayed TX/RX setup ends up too close to the current DW1000 time,
- or the radio enters a bad state,

then the anchor can stop producing slot traffic until the watchdog reinitializes it.

Any such anchor-side stall directly reduces the measurement cadence seen by all tags.

Assessment: high confidence contributor, especially when dips correlate across multiple drones.

### 4.7 RX-failed interrupt visibility is weaker than it looks

Both tag and anchor wrappers attach RX-failed callbacks:

- `src/uwb/uwb_tdoa_tag.cpp:225-228`
- `src/uwb/uwb_tdoa_anchor.cpp:123-126`

But `dwSetDefaults()` disables the RX-failed interrupt mask:

- `lib/libdw1000/src/libdw1000.c:633-639`

`dwHandleInterrupt()` does have a receive-failed path:

- `lib/libdw1000/src/libdw1000.c:1282-1291`

but the current configuration reduces direct visibility and makes debugging/recovery less explicit than it should be.

Assessment: medium confidence contributor. More important as a diagnostics gap than as a sole root cause.

### 4.8 The tag is executed in `loop()`, not in a dedicated scheduled task

The UWB frontend runs in Arduino `loop()` instead of a scheduler task.

Code:

- `src/main.cpp:149-152`

There is even a comment indicating this was done because moving it to a periodic thread caused watchdog trouble.

Why this matters:

- the UWB path is intentionally special-cased,
- its scheduling behavior differs from the rest of the firmware,
- and any extra work added to the main loop context can affect radio service latency.

This is not necessarily the primary bug, but it is a structural risk that should stay in scope during live testing.

Assessment: medium confidence contributor.

### 4.9 Dynamic-anchor positioning can intentionally create short update gaps when enabled

For ESP32-S3 builds, `USE_DYNAMIC_ANCHOR_POSITIONS` is compiled in by default:

- `boards/esp32s3_user_defines.txt`

If enabled at runtime, the tag:

- periodically recalculates anchor positions,
- clears old measurements on first dynamic takeover,
- applies a 300 ms holdoff,
- and resets the estimator warm start.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:257-279`
- `src/uwb/uwb_tdoa_tag.cpp:438-447`
- `src/uwb/uwb_tdoa_tag.cpp:783-839`

This is expected behavior, not necessarily a bug, but it can look like intermittent rate instability if the parameter `uwb.dynamicAnchorPosEnabled` is enabled on only some drones.

Assessment: conditional contributor.

## 5) Ranked Root-Cause Hypotheses

1. ESP32-S3 SPI timing / signal-integrity margin at 20 MHz.
2. Tag-side mutex contention in the measurement hot path.
3. Anchor TDMA delayed scheduling stalls and watchdog-driven recovery.
4. Per-device parameter drift causing silent measurement discard.
5. Anchor-count model mismatch (`6` configurable vs `8` accepted by TDoA logic).
6. Apparent rate loss caused by MAVLink heartbeat gating rather than UWB solver failure.
7. Freshness-counter lost-update race.

## 6) What To Check During Hardware Investigation

### 6.1 Distinguish internal estimator rate from exported position rate

Check whether the rate drop is seen in:

- raw tag debug logs,
- manager telemetry,
- MAVLink output,
- or only on the flight controller side.

If only exported telemetry dips, heartbeat gating becomes much more likely.

### 6.2 Compare good and bad drone configs

Read and compare:

- `uwb.anchorCount`
- anchor IDs and coordinates
- `uwb.channel`
- `uwb.dwMode`
- `uwb.txPowerLevel`
- `uwb.smartPowerEnable`
- `uwb.tdoaSlotCount`
- `uwb.tdoaSlotDurationUs`
- `uwb.dynamicAnchorPosEnabled`
- `uwb.rmseThreshold`

This is mandatory because `/params.txt` can make devices with the same binary behave differently.

### 6.3 Watch tag stats logs

The tag already logs useful stats once per second:

- sent samples,
- rejected samples,
- RMSE rejects,
- NaN rejects,
- insufficient-measurement rejects,
- stale-measurement removals.

Code:

- `src/uwb/uwb_tdoa_tag.cpp:705-724`

If `Insuff` spikes during the issue, focus on anchor visibility, configuration, and silent measurement discard.

### 6.4 Watch anchor stall logs

Monitor:

- `UWB stall detected (...), reinitializing`

Code:

- `src/uwb/uwb_tdoa_anchor.cpp:196-205`

If anchor stalls correlate with tag rate dips, the problem is upstream of the estimator.

### 6.5 Evaluate current LAN debugging and logging capability

What is already available today over the private LAN:

1. UDP discovery heartbeat:
   - Sent every 2 seconds by `WifiDiscovery`.
   - Includes device identity, role, firmware version, UWB short address, `sending_pos`, `anchors_seen`, `origin_sent`, `uwb_enabled`, `rf_enabled`, `rf_healthy`, `rf_forward_enabled`, rate statistics, compiled log level, log UDP port, and whether serial/UDP logging are enabled.
   - For TDoA tags with dynamic anchor positioning enabled, it can also include current dynamic anchor positions.
2. UDP log streaming:
   - Firmware logs can be streamed as JSON over UDP to `wifi.gcsIp:wifi.logUdpPort` when `wifi.logUdpEnabled != 0`.
   - The Tauri manager can receive and buffer these logs per device, which is useful during live testing.
3. WebSocket remote command/control:
   - Devices expose `ws://<ip>/ws` when the web server is enabled.
   - This supports `read`, `readall`, `write`, `firmware-info`, reboot, config management, and `tdoa-distances` on TDoA anchors.
4. UART-to-UDP bridge:
   - This can expose MAVLink or other serial traffic over the network, which is useful when checking whether a rate drop is only on the telemetry/export side.

Main LAN-side limitations:

1. Discovery is unicast to `wifi.gcsIp`, not a general broadcast. If `gcsIp` is wrong, discovery appears broken even if the device is healthy.
2. The heartbeat gives fleet-health level observability, not estimator-internal observability. It does not expose:
   - measurement callback latency,
   - mutex contention,
   - dropped measurements by reason,
   - RX fail / RX timeout counts,
   - estimator iteration counts,
   - anchor watchdog trip counts.
3. Log verbosity is compile-time bounded by `LOG_GLOBAL_LEVEL`. Runtime control only enables or disables serial/UDP outputs; it does not raise verbosity on a running device.
4. There is no dedicated remote diagnostic command for TDoA tag internals. Anchor mode at least has `tdoa-distances`; tag mode has no equivalent JSON snapshot for estimator state.
5. The TCP debug socket is effectively a legacy TWR-only path. It is fed by `UpdateLastTWRSample()` and is not currently useful for TDoA-tag investigation.
6. The standalone CLI log parser expects a numeric `level` field, while firmware emits a string `lvl` field. That makes CLI-side log-level filtering unreliable until fixed.

Assessment:

- For the upcoming hardware session, the current LAN tooling is enough to:
  - discover devices,
  - compare runtime configuration,
  - stream wrapper-level logs,
  - inspect anchor raw-distance diagnostics,
  - and distinguish some telemetry/export issues from core UWB issues.
- It is not enough to directly diagnose the most likely firmware-side tag issues without adding more instrumentation.

## 7) Library-Level Logging Constraint and Refactor Options

### 7.1 Was the earlier concern real?

Yes, mostly.

The core issue is architectural:

1. The firmware logging system lives under `src/logging/` and is tied to firmware configuration via `config/features.hpp`.
2. The runtime logger implementation depends on firmware/platform services such as Arduino and FreeRTOS.
3. The UWB algorithm code is built as PlatformIO libraries under `lib/` (`libdw1000`, `tdoa_algorithm`, `tdoa_estimator`) and is treated as reusable library code.

Because of that split, adding direct dependencies from library code to the application logger couples reusable algorithm code to:

- firmware-only headers under `src/`,
- the project feature system,
- and the current logging transport/runtime implementation.

That coupling is exactly the kind that tends to break builds, reduce reuse, and make library testing harder.

There is also evidence that this repo inherited internal diagnostics concepts from the original Crazyflie codebase, but they are not properly wired here:

- `tdoa_algorithm` still contains dormant `tdoaStats` structures,
- commented-out stats events,
- `DEBUG_MODULE` markers,
- and optional `LOG_GROUP_START` style hooks.

In other words: the libraries are not fully opaque, but their internal diagnostics are currently fragmented and not integrated into the firmware's LAN-visible logging path.

### 7.2 What are the practical options?

#### Option A: Minimal fix, lowest risk

Create a tiny shared diagnostics facade that lives outside `src/` and is safe for library use.

Design goals:

- header-only or dependency-light,
- no Arduino/FreeRTOS requirement in library headers,
- no direct dependency on `src/logging/logging.hpp`,
- no-op by default when diagnostics are disabled.

Typical shape:

- a small `enum` of event IDs,
- a struct payload for counters/snapshots,
- and one or more function pointers or callbacks registered by the wrapper code.

Benefits:

- libraries can emit structured diagnostics without knowing about UDP, WebSocket, serial, or the current logger,
- wrappers decide whether to convert those diagnostics into logs, counters, heartbeats, or command responses.

Assessment: best immediate path.

#### Option B: Callback-based diagnostics sink

Expose explicit instrumentation hooks from the libraries:

- `libdw1000`: RX timeout, RX fail, late schedule, reset/reinit, IRQ timing counters.
- `tdoa_algorithm`: packets received, packets accepted, measurements emitted, anchor-context misses, clock-correction rejects.
- `tdoa_estimator`: iteration count, convergence failures, singular matrix / NaN / covariance issues.

The wrapper layer then aggregates these into:

- periodic logs,
- a remote JSON diagnostics command,
- and optionally discovery-heartbeat summary fields.

Assessment: very good balance between observability and architectural cleanliness.

#### Option C: Move logging primitives into a shared core module

Extract the essential logging abstractions from `src/` into a shared module that both app code and libraries can depend on cleanly.

This would mean separating:

- log macros / interfaces,
- feature flags needed by libraries,
- and transport/runtime backends.

The libraries would depend only on the interface layer; the firmware app would provide the serial/UDP implementation.

Assessment: good medium-term cleanup, but broader than needed for the next hardware phase.

#### Option D: Large refactor of the UWB stack boundaries

Refactor the UWB stack around explicit service interfaces:

- logger,
- clock/time source,
- mutex/locking policy where needed,
- diagnostics sink,
- and possibly transport/config adapters.

That would leave:

1. algorithm libraries focused on pure radio/estimation logic,
2. wrapper layers focused on ESP32/DW1000 integration,
3. app/network layers focused on telemetry, logging, commands, and UI.

Assessment:

- This would be beneficial if UWB debugging, algorithm iteration, and multi-target reuse are expected to continue long-term.
- It would reduce the current mismatch where most observability exists only in wrappers, not in the libraries where several important decisions are actually made.
- It is not the fastest way to unblock the current investigation.

### 7.3 Is a large refactor worth it?

Potentially yes, but not as the first step.

A large refactor would be justified if the team wants:

1. repeatable deep diagnostics from the UWB stack,
2. cleaner separation between reusable algorithms and firmware/platform code,
3. easier unit testing of algorithm diagnostics,
4. and better tooling support in both firmware and `rtls-link-manager`.

However, for the current problem the highest-value approach is staged:

1. Add a small shared diagnostics interface.
2. Expose the most important counters and snapshots over WebSocket/UDP logs.
3. Use that data during hardware investigation.
4. Only then decide whether a broader refactor is worth the churn.

This staged approach gets most of the practical debugging value without immediately paying the cost of a large architectural rewrite.

## 8) Recommended Pre-Hardware Code Improvements

These are not applied by this report, but they are low-risk and would improve live diagnostics:

1. Replace `load()` + `store(0)` on `new_measurement_count` with `exchange(0)`.
2. Avoid blocking indefinitely in `estimatorCallback()` on `measurements_mtx`.
3. Add explicit counters for:
   - tag mutex wait failures / long waits,
   - anchor RX timeout / RX fail events,
   - anchor stall watchdog trips,
   - dropped measurements due to unconfigured anchor IDs.
4. Decide whether the product should support more than 6 anchors in the parameter model. If yes, fix the configuration interface to match the TDoA implementation.
5. Add a remote TDoA-tag diagnostics command that returns a JSON snapshot of:
   - current configured anchors,
   - active measurements,
   - measurement drop reasons,
   - estimator reject reasons,
   - and recent timing / watchdog counters.
6. Add a diagnostic that distinguishes:
   - internal estimator rate,
   - accepted sample rate,
   - exported MAVLink rate.
7. Fix the CLI log parser so it correctly handles the firmware JSON format (`lvl` string instead of numeric `level`).
8. Introduce a shared diagnostics interface for library code instead of wiring `lib/` directly to `src/logging/`.

## 9) Conclusion

The existing SPI-margin hypothesis is still strong, but the firmware also contains real software-side mechanisms that can lower update rate or hide the true source of the drop.

The most important firmware finding is the tag-side blocking mutex in the fresh-measurement path. The most important configuration finding is that locally unconfigured anchors are silently discarded, and the current parameter model only exposes 6 anchors even though the TDoA logic handles 8.

From an observability perspective, the current LAN tooling is useful but not deep enough: it can show device health, configuration, wrapper logs, and some anchor diagnostics, but it cannot yet expose the internal state of the TDoA tag path where several of the most likely software issues live.

The earlier concern about library-level logging was valid in principle. A full refactor could be beneficial long-term, but the best next step is smaller and more targeted: introduce a clean shared diagnostics interface for the UWB libraries and surface its counters/snapshots over the existing network tooling before the next round of hardware investigation.
