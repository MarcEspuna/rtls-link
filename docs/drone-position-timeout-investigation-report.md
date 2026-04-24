# Drone Position Timeout Investigation Report

Date: 2026-03-05

## Scope

This report investigates intermittent position-estimation timeouts observed on some drones running the `rtls-link` firmware.

Observed symptom:

- Some drones consistently show large intermittent gaps in position estimates sent to ArduPilot.
- SPI lines toward the DWM1000 look electrically reasonable from initial scope checks.
- No obvious SPI communication errors have been observed yet.

This investigation focused on firmware-side causes that can produce the same symptom even when SPI is not obviously failing.

## Executive Summary

The issue is likely not explained by a single root cause.

The strongest firmware-side contributors found are:

1. MAVLink position forwarding is heartbeat-gated and can stop entirely if ArduPilot heartbeats are not parsed in time.
2. MAVLink RX servicing is relatively slow for a `921600` baud link and can plausibly starve heartbeat parsing.
3. The TDoA estimator path is fragile to short packet gaps because it requires at least 4 live anchor-pairs and aggressively ages out stale pairs.
4. The upstream TDoA matching and clock-correction logic can suppress measurement generation even while packets are still being received.
5. Anchors have a stall watchdog and resync logic, but the tag does not. A tag-side radio stall can therefore persist longer.
6. On ESP32-S3 boards, DW1000 high-speed SPI runs at `20 MHz`, which is the configured upper-end setting and remains a plausible per-unit hardware margin issue.

The current evidence supports a mixed hypothesis:

- some failures are likely true UWB/radio timing stalls,
- others may be MAVLink output gaps that only look like UWB sampling failures from the ArduPilot side.

## Data Path Reviewed

For TDoA tag mode, the relevant path is:

1. DW1000 IRQ triggers handling in the main Arduino `loop()`.
2. The tag algorithm produces `tdoaMeasurement_t` samples.
3. `estimatorCallback()` stores the most recent sample per anchor-pair.
4. `PosEstimatorTask` runs at 200 Hz and tries to solve position.
5. Valid solutions call `App::SendSample()`.
6. `App::SendSample()` forwards positions to ArduPilot through MAVLink on `Serial1`.

Main references:

- `src/uwb/uwb_tdoa_tag.cpp`
- `lib/tdoa_algorithm/src/tag/tdoa_tag_algorithm.cpp`
- `lib/tdoa_algorithm/src/tag/tdoaEngine.cpp`
- `src/app.cpp`
- `src/mavlink/local_position_sensor.cpp`

## Findings

### 1. MAVLink position output can stop even while the estimator is still healthy

Severity: High

`App::SendSample()` only forwards positions if an ArduPilot heartbeat was received recently:

- `src/app.cpp:658`
- `src/app.hpp:160`

If heartbeat parsing falls behind for more than 3 seconds, valid positions are intentionally dropped. From the flight-controller side, this looks exactly like a position timeout.

This is one of the strongest non-UWB explanations for the observed symptom.

### 2. Heartbeat receive servicing is weak for a 921600 baud UART

Severity: High

The app task runs at 10 Hz:

- `src/main.cpp:37`
- `src/main.cpp:133`

Per cycle it reads at most 1024 bytes from `Serial1` before parsing:

- `src/app.cpp:145`
- `src/app.cpp:257`

That is only about 10 KB/s drained from a much faster serial link. If the FC sends a busier MAVLink stream than expected, RX backlog can accumulate and heartbeat parsing can lag enough to trip the 3-second heartbeat gate.

Implication:

- You may be seeing a MAVLink RX starvation problem, not only a UWB sampling problem.

### 3. Position transmission is synchronous and unqueued

Severity: Medium

MAVLink output is sent inline:

- `src/mavlink/local_position_sensor.cpp:34`
- `src/mavlink/uart_comm.cpp:5`

There is no dedicated TX queue or output task. If UART write blocks or backs up, the estimator task itself is delayed.

Implication:

- UART backpressure can directly inject jitter into the position output path.

### 4. Send success is ignored for position output

Severity: Medium

`App::SendSample()` does not check whether `send_vision_position_estimate()` actually succeeded before updating `last_sample_timestamp_ms_`:

- `src/app.cpp:679`
- `src/app.cpp:691`
- `src/app.cpp:376`

Implication:

- The system can report that it is "sending positions" even if UART writes are failing or being dropped.

This weakens current telemetry and can hide the real failure mode.

### 5. The estimator requires at least 4 live anchor-pairs and ages them out aggressively

Severity: High

The tag stores only the latest measurement per anchor-pair:

- `src/uwb/uwb_tdoa_tag.cpp:398`
- `src/uwb/uwb_tdoa_tag.cpp:414`

It removes pairs older than 350 ms:

- `src/uwb/uwb_tdoa_tag.cpp:463`
- `src/uwb/uwb_tdoa_tag.cpp:485`

It refuses to solve unless at least 4 measurements remain:

- `src/uwb/uwb_tdoa_tag.cpp:436`
- `src/uwb/uwb_tdoa_tag.cpp:516`

Implication:

- A short loss of one or two anchor-pairs can create a longer visible outage.
- Output stops completely until enough pair diversity has rebuilt.

This strongly matches intermittent timeout behavior.

### 6. Upstream TDoA measurement generation is tightly gated

Severity: High

Two mechanisms can suppress measurement generation before the firmware estimator even sees data:

1. Clock-correction reliability gating:
   - `lib/tdoa_algorithm/src/tag/tdoaEngine.cpp:255`
   - `lib/tdoa_algorithm/src/tag/clockCorrectionEngine.cpp:112`

2. Short remote-anchor matching lifetime:
   - `lib/tdoa_algorithm/src/tag/tdoaStorage.cpp:39`
   - `lib/tdoa_algorithm/src/tag/tdoaEngine.cpp:230`

Remote metadata expires after 30 ms. If matching fails or clock correction is not accepted, packets may still be arriving but no `tdoaMeasurement_t` is emitted.

Implication:

- Sampling gaps can originate inside the TDoA algorithm even without obvious SPI errors.

### 7. Anchor-side stalls are explicitly expected and recovered, but only after a visible gap

Severity: High

Anchors have a stall watchdog:

- `src/uwb/uwb_tdoa_anchor.hpp:37`
- `src/uwb/uwb_tdoa_anchor.cpp:192`

The anchor algorithm uses delayed RX/TX scheduling:

- `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:309`
- `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:375`

If scheduling slips or the radio stalls, recovery happens only after a silent period.

Implication:

- A real anchor-side timing stall can reduce sample cadence across all tags.
- This can happen with no obvious SPI bus error signature.

### 8. The tag has no equivalent stall watchdog

Severity: High

Tag update path:

- `src/uwb/uwb_tdoa_tag.cpp:295`

Anchor update path with watchdog:

- `src/uwb/uwb_tdoa_anchor.cpp:192`

The tag does not implement the same explicit stall recovery logic.

Implication:

- If a tag-side DW1000 or event path stalls, the tag may remain degraded much longer than an anchor.
- This is a good match for "some drones are consistently worse than others".

### 9. RX-failed interrupt visibility is weaker than intended

Severity: Medium

RX-failed callbacks are attached:

- `src/uwb/uwb_tdoa_tag.cpp:225`
- `src/uwb/uwb_tdoa_anchor.cpp:123`

But the default driver configuration disables RX-failed interrupts:

- `lib/libdw1000/src/libdw1000.c:633`

Implication:

- Some receive-side failures may surface only as timeouts or later watchdog recovery, reducing observability.

This is especially relevant for debugging marginal units.

### 10. Per-device configuration drift can silently reduce valid measurements

Severity: Medium

The parameter model supports only 6 configured anchors:

- `src/uwb/uwb_params.hpp:113`

But the TDoA logic accepts IDs `0..7` and drops measurements from anchors not configured locally:

- `src/uwb/uwb_tdoa_tag.cpp:390`

Implication:

- If affected drones have different stored configs, they can silently discard good measurements.
- This is a plausible explanation for unit-to-unit differences.

### 11. ESP32-S3 SPI speed remains a credible hardware-margin contributor

Severity: Medium to High

ESP32-S3 board:

- `src/bsp/konex_uwb_v1/board.hpp:20`

uses `20 MHz` fast SPI.

ESP32 board:

- `src/bsp/makerfabs_uwb/board.hpp:18`

uses `16 MHz`.

Implication:

- Even if scope traces look reasonable, 20 MHz can still be marginal on some boards due to routing, assembly, module tolerance, or edge quality.
- This remains a valid hardware-side hypothesis and should not be dismissed.

### 12. Possible UART port mismatch should be verified on hardware

Severity: Open question

Position output currently uses `Serial1` on `uwb_data_uart`:

- `src/app.cpp:53`
- `src/app.cpp:713`

But board definitions label a separate `mavlink_uart`:

- `src/bsp/konex_uwb_v1/board.hpp:23`
- `src/bsp/makerfabs_uwb/board.hpp:21`

and the WiFi UART bridge uses that other port:

- `src/wifi/wifi_uart_bridge.cpp:19`

This may be intentional, but it should be verified against the actual PCB wiring and FC connection.

Implication:

- If some drones are wired expecting MAVLink on the documented MAVLink UART while firmware emits on the other UART, the symptom can appear as a total output timeout.

## Likely Root-Cause Buckets

Ranked by current confidence:

1. MAVLink heartbeat starvation or RX backlog causing intentional position suppression.
2. Estimator/sample starvation caused by pair aging, matching limits, or clock-correction rejection.
3. Real anchor/tag radio timing stalls that do not show up as obvious SPI faults.
4. ESP32-S3 SPI margin differences between units.
5. Per-device parameter/configuration drift.

## Recommendations

### Immediate diagnostic recommendations

1. Distinguish estimator failure from MAVLink forwarding failure.
   Add or expose:
   - heartbeat age,
   - count of dropped positions due to stale heartbeat,
   - MAVLink TX success/failure,
   - live anchor-pair count after stale prune,
   - insufficient-measurement reject count,
   - RMSE reject count,
   - tag-side and anchor-side stall counters.

2. Compare good and bad drone configs.
   Check at least:
   - anchor list/count,
   - `dwMode`,
   - channel,
   - TDMA slot settings,
   - TX power / smart power,
   - antenna delay,
   - any per-drone persisted parameter differences.

3. Verify the actual UART wiring to the flight controller.
   Confirm that the FC is connected to the UART the firmware really uses for position output.

### Fastest hardware A/B tests

1. Reduce ESP32-S3 fast SPI from `20 MHz` to `16 MHz` on affected drones and compare update stability.
2. Compare affected and unaffected drones under identical anchor layout and FC MAVLink stream load.
3. Observe whether position drops correlate with heartbeat age growth or with estimator measurement-count collapse.

### Firmware improvement recommendations

1. Add a tag-side UWB stall watchdog similar to the anchor watchdog.
2. Remove or relax heartbeat gating during debugging, or at minimum log every dropped sample due to stale heartbeat.
3. Improve MAVLink RX servicing:
   - run more often,
   - drain more bytes per cycle,
   - or move RX parsing to a dedicated task.
4. Queue MAVLink TX instead of writing synchronously from the estimator path.
5. Check and propagate send failures instead of always updating `last_sample_timestamp_ms_`.
6. Improve observability for RX-failed events and radio recovery.
7. Revisit estimator robustness:
   - pair aging threshold,
   - minimum-measurement policy,
   - matching window,
   - and rejection counters.
8. Revisit the anchor parameter model so configuration and runtime behavior agree on supported anchor count.

## Suggested Live Debug Order

To get the highest signal quickly:

1. On an affected drone, log heartbeat age and position-drop reason.
2. Confirm whether the estimator is still solving while MAVLink forwarding is stopped.
3. A/B test lower SPI speed on the same unit.
4. Compare stored params from a good drone and a bad drone.
5. Add stall counters on both anchor and tag paths.

## Conclusion

Current evidence does not support treating this as only a PCB/SPI problem.

There are multiple firmware mechanisms that can produce the same externally visible symptom:

- intentional MAVLink output suppression,
- silent estimator starvation,
- delayed radio recovery,
- and weak observability of actual send failures.

The most important next step is to instrument the system so that each dropout is classified as one of:

- `heartbeat-gated output stop`,
- `UART send failure/backpressure`,
- `insufficient live TDoA pairs`,
- `RMSE/solver rejection`,
- `anchor stall`,
- `tag stall`.

Once that classification exists, the issue should narrow quickly.
