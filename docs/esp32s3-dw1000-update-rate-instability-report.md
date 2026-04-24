# ESP32-S3 TDoA Update-Rate Instability: DW1000 Deep-Dive Report

Date: 2026-02-15  
Scope: `esp32s3_application` target (Konex UWB board), TDoA modes based on `libdw1000` + `tdoa_algorithm`.

## 1) Observed Problem

Some UWB minidrones show unstable update rate while others are stable under the same firmware family. Oscilloscope checks indicate very fast SPI clock edges; signal shape appears similar across units, but timing edges are difficult to resolve clearly.

This report analyzes firmware behavior plus DW1000/DWM1000 references to identify likely root causes.

## 2) Executive Summary

Most likely primary cause is SPI timing/signal-integrity margin on ESP32-S3 units running DW1000 at 20 MHz high-speed SPI (the configured maximum). Unit-to-unit PCB/component variation can make only a subset unstable.

Secondary contributors:
- Delayed TX/RX scheduling edge cases in TDoA anchor TDMA flow can produce intermittent stalls.
- Current interrupt mask defaults disable RX-failed interrupt while callbacks are attached, reducing visibility/recovery granularity in marginal conditions.
- Per-device persisted parameter differences can amplify instability if radio settings differ across units.

## 3) Firmware Evidence (Code Audit)

### 3.1 SPI clocks differ between ESP32-S3 and ESP32

- ESP32-S3 board config:  
  - `src/bsp/konex_uwb_v1/board.hpp:20` -> `slow_spi_clk_hz = 1400000`  
  - `src/bsp/konex_uwb_v1/board.hpp:21` -> `fast_spi_clk_hz = 20000000`
- ESP32 board config:  
  - `src/bsp/makerfabs_uwb/board.hpp:18` -> `slow_spi_clk_hz = 2000000`  
  - `src/bsp/makerfabs_uwb/board.hpp:19` -> `fast_spi_clk_hz = 16000000`

Interpretation: ESP32-S3 path runs the DW1000 SPI bus faster than ESP32 (20 MHz vs 16 MHz).

### 3.2 UWB driver uses board SPI speeds directly

- `src/uwb/uwb_ops.hpp:27-30` creates low/high `SPISettings` from board config.
- `src/uwb/uwb_ops.cpp:57-62` switches SPI speed index at runtime via `SpiSetSpeed`.

No extra runtime guard/margin reduction is applied for ESP32-S3 boards.

### 3.3 libdw1000 speed contract and clock switching

- `lib/libdw1000/src/libdw1000Types.h:117-120` documents:
  - low speed <= 4 MHz
  - high speed <= 20 MHz
- `lib/libdw1000/src/libdw1000.c:221-233` (`dwEnableClock`):
  - uses low speed for Auto/XTI
  - uses high speed for PLL

Interpretation: firmware behavior is aligned with the driver model, but ESP32-S3 is configured at the upper bound.

### 3.4 TDoA anchor path includes explicit stall watchdog/reinit

- `src/uwb/uwb_tdoa_anchor.hpp:37` defines `STALL_TIMEOUT_MS = 150`.
- `src/uwb/uwb_tdoa_anchor.cpp:196-205` reinitializes algorithm when no interrupts are seen for >150 ms.

Interpretation: if radio events intermittently stall, user-visible update rate will dip/spike rather than hard-fail.

### 3.5 RX-failed callback attached, but RX-failed interrupt masked off in defaults

- Callbacks are attached in wrappers:
  - anchor: `src/uwb/uwb_tdoa_anchor.cpp:126`
  - tag: `src/uwb/uwb_tdoa_tag.cpp:228`
- Defaults disable RX-failed IRQ mask:
  - `lib/libdw1000/src/libdw1000.c:636` -> `dwInterruptOnReceiveFailed(dev, false);`

Interpretation: receive failure handling still exists in `dwHandleInterrupt`, but masked IRQ behavior can reduce event visibility/reactivity under noisy/marginal conditions.

### 3.6 TDoA anchor algorithm relies heavily on delayed RX/TX scheduling

- Delayed RX setup: `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:309-325`
- Delayed TX setup: `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:375-391`
- 512-tick alignment enforced: `lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp:78-94`

Interpretation: when schedule timing is near the deadline, delayed TX can fail to start (known DW1000 behavior in practice), producing intermittent slot disruption.

### 3.7 Config persistence can create unit-to-unit behavior differences

- Runtime radio params are loaded from LittleFS (`/params.txt`):
  - loader: `src/littlefs_frontend.hpp:91-166`
  - TDoA anchor consumes params at init: `src/uwb/uwb_tdoa_anchor.cpp:131-162`
  - TDoA tag consumes params at init: `src/uwb/uwb_tdoa_tag.cpp:233-247`
- Relevant knobs include `uwb.channel`, `uwb.dwMode`, `uwb.txPowerLevel`, `uwb.smartPowerEnable`, `uwb.tdoaSlotCount`, `uwb.tdoaSlotDurationUs` (`src/uwb/uwb_params.hpp:95-103`).

Interpretation: two devices on "same firmware" can still run different UWB radio/TDMA parameters if their persisted config differs.

## 4) External DW1000/DWM1000 Reference Findings

### 4.1 SPI limits and startup constraint

From DW1000 datasheet references (see Sources):
- Max SPI frequency in high-speed operation: 20 MHz.
- During low-speed/startup phases (until crystal/PLL path is ready), SPI must remain low speed (commonly documented around 3 MHz ceiling).

Why relevant: ESP32-S3 is configured at 20 MHz high-speed, exactly at maximum. Any SI margin reduction can disproportionately impact some units.

### 4.2 Qorvo forum guidance on practical SPI margin

Qorvo support discussions include:
- Startup/init communication should be done at low SPI speed (around 3 MHz guidance).
- Some users report improved reliability when reducing high-speed SPI below nominal max.

Why relevant: this aligns with field symptoms where only some boards are unstable.

### 4.3 Qorvo forum guidance on delayed TX failures

Qorvo discussions describe delayed TX failure modes when scheduling is too close to the current time (`HPDWARN`/`TXPUTE` patterns in Decawave API context).

Why relevant: the anchor TDMA flow in this firmware schedules delayed TX/RX continuously. Any timing jitter or recovery lag can trigger intermittent slot misses/stalls.

### 4.4 DW1000 clock/PLL lock-loss indicators exist in this stack

- Driver checks `CLKPLL_LL_BIT` and `RFPLL_LL_BIT`:
  - `lib/libdw1000/src/libdw1000.c:793-800`
- Interrupt handler can call error callback on clock problem:
  - `lib/libdw1000/src/libdw1000.c:1271-1273`

Why relevant: PLL/clock lock issues can present as intermittent update instability.

## 5) Ranked Root-Cause Hypotheses

1. SPI SI/timing margin at 20 MHz on ESP32-S3 (highest confidence)
- Confidence: high.
- Explains "only some units", "edges too fast", and unstable rate without universal failure.

2. Delayed TX/RX scheduling edge misses in anchor TDMA loop
- Confidence: medium-high.
- Explains intermittent stalls and watchdog-based reinit behavior.

3. Reduced failure visibility due to RX-failed IRQ mask default
- Confidence: medium.
- Not necessarily root cause alone, but can worsen recovery behavior under marginal RF/SI conditions.

4. Persisted per-device UWB parameter drift
- Confidence: medium.
- Can produce large behavior differences with same binary image.

## 6) Practical Verification Plan (Recommended Order)

1. Compare bad vs good device configs (`readall uwb`), especially:
- `uwb.dwMode`
- `uwb.channel`
- `uwb.tdoaSlotCount`
- `uwb.tdoaSlotDurationUs`
- `uwb.txPowerLevel`
- `uwb.smartPowerEnable`

2. A/B firmware test with lower ESP32-S3 fast SPI:
- test 16 MHz first, then 14 MHz or 12 MHz if needed.
- keep all other parameters unchanged.

3. Correlate update-rate dips with stall logs:
- monitor `UWB stall detected ... reinitializing` from `src/uwb/uwb_tdoa_anchor.cpp:197`.

4. Scope quality comparison (good vs bad unit):
- same probe grounding and bandwidth settings.
- check for overshoot/ringing/settling on SCK and MOSI at high speed.

5. Power/clock integrity check near UWB section:
- DW1000 supply rail quality and decoupling.
- reset line behavior.
- any board assembly variation.

## 7) Immediate Mitigation Candidates

No code changes are applied in this report, but low-risk mitigations are:

1. Lower ESP32-S3 fast SPI default from 20 MHz to 16 MHz for validation.
2. Add runtime diagnostics counters for:
- stall watchdog trips
- RX timeout/fail counts
- clock/PLL lock-loss events
3. Revisit RX-failed interrupt mask policy for TDoA modes.
4. Enforce/verify consistent UWB params across deployed units before field testing.

## 8) Notes on Evidence Quality

- Firmware code references above are primary evidence from this repository.
- Qorvo datasheet/user-manual/errata indexes are authoritative for limits and official guidance.
- Qorvo forum threads are valuable engineering guidance but should be treated as advisory/field experience, not normative specification text.

## 9) Sources

Primary vendor pages:
- DW1000 product and documents index: https://www.qorvo.com/products/p/DW1000
- DWM1000 product and documents index: https://www.qorvo.com/products/p/DWM1000

Direct Qorvo DW1000 documents:
- DW1000 User Manual, v2.18 (Mar 2024): https://www.qorvo.com/products/d/da008448
- DW1000 Data Sheet, v2.22 (May 2024): https://www.qorvo.com/products/d/da008149
- DW1000 Errata, v1.4 (Jun 2024): https://www.qorvo.com/products/d/da008865
- APS022, DW1000 Debugging Notes, v1.4 (Feb 2024): https://www.qorvo.com/products/d/da008446

Datasheet mirrors used for searchable excerpts:
- DW1000 datasheet mirror (Digi-Key): https://www.digikey.tw/htmldatasheets/production/1827494/0/0/1/dw1000-datasheet.html
- DWM1000 datasheet mirror (Digi-Key): https://www.digikey.co.th/htmldatasheets/production/1992581/0/0/1/dwm1000-datasheet.html

Qorvo technical forum references:
- Delayed TX scheduling pitfalls (`HPDWARN`/`TXPUTE` context):  
  https://forum.qorvo.com/t/dwt-starttx-always-returns-error-when-using-delay/4521
- Startup SPI speed guidance and practical high-speed stability discussion:  
  https://forum.qorvo.com/t/some-problems-with-rf-init-and-ldeload/8729

Access date for all external references: 2026-02-15.
