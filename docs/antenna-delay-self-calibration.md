# Anchor Antenna-Delay Self‑Calibration (Using “True” Inter‑Anchor Distances)

## Summary / What we want to achieve

We want a **set of anchors (default: 4, but scalable to 6–8)** to automatically compute new **per‑anchor antenna delay ( `uwb.ADelay` )** values so that the system’s **measured inter‑anchor distances match user‑provided “true” distances** as closely as possible.

The intended workflow is:

1. A user precisely measures the physical distances between anchors (typically the rectangle sides; diagonals can be derived if we assume a rectangular layout).
2. The system collects inter‑anchor UWB ranging measurements.
3. We estimate the per‑anchor antenna delays that best explain the difference between measured and “true” distances.
4. We apply the updated `uwb.ADelay` values **immediately (no reboot)** and validate that corrected distances match the targets.

This feature must be **usable from `rtls-link-manager`** and ideally also from **`rtls-link-cli`**, kept in sync with firmware.

---

## Current state (what exists today)

### 1) Single‑device “calibration mode” exists (TWR + known distance)

Firmware already supports a `UWBMode::CALIBRATION_MODE` backend (`src/uwb/uwb_calibration.cpp`) that:

- Starts the device as a **TWR anchor**.
- Uses a **user‑configured target distance** (`uwb.calDistance`).
- Iteratively updates one device’s antenna delay using a sign‑change / step‑halving loop (binary‑search‑like).
- Stores the resulting value into `uwb.ADelay`.

This mode:

- Calibrates **one device at a time** against an external ranging partner.
- Uses **TWR distance output** (`DW1000Ranging.getRange()`).
- Requires reboot to enter calibration mode; applying the final delay does not currently re‑init the UWB backend.

### 2) In TDoA mode, anchors already broadcast inter‑anchor ranges + their antennaDelay

In TDoA Anchor mode (`lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp`), anchors compute inter‑anchor DS‑TWR “distance” in **DW1000 timestamp units** and broadcast it in the TDoA packet:

- `rangePacket->distances[i]`: raw inter‑anchor ToF estimate (un‑corrected for antenna delays)
- `rangePacket->antennaDelay`: the anchor’s configured `uwb.ADelay` value (broadcast metadata)

Important detail:

- TDoA anchors currently call `dwSetAntenaDelay(..., 0)`, so the radio timestamps are **not corrected in hardware**.
- On the tag side (`src/uwb/uwb_tdoa_tag.cpp`), the code already corrects inter‑anchor ranges like:
  - `corrected_ticks = raw_ticks - adelay_from - adelay_to`

So the firmware already contains a clear “model” for how `ADelay` relates to inter‑anchor distance in TDoA mode.

### 2.1) Anchor IDs in TDoA mode (important for configuration + UI)

For the Bitcraze‑derived TDoA implementation, the “anchor ID” is a small integer `0..7` (8 slots maximum).

In the current firmware wrapper:

- Each anchor’s ID is derived from `uwb.devShortAddr` (digits) and ends up in `ctx.anchorId` inside the algorithm.
- Dynamic anchor positioning and other 4‑anchor utilities assume IDs `0..3`.

**Practical rule for the default 4‑anchor setup:** configure anchors as IDs `0`, `1`, `2`, `3` (set `uwb.devShortAddr` accordingly).

### 3) Device configuration and control is via WebSocket text commands

Devices expose a WebSocket endpoint and a SimpleCLI‑backed command set (`src/command_handler/command_handler.cpp`):

- `write -group uwb -name ADelay -data "<value>"`
- `read ...`, `readall ...`, `reboot`, etc.

`rtls-link-manager` and `rtls-link-cli` build these commands from shared Rust code (`tools/rtls-link-manager/crates/rtls-link-core/src/protocol/commands.rs`).

---

## Problem definition (formal)

### Inputs we will have

- A set of anchors `i = 0..N-1` (default `N=4`).
- For a subset of pairs `(i, j)` we know a user‑provided **true distance** `D_true(i,j)` in meters.
- From the running system we can obtain **measured raw inter‑anchor ToF values** `T_raw(i,j)` in DW1000 timestamp units (ticks). In the current TDoA implementation, these are `packet->distances[...]`.

### Output we want

- A per‑anchor antenna delay vector `a[i]` in DW1000 ticks (stored into `uwb.ADelay` on each anchor).

### What “matching distances” means in this system

In the current TDoA distance correction logic, the corrected tick distance is:

```
T_corr(i,j) = T_raw(i,j) - a[i] - a[j]
```

Converted to meters:

```
D_corr(i,j) = T_corr(i,j) * DW1000_TIME_TO_METERS
```

So the target condition is:

```
D_corr(i,j)  ≈  D_true(i,j)
```

Equivalently in ticks:

```
a[i] + a[j]  ≈  T_raw(i,j) - T_true(i,j)
```

where:

```
T_true(i,j) = D_true(i,j) / DW1000_TIME_TO_METERS
```

This becomes a small, well‑structured estimation problem.

---

## “True distance” configuration for the default 4‑anchor rectangular layout

We already have a firmware concept of rectangular 4‑anchor layouts via `UWBParams::anchorLayout` (see `AnchorLayout` in `src/uwb/uwb_params.hpp`).

For the default 4‑anchor rectangle, the simplest user input is:

- `dX`: distance from A0 to the anchor that defines +X
- `dY`: distance from A0 to the anchor that defines +Y
- (optional) `anchorHeight`: if we want 3D distances, incorporate Z; otherwise assume all anchors are at the same height

Given `dX` and `dY`, we derive:

- `dDiag = sqrt(dX^2 + dY^2)` (A0 to the opposite corner)

Then the full set of “true” pair distances is determined by the chosen layout mapping:

- **Layout 0 (`RECTANGULAR_A1X_A3Y`)**: +X=A1, +Y=A3, corner=A2
  - d(0,1)=dX, d(0,3)=dY, d(0,2)=dDiag
  - d(1,2)=dY, d(3,2)=dX, d(1,3)=dDiag

- **Layout 1 (`RECTANGULAR_A1X_A2Y`)**: +X=A1, +Y=A2, corner=A3
  - d(0,1)=dX, d(0,2)=dY, d(0,3)=dDiag
  - d(1,3)=dY, d(2,3)=dX, d(1,2)=dDiag

- **Layout 2 (`RECTANGULAR_A3X_A1Y`)**: +X=A3, +Y=A1, corner=A2
  - same geometry as layout 0, just swapped axis anchors

- **Layout 3 (`RECTANGULAR_A2X_A3Y`)**: +X=A2, +Y=A3, corner=A1

This derived diagonal is what makes the estimation problem well‑posed for 4 anchors (adds an odd cycle).

If we do not want to assume a perfect rectangle, we must allow the user to input at least one diagonal or provide an “advanced mode” with explicit per‑pair distances.

## Scaling beyond 4 anchors (N > 4)

The same estimator generalizes to more anchors as long as we provide enough “true distance” constraints and enough measured pairs.

Recommended “true distance” sources for `N > 4`:

1. **Anchor coordinates (preferred):** if the user configures each anchor’s `(x,y,z)` accurately, we can derive all `D_true(i,j)` as Euclidean distances. This is the most flexible and avoids layout‑specific assumptions.
2. **Explicit pair distances:** allow the user to enter a list of `(i,j, D_true)` edges (graph).

Estimator requirements (for a well‑posed, non‑ambiguous solution):

- The constraint graph should be connected.
- To avoid the “bipartite ambiguity” in sum‑constraints, it should contain at least one odd cycle *or* we must apply a gauge fix (reference anchor) / regularization.

In practice:

- With many anchors and many observed pairs, weighted least squares with mild regularization works well even when some edges are missing.

## Algorithm research: estimating per‑anchor antenna delays

### Key observation: this is (approximately) a linear system

For each anchor pair `(i,j)` for which we have a reliable `T_raw(i,j)` and a `D_true(i,j)`, we get one linear equation:

```
a[i] + a[j] = b(i,j)
```

with:

```
b(i,j) = T_raw(i,j) - D_true(i,j)/DW1000_TIME_TO_METERS
```

This is a “sum‑on‑edges” graph problem.

### Uniqueness / observability (important!)

The system `a[i] + a[j] = b(i,j)` is:

- **Uniquely solvable** (in the noise‑free case) if the measurement graph contains an **odd cycle** (i.e., the graph is **not bipartite**).
- **Underdetermined** by one degree of freedom if the measurement graph is **connected and bipartite** (classic example: a 4‑anchor rectangle where you only include the 4 perimeter edges).

Implication for the 4‑anchor “rectangle” default:

- If we only use the four sides `(0-1, 1-2, 2-3, 3-0)`, the system is bipartite ⇒ **not uniquely solvable** without an additional constraint.

Ways to resolve this (rank‑fix):

1) **Include at least one diagonal pair** (adds a triangle ⇒ odd cycle ⇒ unique solution).
   - The user does not want to measure diagonals, but we can:
     - derive diagonals from measured sides **if** we assume a rectangular geometry, or
     - ask the user to measure one diagonal as a validation step.
2) **Fix a reference**:
   - e.g., hold `a[0]` constant and solve others relative to it.
3) **Regularize toward current values**:
   - minimize both fit error and deviation from current stored `ADelay`.

Recommendation for our project:

- For the **default 4‑anchor rectangular layout**, use **two measured side lengths** plus layout mapping to derive the full set of required “true” pair distances (including diagonals). This yields a non‑bipartite constraint set and avoids an arbitrary gauge.
- Also provide a fallback mode: if the user supplies only a bipartite constraint set, we solve with **regularization** (documented and clearly reported as “relative calibration”).

### Estimator choice (least squares, robustified)

Because real measurements are noisy and sometimes biased (multipath/NLOS), we should not treat equations as exact. We should:

1) **Average** many `T_raw(i,j)` samples per pair (to reduce random noise).
2) **Reject outliers** or use robust statistics:
   - median, trimmed mean, or median‑of‑means across time windows.
3) Solve a **(weighted) least squares** problem:

Minimize:

```
Σ w(i,j) * (a[i] + a[j] - b(i,j))^2
```

Optionally add regularization:

```
+ λ * Σ (a[i] - a0[i])^2
```

where:

- `w(i,j)` can be based on sample count or variance for that pair.
- `a0[i]` is the current stored delay (acts as a “prior”).
- `λ` controls how strongly we keep the solution near existing calibration.

This formulation:

- Handles noise naturally.
- Provides a stable solution even if the constraint graph is weak (regularization adds rank).

### Practical data collection (what we actually average)

For each pair `(i,j)` we want a robust estimate of `T_raw(i,j)` in ticks.

Implementation notes based on the current stack:

- In TDoA, we can observe `packet->distances[j]` from anchor `i` packets on the tag.
- We will see both directions over time (`i→j` and `j→i`). We can:
  - treat them as separate observations of the same undirected pair and combine, or
  - keep directional statistics for debugging (useful when diagnosing a “bad” anchor).

To reduce outlier impact, a simple and effective approach is:

- accumulate samples into windows (for example 20 samples),
- compute a per‑window mean,
- take the median of window means as the final `T_raw(i,j)`.

This is robust and cheap, and works well in multipath‑heavy environments.

### Closed‑form fast path for complete graphs (optional optimization)

If we have all pairs for `N >= 3` (complete graph) and equal weights, there is a closed‑form least‑squares solution:

Let:

- `b(i,j)` for all `i < j`
- `B_i = Σ_{j≠i} b(i,j)`
- `B_total = Σ_{i<j} b(i,j)`

Then:

```
S = B_total / (N-1)
a[i] = (B_i - S) / (N-2)
```

For `N=4`: `a[i] = (B_i - B_total/3) / 2`.

We can keep this as an implementation shortcut, but a small generic solver is simple and more flexible once we add weights/regularization.

---

## Where should the algorithm run? (device vs desktop)

### Option A — Desktop‑side solver (recommended MVP)

**Measurement source:** preferably the TDoA anchors themselves (queried over Wi‑Fi); alternatively a TDoA tag can act as a passive observer.  
**Estimator:** runs in `rtls-link-manager` / `rtls-link-cli` (Rust).  
**Apply:** manager/CLI writes `uwb.ADelay` to each anchor and triggers “apply now”.

Pros:

- Easy to iterate/debug the estimator (log/plot residuals, robust stats).
- Single shared implementation in `rtls-link-core` used by manager + CLI.
- Firmware changes are mostly about exposing measurement stats + applying `ADelay` without reboot.
- Does **not require** a dedicated tag device if anchors can report their inter‑anchor distances.

Cons:

- Requires a desktop coordinator (manager/CLI) for the calibration procedure.
- Needs a telemetry/command to read inter‑anchor measurements from anchors (or from a tag if used as observer).

### Option B — Device‑side solver (future/headless mode)

**Measurement source:** tag device.  
**Estimator:** runs on the tag firmware.  
**Apply:** still easiest via desktop (write to anchors), unless we implement anchor‑to‑anchor update protocol.

Pros:

- Minimal desktop complexity; tools just start/monitor.
- Potentially higher update rate and simpler streaming (send only final solution).

Cons:

- More firmware complexity (robust solver + data structures).
- Harder to debug and evolve.

### Option C — Fully autonomous self‑calibration (future)

Tag computes solution and pushes it to anchors over Wi‑Fi/UWB automatically.

This is the purest “self‑calibration”, but has significant design/security implications.

**Recommendation:** implement **Option A** first, keep Option B/C as follow‑ups.

---

## Validation and iteration strategy

### Important nuance: in current TDoA anchor implementation, changing `ADelay` does NOT change `T_raw`

In TDoA anchor mode, `dwSetAntenaDelay(0)` is used and the inter‑anchor `packet->distances[]` are computed purely from timestamps. The reported `antennaDelay` is metadata.

Therefore:

- Applying a new `ADelay` changes the *correction* (`T_corr`) but not the raw measurement (`T_raw`).
- Iteration is still useful as **a statistical refinement**:
  - collect more samples,
  - detect drift/outliers,
  - recompute the solution,
  - confirm stability.

If we later decide to apply antenna delays in hardware during ranging (TWR mode), then iterative “apply → re‑measure → re‑fit” becomes much more meaningful.

**Clarification:** the DW1000 *does* support writing antenna delay registers (TX_ANTD/RXANTD) and doing so can affect timestamps immediately. However, this particular TDoA implementation intentionally sets the DW1000 antenna delay to 0 and uses scheduled TX times in the protocol. Switching to non‑zero hardware antenna delays in TDoA mode would require revisiting the timestamp semantics (to avoid breaking TDMA alignment / DS‑TWR math and to avoid double‑correction).

### Proposed acceptance metrics

After computing candidate delays `a[i]`, compute residuals:

```
e(i,j) = D_corr(i,j) - D_true(i,j)
```

Report:

- max(|e|)
- RMS(e)
- per‑pair errors (for diagnostics)

Acceptance thresholds (tunable; starting points):

- `max(|e|) <= 0.05 m` (5 cm) for indoor rectangular 4‑anchor setups
- `RMS(e) <= 0.02 m` (2 cm)

### Sanity constraints on the estimated delays

`uwb.ADelay` is a 16‑bit value in DW1000 ticks. Typical values in this codebase are around ~16k.

To avoid applying nonsense results we should enforce:

- hard bounds: `0 <= a[i] <= 65535`
- conservative bounds (recommended for automatic apply): e.g. `14000..22000`
- step bounds relative to current values (recommended): e.g. `|a[i] - a0[i]| <= 1000` unless the user explicitly overrides

When bounds are hit, tools should refuse to auto‑apply and instead show diagnostics (per‑pair residuals + raw inputs).

### Iterative loop (MVP)

1. Reset accumulators on tag.
2. Collect until each required pair has `K` samples (e.g., 200–500).
3. Solve for `a[i]`.
4. Apply to anchors, then re‑collect for a shorter window (e.g., 100 samples) and compute residuals.
5. If residuals unstable or above threshold:
   - increase sample window, or
   - enable more aggressive outlier rejection, and re‑solve.

---

## Implementation plan (deep, end‑to‑end)

### Phase 0 — Define user‑visible workflow and constraints

Decisions to lock:

- Are we calibrating **only anchors** or also tags?
  - MVP: anchors only.
- Do we require a **TDoA tag device** to act as measurement observer?
  - MVP: **no** (anchors already compute inter‑anchor distances in TDoA mode). A tag can still be supported as an optional passive observer.
- What is the “true distance” input format?
  - MVP: rectangular 4‑anchor helper (`dX`, `dY`, layout mapping).
  - Also allow “advanced mode”: explicit per‑pair distances.

### Phase 1 — Firmware: expose inter‑anchor distance statistics

Goal: allow tools to fetch stable, averaged `T_raw(i,j)` values + sample counts **without requiring a tag**.

Implementation outline:

1) **Preferred (anchor‑only): add a reporting path for TDoA anchor inter‑anchor distances**

In TDoA anchor mode, each anchor already computes `ctx.distances[]` (raw ToF ticks to peers). We should expose this over the existing WebSocket command channel:

- Add a small API in `lib/tdoa_algorithm` to retrieve a snapshot of:
  - per‑peer raw ticks (the current `distances[]`)
  - freshness info (last update time or last seen pid), if available
  - (optional) simple accumulation (sum/count) inside the algorithm or wrapper for better stability

- Add new device commands (JSON):
  - `uwb-ia-reset` (optional if we maintain accumulators)
  - `uwb-ia-report` (JSON) — returns this anchor’s per‑peer raw ticks + any stats

Then `rtls-link-manager`/CLI polls all anchors and combines the two directed observations into one undirected pair estimate.

2) **Alternative (tag as observer): add a small accumulator on the TDoA tag backend**

If we also want a passive “observer mode”, we can implement an accumulator on the **TDoA tag backend** (`src/uwb/uwb_tdoa_tag.cpp/.hpp`) that aggregates `packet->distances[]` as it receives anchor packets.

- Keep per‑pair accumulators for raw ticks:
  - sum of ticks, count
  - optional: sum of squares (variance estimate)
  - last update timestamp
- Store per‑anchor reported antennaDelay from packets (already in `lib/tdoa_algorithm` as `uwbTdoa2TagGetAnchorAntennaDelay()`).

3) Provide WebSocket commands (via `src/command_handler/command_handler.cpp`) that return JSON:

- `uwb-ia-reset`  
  Resets accumulators on the tag.

- `uwb-ia-status` (JSON)  
  Returns counts per pair and “ready” status.

- `uwb-ia-report` (JSON)  
  Returns averaged raw ticks `T_raw(i,j)` per pair (and counts / variance).

Notes:

- Keep the JSON small and stable (tools depend on it).
- Prefer returning **DW1000 ticks** (integers) to avoid float precision issues.
- For faster convergence, consider temporarily setting `uwb.tdoaSlotCount` to the active anchor count (e.g. 4) during calibration to increase the effective update rate per pair.

4) Update Rust command builders in:

- `tools/rtls-link-manager/crates/rtls-link-core/src/protocol/commands.rs`
- `tools/rtls-link-cli/src/protocol/commands.rs` (if still used separately)

so manager/CLI can call these.

### Phase 2 — Tools: implement the solver + UX (Manager + CLI)

Goal: compute `a[i]` from (`T_raw`, `D_true`) robustly and present/apply it.

#### 2.1) Shared estimator in Rust (`rtls-link-core`)

Add a “calibration” module to `rtls-link-core` implementing:

- Input: anchor IDs, true distances (meters), measured raw ticks (integers), optional weights
- Output: estimated delays (integers), residual metrics

Implementation details:

- Convert `D_true` to `T_true` using the same constant as firmware (`DW1000_TIME_TO_METERS`).
- Build equations for all available pairs and solve weighted least squares.
- Add optional regularization toward current `ADelay`.
- Add unit tests with synthetic data + noise + missing edges.

#### 2.2) `rtls-link-manager` UI workflow

Add a calibration screen/wizard:

1. Select coordinator device (must be `tag_tdoa`)
2. Select the anchor set (discovered devices in `anchor_tdoa`)
3. Input “true distances”
   - Rectangular helper: `dX`, `dY`, layout mapping, optional height
   - Advanced: explicit per‑pair distances
4. Collect & show progress (`counts per pair`)
5. Solve and present:
   - computed `ADelay` per anchor
   - expected per‑pair residuals
6. Apply:
   - write new `uwb.ADelay` to each anchor
   - trigger “apply now”
7. Verify:
   - re‑collect + show achieved residuals
8. Save/export calibration report (optional but very useful in practice)

#### 2.3) `rtls-link-cli` command

Add a command such as:

```
rtls-link-cli uwb adelay-calib --tag <ip> --anchors <ip,ip,..> --layout <0..3> --dx <m> --dy <m> [--samples 300] [--apply] [--verify]
```

Where:

- `--apply` writes the new delays to anchors.
- `--verify` runs a short post‑apply validation.
- `--dry-run` prints computed delays without changing devices.

### Phase 3 — Firmware: apply `ADelay` immediately (no reboot)

Today, `ADelay` is mainly applied at backend construction (TWR) or just broadcast (TDoA anchor).

We want a command that:

- persists the value (already done by `write` command to LittleFS)
- **also applies it to the running UWB backend**, depending on mode:

Proposed approach:

1) Add a virtual hook in `UWBBackend` (or a dedicated interface):

- `virtual bool ApplyAntennaDelay(uint16_t delay)` (default false)

2) Implement per backend:

- `UWBAnchor` (TWR): call `DW1000.setAntennaDelay(delay)` directly.
- `UWBAnchorTDoA`: update the broadcast `antennaDelay` value used in TDoA packets.
  - This likely requires either:
    - adding a setter into `lib/tdoa_algorithm` to update `ctx.antennaDelay`, or
    - re‑calling `uwbTdoa2Algorithm.init(&m_UwbConfig, &m_Device)` (careful: this resets TDMA state).
- `UWBCalibration`: already adjusts delay while running.

3) Add a command:

- `uwb-apply-adelay` (or similar), which:
  - reads the current stored `uwb.ADelay`
  - calls `ApplyAntennaDelay()` on the active backend
  - returns success/failure text or JSON

### Phase 4 (optional but recommended) — Use calibrated delays inside TDoA engine

Right now, TDoA processing stores raw inter‑anchor ToF values (`packet->distances[]`) without subtracting antenna delays.

If we want TDoA positioning to benefit from `ADelay`, we should consider correcting the ToF values before they are fed into the engine:

- In `lib/tdoa_algorithm/src/tag/tdoa_tag_algorithm.cpp`, when storing remote ToF:
  - `tof_corrected = tof_raw - adelay(sender) - adelay(remote)`

This needs careful validation (it changes estimator behavior), but it is the logical place where calibrated antenna delays can improve TDoA.

**Next feature:** once the rectangle geometry is known (perfect rectangle; diagonal derived), we can go further and constrain inter‑anchor ToF using the “true” distances at runtime (dynamic error correction). See `docs/tdoa-distance-constraint-correction/README.md`.

---

## Risks, pitfalls, and things to watch out for

0) **What exactly is the “true distance”?**
   - For best results, measure between the UWB antenna reference points (as consistently as possible), not between enclosure edges.
   - If anchors are at different heights, “true distance” should be the full 3D Euclidean distance (or we must incorporate height in the model).

1) **Geometry assumptions vs reality**
   - Deriving diagonals from sides assumes a perfect rectangle. If the installation is skewed, “true” diagonals will differ and we may distort the delay solution.
   - Mitigation: allow “advanced mode” (explicit pair distances) and/or ask for one diagonal measurement for validation.

2) **Multipath/NLOS bias**
   - UWB ranges can be systematically biased in cluttered environments.
   - Mitigation: robust averaging, repeated calibration in known‑good environment, show per‑pair residuals so the user can spot problem links.

3) **Underconstrained graphs**
   - With only perimeter edges on 4 anchors, solution is not unique.
   - Mitigation: add diagonal constraints (derived or measured) or regularize against current values.

4) **Immediate apply may disrupt TDMA sync**
   - Re‑initializing the anchor algorithm to update the broadcast delay may cause resync gaps.
   - Mitigation: implement a lightweight setter for `ctx.antennaDelay` inside the algorithm library if possible.

5) **Units and conversion consistency**
   - Tools and firmware must use the *same* `DW1000_TIME_TO_METERS` constant.
   - Mitigation: document it and copy the exact constant into Rust (with tests).

6) **Clamping / invalid delay outputs**
   - Solver could produce negative or out‑of‑range results if measurements are bad.
   - Mitigation: clamp to a safe range, warn, and refuse to apply unless user confirms.

7) **Rollback**
   - Calibration can make things worse if inputs are wrong.
   - Mitigation: automatically back up old `ADelay` values and allow one‑click restore.

---

## Testing strategy

### Unit tests (Rust)

- Synthetic anchors with known delays + known true distances.
- Generate raw ticks: `T_raw = T_true + a[i] + a[j] + noise`.
- Verify solver recovers delays within tolerance under:
  - noise
  - missing edges
  - outliers
  - bipartite graphs (ensure regularization path behaves as expected)

### Firmware validation

- Add a debug command on the tag to dump current accumulated `T_raw` for inspection.
- Manual procedure:
  - run calibration in a controlled rectangular setup
  - verify per‑pair errors shrink and remain stable over time

---

## Open questions (need answers before coding)

1) Do we *require* a dedicated tag device for calibration, or can one anchor temporarily act as tag?
2) Should “true distances” live only in the manager/CLI, or also be stored on a device for headless reruns?
3) What is the expected target accuracy (cm‑level? 10‑cm‑level) and in what environments?
4) Do we want to update the TDoA engine to actually use `ADelay` corrections (Phase 4), or keep calibration scoped to inter‑anchor distance correction only?
