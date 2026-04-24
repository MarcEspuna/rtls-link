# TDoA Distance‑Constraint Error Correction (Using Pre‑Measured Rectangle Distances)

## Goal

After we introduce **precisely measured inter‑anchor distances** (for antenna‑delay calibration), we can optionally use the same information at runtime to **reduce TDoA errors**.

The core idea is to treat the anchor geometry as **known and constant** (perfect rectangle, diagonal derived), and use it to **stabilize / correct the inter‑anchor time‑of‑flight (ToF)** values that are fed into the TDoA estimator.

This document investigates viability and recommends an implementation approach.

---

## Why this can help (high level)

In the Bitcraze‑derived TDoA2 pipeline used here, the tag’s TDoA engine uses:

- **Per‑anchor RX timestamps of other anchors’ packets** (in the receiving anchor’s clock)
- **Inter‑anchor ToF values** (derived from anchor↔anchor DS‑TWR and broadcast in anchor packets)

Those inter‑anchor ToF values are used to translate “packet receive time” into “packet transmit time” between anchors. Any bias/noise in these ToF values directly biases the computed TDoA distance differences.

If we already *know* the true anchor geometry (and we assume it is perfect), we can provide a much more stable ToF for each pair.

---

## Current implementation (what the code does today)

### On anchors (TDoA mode)

- Anchors run the TDMA anchor algorithm (`lib/tdoa_algorithm/src/anchor/tdoa_anchor.cpp`) and send packets containing:
  - `timestamps[]`: TX timestamp for self + RX timestamps of other anchors’ packets (in this anchor clock)
  - `distances[]`: inter‑anchor DS‑TWR ToF estimates (in DW1000 ticks)
  - `antennaDelay`: the configured `uwb.ADelay` (metadata)

Important detail:

- The firmware wrapper configures the DW1000 antenna delay to **0** in TDoA mode (`dwSetAntenaDelay(..., 0)` then `dwCommitConfiguration()`), so hardware antenna delay correction is intentionally disabled.

### On tags (TDoA mode)

- The tag receives anchor packets and stores:
  - `remoteRxTime` from `timestamps[]`
  - `remoteTof` from `distances[]`

Then the TDoA engine computes:

```
delta_txAr_to_txAn_in_cl_An = tof_Ar_to_An_in_cl_An + (txAn_in_cl_An - rxAr_by_An_in_cl_An)
```

Meaning: **ToF is directly additive in the TDoA model**.

### Antenna delay status today

- `uwb.ADelay` exists and is broadcast in packets, but the tag’s TDoA engine currently stores and uses `packet->distances[]` **without subtracting antenna delays**.
- Separately, some features (like dynamic anchor positioning in `src/uwb/uwb_tdoa_tag.cpp`) already apply a correction:
  - `corrected = raw - adelay_from - adelay_to`

So there is a known opportunity to improve TDoA by using calibrated delays consistently inside the engine.

---

## What “distance‑constraint correction” means

We assume:

- 4 anchors form a perfect rectangle.
- The user measures `dX` and `dY` precisely.
- We derive `dDiag = sqrt(dX^2 + dY^2)`.
- From layout mapping we can derive *all* pairwise “true distances” `D_true(i,j)` for the 4‑anchor set.

Convert distance to an ideal ToF in DW1000 ticks:

```
T_true(i,j) = D_true(i,j) / DW1000_TIME_TO_METERS
```

Then, because the currently broadcast `packet->distances[]` is effectively a “raw ToF” that includes per‑anchor antenna delays, we can define an “expected raw ToF”:

```
T_raw_expected(i,j) = T_true(i,j) + ADelay[i] + ADelay[j]
```

Once `ADelay[]` has been calibrated (using the earlier feature), `T_raw_expected` should match the mean of `packet->distances[]` but with much less noise/outlier impact.

---

## Where should the correction be applied?

You suggested two options:

1) **Anchor‑side correction** (anchors adjust what they transmit)
2) **Tag‑side correction** (tags adjust what they consume)

### Option A — Tag‑side correction (recommended first)

**Approach**

- Keep anchor behavior and over‑the‑air packets unchanged.
- On the tag, right before calling `tdoaStorageSetRemoteTimeOfFlight()`, replace (or blend) the received ToF with `T_raw_expected(i,j)` computed from the configured rectangle geometry.

**Modes to support**

- `OFF`: use measured `packet->distances[]`
- `MONITOR`: use measured, but compute and report per‑pair error vs expected
- `BLEND(alpha)`: `tof_used = (1-alpha)*tof_meas + alpha*tof_expected`
- `OVERRIDE`: `tof_used = tof_expected` (maximum constraint)

**Pros**

- Safest: does not change TDMA timing, packet format, or anchor synchronization behavior.
- Easy to disable if it causes issues in real environments.
- Enables runtime health checks: compare measured vs expected to detect anchor displacement/NLOS problems.

**Cons**

- Only benefits tags that implement the feature (but that’s OK: tags are the estimator).

### Option B — Anchor‑side correction (possible, but higher risk)

There are two sub‑variants:

1) **Replace `distances[]` with `T_raw_expected`** before sending packets.
   - This reduces DS‑TWR noise in the broadcast field and benefits all tags immediately.
   - Requires distributing rectangle geometry + per‑anchor ADelay to all anchors (configuration complexity).
   - Loses the ability for tags to compare “measured vs expected” unless we add a second field.

2) **Attempt to “correct timestamps” (`timestamps[]`)** using expected ToF.
   - This is effectively a distributed time‑sync problem and risks breaking protocol semantics.
   - Not recommended until we have strong evidence it is necessary.

### Option C — Switch TDoA mode to use hardware antenna delays (separate project)

It is technically possible to set non‑zero DW1000 antenna delay registers in TDoA mode (via `dwSetAntenaDelay()` + `dwCommitConfiguration()`), but doing so changes timestamp semantics and risks double‑correction if software corrections remain.

Treat this as a future refactor, not as the first implementation of distance‑constraint correction.

---

## Recommended implementation (detailed)

### 1) Represent the rectangle geometry on the tag

We need the tag to know `D_true(i,j)` for the anchor set.

Best practical representation (minimal new params):

- Store anchors’ positions in `UWBParams` such that they match the measured rectangle:
  - A0 = (0, 0)
  - +X anchor = (dX, 0)
  - +Y anchor = (0, dY)
  - corner = (dX, dY)

Then the tag can compute `D_true(i,j)` directly from positions.

Alternative:

- Add explicit parameters `rect_dx`, `rect_dy`, and `anchorLayout` (still required) and derive `D_true` inside the tag algorithm.

### 2) Implement correction in `tdoa_tag_algorithm.cpp`

In `updateRemoteData()` where the code currently does:

- `tof = packet->distances[i]`
- `tdoaStorageSetRemoteTimeOfFlight(anchorCtx, remoteId, tof)`

Add logic:

- Determine if this `(anchorId, remoteId)` pair is part of the 4‑anchor rectangle set.
- Compute `T_true(i,j)` (ticks) from the stored geometry.
- Fetch `ADelay[i]` and `ADelay[j]` from the per‑anchor antennaDelay metadata already tracked by the tag.
- Compute `T_raw_expected = T_true + ADelay[i] + ADelay[j]`.
- Choose `tof_used` based on mode (OFF/MONITOR/BLEND/OVERRIDE).
- Store `tof_used` into `tdoaStorage`.

### 3) Add a feature flag + runtime parameter

Firmware should gate this behavior:

- Compile‑time: `USE_TDOA_DISTANCE_CONSTRAINT_CORRECTION` (name TBD)
- Runtime:
  - enable/mode selector (OFF/MONITOR/BLEND/OVERRIDE)
  - blend factor `alpha` (for BLEND)

This allows safe rollout and A/B testing.

### 4) Validation and safety mechanisms

Even though we assume a perfect rectangle, we should protect against common failure cases:

- Anchors not actually placed perfectly / moved after measurement
- Wrong layout selection (+X/+Y mapping)
- Wrong anchor IDs (0–3 mismatch)
- NLOS/multipath causing DS‑TWR bias

Recommended checks (in MONITOR mode and optionally in others):

- Track `err_ticks = tof_meas - tof_expected`
- Convert to meters for display
- If `|err|` exceeds a threshold consistently (e.g. > 10–20 cm), warn and/or auto‑disable correction.

### 5) Manager/CLI integration

`rtls-link-manager` should:

- Let the user enter `dX`, `dY`, select layout
- Push derived anchor positions (or dx/dy) into the tag config
- Enable the correction mode (start in MONITOR, then OVERRIDE if stable)
- Display per‑pair errors and an overall health score

---

## Expected impact / viability

**Viability:** high, especially tag‑side correction.

Main benefits:

- Removes DS‑TWR noise/outliers from the ToF term used by the TDoA engine.
- Makes TDoA more repeatable when the geometry is truly fixed and known.

Main risks:

- If the “perfect rectangle” assumption is violated, forcing expected ToF can *bias* the estimator.
- Over‑constraining (OVERRIDE) may hide real issues (misplacement/NLOS) unless we keep monitoring enabled.

Recommended rollout:

1) Implement MONITOR first (no behavior change, only diagnostics)
2) Implement BLEND (safe partial correction)
3) Implement OVERRIDE once confidence is high

---

## Relationship to antenna‑delay self‑calibration

This feature should be treated as a follow‑up to `docs/antenna-delay-self-calibration.md`:

- First calibrate anchors’ `uwb.ADelay` against the measured rectangle distances.
- Then enable distance‑constraint correction on tags to improve TDoA runtime accuracy and stability.

