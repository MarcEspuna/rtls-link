# TDoA geometry robustness — scope & intent

Branch: `feature/tdoa-geometry-robustness`

## Problem

TDoA position fixes are solved per *window* of measurements (one snapshot, no temporal
filtering across windows — by design, because ArduPilot's EKF downstream does the
filtering and we must not add latency). The set of anchors/pairs that land in a window
varies. When that set is poorly conditioned — near-coplanar anchors, weak vertical
geometry, a near-collinear subset — the least-squares solve is ill-conditioned. The
weak axis (almost always Z in 3D) gets noise amplified by `1/eigenvalue`, so the fix
jumps. Today the system either rejects the window (a lost fix → ArduPilot dead-reckons)
or reports an over-optimistic covariance (ArduPilot over-trusts a bad fix).

## What we are NOT doing (non-goals)

- **No velocity/EKF/temporal smoothing of the output.** ArduPilot owns filtering. We
  must not add latency to observable axes. (The one exception below is deliberate and
  bounded.)
- **No 3D→2D fallback.** If configured 3D, we always emit Z. ArduPilot expects it;
  silently dropping to 2D would corrupt its state.
- **No change to the wire/MAVLink format or the anchor-pairing transport.**
- **No re-architecture of the windowing/buffer or the solver's LM/QR core.**

## In scope — three changes

### 1. Honest covariance to ArduPilot  (`tdoa_newton_raphson` + integration)
The reported covariance is over-optimistic: (a) each TDoA is a *difference of two ToA
measurements* (factor-of-2 not modelled), and (b) the 28 pairs from 8 anchors are
**correlated** — only `unique_anchors - 1` are independent — but the solver weights them
as independent, shrinking the reported covariance below reality. An over-confident
covariance is the worst failure mode for a downstream EKF.
- Fix: compute the covariance from an **independent reference-anchor TDoA set**
  (`(ref, k)` for each other anchor `k`), modelling the shared-reference correlation
  `C = I + 11ᵀ` structure. The Fisher information of the full correlated set equals
  that of any independent spanning set, so this is exact for the information content and
  side-steps the singular-`C` problem of the fully-redundant set.
- Plus: make the high-variance rejection (`kMax3DPositionVarianceM2 = 9`) a
  **report-with-honest-covariance** path controlled by a runtime param, so a
  weak-but-real fix reaches ArduPilot with a large honest variance instead of being
  dropped. Default preserves current (reject) behaviour.

### 2. Anisotropic null-space regularization  (`tdoa_newton_raphson` + integration)
Pin only the **near-null eigendirection(s)** of `JᵀJ` toward the previous estimate via a
post-convergence MAP blend; leave well-observed directions fully data-driven (zero added
latency on observable axes). This replaces noise-amplified garbage in the weak axis with
the last known value — strictly better than the current implicit warm-start drift.
- **Reported covariance stays data-only**, NOT the MAP posterior — otherwise ArduPilot
  would double-count the prior's information across time and become over-confident again.
- Behind a feature flag + runtime `sigma_prior` (≤0 / disabled = exact current behaviour).

### 3. Geometric (E-optimal) matcher policy  (`tdoa_algorithm` engine + integration)
New `GEOMETRIC` value alongside `YOUNGEST`/`RANDOM` in the TDoA engine's anchor matcher.
When a packet arrives from anchor A, pick the partner B that maximizes the **minimum
eigenvalue gain** (E-optimal) of the current window's Fisher information, evaluated at the
last position estimate. Shapes *inputs*, so no added latency.
- Needs anchor positions + a position prior plumbed into the engine.
- **Cold-start fallback to YOUNGEST** when no prior fix exists.
- Caveat (documented, not a bug): with the same anchor set heard, pairing choice changes
  *conditioning/noise*, not observability — it cannot create Z-observability from
  physically coplanar anchors. Complements (1) and (2), does not replace them.

## Feature flags / params
Each behaviour is gated (feature switch in `user_defines.txt` / `features.hpp`, runtime
param in the registry) and **defaults to current behaviour**, so the build is a no-op
until explicitly enabled. Desktop `rtls-link-manager` param sync tracked separately.

## Note on a parallel effort
A separate worktree (`feature/geometric-tdoa-matcher`) contains a concurrent
implementation of overlapping ideas by other agents. This branch is an independent
implementation built off `main`; reconciliation between the two is a follow-up.

## Audit rules
High-signal only: correctness of the numerics, the covariance-honesty contract above,
the cold-start path, feature-flag default-off guarantee, build integrity on
esp32/esp32s3/native. No style nitpicks.
