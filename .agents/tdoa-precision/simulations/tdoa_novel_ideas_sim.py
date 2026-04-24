#!/usr/bin/env python3
"""Novel-ideas TDoA precision experiments for RTLS-link.

Extends the existing repo sim model with a more realistic static-error
structure and evaluates ideas the current sim cannot distinguish:

 1) online_anchor_cal
    Per-anchor TX/RX delay decomposition from round-robin inter-anchor
    distance measurements (least squares). TX correction is applied to the
    tag-side TDoA pipeline. Requires no manual ADelay calibration and no
    precise anchor survey (uses measured anchor distances, optionally
    combined with a self-survey cuboid / MDS).

 2) pair_inv_var_weight
    Running variance of (measured_AB - expected_AB) per anchor pair is used
    as an inverse-variance weight in the TDoA solver. Pair-specific instead
    of Huber's uniform per-residual scaling.

 3) pair_residual_cancel
    After LS decomposition, the residual (measured - D - TX_i - RX_j) is a
    static-multipath estimate for that pair. Subtract it from measured ToF.

 4) symmetric_averaging
    Use (d_AB + d_BA)/2 as a lower-bias estimator for inter-anchor distance
    (cancels TX_i - RX_j asymmetry in each direction).

 5) triangle_reject
    Reject tag pair measurements that violate |dtag_A - dtag_B| <= D_AB
    given the calibrated inter-anchor distance D_AB.

Models:
 - per-anchor TX and RX delays (static) -- cause per-pair additive bias
   decomposable into sums TX_i + RX_j
 - per-pair multipath (static) -- not decomposable, remains as residual
 - time-varying zero-mean noise (tag-side and anchor-anchor)
 - NLOS-like outliers on tag-side pair measurements

Focus metrics for the "light show" use case:
 - raw_p95 (absolute error)
 - off_p95 (after one constant translation removal)
 - rigid_p95 (after best rigid transform)
 - rel_p95 (random two-tag formation error)
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass

import numpy as np


# ---------- geometry ----------

def anchors_3d(width=5.0, depth=5.0, height=3.0):
    lower = np.array([[0, 0, 0], [width, 0, 0], [width, depth, 0], [0, depth, 0]], dtype=float)
    upper = lower.copy()
    upper[:, 2] = height
    return np.vstack([lower, upper])


def anchors_2d(width=5.0, depth=5.0, z=0.0):
    return np.array([[0, 0, z], [width, 0, z], [width, depth, z], [0, depth, z]], dtype=float)


def anchor_pairs(n):
    return list(itertools.combinations(range(n), 2))


def grid_points(width, depth, heights, n=5, pad=0.4):
    xs = np.linspace(pad, width - pad, n)
    ys = np.linspace(pad, depth - pad, n)
    return np.array([[x, y, z] for z in heights for y in ys for x in xs])


LAYOUTS = {
    "8_3d": {
        "anchors_fn": lambda: anchors_3d(5.0, 5.0, 3.0),
        "grid_fn": lambda: grid_points(5.0, 5.0, heights=[0.6, 1.5, 2.4], n=5),
        "fixed_z": None,
        "desc": "8 anchors, 5 m x 5 m x 3 m cuboid, 3 heights",
    },
    "4_2d": {
        "anchors_fn": lambda: anchors_2d(5.0, 5.0, z=0.0),
        "grid_fn": lambda: grid_points(5.0, 5.0, heights=[1.0], n=5),
        "fixed_z": 1.0,
        "desc": "4 anchors at corners z=0, tag fixed_z=1.0 m (current real HW)",
    },
}


# ---------- error model ----------

@dataclass
class ErrorModel:
    anchor_tx_sigma_m: float = 0.030   # per-anchor TX delay bias (static)
    anchor_rx_sigma_m: float = 0.030   # per-anchor RX delay bias (static)
    pair_multipath_sigma_m: float = 0.020  # per-pair multipath bias (static, non-separable)
    tag_sigma_m: float = 0.025         # tag-side zero-mean noise
    anchor_sigma_m: float = 0.025      # anchor-anchor zero-mean noise
    outlier_prob: float = 0.01
    outlier_sigma_m: float = 0.150


def sample_static_state(rng, n_anchors, model):
    tx = rng.normal(0, model.anchor_tx_sigma_m, size=n_anchors)
    rx = rng.normal(0, model.anchor_rx_sigma_m, size=n_anchors)
    # Pair multipath bias is symmetric for (i,j) but may differ between
    # direction i->j and j->i in reality. We model the average.
    pair_bias = {}
    for i, j in itertools.combinations(range(n_anchors), 2):
        pair_bias[(i, j)] = rng.normal(0, model.pair_multipath_sigma_m)
    return tx, rx, pair_bias


# ---------- anchor-anchor measurement generation ----------

def sample_anchor_anchor(rng, anchors, tx, rx, pair_bias, model, samples_per_direction=200):
    """Generate round-robin anchor-anchor distance measurements.

    Returns dict (i,j) -> list of measured distances (both directions
    intermixed; direction is also recorded).
    """
    n = len(anchors)
    obs = {(i, j): [] for i in range(n) for j in range(n) if i != j}
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            D = np.linalg.norm(anchors[i] - anchors[j])
            bias_key = (i, j) if i < j else (j, i)
            pm = pair_bias[bias_key]
            # i transmits -> j receives: TX_i + RX_j + pair multipath + noise
            mean = D + tx[i] + rx[j] + pm
            samples = rng.normal(mean, model.anchor_sigma_m, size=samples_per_direction)
            obs[(i, j)].extend(samples.tolist())
    return obs


# ---------- strategies ----------

def estimate_per_anchor_delays(anchor_anchor_obs, anchors_true_positions):
    """LS decomposition of measured_ij = D_ij + TX_i + RX_j + residual.

    Uses averaged directional measurements. Solves for TX and RX per anchor
    (with a gauge fix: TX_0 = 0 to break the null space).

    Assumes anchors_true_positions is a good self-survey/manual survey.
    Returns tx_hat[n], rx_hat[n], pair_residual[(i,j)].
    """
    n = len(anchors_true_positions)
    pair_means = {}
    for (i, j), vals in anchor_anchor_obs.items():
        pair_means[(i, j)] = float(np.mean(vals))

    # Build observation y_k = tx[i] + rx[j] + pair_bias[i<j], with true D_ij subtracted.
    # Unknowns: tx[0..n-1], rx[0..n-1], pair_bias[pair_idx]
    # Gauge: tx[0] = 0 (absolute delay is unobservable without external truth).
    undir_pairs = list(itertools.combinations(range(n), 2))
    pair_idx = {p: k for k, p in enumerate(undir_pairs)}
    n_pairs = len(undir_pairs)

    rows = []
    ys = []
    for (i, j), mean in pair_means.items():
        D = np.linalg.norm(anchors_true_positions[i] - anchors_true_positions[j])
        row = np.zeros(n + n + n_pairs)
        if i != 0:
            row[i - 1] = 1.0            # tx[i], skipping tx[0]=0 gauge
        # tx[0] omitted
        # rx uses indices n-1 .. n-1 + n
        # We'll lay out params as: tx[1..n-1] (n-1), rx[0..n-1] (n), pair[0..n_pairs-1]
        # Rebuild row accordingly.
        row = np.zeros((n - 1) + n + n_pairs)
        if i != 0:
            row[i - 1] = 1.0
        row[(n - 1) + j] = 1.0
        pair_key = (i, j) if i < j else (j, i)
        row[(n - 1) + n + pair_idx[pair_key]] = 1.0
        rows.append(row)
        ys.append(mean - D)

    A = np.asarray(rows)
    y = np.asarray(ys)
    # Regularize pair_bias toward zero (stronger than delays) to keep the
    # model identifiable and favor per-anchor structure.
    lam_tx = 1e-6
    lam_rx = 1e-6
    lam_pair = 1.0  # much stronger pull -> pair residual absorbs only what TX/RX cannot explain
    reg = np.zeros_like(A[0])
    diag = np.concatenate([
        np.full(n - 1, lam_tx),
        np.full(n, lam_rx),
        np.full(n_pairs, lam_pair),
    ])
    AtA = A.T @ A + np.diag(diag)
    Atb = A.T @ y
    params = np.linalg.solve(AtA, Atb)
    tx_hat = np.concatenate([[0.0], params[:n - 1]])
    rx_hat = params[n - 1:(n - 1) + n]
    pair_vec = params[(n - 1) + n:]
    pair_resid = {p: float(pair_vec[k]) for p, k in pair_idx.items()}
    return tx_hat, rx_hat, pair_resid


def measured_tdoa_from_tag(rng, truth, anchors_true, pairs, tx, rx, pair_bias, model):
    """Generate tag-side TDoA distance-difference measurements for pairs (A,B).

    TDoA sees the difference of two anchor transmissions arriving at the tag.
    Each contributes TX_i of the transmitting anchor; tag RX delay cancels.
    Pair multipath from anchor-anchor is NOT the same as multipath on anchor->tag,
    so we model tag-side multipath as independent, with a smaller pair-specific
    but truth-independent bias plus zero-mean noise plus occasional NLOS.

    Returns measurement vector indexed by pairs.
    """
    meas = []
    for a, b in pairs:
        da = np.linalg.norm(truth - anchors_true[a])
        db = np.linalg.norm(truth - anchors_true[b])
        # TDoA distance diff = (db - da), with TX_b - TX_a appearing as bias
        # (anchors' TX delays leak into tag-side TDoA):
        bias = (tx[b] + rx[b] * 0.0) - (tx[a] + rx[a] * 0.0)  # only TX leaks to tag
        noise = rng.normal(0, model.tag_sigma_m)
        # Add a small anchor->tag multipath effect that partially correlates with
        # anchor-anchor multipath (same anchors, different paths). We take a
        # fraction of pair multipath to model "bad-pair-bad-tag" tendency.
        pm_key = (a, b) if a < b else (b, a)
        tag_mp = 0.4 * pair_bias[pm_key]
        if rng.random() < model.outlier_prob:
            noise += rng.normal(0, model.outlier_sigma_m)
        meas.append(db - da + bias + tag_mp + noise)
    return np.asarray(meas)


# ---------- solver ----------

def tdoa_solve(anchors_cfg, pairs, measurements, initial, weights=None, fixed_z=None,
               huber_delta=None, max_iter=20):
    x = initial.astype(float).copy()
    if fixed_z is not None:
        x[2] = fixed_z
    active = 2 if fixed_z is not None else 3
    damp = 1e-5
    n = len(pairs)
    w = np.ones(n) if weights is None else np.asarray(weights).astype(float).copy()
    for _ in range(max_iter):
        residuals = np.zeros(n)
        jac = np.zeros((n, active))
        for k, (a, b) in enumerate(pairs):
            va = x - anchors_cfg[a]
            vb = x - anchors_cfg[b]
            da = max(np.linalg.norm(va), 1e-9)
            db = max(np.linalg.norm(vb), 1e-9)
            residuals[k] = (db - da) - measurements[k]
            jac[k] = (vb / db - va / da)[:active]
        ww = w.copy()
        if huber_delta is not None:
            abs_r = np.abs(residuals)
            mask = abs_r > huber_delta
            ww = ww.copy()
            ww[mask] = ww[mask] * (huber_delta / np.maximum(abs_r[mask], 1e-12))
        sw = np.sqrt(ww)
        jw = jac * sw[:, None]
        rw = residuals * sw
        lhs = jw.T @ jw + damp * np.eye(active)
        rhs = jw.T @ rw
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        x[:active] -= step
        if fixed_z is not None:
            x[2] = fixed_z
        if np.linalg.norm(step) < 1e-7:
            break
    return x


# ---------- consistency metrics ----------

def procrustes_rigid(est, truth):
    xm = est.mean(0)
    ym = truth.mean(0)
    x0 = est - xm
    y0 = truth - ym
    h = x0.T @ y0
    u, _, vt = np.linalg.svd(h)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return x0 @ r + ym


def metrics(rng, est, truth, rel_samples=8000):
    raw = np.linalg.norm(est - truth, axis=1)
    off = est - (est - truth).mean(0)
    off_err = np.linalg.norm(off - truth, axis=1)
    rigid = procrustes_rigid(est, truth)
    rigid_err = np.linalg.norm(rigid - truth, axis=1)
    ia = rng.integers(0, len(truth), size=rel_samples)
    ib = rng.integers(0, len(truth), size=rel_samples)
    rel = np.linalg.norm((est[ia] - est[ib]) - (truth[ia] - truth[ib]), axis=1)
    return {
        "raw_p95": float(np.percentile(raw, 95)),
        "off_p95": float(np.percentile(off_err, 95)),
        "rigid_p95": float(np.percentile(rigid_err, 95)),
        "rel_p95": float(np.percentile(rel, 95)),
    }


# ---------- experiment ----------

def run_experiment(seed=1, points_per_grid=40, model=None, label="default",
                   verbose=True, layout="8_3d"):
    if model is None:
        model = ErrorModel()
    rng = np.random.default_rng(seed)
    layout_cfg = LAYOUTS[layout]
    anchors = layout_cfg["anchors_fn"]()
    fixed_z = layout_cfg["fixed_z"]
    n = len(anchors)
    all_pairs = anchor_pairs(n)

    # 1) static error state
    tx, rx, pair_bias = sample_static_state(rng, n, model)

    # 2) anchor-anchor round-robin observations + per-pair variance tracking
    obs = sample_anchor_anchor(rng, anchors, tx, rx, pair_bias, model, samples_per_direction=300)
    # Symmetric averaging (d_ij + d_ji) / 2
    sym_pair_mean = {}
    pair_var = {}
    for (i, j) in all_pairs:
        fwd = np.asarray(obs[(i, j)])
        bwd = np.asarray(obs[(j, i)])
        merged = np.concatenate([fwd, bwd])
        sym_pair_mean[(i, j)] = float(merged.mean())
        # residual variance around the grand mean = proxy for pair quality;
        # smaller means more consistent pair.
        pair_var[(i, j)] = float(merged.var() + 1e-8)

    # 3) Estimate per-anchor delays (assuming anchor positions known)
    tx_hat, rx_hat, pair_resid = estimate_per_anchor_delays(obs, anchors)

    # 4) Ground truth grid
    grid = layout_cfg["grid_fn"]()
    truth = np.repeat(grid, points_per_grid, axis=0)
    center = anchors.mean(0)
    if fixed_z is not None:
        center = center.copy()
        center[2] = fixed_z

    def solve_all(name, correct_tx=False, pair_weights=None, cancel_pair_resid=False,
                  triangle_reject=False, huber=False):
        ests = []
        for p in truth:
            meas = measured_tdoa_from_tag(rng, p, anchors, all_pairs, tx, rx, pair_bias, model)
            # Correction: subtract estimated (TX_b - TX_a) from tag measurement
            if correct_tx:
                for k, (a, b) in enumerate(all_pairs):
                    meas[k] -= (tx_hat[b] - tx_hat[a])
            if cancel_pair_resid:
                for k, (a, b) in enumerate(all_pairs):
                    # Cancel static per-pair multipath estimated from anchor-anchor LS.
                    # Scale by 0.4 because tag-side multipath is modeled as 0.4*anchor_pair_bias.
                    # In real HW the coupling is unknown; we conservatively use the full
                    # residual here to test the best-case of direct cancellation.
                    meas[k] -= pair_resid[(a, b)] * 0.4
            # Triangle-inequality rejection: |meas| must be <= D_AB
            pairs_used = list(all_pairs)
            meas_used = meas.copy()
            if triangle_reject:
                keep = []
                D_est = {}
                for (a, b) in all_pairs:
                    # Use symmetric average minus estimated TX/RX delays as D hat
                    D_est[(a, b)] = sym_pair_mean[(a, b)] - 0.5 * (tx_hat[a] + rx_hat[b] + tx_hat[b] + rx_hat[a])
                    D_est[(a, b)] -= pair_resid[(a, b)]
                for k, (a, b) in enumerate(all_pairs):
                    if abs(meas[k]) <= D_est[(a, b)] * 1.02 + 0.02:
                        keep.append(k)
                if len(keep) < 4:
                    keep = list(range(len(all_pairs)))
                pairs_used = [all_pairs[k] for k in keep]
                meas_used = meas[keep]
                if pair_weights is not None:
                    pair_weights_used = np.asarray(pair_weights)[keep]
                else:
                    pair_weights_used = None
            else:
                pair_weights_used = pair_weights

            est = tdoa_solve(anchors, pairs_used, meas_used, initial=center,
                             weights=pair_weights_used,
                             fixed_z=fixed_z,
                             huber_delta=0.08 if huber else None,
                             max_iter=20)
            ests.append(est)
        ests = np.asarray(ests)
        m = metrics(rng, ests, truth)
        if verbose:
            print(f"  {name:36s} raw_p95={100*m['raw_p95']:5.2f} cm  "
                  f"off_p95={100*m['off_p95']:5.2f} cm  "
                  f"rigid_p95={100*m['rigid_p95']:5.2f} cm  "
                  f"rel_p95={100*m['rel_p95']:5.2f} cm")
        return m

    if verbose:
        print(f"\n=== {label} [{layout}: {layout_cfg['desc']}] ===")
        print(f"  model: tx_sigma={model.anchor_tx_sigma_m*100:.1f} cm  "
              f"rx_sigma={model.anchor_rx_sigma_m*100:.1f} cm  "
              f"pair_multipath={model.pair_multipath_sigma_m*100:.1f} cm  "
              f"outlier_p={model.outlier_prob}  outlier_sigma={model.outlier_sigma_m*100:.0f} cm")
        # LS diagnostics: how well TX/RX are recovered relative to the truth.
        tx_true_zero_fixed = tx - tx[0]  # match gauge TX_0=0
        tx_err = tx_hat - tx_true_zero_fixed
        rx_err = rx_hat - rx  # RX has no gauge fix in our LS (regularized weakly)
        print(f"  LS: tx_err_rms={100*np.sqrt(np.mean(tx_err**2)):.2f} cm  "
              f"tx_err_max={100*np.max(np.abs(tx_err)):.2f} cm  "
              f"rx_err_rms={100*np.sqrt(np.mean(rx_err**2)):.2f} cm  "
              f"(anchors={n}, directional_obs={n*(n-1)}, "
              f"effective_unknowns={2*n-1}+{n*(n-1)//2}pair_reg)")
    # Inverse-variance pair weights
    inv_var = np.array([1.0 / pair_var[p] for p in all_pairs])
    inv_var /= inv_var.mean()

    results = {}
    results["baseline_no_correction"] = solve_all("baseline (no correction)")
    results["huber_only"] = solve_all("huber only", huber=True)
    results["tx_cal"] = solve_all("per-anchor TX calibration", correct_tx=True)
    results["tx_cal_huber"] = solve_all("per-anchor TX cal + huber", correct_tx=True, huber=True)
    results["tx_cal_pair_w"] = solve_all("per-anchor TX cal + pair inv-var",
                                         correct_tx=True, pair_weights=inv_var)
    results["tx_cal_pair_w_huber"] = solve_all("TX cal + pair inv-var + huber",
                                               correct_tx=True, pair_weights=inv_var, huber=True)
    results["tx_cal_resid_cancel"] = solve_all("TX cal + pair residual cancel",
                                                correct_tx=True, cancel_pair_resid=True)
    results["full_stack"] = solve_all("TX cal + residual cancel + pair inv-var + huber",
                                       correct_tx=True, cancel_pair_resid=True,
                                       pair_weights=inv_var, huber=True)
    results["triangle_reject"] = solve_all("TX cal + triangle reject + huber",
                                           correct_tx=True, triangle_reject=True, huber=True)
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--repeats", type=int, default=5,
                        help="how many different static realizations to average over")
    parser.add_argument("--layout", default="all",
                        choices=["all"] + list(LAYOUTS.keys()))
    args = parser.parse_args()

    cases = [
        ("baseline realistic", ErrorModel()),
        ("hard RF / more outliers", ErrorModel(
            anchor_tx_sigma_m=0.04, anchor_rx_sigma_m=0.04,
            pair_multipath_sigma_m=0.04, tag_sigma_m=0.035,
            anchor_sigma_m=0.035, outlier_prob=0.05, outlier_sigma_m=0.26)),
        ("low-bias calibrated hw", ErrorModel(
            anchor_tx_sigma_m=0.015, anchor_rx_sigma_m=0.015,
            pair_multipath_sigma_m=0.015, tag_sigma_m=0.02,
            anchor_sigma_m=0.02, outlier_prob=0.005)),
    ]

    layouts = list(LAYOUTS.keys()) if args.layout == "all" else [args.layout]

    for layout in layouts:
        print(f"\n############ LAYOUT {layout} ############")
        for label, model in cases:
            acc = {}
            for r in range(args.repeats):
                res = run_experiment(seed=args.seed + r, model=model,
                                     label=f"{label} (seed {args.seed + r})",
                                     verbose=(r == 0), layout=layout)
                for k, v in res.items():
                    acc.setdefault(k, []).append(v)
            print(f"\n  --- averaged over {args.repeats} static realizations ---")
            for k, v_list in acc.items():
                mean_m = {key: float(np.mean([d[key] for d in v_list])) for key in v_list[0]}
                print(f"  AVG {k:32s} raw_p95={100*mean_m['raw_p95']:5.2f} cm  "
                      f"off_p95={100*mean_m['off_p95']:5.2f} cm  "
                      f"rigid_p95={100*mean_m['rigid_p95']:5.2f} cm  "
                      f"rel_p95={100*mean_m['rel_p95']:5.2f} cm")


if __name__ == "__main__":
    main()
