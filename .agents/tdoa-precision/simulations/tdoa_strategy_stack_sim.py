#!/usr/bin/env python3
"""Compare geometry lock, per-anchor bias calibration, and Huber together.

This is a second-pass harness for deciding implementation order. It models two
separable static error sources:

1. Inter-anchor path bias that geometry lock can remove.
2. Per-anchor transmit-side bias that leaks into tag TDoA and can be estimated
   from anchor-anchor traffic.

The goal is not to predict field performance exactly. The goal is to test
whether the proposed corrections are substitutes or complementary, and how much
we lose if the anchor-anchor stream only supports a symmetric per-anchor
correction instead of directional TX/RX separation.
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass

import numpy as np


def anchors_2d(width=5.0, depth=5.0, z=0.0):
    return np.array([[0, 0, z], [width, 0, z], [width, depth, z], [0, depth, z]], dtype=float)


def anchors_3d(width=5.0, depth=5.0, height=3.0):
    lower = anchors_2d(width, depth, 0.0)
    upper = lower.copy()
    upper[:, 2] = height
    return np.vstack([lower, upper])


def grid_points(width, depth, heights, n=5, pad=0.4):
    xs = np.linspace(pad, width - pad, n)
    ys = np.linspace(pad, depth - pad, n)
    return np.array([[x, y, z] for z in heights for y in ys for x in xs])


LAYOUTS = {
    "4_2d": {
        "anchors": lambda: anchors_2d(5.0, 5.0, 0.0),
        "grid": lambda: grid_points(5.0, 5.0, [1.0], n=5),
        "fixed_z": 1.0,
    },
    "8_3d": {
        "anchors": lambda: anchors_3d(5.0, 5.0, 3.0),
        "grid": lambda: grid_points(5.0, 5.0, [0.6, 1.5, 2.4], n=5),
        "fixed_z": None,
    },
}


@dataclass(frozen=True)
class ErrorModel:
    name: str
    tx_sigma: float
    rx_sigma: float
    pair_sigma: float
    tag_pair_sigma: float
    tag_sigma: float
    anchor_sigma: float
    outlier_prob: float
    outlier_sigma: float
    tag_pair_coupling: float = 0.35
    tx_rx_corr: float = 0.0


MODELS = {
    "baseline": ErrorModel(
        "baseline", tx_sigma=0.030, rx_sigma=0.030, pair_sigma=0.020,
        tag_pair_sigma=0.015, tag_sigma=0.025, anchor_sigma=0.025,
        outlier_prob=0.01, outlier_sigma=0.15, tx_rx_corr=0.0),
    "hard_rf": ErrorModel(
        "hard_rf", tx_sigma=0.040, rx_sigma=0.040, pair_sigma=0.040,
        tag_pair_sigma=0.030, tag_sigma=0.035, anchor_sigma=0.035,
        outlier_prob=0.05, outlier_sigma=0.26, tx_rx_corr=0.0),
    "low_bias": ErrorModel(
        "low_bias", tx_sigma=0.015, rx_sigma=0.015, pair_sigma=0.015,
        tag_pair_sigma=0.010, tag_sigma=0.020, anchor_sigma=0.020,
        outlier_prob=0.005, outlier_sigma=0.15, tx_rx_corr=0.0),
    "symmetric_good": ErrorModel(
        "symmetric_good", tx_sigma=0.030, rx_sigma=0.030, pair_sigma=0.020,
        tag_pair_sigma=0.015, tag_sigma=0.025, anchor_sigma=0.025,
        outlier_prob=0.01, outlier_sigma=0.15, tx_rx_corr=0.85),
}


def pairs_for(n):
    return list(itertools.combinations(range(n), 2))


def sample_state(rng, n, model):
    tx = rng.normal(0.0, model.tx_sigma, n)
    rx_ind = rng.normal(0.0, model.rx_sigma, n)
    if model.tx_rx_corr == 0.0:
        rx = rx_ind
    else:
        # Correlated TX/RX endpoint biases, preserving rx_sigma.
        tx_scaled = tx * (model.rx_sigma / max(model.tx_sigma, 1e-12))
        rx = model.tx_rx_corr * tx_scaled + np.sqrt(max(0.0, 1.0 - model.tx_rx_corr**2)) * rx_ind

    pair_bias = {}
    tag_pair_ind = {}
    for p in pairs_for(n):
        pair_bias[p] = rng.normal(0.0, model.pair_sigma)
        tag_pair_ind[p] = rng.normal(0.0, model.tag_pair_sigma)
    return tx, rx, pair_bias, tag_pair_ind


def sample_anchor_anchor(rng, anchors, tx, rx, pair_bias, model, samples=120):
    n = len(anchors)
    obs = {(i, j): [] for i in range(n) for j in range(n) if i != j}
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            p = (i, j) if i < j else (j, i)
            d = np.linalg.norm(anchors[i] - anchors[j])
            mean = d + tx[i] + rx[j] + pair_bias[p]
            obs[(i, j)] = rng.normal(mean, model.anchor_sigma, samples)
    return obs


def estimate_directional_tx(obs, anchors, pair_reg=1.0):
    """Estimate TX_i from directional measurements y_ij = TX_i + RX_j + pair_ij."""
    n = len(anchors)
    undir = pairs_for(n)
    pair_index = {p: k for k, p in enumerate(undir)}
    rows = []
    ys = []
    for (i, j), samples in obs.items():
        d = np.linalg.norm(anchors[i] - anchors[j])
        row = np.zeros((n - 1) + n + len(undir))
        if i != 0:
            row[i - 1] = 1.0
        row[(n - 1) + j] = 1.0
        p = (i, j) if i < j else (j, i)
        row[(n - 1) + n + pair_index[p]] = 1.0
        rows.append(row)
        ys.append(float(np.mean(samples)) - d)

    a = np.asarray(rows)
    y = np.asarray(ys)
    diag = np.concatenate([
        np.full(n - 1, 1e-6),
        np.full(n, 1e-6),
        np.full(len(undir), pair_reg),
    ])
    x = np.linalg.solve(a.T @ a + np.diag(diag), a.T @ y)
    tx_hat = np.concatenate([[0.0], x[:n - 1]])
    return tx_hat


def estimate_symmetric_composite(obs, anchors, pair_reg=1.0):
    """Estimate per-anchor composite u_i from symmetric pair means.

    Symmetric pair means follow approximately:
      mean_ij - D_ij = u_i + u_j + pair_ij
    where u_i ~= 0.5 * (TX_i + RX_i). This is only a good tag-side correction
    if TX and RX endpoint biases are correlated.
    """
    n = len(anchors)
    undir = pairs_for(n)
    pair_index = {p: k for k, p in enumerate(undir)}
    rows = []
    ys = []
    for i, j in undir:
        d = np.linalg.norm(anchors[i] - anchors[j])
        mean = 0.5 * (float(np.mean(obs[(i, j)])) + float(np.mean(obs[(j, i)])))
        row = np.zeros(n + len(undir))
        row[i] = 1.0
        row[j] = 1.0
        row[n + pair_index[(i, j)]] = 1.0
        rows.append(row)
        ys.append(mean - d)

    a = np.asarray(rows)
    y = np.asarray(ys)
    diag = np.concatenate([np.full(n, 1e-6), np.full(len(undir), pair_reg)])
    x = np.linalg.solve(a.T @ a + np.diag(diag), a.T @ y)
    return x[:n] - x[0]


def tdoa_solve(anchors, pairs, measurements, initial, fixed_z=None, huber_delta=None, weights=None, max_iter=20):
    x = initial.copy().astype(float)
    if fixed_z is not None:
        x[2] = fixed_z
    active = 2 if fixed_z is not None else 3
    w_base = np.ones(len(pairs)) if weights is None else np.asarray(weights, dtype=float)

    for _ in range(max_iter):
        residuals = np.empty(len(pairs))
        jac = np.empty((len(pairs), active))
        for k, (a, b) in enumerate(pairs):
            va = x - anchors[a]
            vb = x - anchors[b]
            da = max(np.linalg.norm(va), 1e-9)
            db = max(np.linalg.norm(vb), 1e-9)
            residuals[k] = (db - da) - measurements[k]
            jac[k] = (vb / db - va / da)[:active]

        w = w_base.copy()
        if huber_delta is not None:
            ar = np.abs(residuals)
            mask = ar > huber_delta
            w[mask] *= huber_delta / np.maximum(ar[mask], 1e-12)

        sw = np.sqrt(w)
        jw = jac * sw[:, None]
        rw = residuals * sw
        lhs = jw.T @ jw + 1e-5 * np.eye(active)
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


def rigid_align(est, truth):
    em = est.mean(axis=0)
    tm = truth.mean(axis=0)
    e0 = est - em
    t0 = truth - tm
    u, _, vt = np.linalg.svd(e0.T @ t0)
    r = u @ vt
    if np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    return e0 @ r + tm


def compute_metrics(rng, est, truth, samples=6000):
    raw = np.linalg.norm(est - truth, axis=1)
    off = est - (est - truth).mean(axis=0)
    off_err = np.linalg.norm(off - truth, axis=1)
    rigid = rigid_align(est, truth)
    rigid_err = np.linalg.norm(rigid - truth, axis=1)
    ia = rng.integers(0, len(truth), samples)
    ib = rng.integers(0, len(truth), samples)
    rel = np.linalg.norm((est[ia] - est[ib]) - (truth[ia] - truth[ib]), axis=1)
    return {
        "raw_p95": float(np.percentile(raw, 95)),
        "off_p95": float(np.percentile(off_err, 95)),
        "rigid_p95": float(np.percentile(rigid_err, 95)),
        "rel_p95": float(np.percentile(rel, 95)),
    }


STRATEGIES = [
    ("current", False, None, False, False),
    ("huber", False, None, True, False),
    ("geom", True, None, False, False),
    ("geom+huber", True, None, True, False),
    ("tx+huber", False, "directional", True, False),
    ("geom+tx+huber", True, "directional", True, False),
    ("geom+sym+huber", True, "symmetric", True, False),
    ("geom+tx+tri+huber", True, "directional", True, True),
]


def simulate_once(seed, layout_name, model, point_repeats):
    rng = np.random.default_rng(seed)
    layout = LAYOUTS[layout_name]
    anchors = layout["anchors"]()
    fixed_z = layout["fixed_z"]
    pairs = pairs_for(len(anchors))
    truth = np.repeat(layout["grid"](), point_repeats, axis=0)
    center = anchors.mean(axis=0)
    if fixed_z is not None:
        center = center.copy()
        center[2] = fixed_z

    tx, rx, pair_bias, tag_pair_ind = sample_state(rng, len(anchors), model)
    obs = sample_anchor_anchor(rng, anchors, tx, rx, pair_bias, model)
    tx_hat = estimate_directional_tx(obs, anchors)
    sym_hat = estimate_symmetric_composite(obs, anchors)

    tx_true_gauge = tx - tx[0]
    tx_err_rms = float(np.sqrt(np.mean((tx_hat - tx_true_gauge) ** 2)))
    sym_err_rms = float(np.sqrt(np.mean((sym_hat - tx_true_gauge) ** 2)))

    out = {}
    for name, geom_lock, cal_mode, huber, triangle in STRATEGIES:
        est = []
        for pos in truth:
            meas = []
            used_pairs = []
            for a, b in pairs:
                da = np.linalg.norm(pos - anchors[a])
                db = np.linalg.norm(pos - anchors[b])
                p = (a, b)
                remote_path = 0.0 if geom_lock else pair_bias[p]
                tag_pair = model.tag_pair_coupling * pair_bias[p] + tag_pair_ind[p]
                noise = rng.normal(0.0, model.tag_sigma)
                if rng.random() < model.outlier_prob:
                    noise += rng.normal(0.0, model.outlier_sigma)
                m = (db - da) + (tx[b] - tx[a]) + remote_path + tag_pair + noise
                if cal_mode == "directional":
                    m -= (tx_hat[b] - tx_hat[a])
                elif cal_mode == "symmetric":
                    m -= (sym_hat[b] - sym_hat[a])
                if triangle:
                    d_ab = np.linalg.norm(anchors[a] - anchors[b])
                    if abs(m) > d_ab * 1.02 + 0.02:
                        continue
                meas.append(m)
                used_pairs.append((a, b))
            if len(used_pairs) < (3 if fixed_z is not None else 4):
                used_pairs = pairs
                meas = []
                for a, b in pairs:
                    da = np.linalg.norm(pos - anchors[a])
                    db = np.linalg.norm(pos - anchors[b])
                    p = (a, b)
                    remote_path = 0.0 if geom_lock else pair_bias[p]
                    tag_pair = model.tag_pair_coupling * pair_bias[p] + tag_pair_ind[p]
                    m = (db - da) + (tx[b] - tx[a]) + remote_path + tag_pair + rng.normal(0.0, model.tag_sigma)
                    if cal_mode == "directional":
                        m -= (tx_hat[b] - tx_hat[a])
                    elif cal_mode == "symmetric":
                        m -= (sym_hat[b] - sym_hat[a])
                    meas.append(m)
            est.append(tdoa_solve(
                anchors,
                used_pairs,
                np.asarray(meas),
                center,
                fixed_z=fixed_z,
                huber_delta=0.08 if huber else None,
            ))
        out[name] = compute_metrics(rng, np.asarray(est), truth)
    return out, tx_err_rms, sym_err_rms


def summarize(values):
    keys = values[0].keys()
    return {k: float(np.mean([v[k] for v in values])) for k in keys}


def run(layouts, models, seeds, point_repeats):
    for layout in layouts:
        print(f"\n# Layout: {layout}")
        for model_name in models:
            model = MODELS[model_name]
            acc = {name: [] for name, *_ in STRATEGIES}
            tx_errs = []
            sym_errs = []
            for seed in range(seeds):
                result, tx_err, sym_err = simulate_once(10_000 + seed, layout, model, point_repeats)
                tx_errs.append(tx_err)
                sym_errs.append(sym_err)
                for k, v in result.items():
                    acc[k].append(v)
            print(f"\n## Model: {model_name}")
            print(f"calibration error: directional_tx_rms={100*np.mean(tx_errs):.2f} cm, "
                  f"symmetric_proxy_rms={100*np.mean(sym_errs):.2f} cm")
            print("| strategy | raw_p95 | off_p95 | rigid_p95 | rel_p95 | rel drop vs current |")
            print("|---|---:|---:|---:|---:|---:|")
            current_rel = summarize(acc["current"])["rel_p95"]
            for name, *_ in STRATEGIES:
                m = summarize(acc[name])
                drop = 100.0 * (current_rel - m["rel_p95"]) / current_rel if current_rel > 0 else 0.0
                print(f"| {name} | {100*m['raw_p95']:.2f} | {100*m['off_p95']:.2f} | "
                      f"{100*m['rigid_p95']:.2f} | {100*m['rel_p95']:.2f} | {drop:.0f}% |")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--layout", choices=["all", *LAYOUTS.keys()], default="all")
    parser.add_argument("--model", choices=["all", *MODELS.keys()], default="all")
    parser.add_argument("--seeds", type=int, default=12)
    parser.add_argument("--point-repeats", type=int, default=16)
    args = parser.parse_args()

    layouts = list(LAYOUTS.keys()) if args.layout == "all" else [args.layout]
    models = ["baseline", "hard_rf", "low_bias"] if args.model == "all" else [args.model]
    run(layouts, models, args.seeds, args.point_repeats)


if __name__ == "__main__":
    main()
