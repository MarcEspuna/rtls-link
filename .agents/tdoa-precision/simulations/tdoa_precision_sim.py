#!/usr/bin/env python3
"""Strategy-comparison simulations for RTLS-link TDoA precision work.

The model is deliberately simple and hardware-constrained. It compares
correction strategies that can plausibly be implemented on the current
DW1000/ESP32 firmware:

- static antenna-delay correction,
- static inter-anchor geometry lock/blend,
- robust TDoA residual weighting,
- self-survey from anchor distances,
- low-latency smoothing,
- dynamic antenna-delay adaptation as a negative-control experiment.

It is not a replacement for real static-tag logs.
"""

from __future__ import annotations

import argparse
import itertools
import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


DW1000_TIME_TO_METERS = 0.004691763978616


@dataclass(frozen=True)
class NoiseProfile:
    tag_sigma_m: float = 0.025
    interanchor_sigma_m: float = 0.035
    adelay_bias_sigma_m: float = 0.030
    pair_bias_sigma_m: float = 0.025
    outlier_probability: float = 0.010
    outlier_sigma_m: float = 0.160


@dataclass(frozen=True)
class Mode:
    name: str
    adelay_factor: float
    interanchor_factor: float
    outlier_factor: float
    blend_note: str = ""


MODES = {
    "current_raw": Mode("current_raw", adelay_factor=1.0, interanchor_factor=1.0, outlier_factor=1.0),
    "static_adelay": Mode("static_adelay", adelay_factor=0.15, interanchor_factor=1.0, outlier_factor=1.0),
    "blend50": Mode("blend50", adelay_factor=0.15, interanchor_factor=0.5, outlier_factor=0.6),
    "geom_locked": Mode("geom_locked", adelay_factor=0.15, interanchor_factor=0.05, outlier_factor=0.45),
}


def anchor_pairs(n: int) -> list[tuple[int, int]]:
    return list(itertools.combinations(range(n), 2))


def anchors_2d(width: float = 5.0, depth: float = 5.0) -> np.ndarray:
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [width, 0.0, 0.0],
            [width, depth, 0.0],
            [0.0, depth, 0.0],
        ],
        dtype=float,
    )


def anchors_2d_six(width: float = 5.0, depth: float = 5.0, variant: str = "top_bottom") -> np.ndarray:
    base = anchors_2d(width, depth)
    if variant == "top_bottom":
        extra = np.array([[width / 2.0, 0.0, 0.0], [width / 2.0, depth, 0.0]])
    else:
        extra = np.array([[0.0, depth / 2.0, 0.0], [width, depth / 2.0, 0.0]])
    return np.vstack([base, extra])


def anchors_3d(width: float = 5.0, depth: float = 5.0, height: float = 4.0) -> np.ndarray:
    lower = anchors_2d(width, depth)
    upper = lower.copy()
    upper[:, 2] = height
    return np.vstack([lower, upper])


def random_points_2d(rng: np.random.Generator, count: int, width: float, depth: float, z: float = 0.0) -> np.ndarray:
    xy = rng.uniform([0.25, 0.25], [width - 0.25, depth - 0.25], size=(count, 2))
    zcol = np.full((count, 1), z)
    return np.hstack([xy, zcol])


def random_points_3d(
    rng: np.random.Generator,
    count: int,
    width: float,
    depth: float,
    z_min: float = 0.4,
    z_max: float = 3.4,
) -> np.ndarray:
    return rng.uniform([0.25, 0.25, z_min], [width - 0.25, depth - 0.25, z_max], size=(count, 3))


def grid_points_2d(width: float, depth: float, n: int = 7, z: float = 0.0) -> np.ndarray:
    xs = np.linspace(0.35, width - 0.35, n)
    ys = np.linspace(0.35, depth - 0.35, n)
    return np.array([[x, y, z] for y in ys for x in xs], dtype=float)


def grid_points_3d(width: float, depth: float, heights: Iterable[float], n: int = 5) -> np.ndarray:
    pts = []
    for z in heights:
        pts.extend(grid_points_2d(width, depth, n=n, z=z))
    return np.array(pts, dtype=float)


def tdoa_prediction(position: np.ndarray, anchors: np.ndarray, pair_list: list[tuple[int, int]]) -> np.ndarray:
    values = []
    for a, b in pair_list:
        da = np.linalg.norm(position - anchors[a])
        db = np.linalg.norm(position - anchors[b])
        values.append(db - da)
    return np.asarray(values)


def solve_tdoa(
    anchors: np.ndarray,
    pair_list: list[tuple[int, int]],
    measurements: np.ndarray,
    initial: np.ndarray,
    fixed_z: float | None,
    huber_delta: float | None = None,
    max_iter: int = 18,
) -> tuple[np.ndarray, float]:
    x = initial.astype(float).copy()
    if fixed_z is not None:
        x[2] = fixed_z

    active_dims = 2 if fixed_z is not None else 3
    damping = 1e-5

    for _ in range(max_iter):
        residuals = []
        jacobian = []

        for (a, b), meas in zip(pair_list, measurements):
            va = x - anchors[a]
            vb = x - anchors[b]
            da = max(np.linalg.norm(va), 1e-9)
            db = max(np.linalg.norm(vb), 1e-9)
            pred = db - da
            grad = vb / db - va / da
            residuals.append(pred - meas)
            jacobian.append(grad[:active_dims])

        r = np.asarray(residuals)
        j = np.asarray(jacobian)

        if huber_delta is not None:
            abs_r = np.abs(r)
            w = np.ones_like(r)
            mask = abs_r > huber_delta
            w[mask] = huber_delta / np.maximum(abs_r[mask], 1e-12)
            sw = np.sqrt(w)
            jw = j * sw[:, None]
            rw = r * sw
        else:
            jw = j
            rw = r

        lhs = jw.T @ jw + damping * np.eye(active_dims)
        rhs = jw.T @ rw
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(lhs, rhs, rcond=None)[0]

        x[:active_dims] -= step
        if fixed_z is not None:
            x[2] = fixed_z
        if np.linalg.norm(step) < 1e-7:
            break

    rmse = float(np.sqrt(np.mean((tdoa_prediction(x, anchors, pair_list) - measurements) ** 2)))
    return x, rmse


def make_pair_error_state(
    rng: np.random.Generator,
    pair_count: int,
    profile: NoiseProfile,
) -> tuple[np.ndarray, np.ndarray]:
    adelay_bias = rng.normal(0.0, profile.adelay_bias_sigma_m, size=pair_count)
    pair_bias = rng.normal(0.0, profile.pair_bias_sigma_m, size=pair_count)
    return adelay_bias, pair_bias


def noisy_measurements(
    rng: np.random.Generator,
    truth: np.ndarray,
    true_anchors: np.ndarray,
    pair_list: list[tuple[int, int]],
    mode: Mode,
    profile: NoiseProfile,
    adelay_bias: np.ndarray,
    pair_bias: np.ndarray,
) -> np.ndarray:
    base = tdoa_prediction(truth, true_anchors, pair_list)
    n = len(pair_list)

    tag_noise = rng.normal(0.0, profile.tag_sigma_m, size=n)
    interanchor_noise = rng.normal(0.0, profile.interanchor_sigma_m, size=n)
    outliers = np.zeros(n)
    mask = rng.random(n) < profile.outlier_probability * mode.outlier_factor
    outliers[mask] = rng.normal(0.0, profile.outlier_sigma_m, size=np.count_nonzero(mask))

    return (
        base
        + tag_noise
        + mode.adelay_factor * adelay_bias
        + mode.interanchor_factor * (pair_bias + interanchor_noise)
        + outliers
    )


def simulate_positions(
    rng: np.random.Generator,
    true_anchors: np.ndarray,
    configured_anchors: np.ndarray,
    points: np.ndarray,
    mode_name: str,
    profile: NoiseProfile,
    fixed_z: float | None,
    huber: bool = False,
    live_pair_count: int | None = None,
    pair_subset: list[tuple[int, int]] | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    mode = MODES[mode_name]
    all_pairs = anchor_pairs(len(true_anchors))
    pair_to_index = {tuple(sorted(pair)): idx for idx, pair in enumerate(all_pairs)}
    adelay_bias, pair_bias = make_pair_error_state(rng, len(all_pairs), profile)

    estimates = []
    rmses = []
    center = np.mean(configured_anchors, axis=0)
    if fixed_z is not None:
        center[2] = fixed_z

    for truth in points:
        if pair_subset is not None:
            pairs = pair_subset
            pair_idx = np.asarray([pair_to_index[tuple(sorted(p))] for p in pairs])
        elif live_pair_count is None or live_pair_count >= len(all_pairs):
            pair_idx = np.arange(len(all_pairs))
            pairs = all_pairs
        else:
            pair_idx = np.sort(rng.choice(len(all_pairs), size=live_pair_count, replace=False))
            pairs = [all_pairs[i] for i in pair_idx]
        meas = noisy_measurements(
            rng,
            truth,
            true_anchors,
            pairs,
            mode,
            profile,
            adelay_bias[pair_idx],
            pair_bias[pair_idx],
        )
        estimate, rmse = solve_tdoa(
            configured_anchors,
            pairs,
            meas,
            initial=center,
            fixed_z=fixed_z,
            huber_delta=0.08 if huber else None,
        )
        estimates.append(estimate)
        rmses.append(rmse)

    return np.asarray(estimates), np.asarray(rmses)


def error_stats(errors_m: np.ndarray) -> dict[str, float]:
    return {
        "mean": float(np.mean(errors_m)),
        "median": float(np.median(errors_m)),
        "p90": float(np.percentile(errors_m, 90)),
        "p95": float(np.percentile(errors_m, 95)),
        "p99": float(np.percentile(errors_m, 99)),
    }


def print_stats(name: str, errors_m: np.ndarray) -> None:
    s = error_stats(errors_m)
    print(
        f"{name:24s} mean={s['mean']:.3f} m  median={s['median']:.3f} m  "
        f"p95={s['p95']:.3f} m  p99={s['p99']:.3f} m"
    )


def procrustes_map(est: np.ndarray, truth: np.ndarray, scale: bool = False, allow_reflection: bool = False) -> np.ndarray:
    x_mu = np.mean(est, axis=0)
    y_mu = np.mean(truth, axis=0)
    x0 = est - x_mu
    y0 = truth - y_mu
    h = x0.T @ y0
    u, singular_values, vt = np.linalg.svd(h)
    # Row-vector Kabsch alignment for min ||X R - Y||.
    r = u @ vt
    if not allow_reflection and np.linalg.det(r) < 0:
        u[:, -1] *= -1
        r = u @ vt
    s = 1.0
    if scale:
        denom = np.sum(x0 * x0)
        if denom > 1e-12:
            s = float(np.sum(singular_values) / denom)
    return s * (x0 @ r) + y_mu


def consistency_metrics(
    rng: np.random.Generator,
    estimates: np.ndarray,
    truth: np.ndarray,
    relative_samples: int = 8000,
) -> dict[str, float]:
    raw = np.linalg.norm(estimates - truth, axis=1)
    offset = np.mean(estimates - truth, axis=0)
    off = np.linalg.norm((estimates - offset) - truth, axis=1)
    rigid = np.linalg.norm(procrustes_map(estimates, truth, scale=False) - truth, axis=1)
    sim = np.linalg.norm(procrustes_map(estimates, truth, scale=True) - truth, axis=1)

    idx_a = rng.integers(0, len(truth), size=relative_samples)
    idx_b = rng.integers(0, len(truth), size=relative_samples)
    rel_est = estimates[idx_a] - estimates[idx_b]
    rel_truth = truth[idx_a] - truth[idx_b]
    rel = np.linalg.norm(rel_est - rel_truth, axis=1)

    return {
        "raw_p95": float(np.percentile(raw, 95)),
        "off_p95": float(np.percentile(off, 95)),
        "rigid_p95": float(np.percentile(rigid, 95)),
        "sim_p95": float(np.percentile(sim, 95)),
        "rel_p95": float(np.percentile(rel, 95)),
    }


def print_consistency(name: str, metrics: dict[str, float]) -> None:
    print(
        f"{name:34s} raw_p95={100*metrics['raw_p95']:.2f} cm  "
        f"off_p95={100*metrics['off_p95']:.2f} cm  "
        f"rigid_p95={100*metrics['rigid_p95']:.2f} cm  "
        f"sim_p95={100*metrics['sim_p95']:.2f} cm  "
        f"rel_p95={100*metrics['rel_p95']:.2f} cm"
    )


def geometry_dop_stats(
    anchors: np.ndarray,
    points: np.ndarray,
    pair_list: list[tuple[int, int]],
    measurement_sigma_m: float = 0.03,
) -> dict[str, float]:
    z_sigmas = []
    xy_sigmas = []
    conds = []

    for point in points:
        j_rows = []
        for a, b in pair_list:
            va = point - anchors[a]
            vb = point - anchors[b]
            da = max(np.linalg.norm(va), 1e-9)
            db = max(np.linalg.norm(vb), 1e-9)
            j_rows.append(va / da - vb / db)

        j = np.asarray(j_rows)
        info = j.T @ j
        singular_values = np.linalg.svd(info, compute_uv=False)
        conds.append(float(singular_values[0] / max(singular_values[-1], 1e-12)))
        cov = np.linalg.pinv(info) * (measurement_sigma_m**2)
        xy_sigmas.append(float(np.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))))
        z_sigmas.append(float(np.sqrt(max(cov[2, 2], 0.0))))

    return {
        "xy_sigma_p50": float(np.percentile(xy_sigmas, 50)),
        "xy_sigma_p95": float(np.percentile(xy_sigmas, 95)),
        "z_sigma_p50": float(np.percentile(z_sigmas, 50)),
        "z_sigma_p95": float(np.percentile(z_sigmas, 95)),
        "condition_p95": float(np.percentile(conds, 95)),
    }


def component_error_stats(estimates: np.ndarray, truth: np.ndarray) -> dict[str, float]:
    delta = estimates - truth
    xy = np.linalg.norm(delta[:, :2], axis=1)
    z = np.abs(delta[:, 2])
    total = np.linalg.norm(delta, axis=1)
    return {
        "xy_p95": float(np.percentile(xy, 95)),
        "z_p50": float(np.percentile(z, 50)),
        "z_p95": float(np.percentile(z, 95)),
        "z_p99": float(np.percentile(z, 99)),
        "total_p95": float(np.percentile(total, 95)),
    }


def print_height_row(
    label: str,
    height: float,
    dop: dict[str, float],
    err: dict[str, float],
) -> None:
    print(
        f"{label:21s} H={height:4.2f}m  "
        f"DOP z1_p95={100*dop['z_sigma_p95']:5.2f}cm  "
        f"DOP xy1_p95={100*dop['xy_sigma_p95']:5.2f}cm  "
        f"cond95={dop['condition_p95']:6.1f}  "
        f"MC z_p50={100*err['z_p50']:5.2f}cm  "
        f"z_p95={100*err['z_p95']:5.2f}cm  "
        f"z_p99={100*err['z_p99']:5.2f}cm  "
        f"xy_p95={100*err['xy_p95']:5.2f}cm  "
        f"total_p95={100*err['total_p95']:5.2f}cm"
    )


def scenario_baseline_2d(rng: np.random.Generator) -> None:
    print("\n# Baseline 2D, 5 m x 5 m, 4 anchors")
    true_anchors = anchors_2d(5.0, 5.0)
    points = random_points_2d(rng, 2500, 5.0, 5.0, z=0.0)
    profile = NoiseProfile()
    for mode_name, huber in [
        ("current_raw", False),
        ("static_adelay", False),
        ("geom_locked", False),
        ("blend50", False),
        ("current_raw", True),
        ("geom_locked", True),
    ]:
        est, _ = simulate_positions(
            rng,
            true_anchors,
            true_anchors.copy(),
            points,
            mode_name,
            profile,
            fixed_z=0.0,
            huber=huber,
        )
        label = mode_name + ("+huber" if huber else "")
        print_stats(label, np.linalg.norm(est - points, axis=1))

    print("\n# Hard RF / outlier-heavy 2D")
    hard = NoiseProfile(
        tag_sigma_m=0.035,
        interanchor_sigma_m=0.060,
        adelay_bias_sigma_m=0.040,
        pair_bias_sigma_m=0.040,
        outlier_probability=0.035,
        outlier_sigma_m=0.260,
    )
    for mode_name, huber in [
        ("current_raw", False),
        ("static_adelay", False),
        ("geom_locked", False),
        ("blend50", False),
        ("geom_locked", True),
    ]:
        est, _ = simulate_positions(
            rng,
            true_anchors,
            true_anchors.copy(),
            points,
            mode_name,
            hard,
            fixed_z=0.0,
            huber=huber,
        )
        label = mode_name + ("+huber" if huber else "")
        print_stats(label, np.linalg.norm(est - points, axis=1))

    print("\n# Minimum live diversity, 4 of 6 pairs")
    for mode_name in ["current_raw", "static_adelay", "geom_locked", "blend50"]:
        est, _ = simulate_positions(
            rng,
            true_anchors,
            true_anchors.copy(),
            points,
            mode_name,
            profile,
            fixed_z=0.0,
            live_pair_count=4,
        )
        print_stats(mode_name, np.linalg.norm(est - points, axis=1))


def scenario_anchor_count_2d(rng: np.random.Generator) -> None:
    print("\n# Anchor count and geometry, 2D geom_locked+huber")
    configs = [
        ("4 corners", anchors_2d(5.0, 5.0)),
        ("5 incl center", np.vstack([anchors_2d(5.0, 5.0), [[2.5, 2.5, 0.0]]])),
        ("6 mid top/bottom", anchors_2d_six(5.0, 5.0, "top_bottom")),
        ("6 mid sides", anchors_2d_six(5.0, 5.0, "sides")),
    ]
    points = random_points_2d(rng, 3000, 5.0, 5.0, z=0.0)
    profile = NoiseProfile(tag_sigma_m=0.022, interanchor_sigma_m=0.025, outlier_probability=0.006)
    for name, anchors in configs:
        est, _ = simulate_positions(rng, anchors, anchors.copy(), points, "geom_locked", profile, fixed_z=0.0, huber=True)
        print_stats(name, np.linalg.norm(est - points, axis=1))


def scenario_consistency_2d(rng: np.random.Generator) -> None:
    print("\n# 2D consistency, 5 m x 5 m")
    true_anchors = anchors_2d(5.0, 5.0)
    grid = grid_points_2d(5.0, 5.0, n=9, z=0.0)
    repeats = 60
    points = np.repeat(grid, repeats, axis=0)
    profile = NoiseProfile()

    for mode_name, huber in [
        ("current_raw", False),
        ("static_adelay", False),
        ("geom_locked", False),
        ("geom_locked", True),
    ]:
        est, _ = simulate_positions(rng, true_anchors, true_anchors.copy(), points, mode_name, profile, fixed_z=0.0, huber=huber)
        label = "manual exact " + mode_name + ("+huber" if huber else "")
        print_consistency(label, consistency_metrics(rng, est, points))

    configured = true_anchors.copy()
    configured[1, 0] += 0.05
    est, _ = simulate_positions(rng, true_anchors, configured, points, "geom_locked", profile, fixed_z=0.0, huber=True)
    print_consistency("manual 5 cm survey error", consistency_metrics(rng, est, points))


def scenario_consistency_3d(rng: np.random.Generator) -> None:
    print("\n# 3D consistency, 8 anchors, 5 m x 5 m x 4 m")
    true_anchors = anchors_3d(5.0, 5.0, 4.0)
    grid = grid_points_3d(5.0, 5.0, heights=[0.7, 1.8, 3.0], n=5)
    points = np.repeat(grid, 70, axis=0)
    profile = NoiseProfile(tag_sigma_m=0.025, interanchor_sigma_m=0.032, outlier_probability=0.008)

    for mode_name, huber in [
        ("current_raw", False),
        ("static_adelay", False),
        ("geom_locked", False),
        ("geom_locked", True),
    ]:
        est, _ = simulate_positions(rng, true_anchors, true_anchors.copy(), points, mode_name, profile, fixed_z=None, huber=huber)
        label = "manual exact " + mode_name + ("+huber" if huber else "")
        print_consistency(label, consistency_metrics(rng, est, points))

    configured = true_anchors.copy()
    configured[6] += np.array([0.05, -0.03, 0.03])
    est, _ = simulate_positions(rng, true_anchors, configured, points, "geom_locked", profile, fixed_z=None, huber=True)
    print_consistency("manual 5 cm survey error", consistency_metrics(rng, est, points))

    print("\n# 3D sparse live pairs, 12 of 28")
    for mode_name, huber in [
        ("static_adelay", False),
        ("geom_locked", False),
        ("blend50", False),
        ("geom_locked", True),
    ]:
        est, _ = simulate_positions(
            rng,
            true_anchors,
            true_anchors.copy(),
            points,
            mode_name,
            profile,
            fixed_z=None,
            huber=huber,
            live_pair_count=12,
        )
        label = mode_name + ("+huber" if huber else "")
        print_consistency(label, consistency_metrics(rng, est, points))


def scenario_height_separation_3d(rng: np.random.Generator) -> None:
    print("\n# 3D height-separation sensitivity, 8 anchors, 5 m x 5 m footprint")
    print("# DOP assumes 3 cm independent TDoA distance-difference noise.")
    print("# MC uses geom_locked+huber with the compact stochastic model.")

    width = 5.0
    depth = 5.0
    heights = [2.0, 3.0, 4.0, 5.0]
    profile = NoiseProfile(tag_sigma_m=0.025, interanchor_sigma_m=0.025, outlier_probability=0.006, outlier_sigma_m=0.120)

    volumes = [
        ("inside scaled", lambda h: (0.20 * h, 0.80 * h)),
        ("fixed 0.4-1.6", lambda h: (0.4, min(1.6, h - 0.1))),
        ("stress 0.4-2.4", lambda h: (0.4, 2.4)),
        ("high 1.6-2.8", lambda h: (1.6, 2.8)),
    ]

    for label, z_range_fn in volumes:
        print(f"\n## Volume: {label}")
        for height in heights:
            z_min, z_max = z_range_fn(height)
            if z_max <= z_min:
                continue

            anchors = anchors_3d(width, depth, height)
            pairs = anchor_pairs(len(anchors))
            points = random_points_3d(rng, 1800, width, depth, z_min=z_min, z_max=z_max)

            dop = geometry_dop_stats(anchors, points, pairs, measurement_sigma_m=0.03)
            est, _ = simulate_positions(
                rng,
                anchors,
                anchors.copy(),
                points,
                "geom_locked",
                profile,
                fixed_z=None,
                huber=True,
            )
            err = component_error_stats(est, points)
            print_height_row(label, height, dop, err)

    print("\n## Sparse 12-pair comparison, fixed 0.4-1.6 m volume")
    sparse = sparse_pairs_for_eight_anchor_layout()
    for height in heights:
        anchors = anchors_3d(width, depth, height)
        points = random_points_3d(rng, 1800, width, depth, z_min=0.4, z_max=min(1.6, height - 0.1))
        dop = geometry_dop_stats(anchors, points, sparse, measurement_sigma_m=0.03)
        est, _ = simulate_positions(
            rng,
            anchors,
            anchors.copy(),
            points,
            "geom_locked",
            profile,
            fixed_z=None,
            huber=True,
            pair_subset=sparse,
        )
        err = component_error_stats(est, points)
        print_height_row("sparse fixed", height, dop, err)

    print("\n## Sparse 12-pair comparison, high 1.6-2.8 m volume")
    for height in heights:
        anchors = anchors_3d(width, depth, height)
        points = random_points_3d(rng, 1800, width, depth, z_min=1.6, z_max=2.8)
        dop = geometry_dop_stats(anchors, points, sparse, measurement_sigma_m=0.03)
        est, _ = simulate_positions(
            rng,
            anchors,
            anchors.copy(),
            points,
            "geom_locked",
            profile,
            fixed_z=None,
            huber=True,
            pair_subset=sparse,
        )
        err = component_error_stats(est, points)
        print_height_row("sparse high", height, dop, err)


def scenario_height_separation_fine_3d(rng: np.random.Generator) -> None:
    print("\n# Fine 3D height-separation sweep, 8 anchors, 5 m x 5 m footprint")
    print("# Heights: 2.00 m to 4.00 m in 0.25 m increments.")
    print("# DOP assumes 3 cm independent TDoA distance-difference noise.")
    print("# MC uses geom_locked+huber with the compact stochastic model.")

    width = 5.0
    depth = 5.0
    heights = np.arange(2.0, 4.0001, 0.25)
    profile = NoiseProfile(tag_sigma_m=0.025, interanchor_sigma_m=0.025, outlier_probability=0.006, outlier_sigma_m=0.120)

    cases = [
        ("all28 fixed", None, 0.4, 1.6),
        ("all28 high", None, 1.6, 2.8),
        ("sparse12 fixed", sparse_pairs_for_eight_anchor_layout(), 0.4, 1.6),
        ("sparse12 high", sparse_pairs_for_eight_anchor_layout(), 1.6, 2.8),
    ]

    for label, pair_subset, z_min, z_max in cases:
        print(f"\n## {label}, z={z_min:.1f}-{z_max:.1f} m")
        for height in heights:
            anchors = anchors_3d(width, depth, float(height))
            pairs = pair_subset if pair_subset is not None else anchor_pairs(len(anchors))
            points = random_points_3d(rng, 1400, width, depth, z_min=z_min, z_max=z_max)
            dop = geometry_dop_stats(anchors, points, pairs, measurement_sigma_m=0.03)
            est, _ = simulate_positions(
                rng,
                anchors,
                anchors.copy(),
                points,
                "geom_locked",
                profile,
                fixed_z=None,
                huber=True,
                pair_subset=pair_subset,
            )
            err = component_error_stats(est, points)
            print_height_row(label, float(height), dop, err)


def sparse_pairs_for_eight_anchor_layout() -> list[tuple[int, int]]:
    return [
        (0, 1), (1, 2), (2, 3), (3, 0),
        (4, 5), (5, 6), (6, 7), (7, 4),
        (0, 4), (1, 5), (2, 6), (3, 7),
    ]


def classical_mds(distances: np.ndarray, dims: int) -> np.ndarray:
    n = distances.shape[0]
    d2 = distances**2
    j = np.eye(n) - np.ones((n, n)) / n
    b = -0.5 * j @ d2 @ j
    eigvals, eigvecs = np.linalg.eigh(b)
    order = np.argsort(eigvals)[::-1]
    eigvals = eigvals[order]
    eigvecs = eigvecs[:, order]
    vals = np.maximum(eigvals[:dims], 0.0)
    return eigvecs[:, :dims] * np.sqrt(vals)


def average_anchor_distances(
    rng: np.random.Generator,
    anchors: np.ndarray,
    samples_per_pair: int,
    corrected: bool,
) -> np.ndarray:
    n = len(anchors)
    distances = np.zeros((n, n), dtype=float)
    for i, j in itertools.combinations(range(n), 2):
        true_d = np.linalg.norm(anchors[i] - anchors[j])
        static_bias = rng.normal(0.0, 0.018 if corrected else 0.075)
        sample_noise = rng.normal(0.0, 0.045 if corrected else 0.055, size=samples_per_pair)
        measured = true_d + static_bias + sample_noise
        d = max(float(np.mean(measured)), 0.01)
        distances[i, j] = distances[j, i] = d
    return distances


def cuboid_from_distances(distances: np.ndarray) -> np.ndarray:
    bottom = [0, 1, 2, 3]
    top = [4, 5, 6, 7]
    width = np.mean([distances[0, 1], distances[2, 3], distances[4, 5], distances[6, 7]])
    depth = np.mean([distances[1, 2], distances[3, 0], distances[5, 6], distances[7, 4]])
    height = np.mean([distances[b, t] for b, t in zip(bottom, top)])
    return anchors_3d(float(width), float(depth), float(height))


def scenario_self_survey(rng: np.random.Generator) -> None:
    print("\n# Self-survey dimension error, 3D cuboid")
    true_anchors = anchors_3d(5.0, 5.0, 4.0)
    true_dims = np.array([5.0, 5.0, 4.0])
    for corrected in [True, False]:
        label = "corrected" if corrected else "raw_uncorrected"
        for samples in [10, 20, 50, 250]:
            dim_errors = []
            mds_rms = []
            for _ in range(300):
                distances = average_anchor_distances(rng, true_anchors, samples, corrected=corrected)
                cuboid = cuboid_from_distances(distances)
                dims = np.array([cuboid[1, 0] - cuboid[0, 0], cuboid[2, 1] - cuboid[1, 1], cuboid[4, 2] - cuboid[0, 2]])
                dim_errors.append(np.max(np.abs(dims - true_dims)))

                mds = classical_mds(distances, dims=3)
                aligned = procrustes_map(mds, true_anchors, scale=False, allow_reflection=True)
                mds_rms.append(np.sqrt(np.mean(np.sum((aligned - true_anchors) ** 2, axis=1))))

            print(
                f"{label:16s} samples={samples:3d}  "
                f"cuboid_dim_p95={100*np.percentile(dim_errors,95):.2f} cm  "
                f"mds_anchor_rms_p95={100*np.percentile(mds_rms,95):.2f} cm"
            )


def scenario_quantization(rng: np.random.Generator) -> None:
    print("\n# Timestamp tick quantization")
    anchors = anchors_2d(5.0, 5.0)
    pairs = anchor_pairs(len(anchors))
    points = random_points_2d(rng, 3000, 5.0, 5.0, z=0.0)
    center = np.mean(anchors, axis=0)

    for fraction in [1.0, 0.1]:
        estimates = []
        for p in points:
            meas = tdoa_prediction(p, anchors, pairs)
            quantized = np.round(meas / (DW1000_TIME_TO_METERS * fraction)) * (DW1000_TIME_TO_METERS * fraction)
            est, _ = solve_tdoa(anchors, pairs, quantized, center, fixed_z=0.0)
            estimates.append(est)
        errors = np.linalg.norm(np.asarray(estimates) - points, axis=1)
        print(
            f"tick_fraction={fraction:3.1f}  mean={1000*np.mean(errors):.3f} mm  "
            f"p95={1000*np.percentile(errors,95):.3f} mm  p99={1000*np.percentile(errors,99):.3f} mm"
        )


def scenario_smoothing(rng: np.random.Generator) -> None:
    print("\n# Low-latency EMA smoothing")
    rate_hz = 50.0
    dt = 1.0 / rate_hz
    t = np.arange(0.0, 20.0, dt)
    truth = np.column_stack(
        [
            2.5 + 1.3 * np.sin(2 * np.pi * 0.10 * t),
            2.5 + 1.0 * np.cos(2 * np.pi * 0.13 * t),
            1.5 + 0.3 * np.sin(2 * np.pi * 0.20 * t),
        ]
    )
    meas = truth + rng.normal(0.0, 0.05, size=truth.shape)
    outlier_mask = rng.random(len(t)) < 0.01
    meas[outlier_mask] += rng.normal(0.0, 0.35, size=(np.count_nonzero(outlier_mask), 3))

    for alpha in [0.60, 0.40, 0.25]:
        filt = np.empty_like(meas)
        filt[0] = meas[0]
        for i in range(1, len(meas)):
            filt[i] = alpha * meas[i] + (1.0 - alpha) * filt[i - 1]

        raw_err = np.linalg.norm(meas - truth, axis=1)
        filt_err = np.linalg.norm(filt - truth, axis=1)
        approx_delay_ms = 1000.0 * ((1.0 - alpha) / alpha) * dt
        print(
            f"alpha={alpha:.2f}  raw_p95={100*np.percentile(raw_err,95):.2f} cm  "
            f"filtered_p95={100*np.percentile(filt_err,95):.2f} cm  "
            f"filtered_mean={100*np.mean(filt_err):.2f} cm  "
            f"approx_delay={approx_delay_ms:.1f} ms"
        )


def scenario_dynamic_adelay(rng: np.random.Generator) -> None:
    print("\n# Dynamic antenna-delay adaptation")
    true_distance = 5.0
    static_bias = 0.035
    samples = 4000
    raw = true_distance + static_bias + rng.normal(0.0, 0.035, size=samples)
    outlier_mask = rng.random(samples) < 0.025
    raw[outlier_mask] += rng.normal(0.0, 0.30, size=np.count_nonzero(outlier_mask))

    for beta in [0.0, 0.02, 0.10, 0.50, 1.00]:
        correction = 0.0
        corrected = []
        jumps = []
        for z in raw:
            residual = (z - correction) - true_distance
            delta = beta * residual
            correction += delta
            corrected.append(z - correction)
            jumps.append(abs(delta))
        corrected = np.asarray(corrected)
        residuals = corrected - true_distance
        print(
            f"beta={beta:4.2f}  residual_p95={100*np.percentile(np.abs(residuals),95):.2f} cm  "
            f"jump_p95={100*np.percentile(jumps,95):.2f} cm  "
            f"final_correction={100*correction:.2f} cm"
        )


def run_all(rng: np.random.Generator) -> None:
    scenario_baseline_2d(rng)
    scenario_anchor_count_2d(rng)
    scenario_consistency_2d(rng)
    scenario_consistency_3d(rng)
    scenario_height_separation_3d(rng)
    scenario_height_separation_fine_3d(rng)
    scenario_self_survey(rng)
    scenario_quantization(rng)
    scenario_smoothing(rng)
    scenario_dynamic_adelay(rng)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        default="all",
        choices=[
            "all",
            "baseline-2d",
            "anchor-count-2d",
            "consistency-2d",
            "consistency-3d",
            "height-separation-3d",
            "height-separation-fine-3d",
            "self-survey",
            "quantization",
            "smoothing",
            "dynamic-adelay",
        ],
    )
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)
    if args.scenario == "all":
        run_all(rng)
    elif args.scenario == "baseline-2d":
        scenario_baseline_2d(rng)
    elif args.scenario == "anchor-count-2d":
        scenario_anchor_count_2d(rng)
    elif args.scenario == "consistency-2d":
        scenario_consistency_2d(rng)
    elif args.scenario == "consistency-3d":
        scenario_consistency_3d(rng)
    elif args.scenario == "height-separation-3d":
        scenario_height_separation_3d(rng)
    elif args.scenario == "height-separation-fine-3d":
        scenario_height_separation_fine_3d(rng)
    elif args.scenario == "self-survey":
        scenario_self_survey(rng)
    elif args.scenario == "quantization":
        scenario_quantization(rng)
    elif args.scenario == "smoothing":
        scenario_smoothing(rng)
    elif args.scenario == "dynamic-adelay":
        scenario_dynamic_adelay(rng)
    else:
        raise AssertionError(args.scenario)


if __name__ == "__main__":
    main()
