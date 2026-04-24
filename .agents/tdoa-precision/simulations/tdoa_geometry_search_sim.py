#!/usr/bin/env python3
"""Search practical anchor geometries for better Z observability.

The focus is RTLS-link's planned light-show layout:

- compact 5 m x 5 m show footprint,
- anchors mounted on tripods/stations,
- no very tall anchors,
- evaluate Z reliability near the floor, inside the volume, near the top
  anchor plane, and above the top plane.

This is a geometry/sensitivity harness. It is not a hardware-accurate UWB
propagation model.
"""

from __future__ import annotations

import argparse
import itertools
from dataclasses import dataclass

import numpy as np


def anchor_pairs(n: int) -> list[tuple[int, int]]:
    return list(itertools.combinations(range(n), 2))


@dataclass(frozen=True)
class Layout:
    name: str
    anchors: np.ndarray
    station_count: int
    description: str


def rect_corners(width: float, depth: float, margin: float = 0.0) -> list[tuple[float, float]]:
    return [
        (-margin, -margin),
        (width + margin, -margin),
        (width + margin, depth + margin),
        (-margin, depth + margin),
    ]


def station_pair_layout(
    name: str,
    xy: list[tuple[float, float]],
    z_low: float,
    z_high: float,
    description: str,
    top_xy: list[tuple[float, float]] | None = None,
) -> Layout:
    top_xy = xy if top_xy is None else top_xy
    lows = np.array([[x, y, z_low] for x, y in xy], dtype=float)
    highs = np.array([[x, y, z_high] for x, y in top_xy], dtype=float)
    return Layout(name=name, anchors=np.vstack([lows, highs]), station_count=len(xy), description=description)


def staggered_station_layout(
    name: str,
    xy: list[tuple[float, float]],
    z_lows: list[float],
    z_highs: list[float],
    description: str,
) -> Layout:
    lows = np.array([[x, y, z] for (x, y), z in zip(xy, z_lows)], dtype=float)
    highs = np.array([[x, y, z] for (x, y), z in zip(xy, z_highs)], dtype=float)
    return Layout(name=name, anchors=np.vstack([lows, highs]), station_count=len(xy), description=description)


def build_layouts(width: float = 5.0, depth: float = 5.0) -> list[Layout]:
    corners = rect_corners(width, depth)
    center = (width / 2.0, depth / 2.0)
    mid_sides = [(width / 2.0, 0.0), (width, depth / 2.0), (width / 2.0, depth), (0.0, depth / 2.0)]

    layouts = [
        station_pair_layout(
            "4tripod_cube_0p25_3p0",
            corners,
            0.25,
            3.0,
            "Baseline: 4 corner tripods, lower anchors 25 cm, upper anchors 3 m.",
        ),
        station_pair_layout(
            "4tripod_cube_0p10_3p0",
            corners,
            0.10,
            3.0,
            "Same as baseline, but lower anchors as low as practical.",
        ),
        station_pair_layout(
            "4tripod_cube_0p25_3p4",
            corners,
            0.25,
            3.4,
            "4 corner tripods with modestly higher top anchors, still not very tall.",
        ),
        staggered_station_layout(
            "4tripod_staggered_0p15_3p2",
            corners,
            [0.15, 0.35, 0.15, 0.35],
            [3.0, 3.3, 3.0, 3.3],
            "4 tripods with fixed small height staggering to break planar symmetry.",
        ),
        station_pair_layout(
            "4tripod_top_outset_0p5",
            corners,
            0.25,
            3.0,
            "Top anchors pushed 0.5 m outward from the footprint using short arms.",
            top_xy=rect_corners(width, depth, margin=0.5),
        ),
        station_pair_layout(
            "4tripod_top_inset_0p5",
            corners,
            0.25,
            3.0,
            "Top anchors inset 0.5 m toward the center.",
            top_xy=[(0.5, 0.5), (width - 0.5, 0.5), (width - 0.5, depth - 0.5), (0.5, depth - 0.5)],
        ),
        station_pair_layout(
            "5tripod_center_0p25_3p0",
            [*corners, center],
            0.25,
            3.0,
            "5 tripods / 10 anchors: four corners plus one center station.",
        ),
        station_pair_layout(
            "5tripod_center_0p10_3p0",
            [*corners, center],
            0.10,
            3.0,
            "5 tripods / 10 anchors: center station plus lower anchors at 10 cm.",
        ),
        station_pair_layout(
            "5tripod_center_0p25_3p4",
            [*corners, center],
            0.25,
            3.4,
            "5 tripods / 10 anchors: center station and modest 3.4 m top anchors.",
        ),
        staggered_station_layout(
            "5tripod_center_staggered",
            [*corners, center],
            [0.15, 0.35, 0.15, 0.35, 0.25],
            [3.0, 3.3, 3.0, 3.3, 3.15],
            "5 tripods with center station and staggered heights.",
        ),
        station_pair_layout(
            "5tripod_mid_side_plus_center",
            [*mid_sides, center],
            0.25,
            3.0,
            "5 stations at side midpoints plus center, no corner stations.",
        ),
        station_pair_layout(
            "6tripod_corners_plus_mid_x",
            [*corners, (width / 2.0, 0.0), (width / 2.0, depth)],
            0.25,
            3.0,
            "6 tripods / 12 anchors: corners plus two mid-side stations.",
        ),
    ]
    return layouts


def pair_set_for_layout(layout: Layout) -> list[tuple[int, int]]:
    """A sparse but plausible local pair set for station-pair layouts.

    Anchor ordering is all lows first, then all highs. The sparse set includes
    lower ring, upper ring, and vertical station pairs. If the layout has a
    center station after four perimeter stations, include radial center-to-corner
    pairs instead of making the center part of the perimeter ring.
    """
    n = layout.station_count
    pairs: list[tuple[int, int]] = []

    center_index = None
    if n >= 5:
        xy = layout.anchors[:n, :2]
        centroid = xy[:4].mean(axis=0)
        if np.linalg.norm(xy[4] - centroid) < 0.2:
            center_index = 4

    ring_count = 4 if center_index == 4 else n
    for i in range(ring_count):
        pairs.append((i, (i + 1) % ring_count))
        pairs.append((n + i, n + ((i + 1) % ring_count)))

    for i in range(n):
        pairs.append((i, n + i))

    if center_index is not None:
        c_low = center_index
        c_high = n + center_index
        for i in range(4):
            pairs.append((c_low, i))
            pairs.append((c_high, n + i))

    return sorted(set(tuple(sorted(p)) for p in pairs))


def grid_points(width: float, depth: float, z_values: list[float], n_xy: int = 9, pad: float = 0.35) -> np.ndarray:
    xs = np.linspace(pad, width - pad, n_xy)
    ys = np.linspace(pad, depth - pad, n_xy)
    return np.array([[x, y, z] for z in z_values for y in ys for x in xs], dtype=float)


def random_points(rng: np.random.Generator, width: float, depth: float, z_min: float, z_max: float, count: int) -> np.ndarray:
    return rng.uniform([0.35, 0.35, z_min], [width - 0.35, depth - 0.35, z_max], size=(count, 3))


def geometry_stats(anchors: np.ndarray, points: np.ndarray, pairs: list[tuple[int, int]], sigma: float = 0.03) -> dict[str, float]:
    z_sigmas = []
    xy_sigmas = []
    total_sigmas = []
    conds = []
    for point in points:
        rows = []
        for a, b in pairs:
            va = point - anchors[a]
            vb = point - anchors[b]
            da = max(np.linalg.norm(va), 1e-9)
            db = max(np.linalg.norm(vb), 1e-9)
            rows.append(va / da - vb / db)
        j = np.asarray(rows)
        info = j.T @ j
        singular_values = np.linalg.svd(info, compute_uv=False)
        conds.append(float(singular_values[0] / max(singular_values[-1], 1e-12)))
        cov = np.linalg.pinv(info) * sigma**2
        z_sigmas.append(float(np.sqrt(max(cov[2, 2], 0.0))))
        xy_sigmas.append(float(np.sqrt(max(cov[0, 0] + cov[1, 1], 0.0))))
        total_sigmas.append(float(np.sqrt(max(np.trace(cov), 0.0))))
    return {
        "z_p50": float(np.percentile(z_sigmas, 50)),
        "z_p95": float(np.percentile(z_sigmas, 95)),
        "xy_p95": float(np.percentile(xy_sigmas, 95)),
        "total_p95": float(np.percentile(total_sigmas, 95)),
        "cond_p95": float(np.percentile(conds, 95)),
    }


def predict(anchors: np.ndarray, pairs: list[tuple[int, int]], point: np.ndarray) -> np.ndarray:
    return np.asarray([np.linalg.norm(point - anchors[b]) - np.linalg.norm(point - anchors[a]) for a, b in pairs])


def solve_position(anchors: np.ndarray, pairs: list[tuple[int, int]], measurements: np.ndarray, initial: np.ndarray, huber_delta: float = 0.08) -> np.ndarray:
    x = initial.copy()
    for _ in range(20):
        residuals = []
        jac = []
        for (a, b), meas in zip(pairs, measurements):
            va = x - anchors[a]
            vb = x - anchors[b]
            da = max(np.linalg.norm(va), 1e-9)
            db = max(np.linalg.norm(vb), 1e-9)
            residuals.append((db - da) - meas)
            jac.append(vb / db - va / da)
        r = np.asarray(residuals)
        j = np.asarray(jac)
        w = np.ones_like(r)
        mask = np.abs(r) > huber_delta
        w[mask] = huber_delta / np.maximum(np.abs(r[mask]), 1e-12)
        sw = np.sqrt(w)
        lhs = (j * sw[:, None]).T @ (j * sw[:, None]) + 1e-5 * np.eye(3)
        rhs = (j * sw[:, None]).T @ (r * sw)
        try:
            step = np.linalg.solve(lhs, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
        x -= step
        if np.linalg.norm(step) < 1e-7:
            break
    return x


def monte_carlo_stats(
    rng: np.random.Generator,
    layout: Layout,
    pairs: list[tuple[int, int]],
    z_min: float,
    z_max: float,
    profile: str,
    count: int = 1200,
) -> dict[str, float]:
    if profile == "hard_rf":
        tag_sigma = 0.035
        pair_sigma = 0.040
        outlier_prob = 0.035
        outlier_sigma = 0.260
    else:
        tag_sigma = 0.025
        pair_sigma = 0.020
        outlier_prob = 0.006
        outlier_sigma = 0.120

    anchors = layout.anchors
    pts = random_points(rng, 5.0, 5.0, z_min, z_max, count)
    pair_bias = rng.normal(0.0, pair_sigma, len(pairs))
    center = anchors.mean(axis=0)
    estimates = []
    for point in pts:
        meas = predict(anchors, pairs, point)
        meas += pair_bias
        meas += rng.normal(0.0, tag_sigma, len(pairs))
        outliers = rng.random(len(pairs)) < outlier_prob
        meas[outliers] += rng.normal(0.0, outlier_sigma, int(np.count_nonzero(outliers)))
        estimates.append(solve_position(anchors, pairs, meas, center))
    est = np.asarray(estimates)
    delta = est - pts
    z = np.abs(delta[:, 2])
    xy = np.linalg.norm(delta[:, :2], axis=1)
    total = np.linalg.norm(delta, axis=1)
    return {
        "z_p95": float(np.percentile(z, 95)),
        "z_p99": float(np.percentile(z, 99)),
        "xy_p95": float(np.percentile(xy, 95)),
        "total_p95": float(np.percentile(total, 95)),
    }


def print_dop_summary(layouts: list[Layout]) -> None:
    volumes = [
        ("floor_0_0p5", [0.05, 0.25, 0.5]),
        ("show_0p5_2p5", [0.5, 1.0, 1.5, 2.0, 2.5]),
        ("near_top_2p5_3p2", [2.5, 2.8, 3.0, 3.2]),
        ("above_3p2_4p5", [3.2, 3.5, 4.0, 4.5]),
    ]
    print("# Geometry DOP summary, sigma=3 cm, units in cm")
    print("| layout | anchors | pair_mode | volume | z_p95 | xy_p95 | total_p95 | cond95 |")
    print("|---|---:|---|---|---:|---:|---:|---:|")
    for layout in layouts:
        for pair_mode, pairs in [("all", anchor_pairs(len(layout.anchors))), ("sparse", pair_set_for_layout(layout))]:
            for volume, zs in volumes:
                stats = geometry_stats(layout.anchors, grid_points(5.0, 5.0, zs), pairs)
                print(
                    f"| {layout.name} | {len(layout.anchors)} | {pair_mode} | {volume} | "
                    f"{100*stats['z_p95']:.2f} | {100*stats['xy_p95']:.2f} | "
                    f"{100*stats['total_p95']:.2f} | {stats['cond_p95']:.1f} |"
                )


def rank_layouts(layouts: list[Layout]) -> list[tuple[float, Layout]]:
    # Weighted objective: prioritize show volume and near-top volume, include
    # sparse availability so layouts do not win only in the all-pair ideal.
    weights = [
        ("show", [0.5, 1.0, 1.5, 2.0, 2.5], 0.40),
        ("near_top", [2.5, 2.8, 3.0, 3.2], 0.35),
        ("above", [3.2, 3.5, 4.0], 0.20),
        ("floor", [0.05, 0.25, 0.5], 0.05),
    ]
    ranked: list[tuple[float, Layout]] = []
    for layout in layouts:
        score = 0.0
        for _, zs, w in weights:
            pts = grid_points(5.0, 5.0, zs)
            all_stats = geometry_stats(layout.anchors, pts, anchor_pairs(len(layout.anchors)))
            sparse_stats = geometry_stats(layout.anchors, pts, pair_set_for_layout(layout))
            score += w * (0.45 * all_stats["z_p95"] + 0.55 * sparse_stats["z_p95"])
        # Small setup-complexity penalty.
        score += 0.0015 * max(0, layout.station_count - 4)
        ranked.append((score, layout))
    return sorted(ranked, key=lambda x: x[0])


def print_ranking(layouts: list[Layout]) -> list[Layout]:
    ranked = rank_layouts(layouts)
    print("\n# Ranked layouts, lower score is better")
    print("| rank | layout | stations | anchors | score_cm | description |")
    print("|---:|---|---:|---:|---:|---|")
    for i, (score, layout) in enumerate(ranked, start=1):
        print(f"| {i} | {layout.name} | {layout.station_count} | {len(layout.anchors)} | {100*score:.2f} | {layout.description} |")
    return [layout for _, layout in ranked]


def print_mc_for_finalists(layouts: list[Layout], seed: int) -> None:
    rng = np.random.default_rng(seed)
    volumes = [
        ("show_0p5_2p5", 0.5, 2.5),
        ("near_top_2p5_3p2", 2.5, 3.2),
        ("above_3p2_4p5", 3.2, 4.5),
    ]
    print("\n# Monte Carlo finalists, geom_locked+huber-like solve, units in cm")
    print("| layout | profile | pair_mode | volume | z_p95 | z_p99 | xy_p95 | total_p95 |")
    print("|---|---|---|---|---:|---:|---:|---:|")
    for layout in layouts:
        for pair_mode, pairs in [("all", anchor_pairs(len(layout.anchors))), ("sparse", pair_set_for_layout(layout))]:
            for profile in ["compact", "hard_rf"]:
                for volume, z_min, z_max in volumes:
                    stats = monte_carlo_stats(rng, layout, pairs, z_min, z_max, profile, count=1000)
                    print(
                        f"| {layout.name} | {profile} | {pair_mode} | {volume} | "
                        f"{100*stats['z_p95']:.2f} | {100*stats['z_p99']:.2f} | "
                        f"{100*stats['xy_p95']:.2f} | {100*stats['total_p95']:.2f} |"
                    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, default=20260423)
    parser.add_argument("--mc-finalists", type=int, default=5)
    parser.add_argument("--dop-only", action="store_true")
    args = parser.parse_args()

    layouts = build_layouts()
    print_dop_summary(layouts)
    ranked = print_ranking(layouts)
    if not args.dop_only:
        finalists = []
        baseline = next(layout for layout in layouts if layout.name == "4tripod_cube_0p25_3p0")
        finalists.append(baseline)
        for layout in ranked:
            if layout.name != baseline.name and layout not in finalists:
                finalists.append(layout)
            if len(finalists) >= args.mc_finalists:
                break
        print_mc_for_finalists(finalists, args.seed)


if __name__ == "__main__":
    main()
