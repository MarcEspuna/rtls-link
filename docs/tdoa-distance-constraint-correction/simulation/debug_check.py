"""Diagnose error floor: how many unique pairs, what's the raw TDoA accuracy?"""
import numpy as np
import tdoa_sim as s

def main():
    cfg = s.make_base_cfg(twr_noise_ticks=0.0, ts_noise_ticks=0.0, nlos_prob=0.0,
                           clock_spread_ppm=0.0, n_frames=30, solve_2d=True)
    # noise-free, but still check pair coverage
    tag = np.array([1.3, 0.8, 1.0])
    rng = np.random.default_rng(0)
    obs = s.simulate_frames(cfg, tag, rng)
    tdoas = s.compute_tdoa_measurements(obs, cfg, s.AdelaySubtract())
    pairs_seen = set()
    for a, b, d in tdoas:
        pairs_seen.add(tuple(sorted((a, b))))
    print(f"Noise-free tdoas: {len(tdoas)}, unique unordered pairs: {len(pairs_seen)}")
    print(f"Pairs: {sorted(pairs_seen)}")

    # With noise, check TDoA error distribution
    cfg_n = s.make_base_cfg(twr_noise_ticks=20.0, ts_noise_ticks=2.0, solve_2d=True)
    errs = []
    for trial in range(50):
        rng = np.random.default_rng(trial)
        obs = s.simulate_frames(cfg_n, tag, rng)
        tdoas = s.compute_tdoa_measurements(obs, cfg_n, s.AdelaySubtract())
        latest = s.latest_tdoa_per_pair(tdoas)
        for (a, b, d) in latest.values():
            truth = (np.linalg.norm(cfg_n.anchor_positions[a] - tag)
                     - np.linalg.norm(cfg_n.anchor_positions[b] - tag))
            # Firmware convention: distDiff = d(b) - d(a) = -(d(a) - d(b))
            errs.append(d - (-truth))
    errs = np.asarray(errs)
    print(f"\nTDoA per-pair error (AdelaySubtract, 50 trials at tag={tag}):")
    print(f"  bias (mean): {np.mean(errs)*100:+.2f} cm")
    print(f"  std: {np.std(errs)*100:.2f} cm")
    print(f"  max |err|: {np.max(np.abs(errs))*100:.2f} cm")

    # Same for static_truth
    errs = []
    for trial in range(50):
        rng = np.random.default_rng(trial)
        obs = s.simulate_frames(cfg_n, tag, rng)
        tdoas = s.compute_tdoa_measurements(obs, cfg_n, s.StaticTruth(cfg_n.anchor_positions))
        latest = s.latest_tdoa_per_pair(tdoas)
        for (a, b, d) in latest.values():
            truth = (np.linalg.norm(cfg_n.anchor_positions[a] - tag)
                     - np.linalg.norm(cfg_n.anchor_positions[b] - tag))
            errs.append(d - (-truth))
    errs = np.asarray(errs)
    print(f"\nTDoA per-pair error (StaticTruth, 50 trials):")
    print(f"  bias: {np.mean(errs)*100:+.2f} cm")
    print(f"  std: {np.std(errs)*100:.2f} cm")

    # Position error at a few positions with 100 trials
    print("\nPosition error across tag positions (AdelaySubtract, 30 trials):")
    for pos in [np.array([1.3, 0.8, 1.0]),
                np.array([2.0, 1.5, 1.0]),
                np.array([3.8, 3.0, 1.0]),
                np.array([4.5, 3.5, 1.0])]:
        errs = []
        for trial in range(30):
            rng = np.random.default_rng(trial + 1000)
            obs = s.simulate_frames(cfg_n, pos, rng)
            tdoas = s.compute_tdoa_measurements(obs, cfg_n, s.AdelaySubtract())
            initial = pos + rng.normal(0, 0.2, 3)
            est = s.estimate_position(cfg_n, tdoas, initial)
            if est is not None:
                errs.append(np.linalg.norm(est - pos))
        errs = np.asarray(errs)
        print(f"  tag={pos}  mean={np.mean(errs)*100:.1f}cm  std={np.std(errs)*100:.1f}cm  p90={np.percentile(errs, 90)*100:.1f}cm")


if __name__ == "__main__":
    main()
