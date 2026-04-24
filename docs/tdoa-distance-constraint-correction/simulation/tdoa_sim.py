"""
Hardware-realistic TDoA simulation for DW1000 + ESP32-S3 RTLS.

Mirrors firmware semantics in lib/tdoa_algorithm (Bitcraze TDoA2 port):

    delta_txAr_to_txAn_in_cl_An = tof_Ar_to_An_in_cl_An + (txAn_in_cl_An - rxAr_by_An_in_cl_An)
    TDoA_in_cl_T               = (rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T)
                                  - delta_txAr_to_txAn_in_cl_An * clockCorrection
    distDiff                   = SPEED_OF_LIGHT * TDoA / LOCODECK_TS_FREQ

Models:
  * Per-device clock rate (±20 ppm).
  * Per-anchor antenna delay (a[i] in ticks); measured ToF between i,j = true + a[i] + a[j]
    (in TDoA mode firmware sets hardware ADelay = 0).
  * Gaussian DW1000 timestamp noise on every TX/RX event.
  * Independent DS-TWR noise on the inter-anchor distances[] broadcast.
  * Optional NLOS outliers on the DS-TWR inter-anchor ToF.
  * 4-anchor TDMA round-robin, frame = slot_len * N.

Position is solved with the same Newton-Raphson formulation the firmware uses
(see lib/tdoa_newton_raphson/desktop_implementation/tdoa_newton_raphson.cpp).

Run this file directly to execute the sweeps declared in run_sweeps().
"""

from __future__ import annotations

import dataclasses
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np

# --------------------------------------------------------------------------
# Physical / hardware constants (match firmware exactly)
# --------------------------------------------------------------------------
SPEED_OF_LIGHT = 299_792_458.0            # physicalConstants.h
LOCODECK_TS_FREQ = 499.2e6 * 128          # locodeck.h, 63_897_600_000 Hz
TICKS_PER_METER = LOCODECK_TS_FREQ / SPEED_OF_LIGHT   # ~213.14 ticks/m
DW1000_TIME_TO_METERS = 1.0 / TICKS_PER_METER         # 0.004691763978616 m/tick

# DW1000 uses 40-bit timestamps
DW_TS_MASK = (1 << 40) - 1

# --------------------------------------------------------------------------
# World configuration
# --------------------------------------------------------------------------
@dataclass
class AnchorConfig:
    id: int
    pos: np.ndarray               # 3-vector meters
    adelay_ticks: int             # calibrated antenna delay (per device, one side)
    clock_ppm: float              # actual clock rate deviation in ppm


@dataclass
class SimConfig:
    # Geometry
    anchor_positions: np.ndarray    # (N, 3)
    # Hardware
    anchor_adelays_ticks: np.ndarray          # (N,) per-anchor antenna delay
    anchor_clock_ppm: np.ndarray              # (N,) per-anchor clock ppm
    tag_clock_ppm: float = 0.0
    # DW1000 timestamp noise applied to every TX/RX event, in ticks (1 tick ~4.7 mm)
    ts_noise_ticks: float = 2.0               # ~30 ps RMS, ~1 cm @ 1σ
    # Independent DS-TWR noise on packet.distances[] broadcast by anchors (ticks)
    twr_noise_ticks: float = 30.0             # ~14 cm 1σ (representative)
    # NLOS outlier probability + magnitude (ticks) for DS-TWR
    twr_nlos_prob: float = 0.0
    twr_nlos_bias_ticks: float = 200.0        # ~1 m bias when NLOS
    # TDMA
    slot_len_us: float = 2000.0
    active_slots: int = 4
    n_frames: int = 30                        # simulate this many frames per trial
    # How many frames to "warm up" before collecting measurements
    warmup_frames: int = 5
    # Position solver
    solver_iterations: int = 20
    solve_2d: bool = False
    fixed_z_2d: float = 1.0                   # assumed tag z in 2D solve (matches firmware ASSUMED_TAG_Z)
    initial_guess_noise_m: float = 0.1        # initial position noise (warm-start-like)


def rectangle_anchors(dx: float, dy: float, z: float = 2.5) -> np.ndarray:
    """4-anchor rectangle: A0=(0,0), A1=(dx,0), A2=(dx,dy), A3=(0,dy)."""
    return np.array([
        [0.0, 0.0, z],
        [dx,  0.0, z],
        [dx,  dy,  z],
        [0.0, dy,  z],
    ])


# --------------------------------------------------------------------------
# Core simulation: generate per-frame packet observations and TDoA measurements
# --------------------------------------------------------------------------
@dataclass
class PacketObservation:
    """What the tag observes when it receives anchor `sender`'s packet in a given frame."""
    sender_id: int
    frame_idx: int
    # Tag's RX timestamp of this packet (in tag ticks), with noise
    tag_rx_ticks: int
    # The broadcast `timestamps[anchor]` field: sender's own TX timestamp in sender clock
    tx_in_sender_clock: int
    # sender broadcast rx timestamps of *other* anchors (in sender's clock)
    # dict[other_id] -> rx_ticks (0 if not available)
    remote_rx_in_sender_clock: Dict[int, int]
    # sender broadcast inter-anchor DS-TWR ToF (ticks) for each other anchor (raw = includes adelays)
    # dict[other_id] -> raw tof_ticks
    remote_tof_raw: Dict[int, int]
    # metadata
    sender_adelay: int


def simulate_frames(cfg: SimConfig, tag_pos: np.ndarray, rng: np.random.Generator) -> List[PacketObservation]:
    """
    Simulate the TDMA protocol end-to-end.

    Returns the ordered list of every packet the tag receives.
    Each PacketObservation carries everything the tag engine needs.
    """
    N = cfg.anchor_positions.shape[0]
    assert cfg.active_slots == N, "sim supports active_slots == N for now"

    # Clock rates (fs * (1 + ppm * 1e-6))
    anchor_rate = LOCODECK_TS_FREQ * (1.0 + cfg.anchor_clock_ppm * 1e-6)   # (N,)
    tag_rate = LOCODECK_TS_FREQ * (1.0 + cfg.tag_clock_ppm * 1e-6)

    # Random initial clock offsets (not important for TDoA but realistic)
    anchor_offset = rng.uniform(0, 1 << 30, size=N)
    tag_offset = rng.uniform(0, 1 << 30)

    # Propagation distances (meters) and ideal times (seconds)
    # anchor <-> anchor
    d_aa = np.linalg.norm(
        cfg.anchor_positions[:, None, :] - cfg.anchor_positions[None, :, :], axis=-1
    )  # (N, N)
    # anchor -> tag
    d_at = np.linalg.norm(cfg.anchor_positions - tag_pos[None, :], axis=-1)  # (N,)
    t_at = d_at / SPEED_OF_LIGHT
    t_aa = d_aa / SPEED_OF_LIGHT

    slot_len_s = cfg.slot_len_us * 1e-6
    frame_len_s = slot_len_s * cfg.active_slots

    # Per-anchor broadcast state
    # last_tx_ticks[sender] = the sender's TX timestamp of its most recent frame (in sender ticks)
    last_tx_ticks: Dict[int, int] = {}
    # last_rx_ticks[receiver][sender] = receiver's RX timestamp of sender's most recent packet (in receiver ticks)
    last_rx_ticks: Dict[Tuple[int, int], int] = {}
    # last_tof_broadcast[sender][other] = the DS-TWR ToF sender computed for other (ticks)
    last_tof_ticks: Dict[Tuple[int, int], int] = {}

    observations: List[PacketObservation] = []

    def add_noise_int(val_ticks: float, sigma: float) -> int:
        return int(round(val_ticks + rng.normal(0.0, sigma)))

    for frame in range(cfg.n_frames):
        frame_start_s = frame * frame_len_s

        # Each anchor transmits in its own slot in order 0, 1, ..., N-1
        # We need to model the fact that each anchor schedules its TX relative to its own clock.
        # Use the wall-time slot start as the "intended" TX time for simplicity;
        # real TDMA alignment via slot 0 resync gives ~the same result.
        for sender in range(N):
            wall_tx_s = frame_start_s + sender * slot_len_s + 1e-4  # +offset within slot

            # Sender's scheduled TX timestamp in its own clock (no ts noise, it's scheduled)
            sender_tx_ticks = int(wall_tx_s * anchor_rate[sender] + anchor_offset[sender])
            last_tx_ticks[sender] = sender_tx_ticks

            # Compute DS-TWR inter-anchor ToF the sender would broadcast for each peer.
            # In the firmware, this is calculated inside handleRxPacket() whenever
            # peer's packet sequence aligns. We simplify by broadcasting the true value
            # plus the per-pair raw offset a[sender]+a[peer] and TWR noise.
            sender_distances: Dict[int, int] = {}
            for peer in range(N):
                if peer == sender:
                    continue
                true_tof_ticks = t_aa[sender, peer] * anchor_rate[sender]
                raw = (true_tof_ticks
                       + cfg.anchor_adelays_ticks[sender]
                       + cfg.anchor_adelays_ticks[peer]
                       + rng.normal(0.0, cfg.twr_noise_ticks))
                if cfg.twr_nlos_prob > 0 and rng.random() < cfg.twr_nlos_prob:
                    raw += rng.uniform(0.5, 1.0) * cfg.twr_nlos_bias_ticks
                sender_distances[peer] = max(int(round(raw)), 0)
                last_tof_ticks[(sender, peer)] = sender_distances[peer]

            # Anchor-to-anchor packet reception (receivers)
            remote_rx_in_sender: Dict[int, int] = {}
            for peer in range(N):
                if peer == sender:
                    continue
                # sender just transmitted; peer receives it at wall_tx_s + t_aa[sender,peer]
                # but this loop represents the sender describing *its own* RX of previous peer packets.
                pass

            # The sender broadcasts its RX of *other anchors' previous packets*.
            # Those values were recorded earlier in last_rx_ticks[(sender, other)].
            for other in range(N):
                if other == sender:
                    continue
                key = (sender, other)
                remote_rx_in_sender[other] = last_rx_ticks.get(key, 0)

            # Now model reception of this packet by every other anchor AND by the tag.
            # Each receiver records their RX timestamp.
            for receiver in range(N):
                if receiver == sender:
                    continue
                wall_rx_s = wall_tx_s + t_aa[sender, receiver]
                # RX timestamp in receiver clock = wall_rx * anchor_rate[receiver] + offset
                ideal = wall_rx_s * anchor_rate[receiver] + anchor_offset[receiver]
                # Add antenna-delay-equivalent shift: in TDoA mode the firmware sets hw ADelay=0,
                # so antenna delay does NOT appear in rxAr_by_An_in_cl_An (the raw RX timestamp).
                # The adelays only show up on the DS-TWR `distances[]` broadcast.
                rx_noisy = add_noise_int(ideal, cfg.ts_noise_ticks)
                last_rx_ticks[(receiver, sender)] = rx_noisy

            # Tag reception of this packet
            wall_rx_s_tag = wall_tx_s + t_at[sender]
            tag_ideal = wall_rx_s_tag * tag_rate + tag_offset
            tag_rx_ticks = add_noise_int(tag_ideal, cfg.ts_noise_ticks)

            observations.append(PacketObservation(
                sender_id=sender,
                frame_idx=frame,
                tag_rx_ticks=tag_rx_ticks,
                tx_in_sender_clock=sender_tx_ticks,
                remote_rx_in_sender_clock=remote_rx_in_sender,
                remote_tof_raw=sender_distances,
                sender_adelay=int(cfg.anchor_adelays_ticks[sender]),
            ))

    return observations


# --------------------------------------------------------------------------
# Correction strategies
# --------------------------------------------------------------------------
class ToFCorrector:
    """Interface: takes the raw inter-anchor ToF in packet.distances[] (DS-TWR raw ticks
    including a[i] + a[j]) and returns the value to feed into the TDoA engine, which
    should be pure propagation time (ticks). May keep internal state across packets.
    """
    name: str = "base"

    def correct(self, sender: int, other: int, raw_tof_ticks: int,
                sender_adelay: int, other_adelay: int) -> int:
        return raw_tof_ticks


class Baseline(ToFCorrector):
    """Current firmware: feed raw packet.distances[i] directly (includes a[i]+a[j] bias)."""
    name = "baseline_current_fw"


class AdelaySubtract(ToFCorrector):
    """ToF := raw - a[i] - a[j]. Minimal firmware change to remove pair-dependent bias."""
    name = "adelay_subtract"

    def correct(self, sender, other, raw, sa, oa):
        return max(raw - sa - oa, 0)


class StaticTruth(ToFCorrector):
    """ToF := T_true(i,j) from known anchor geometry. Zero-variance, deterministic."""
    name = "static_truth"

    def __init__(self, anchor_positions: np.ndarray):
        self.d = np.linalg.norm(
            anchor_positions[:, None, :] - anchor_positions[None, :, :], axis=-1
        )
        self.t_true_ticks = self.d * TICKS_PER_METER

    def correct(self, sender, other, raw, sa, oa):
        return max(int(round(self.t_true_ticks[sender, other])), 0)


class Blend(ToFCorrector):
    """ToF := (1-alpha)*(raw - a[i] - a[j]) + alpha*T_true(i,j).
    Trades noise reduction vs. sensitivity to geometry inaccuracy."""
    def __init__(self, anchor_positions: np.ndarray, alpha: float):
        self.truth = StaticTruth(anchor_positions)
        self.alpha = alpha
        self.name = f"blend_a{alpha:.2f}"

    def correct(self, sender, other, raw, sa, oa):
        meas = raw - sa - oa
        exp = self.truth.t_true_ticks[sender, other]
        v = (1.0 - self.alpha) * meas + self.alpha * exp
        return max(int(round(v)), 0)


class EWMA(ToFCorrector):
    """Per-pair EWMA on (raw - a[i] - a[j]). Cheap noise reduction, no geometry assumption."""
    def __init__(self, alpha: float = 0.1):
        self.alpha = alpha
        self.state: Dict[Tuple[int, int], float] = {}
        self.name = f"ewma_a{alpha:.2f}"

    def correct(self, sender, other, raw, sa, oa):
        meas = raw - sa - oa
        key = (sender, other)
        s = self.state.get(key)
        s = meas if s is None else self.alpha * meas + (1.0 - self.alpha) * s
        self.state[key] = s
        return max(int(round(s)), 0)


class RunningMedian(ToFCorrector):
    """Per-pair rolling median of last N (raw - a[i] - a[j]) samples.
    Best outlier/NLOS rejection among the simple filters."""
    def __init__(self, window: int = 9):
        self.window = window
        self.buf: Dict[Tuple[int, int], List[int]] = {}
        self.name = f"median_w{window}"

    def correct(self, sender, other, raw, sa, oa):
        meas = raw - sa - oa
        key = (sender, other)
        lst = self.buf.setdefault(key, [])
        lst.append(meas)
        if len(lst) > self.window:
            lst.pop(0)
        return max(int(np.median(lst)), 0)


class GatedStaticTruth(ToFCorrector):
    """Use T_true(i,j) unless the measured (raw - a - a) differs by more than `threshold_ticks`
    for more than `quorum` out of `window` recent samples — then raise a health flag and
    fall back to the (noisy) adelay-subtracted measurement so a geometry error can't silently
    bias results."""
    def __init__(self, anchor_positions: np.ndarray,
                 threshold_ticks: float = 60.0,       # ~28 cm
                 window: int = 20,
                 quorum: int = 10):
        self.truth = StaticTruth(anchor_positions)
        self.threshold = threshold_ticks
        self.window = window
        self.quorum = quorum
        self.name = f"static_truth_gated_{int(threshold_ticks)}t"
        self.health: Dict[Tuple[int, int], List[bool]] = {}

    def correct(self, sender, other, raw, sa, oa):
        exp = self.truth.t_true_ticks[sender, other]
        meas = raw - sa - oa
        err = abs(meas - exp)
        bad = err > self.threshold
        key = (sender, other)
        lst = self.health.setdefault(key, [])
        lst.append(bad)
        if len(lst) > self.window:
            lst.pop(0)
        if sum(lst) >= self.quorum:
            # geometry disagrees with reality → fall back to measurement
            return max(int(round(meas)), 0)
        return max(int(round(exp)), 0)


# --------------------------------------------------------------------------
# Tag-side TDoA engine (faithful to firmware calcTDoA)
# --------------------------------------------------------------------------
def compute_tdoa_measurements(
    observations: List[PacketObservation],
    cfg: SimConfig,
    corrector: ToFCorrector,
) -> List[Tuple[int, int, float]]:
    """
    Mirror tdoaEngine.cpp + tdoa_tag_algorithm.cpp:
      - For each received packet (An), update per-anchor context.
      - Estimate clockCorrection from consecutive TX/RX of An in tag.
      - Pick a suitable Ar, compute TDoA distance diff.
      - Output (anchorA_id, anchorB_id, distance_diff_m) using the firmware
        convention anchorIds[0]=otherAnchor (Ar), [1]=thisAnchor (An)
        so distDiff = c * (rxAn_by_T - rxAr_by_T - delta_txAr_to_txAn * k) / fs
        which equals d(Ar,T) - d(An,T)  (same sign firmware uses internally).
    """
    # Per-anchor storage (mirrors tdoaStorage)
    last_tag_rx: Dict[int, int] = {}            # tag RX of that anchor's previous packet
    last_sender_tx: Dict[int, int] = {}          # that anchor's TX of its previous packet
    clock_correction: Dict[int, float] = {}      # ratio tag_rate/anchor_rate
    # remote_rx_from_sender[sender][other] = sender's RX of other's previous packet (sender clock)
    remote_rx: Dict[Tuple[int, int], int] = {}
    # remote_tof[sender][other] = corrected ToF sender->other in sender clock (ticks)
    remote_tof: Dict[Tuple[int, int], int] = {}

    warmup_frames = cfg.warmup_frames
    results: List[Tuple[int, int, float]] = []

    for obs in observations:
        sender = obs.sender_id
        # Update remote data (what's in the packet)
        for other_id, rx_ticks in obs.remote_rx_in_sender_clock.items():
            if rx_ticks != 0:
                remote_rx[(sender, other_id)] = rx_ticks
        for other_id, raw_tof in obs.remote_tof_raw.items():
            if raw_tof != 0:
                other_adelay = int(cfg.anchor_adelays_ticks[other_id])
                sender_adelay = obs.sender_adelay
                corrected = corrector.correct(sender, other_id, raw_tof,
                                              sender_adelay, other_adelay)
                remote_tof[(sender, other_id)] = corrected

        # Clock correction update (needs two successive packets from sender)
        if sender in last_tag_rx and sender in last_sender_tx:
            dt_tag = (obs.tag_rx_ticks - last_tag_rx[sender]) & DW_TS_MASK
            dt_an = (obs.tx_in_sender_clock - last_sender_tx[sender]) & DW_TS_MASK
            if dt_an > 0:
                cc = dt_tag / dt_an
                if 1.0 - 5e-5 < cc < 1.0 + 5e-5:
                    clock_correction[sender] = cc

        # Compute TDoA only if we have clock correction and are past warmup
        if obs.frame_idx >= warmup_frames and sender in clock_correction:
            # Pick "youngest" other anchor that has valid data
            best_other = None
            best_frame = -1
            for other in range(cfg.anchor_positions.shape[0]):
                if other == sender:
                    continue
                if (sender, other) not in remote_tof:
                    continue
                if other not in last_tag_rx:
                    continue
                # treat "youngest" as latest sender-update-time; in sim, just use most recent tag RX
                if last_tag_rx[other] > best_frame:
                    best_frame = last_tag_rx[other]
                    best_other = other
            if best_other is not None:
                other = best_other
                cc = clock_correction[sender]
                tof_Ar_to_An = remote_tof[(sender, other)]
                rxAr_by_An = remote_rx.get((sender, other), 0)
                rxAr_by_T = last_tag_rx[other]
                rxAn_by_T = obs.tag_rx_ticks
                txAn = obs.tx_in_sender_clock

                delta_tx = (tof_Ar_to_An + ((txAn - rxAr_by_An) & DW_TS_MASK)) & DW_TS_MASK
                # Preserve sign: we don't wrap below zero in practice
                raw_tdoa = (rxAn_by_T - rxAr_by_T) - delta_tx * cc
                distance_diff = SPEED_OF_LIGHT * raw_tdoa / LOCODECK_TS_FREQ
                # Firmware enqueue uses (otherAnchorCtx=Ar, anchorCtx=An, distDiff)
                # anchorIds[0] = Ar = other, anchorIds[1] = An = sender
                results.append((other, sender, distance_diff))

        # Store current reception for next iteration
        last_tag_rx[sender] = obs.tag_rx_ticks
        last_sender_tx[sender] = obs.tx_in_sender_clock

    return results


# --------------------------------------------------------------------------
# Newton-Raphson position solver (matches lib/tdoa_newton_raphson)
# --------------------------------------------------------------------------
def solve_position_nr(
    anchor_left: np.ndarray,        # (M, 3)  positions of anchor Ar
    anchor_right: np.ndarray,       # (M, 3)  positions of anchor An
    doas: np.ndarray,               # (M,)    d(Ar,T) - d(An,T) in meters
    initial: np.ndarray,            # (3,) or (2,)
    iterations: int = 20,
    solve_2d: bool = False,
) -> np.ndarray:
    pos = initial.copy().astype(float)
    for _ in range(iterations):
        dL = np.linalg.norm(anchor_left - pos, axis=1)
        dR = np.linalg.norm(anchor_right - pos, axis=1)
        dL = np.maximum(dL, 1e-6)
        dR = np.maximum(dR, 1e-6)
        f = dL - dR - doas
        # gradient: for each measurement i, grad_i = (pos - L_i)/dL_i - (pos - R_i)/dR_i
        grad = (pos[None, :] - anchor_left) / dL[:, None] - (pos[None, :] - anchor_right) / dR[:, None]
        if solve_2d:
            A = grad[:, :2]
        else:
            A = grad
        try:
            delta, *_ = np.linalg.lstsq(A, f, rcond=None)
        except np.linalg.LinAlgError:
            break
        if solve_2d:
            pos[:2] -= delta
        else:
            pos -= delta
        if np.linalg.norm(delta) < 1e-6:
            break
    return pos


def latest_tdoa_per_pair(
    tdoas: List[Tuple[int, int, float]],
) -> Dict[Tuple[int, int], float]:
    """Keep only the most recent value per unordered pair (mirrors firmware behavior)."""
    out: Dict[Tuple[int, int], float] = {}
    for a, b, d in tdoas:
        key = tuple(sorted((a, b)))
        # preserve direction: store (a, b, d) as-is in the final pass
        out[key] = (a, b, d)
    return out


def estimate_position(
    cfg: SimConfig,
    tdoas: List[Tuple[int, int, float]],
    initial_guess: np.ndarray,
) -> Optional[np.ndarray]:
    latest = latest_tdoa_per_pair(tdoas)
    if len(latest) < 3:
        return None
    left_pos, right_pos, doas = [], [], []
    for (a, b, d) in latest.values():
        left_pos.append(cfg.anchor_positions[a])
        right_pos.append(cfg.anchor_positions[b])
        # Firmware uses tdoas(i) = -meas.tdoa because distanceDiff = d(right) - d(left)
        # in the measurement convention, while NR expects d(left) - d(right).
        doas.append(-d)
    return solve_position_nr(
        np.asarray(left_pos), np.asarray(right_pos), np.asarray(doas),
        initial_guess, cfg.solver_iterations, cfg.solve_2d,
    )


# --------------------------------------------------------------------------
# Monte-Carlo driver
# --------------------------------------------------------------------------
@dataclass
class TrialResult:
    method: str
    tag_pos: np.ndarray
    est: Optional[np.ndarray]
    err: Optional[float]


def run_trial(cfg: SimConfig, tag_pos: np.ndarray, corrector: ToFCorrector,
              rng: np.random.Generator) -> TrialResult:
    obs = simulate_frames(cfg, tag_pos, rng)
    tdoas = compute_tdoa_measurements(obs, cfg, corrector)
    initial = tag_pos + rng.normal(0.0, 0.5, size=3)
    est = estimate_position(cfg, tdoas, initial)
    if est is None:
        return TrialResult(corrector.name, tag_pos, None, None)
    err = float(np.linalg.norm(est - tag_pos))
    return TrialResult(corrector.name, tag_pos, est, err)


def default_methods(anchor_positions: np.ndarray) -> List[ToFCorrector]:
    return [
        Baseline(),
        AdelaySubtract(),
        EWMA(0.10),
        EWMA(0.05),
        RunningMedian(9),
        RunningMedian(21),
        Blend(anchor_positions, 0.25),
        Blend(anchor_positions, 0.50),
        Blend(anchor_positions, 0.75),
        StaticTruth(anchor_positions),
        GatedStaticTruth(anchor_positions, threshold_ticks=60.0, window=20, quorum=10),
    ]


def sample_tag_positions(n: int, dx: float, dy: float, z_range=(0.5, 2.0),
                         rng: Optional[np.random.Generator] = None,
                         avoid_center: bool = True,
                         avoid_radius: float = 0.5) -> np.ndarray:
    """Sample tag positions inside the rectangle. Avoid a small disk around the
    geometric center where coplanar-anchor TDoA is rank-deficient."""
    rng = rng or np.random.default_rng(42)
    cx, cy = 0.5 * dx, 0.5 * dy
    out = []
    while len(out) < n:
        x = rng.uniform(0.1 * dx, 0.9 * dx)
        y = rng.uniform(0.1 * dy, 0.9 * dy)
        z = rng.uniform(z_range[0], z_range[1])
        if avoid_center and (x - cx) ** 2 + (y - cy) ** 2 < avoid_radius ** 2:
            continue
        out.append((x, y, z))
    return np.asarray(out)


def run_mc(cfg: SimConfig, tag_positions: np.ndarray, methods: List[ToFCorrector],
           n_trials_per_pos: int = 3, seed: int = 0) -> List[TrialResult]:
    rng_master = np.random.default_rng(seed)
    all_results: List[TrialResult] = []
    for pos in tag_positions:
        for _ in range(n_trials_per_pos):
            # All methods see the *same* random noise so comparison is fair.
            trial_seed = int(rng_master.integers(0, 2**31 - 1))
            for m in methods:
                rng = np.random.default_rng(trial_seed)
                res = run_trial(cfg, pos, m, rng)
                all_results.append(res)
    return all_results


def summarize(results: List[TrialResult]) -> Dict[str, Dict[str, float]]:
    by_method: Dict[str, List[float]] = {}
    for r in results:
        if r.err is None:
            continue
        by_method.setdefault(r.method, []).append(r.err)
    out = {}
    for m, errs in by_method.items():
        e = np.asarray(errs)
        out[m] = {
            "n": int(e.size),
            "mean_m": float(np.mean(e)),
            "rms_m": float(np.sqrt(np.mean(e**2))),
            "p50_m": float(np.median(e)),
            "p90_m": float(np.percentile(e, 90)),
            "p99_m": float(np.percentile(e, 99)),
            "max_m": float(np.max(e)),
        }
    return out


def print_summary(tag: str, summary: Dict[str, Dict[str, float]]):
    print(f"\n=== {tag} ===")
    print(f"{'method':<26} {'n':>4} {'RMS (m)':>8} {'p50':>8} {'p90':>8} {'p99':>8} {'max':>8}")
    # Sort by RMS
    for m, s in sorted(summary.items(), key=lambda kv: kv[1]["rms_m"]):
        print(f"{m:<26} {s['n']:>4} {s['rms_m']:>8.4f} {s['p50_m']:>8.4f} "
              f"{s['p90_m']:>8.4f} {s['p99_m']:>8.4f} {s['max_m']:>8.4f}")


# --------------------------------------------------------------------------
# Scenario presets
# --------------------------------------------------------------------------
def make_base_cfg(twr_noise_ticks=30.0, ts_noise_ticks=2.0,
                  nlos_prob=0.0, nlos_bias_ticks=200.0,
                  clock_spread_ppm=10.0, n_frames=30,
                  solve_2d: bool = True) -> SimConfig:
    dx, dy = 5.0, 4.0
    anchor_positions = rectangle_anchors(dx, dy, z=2.5)
    N = anchor_positions.shape[0]
    adelays = np.array([15800, 16200, 15950, 16100], dtype=float)
    rng = np.random.default_rng(123)
    ppm = rng.uniform(-clock_spread_ppm, clock_spread_ppm, size=N)
    return SimConfig(
        anchor_positions=anchor_positions,
        anchor_adelays_ticks=adelays,
        anchor_clock_ppm=ppm,
        tag_clock_ppm=rng.uniform(-clock_spread_ppm, clock_spread_ppm),
        ts_noise_ticks=ts_noise_ticks,
        twr_noise_ticks=twr_noise_ticks,
        twr_nlos_prob=nlos_prob,
        twr_nlos_bias_ticks=nlos_bias_ticks,
        n_frames=n_frames,
        warmup_frames=4,
        solve_2d=solve_2d,
    )


# --------------------------------------------------------------------------
# Entrypoint
# --------------------------------------------------------------------------
def run_sweeps(n_positions: int = 12, trials_per_pos: int = 6):
    summaries = {}

    base_positions = sample_tag_positions(
        n_positions, dx=5.0, dy=4.0, z_range=(0.5, 2.0),
        rng=np.random.default_rng(11), avoid_center=True, avoid_radius=0.7,
    )

    # Scenario A: realistic indoor — representative DS-TWR noise, 1σ ts ~1 cm, LOS
    cfg_a = make_base_cfg(twr_noise_ticks=20.0, ts_noise_ticks=2.0, nlos_prob=0.0)
    methods = default_methods(cfg_a.anchor_positions)
    res = run_mc(cfg_a, base_positions, methods, n_trials_per_pos=trials_per_pos, seed=101)
    summaries["A: clean indoor (TWR σ≈9 cm, ts σ≈1 cm)"] = summarize(res)

    # Scenario B: very clean (well-tuned antennas, low multipath)
    cfg_b = make_base_cfg(twr_noise_ticks=8.0, ts_noise_ticks=1.5)
    res = run_mc(cfg_b, base_positions, methods, n_trials_per_pos=trials_per_pos, seed=102)
    summaries["B: very clean (TWR σ≈4 cm, ts σ≈0.7 cm)"] = summarize(res)

    # Scenario C: noisy DS-TWR (longer range or reflective walls)
    cfg_c = make_base_cfg(twr_noise_ticks=60.0, ts_noise_ticks=3.0)
    res = run_mc(cfg_c, base_positions, methods, n_trials_per_pos=trials_per_pos, seed=103)
    summaries["C: noisy DS-TWR (σ≈28 cm)"] = summarize(res)

    # Scenario D: LOS + NLOS outliers on DS-TWR (3% probability)
    cfg_d = make_base_cfg(twr_noise_ticks=25.0, ts_noise_ticks=2.5,
                           nlos_prob=0.03, nlos_bias_ticks=250.0)
    res = run_mc(cfg_d, base_positions, methods, n_trials_per_pos=trials_per_pos, seed=104)
    summaries["D: NLOS 3% prob, ~1 m bias"] = summarize(res)

    # Scenario E: geometry mismatch — anchors offset from the configured rectangle
    cfg_e = make_base_cfg(twr_noise_ticks=20.0, ts_noise_ticks=2.0)
    true_positions = cfg_e.anchor_positions.copy()
    rng_p = np.random.default_rng(777)
    true_positions += rng_p.normal(0.0, 0.05, size=true_positions.shape)
    methods_e = default_methods(cfg_e.anchor_positions)  # correctors still think rectangle is exact
    cfg_e_true = dataclasses.replace(cfg_e, anchor_positions=true_positions)
    res = run_mc(cfg_e_true, base_positions, methods_e, n_trials_per_pos=trials_per_pos, seed=105)
    summaries["E: anchors 5 cm off configured rectangle"] = summarize(res)

    # Scenario F: large geometry mismatch — what happens to pure static_truth vs gated?
    cfg_f = make_base_cfg(twr_noise_ticks=20.0, ts_noise_ticks=2.0)
    true_positions_f = cfg_f.anchor_positions.copy()
    rng_p = np.random.default_rng(888)
    true_positions_f += rng_p.normal(0.0, 0.20, size=true_positions_f.shape)  # 20 cm 1σ per axis
    methods_f = default_methods(cfg_f.anchor_positions)
    cfg_f_true = dataclasses.replace(cfg_f, anchor_positions=true_positions_f)
    res = run_mc(cfg_f_true, base_positions, methods_f, n_trials_per_pos=trials_per_pos, seed=106)
    summaries["F: anchors 20 cm off (bad geometry)"] = summarize(res)

    for tag, s in summaries.items():
        print_summary(tag, s)

    return summaries


if __name__ == "__main__":
    run_sweeps()
