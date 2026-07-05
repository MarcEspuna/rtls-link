// Simulation comparing the current (main) batch+gates 3D robust pipeline
// against the new sliding-window MAP estimator on an identical synthetic
// UWB TDoA measurement stream.
//
// The "current" pipeline replicates src/uwb/uwb_tdoa_tag.cpp behaviour:
//  - producer updates canonical pair slots, notifies at >=4 fresh + 5ms debounce
//  - consumer snapshots fresh slots (min 8 rows / 6 unique anchors / 120ms span,
//    350ms stale) and CONSUMES them
//  - geometry gates (plane split, spanning pairs, information, det ratio)
//  - estimateRobust3D with rmse threshold 0.8, accept = valid && converged
//
// The "window" pipeline solves on a fixed 20ms cadence over every non-stale
// slot (no consumption) with estimateWindow3D.

#include <gtest/gtest.h>

#include "tdoa_matcher_score.hpp"
#include "tdoa_robust_estimator.hpp"
#include "tdoa_window_estimator.hpp"
#include "uwb/tdoa_measurement_buffer.hpp"

#include <etl/array.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <random>
#include <string>
#include <vector>

namespace {

using tdoa_estimator::PosVector3D;
using tdoa_estimator::Scalar;

constexpr uint8_t kNumAnchors = 8;
constexpr uint8_t kNumPairs = kNumAnchors * (kNumAnchors - 1) / 2;

// --- Firmware constants replicated from src/uwb/uwb_tdoa_tag.cpp ---
constexpr size_t kRobustMeasForSolve = 8;
constexpr uint8_t kRobustUniqueAnchors = 6;
constexpr uint64_t kStaleThresholdUs = 350000;
constexpr uint64_t kMaxBatchSpanUs = 120000;
constexpr size_t kMinFreshForNotify = 4;
constexpr uint64_t kNotifyDebounceUs = 5000;
constexpr uint64_t kWatchdogUs = 50000;
constexpr Scalar kRmseThreshold = 0.8f;

constexpr uint8_t kMinPlaneAnchorsPerSide = 2;
constexpr Scalar kMinPlaneSeparationM = 0.5f;
constexpr Scalar kMinAxisSeparationM = 0.5f;
constexpr Scalar kMinHorizontalInformation = 0.25f;
constexpr Scalar kMinZInformation = 0.25f;
constexpr Scalar kMinDeterminantRatio = 3.0e-4f;
constexpr uint8_t kMinAxisSpanningPairs = 1;
constexpr uint8_t kMinCrossPlanePairs = 2;
constexpr uint8_t kMinSamePlanePairs = 1;

// TDMA schedule (legacy defaults): 8 slots x 2ms = 16ms frame.
constexpr uint64_t kSlotUs = 2000;

uint8_t pairIndex(uint8_t a, uint8_t b)
{
    if (a > b) std::swap(a, b);
    // canonical index matching tdoa::PairIndex ordering
    uint8_t idx = 0;
    for (uint8_t i = 0; i < kNumAnchors; ++i) {
        for (uint8_t j = static_cast<uint8_t>(i + 1); j < kNumAnchors; ++j) {
            if (i == a && j == b) return idx;
            ++idx;
        }
    }
    return 0;
}

// ---------------------------------------------------------------------------
// Scenario definition
// ---------------------------------------------------------------------------

struct NlosEvent {
    uint8_t anchor = 0;
    double start_s = 0.0;
    double end_s = 0.0;
    Scalar bias_m = 0.0f;
};

enum class MatcherPolicy { YOUNGEST, RANDOM, GEOMETRIC };

struct Scenario {
    std::string name;
    std::array<PosVector3D, kNumAnchors> anchors;
    std::function<PosVector3D(double)> trajectory;
    double duration_s = 60.0;
    double packet_loss = 0.10;      // P(anchor packet not received)
    double unreliable_drop = 0.10;  // P(TDoA dropped upstream: clock corr unreliable)
    Scalar noise_sigma_m = 0.08f;   // per-row TDoA white noise
    // Pair-persistent errors (multipath, antenna-delay residuals): constant
    // for the run, so re-measuring the same pair repeats the bias instead of
    // averaging it out — this is where pair diversity actually pays.
    Scalar anchor_bias_sigma_m = 0.0f;
    Scalar pair_bias_sigma_m = 0.0f;
    std::vector<NlosEvent> nlos;
    MatcherPolicy matcher = MatcherPolicy::YOUNGEST;
    // GEOMETRIC matcher tuning (header defaults unless overridden)
    Scalar geo_trace_blend = 0.05f;
    Scalar geo_refresh_discount = 1.6f;
    uint64_t window_cadence_us = 20000;  // window pipeline solve/emit period
    // Covariance output model (mirrors firmware): correlation scaling on by
    // default; set false + var_max=25 to reproduce the pre-fix firmware.
    bool cov_reuse_scaling = true;
    Scalar cov_var_max = 4.0f;
    uint32_t seed = 1234;
};

std::array<PosVector3D, kNumAnchors> makeTwoPlaneAnchors(Scalar w, Scalar h,
                                                         Scalar z_low, Scalar z_high)
{
    std::array<PosVector3D, kNumAnchors> a;
    a[0] << 0, 0, z_low;
    a[1] << w, 0, z_low;
    a[2] << w, h, z_low;
    a[3] << 0, h, z_low;
    a[4] << 0, 0, z_high;
    a[5] << w, 0, z_high;
    a[6] << w, h, z_high;
    a[7] << 0, h, z_high;
    return a;
}

// ---------------------------------------------------------------------------
// Geometry gate — replicated from uwb_tdoa_tag.cpp evaluate3DGeometry /
// is3DGeometryAcceptable so the sim matches firmware behaviour exactly.
// ---------------------------------------------------------------------------

struct GeometryStats {
    uint8_t uniqueAnchors = 0;
    uint8_t lowPlane = 0;
    uint8_t highPlane = 0;
    uint8_t xSpanningPairs = 0;
    uint8_t ySpanningPairs = 0;
    uint8_t crossPlanePairs = 0;
    uint8_t samePlanePairs = 0;
    Scalar xInfo = 0, yInfo = 0, zInfo = 0;
    Scalar detRatio = 0;
};

GeometryStats evaluateGeometry(const tdoa::MeasurementSlot* rows, size_t count,
                               const std::array<PosVector3D, kNumAnchors>& anchors,
                               const PosVector3D& reference)
{
    GeometryStats st;
    bool seen[kNumAnchors] = {};
    Scalar minZ = std::numeric_limits<Scalar>::max();
    Scalar maxZ = -std::numeric_limits<Scalar>::max();
    Eigen::Matrix<Scalar, 3, 3> info = Eigen::Matrix<Scalar, 3, 3>::Zero();

    for (size_t i = 0; i < count; ++i) {
        const auto& r = rows[i];
        const PosVector3D& A = anchors[r.anchor_a];
        const PosVector3D& B = anchors[r.anchor_b];
        if (std::fabs(A(0) - B(0)) >= kMinAxisSeparationM) st.xSpanningPairs++;
        if (std::fabs(A(1) - B(1)) >= kMinAxisSeparationM) st.ySpanningPairs++;
        if (std::fabs(A(2) - B(2)) >= kMinPlaneSeparationM) st.crossPlanePairs++;
        else st.samePlanePairs++;

        for (uint8_t id : {r.anchor_a, r.anchor_b}) {
            if (!seen[id]) {
                seen[id] = true;
                st.uniqueAnchors++;
                minZ = std::min(minZ, anchors[id](2));
                maxZ = std::max(maxZ, anchors[id](2));
            }
        }

        Scalar dA = (reference - A).norm();
        Scalar dB = (reference - B).norm();
        dA = std::max(dA, Scalar(1e-4));
        dB = std::max(dB, Scalar(1e-4));
        const PosVector3D g = (reference - A) / dA - (reference - B) / dB;
        info += g * g.transpose();
    }

    if (st.uniqueAnchors == 0) return st;

    if ((maxZ - minZ) >= kMinPlaneSeparationM) {
        const Scalar midZ = (minZ + maxZ) / 2;
        for (uint8_t i = 0; i < kNumAnchors; ++i) {
            if (!seen[i]) continue;
            if (anchors[i](2) < midZ) st.lowPlane++;
            else st.highPlane++;
        }
    }

    const Scalar trace = info.trace();
    if (trace > Scalar(1e-6)) {
        const Scalar det =
            info(0, 0) * (info(1, 1) * info(2, 2) - info(1, 2) * info(2, 1))
            - info(0, 1) * (info(1, 0) * info(2, 2) - info(1, 2) * info(2, 0))
            + info(0, 2) * (info(1, 0) * info(2, 1) - info(1, 1) * info(2, 0));
        if (std::isfinite(static_cast<double>(det)) && det > 0) {
            st.detRatio = det / (trace * trace * trace);
        }
    }
    st.xInfo = info(0, 0);
    st.yInfo = info(1, 1);
    st.zInfo = info(2, 2);
    return st;
}

bool geometryAcceptable(const GeometryStats& st)
{
    return st.uniqueAnchors >= kRobustUniqueAnchors
        && st.lowPlane >= kMinPlaneAnchorsPerSide
        && st.highPlane >= kMinPlaneAnchorsPerSide
        && st.xSpanningPairs >= kMinAxisSpanningPairs
        && st.ySpanningPairs >= kMinAxisSpanningPairs
        && st.crossPlanePairs >= kMinCrossPlanePairs
        && st.samePlanePairs >= kMinSamePlanePairs
        && st.xInfo >= kMinHorizontalInformation
        && st.yInfo >= kMinHorizontalInformation
        && st.zInfo >= kMinZInformation
        && st.detRatio >= kMinDeterminantRatio;
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

struct EmitRecord {
    double t_s = 0.0;
    PosVector3D est = PosVector3D::Zero();
    double sigma_reported_m = -1.0;  // sqrt(trace(cov)/3), -1 if unavailable
    double var_reported[3] = {-1.0, -1.0, -1.0};  // covariance diagonal (m^2)
};

struct Metrics {
    std::vector<double> errors3d;
    std::vector<double> errorsXY;
    std::vector<double> errorsZ;
    std::vector<double> emit_times_s;
    std::vector<EmitRecord> records;  // post-warmup emits with full detail
    double duration_s = 0.0;
    double warmup_s = 2.0;

    void addEmit(double t_s, const PosVector3D& est, const PosVector3D& truth,
                 double sigma_reported_m = -1.0,
                 const double* var_reported = nullptr)
    {
        emit_times_s.push_back(t_s);
        if (t_s < warmup_s) return;
        const PosVector3D d = est - truth;
        errors3d.push_back(d.norm());
        errorsXY.push_back(d.head<2>().norm());
        errorsZ.push_back(std::fabs(d(2)));
        EmitRecord rec{t_s, est, sigma_reported_m, {-1.0, -1.0, -1.0}};
        if (var_reported != nullptr) {
            rec.var_reported[0] = var_reported[0];
            rec.var_reported[1] = var_reported[1];
            rec.var_reported[2] = var_reported[2];
        }
        records.push_back(rec);
    }

    double rateHz() const
    {
        size_t count = 0;
        for (double t : emit_times_s) {
            if (t >= warmup_s) ++count;
        }
        const double span = duration_s - warmup_s;
        return span > 0 ? static_cast<double>(count) / span : 0.0;
    }

    static double rms(const std::vector<double>& v)
    {
        if (v.empty()) return 0.0;
        double s = 0;
        for (double e : v) s += e * e;
        return std::sqrt(s / static_cast<double>(v.size()));
    }

    static double percentile(std::vector<double> v, double p)
    {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        const size_t idx = static_cast<size_t>(p * static_cast<double>(v.size() - 1));
        return v[idx];
    }

    double maxGapMs() const
    {
        double max_gap = 0.0;
        double prev = warmup_s;
        for (double t : emit_times_s) {
            if (t < warmup_s) continue;
            max_gap = std::max(max_gap, t - prev);
            prev = t;
        }
        max_gap = std::max(max_gap, duration_s - prev);
        return max_gap * 1000.0;
    }
};

// ---------------------------------------------------------------------------
// The simulation
// ---------------------------------------------------------------------------

struct SimResult {
    Metrics current;   // main pipeline (batch + gates + robust estimator)
    Metrics window;    // new sliding-window estimator
    // Window-geometry diagnostics (means over all window solves)
    double window_mean_rows = 0.0;
    double window_mean_distinct_pairs = 0.0;   // == rows (freshest per pair)
    double window_cross_plane_frac = 0.0;      // fraction of rows spanning planes
};

Scalar nlosBias(const Scenario& sc, uint8_t anchor, double t_s)
{
    for (const auto& ev : sc.nlos) {
        if (ev.anchor == anchor && t_s >= ev.start_s && t_s < ev.end_s) {
            return ev.bias_m;
        }
    }
    return 0.0f;
}

SimResult runScenario(const Scenario& sc, bool verbose = false,
                      const tdoa_estimator::WindowEstimatorOptions& window_options = {})
{
    std::mt19937 rng(sc.seed);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::normal_distribution<double> gauss(0.0, 1.0);

    // Shared measurement state (one copy per pipeline: the current pipeline
    // consumes fresh flags, the window pipeline must not see that side effect).
    etl::array<tdoa::MeasurementSlot, kNumPairs> slots_current = {};
    etl::array<tdoa::MeasurementSlot, kNumPairs> slots_window = {};
    etl::array<bool, kNumAnchors> configured;
    configured.fill(true);

    // --- Current pipeline state ---
    size_t fresh_count = 0;
    uint64_t last_notify_us = 0;
    bool notify_pending = false;
    uint64_t last_wake_us = 0;
    PosVector3D cur_last_pos = PosVector3D::Zero();
    bool cur_first = true;

    // --- Window pipeline state ---
    // Wake mechanics mirror the firmware task: producer notify (>=4 fresh +
    // 5ms debounce) plus a dynamic timeout re-armed to the remaining cadence
    // (capped at the 50ms watchdog), as set by estimatorProcessWindow.
    tdoa_estimator::WindowEstimatorState win_state;
    tdoa_estimator::WindowEstimatorOptions win_opts = window_options;
    const uint64_t kWindowCadenceUs = sc.window_cadence_us;
    win_opts.report_var_max_m2 = sc.cov_var_max;
    win_opts.covariance_reuse_scale = sc.cov_reuse_scaling
        ? std::max(1.0f, static_cast<float>(win_opts.window_max_age_us)
                       / static_cast<float>(kWindowCadenceUs))
        : 1.0f;
    size_t fresh_count_w = 0;
    uint64_t last_notify_w_us = 0;
    bool notify_pending_w = false;
    uint64_t last_wake_w_us = 0;
    uint64_t last_solve_w_us = 0;
    uint64_t wake_timeout_w_us = kWatchdogUs;

    // Youngest-anchor matching state: last reception time per anchor.
    std::array<uint64_t, kNumAnchors> last_rx_us = {};
    std::array<bool, kNumAnchors> rx_seen = {};

    SimResult result;
    result.current.duration_s = sc.duration_s;
    result.window.duration_s = sc.duration_s;

    const uint64_t duration_us = static_cast<uint64_t>(sc.duration_s * 1e6);

    // Pair-persistent error draws (constant for the run).
    std::array<Scalar, kNumAnchors> anchor_bias = {};
    std::array<Scalar, kNumPairs> pair_bias = {};
    for (auto& b : anchor_bias) {
        b = static_cast<Scalar>(gauss(rng)) * sc.anchor_bias_sigma_m;
    }
    for (auto& b : pair_bias) {
        b = static_cast<Scalar>(gauss(rng)) * sc.pair_bias_sigma_m;
    }

    auto trueDistance = [&](uint8_t anchor, double t_s) -> Scalar {
        const PosVector3D tag = sc.trajectory(t_s);
        return (tag - sc.anchors[anchor]).norm() + nlosBias(sc, anchor, t_s)
            + anchor_bias[anchor];
    };

    auto runCurrentConsumer = [&](uint64_t now_us) {
        last_wake_us = now_us;
        tdoa::MeasurementSlot snapshot[kNumPairs];
        const auto snap = tdoa::SnapshotFreshMeasurements(
            slots_current, configured, now_us, kStaleThresholdUs,
            kRobustMeasForSolve, snapshot, kNumPairs,
            kRobustUniqueAnchors, kMaxBatchSpanUs);
        fresh_count -= std::min<size_t>(fresh_count, snap.consumed + snap.expired);
        if (!snap.haveEnough) {
            return;
        }

        if (cur_first) {
            PosVector3D avg = PosVector3D::Zero();
            for (size_t i = 0; i < snap.copied; ++i) {
                avg += sc.anchors[snapshot[i].anchor_a] + sc.anchors[snapshot[i].anchor_b];
            }
            avg /= static_cast<Scalar>(2 * snap.copied);
            cur_last_pos = avg;
            cur_first = false;
        }

        const GeometryStats geo =
            evaluateGeometry(snapshot, snap.copied, sc.anchors, cur_last_pos);
        if (!geometryAcceptable(geo)) {
            return;
        }

        tdoa_estimator::RobustTdoaRow rows[kNumPairs];
        for (size_t i = 0; i < snap.copied; ++i) {
            const auto& s = snapshot[i];
            auto& r = rows[i];
            r.anchor_a = s.anchor_a;
            r.anchor_b = s.anchor_b;
            r.anchor_a_pos = sc.anchors[s.anchor_a];
            r.anchor_b_pos = sc.anchors[s.anchor_b];
            r.tdoa = -s.tdoa;  // firmware flips slot convention for the solver
            r.age_us = static_cast<uint32_t>(now_us - s.timestamp_us);
            r.nominal_sigma_m = s.sigma_m;
            r.health = 1.0f;
        }

        tdoa_estimator::RobustEstimatorOptions opts;
        opts.min_rows = kRobustMeasForSolve;
        opts.min_unique_anchors = kRobustUniqueAnchors;
        opts.max_selected_rows = 20;
        opts.max_iterations = 10;
        opts.convergence_threshold = 1e-3f;
        opts.rmse_threshold = kRmseThreshold;
        opts.enable_pair_selection = true;
        opts.enable_robust_pass = true;
        opts.reference_sigma_m = 0.15f;

        const auto res = tdoa_estimator::estimateRobust3D(
            rows, snap.copied, cur_last_pos, opts);
        if (res.solve.valid && res.solve.converged && !res.solve.position.hasNaN()) {
            const double t_s = static_cast<double>(now_us) * 1e-6;
            const double sigma = res.solve.covarianceValid
                ? std::sqrt(res.solve.positionCovariance.trace() / 3.0)
                : -1.0;
            result.current.addEmit(t_s, res.solve.position, sc.trajectory(t_s), sigma);
            cur_last_pos = res.solve.position;
        }
    };

    size_t window_solve_count = 0;
    size_t window_rows_sum = 0;
    size_t window_cross_sum = 0;

    // Matcher snapshot published by the window solve — mirrors the firmware's
    // seqlock-published state (only updated at solve time, stale -> fallback).
    struct SimMatcherSnapshot {
        bool valid = false;
        uint64_t t_us = 0;
        tdoa_estimator::Scalar info[6] = {};
        PosVector3D tag = PosVector3D::Zero();
        std::array<Scalar, kNumPairs> pair_weight = {};
    } matcher_snap;

    auto runWindowConsumer = [&](uint64_t now_us) {
        tdoa::MeasurementSlot snapshot[kNumPairs];
        const auto snap = tdoa::SnapshotWindowMeasurements(
            slots_window, configured, now_us, kStaleThresholdUs,
            win_opts.window_max_age_us, snapshot, kNumPairs);
        fresh_count_w -= std::min<size_t>(fresh_count_w, snap.consumed + snap.expired);

        window_solve_count++;
        window_rows_sum += snap.copied;
        for (size_t i = 0; i < snap.copied; ++i) {
            if (std::fabs(sc.anchors[snapshot[i].anchor_a](2)
                          - sc.anchors[snapshot[i].anchor_b](2)) >= kMinPlaneSeparationM) {
                window_cross_sum++;
            }
        }

        tdoa_estimator::RobustTdoaRow rows[kNumPairs];
        for (size_t i = 0; i < snap.copied; ++i) {
            const auto& s = snapshot[i];
            auto& r = rows[i];
            r.anchor_a = s.anchor_a;
            r.anchor_b = s.anchor_b;
            r.anchor_a_pos = sc.anchors[s.anchor_a];
            r.anchor_b_pos = sc.anchors[s.anchor_b];
            r.tdoa = -s.tdoa;
            r.age_us = static_cast<uint32_t>(now_us - s.timestamp_us);
            r.nominal_sigma_m = s.sigma_m;
            r.health = 1.0f;
        }
        const auto res = tdoa_estimator::estimateWindow3D(
            rows, snap.copied, now_us, win_state, win_opts);
        if (res.solve.valid) {
            const double t_s = static_cast<double>(now_us) * 1e-6;
            const double sigma = res.solve.covarianceValid
                ? std::sqrt(res.solve.positionCovariance.trace() / 3.0)
                : -1.0;
            const double vars[3] = {res.solve.positionCovariance(0, 0),
                                    res.solve.positionCovariance(1, 1),
                                    res.solve.positionCovariance(2, 2)};
            result.window.addEmit(t_s, res.solve.position, sc.trajectory(t_s), sigma,
                                  res.solve.covarianceValid ? vars : nullptr);

            // Publish the matcher snapshot (same weight model as the
            // estimator: age decay / sigma^2), mirroring the firmware.
            matcher_snap.valid = true;
            matcher_snap.t_us = now_us;
            matcher_snap.tag = res.solve.position;
            matcher_snap.pair_weight.fill(0.0f);
            tdoa_estimator::PackedInfo3 info;
            for (size_t i = 0; i < snap.copied; ++i) {
                const auto& s = snapshot[i];
                const Scalar age_s_v = static_cast<Scalar>(now_us - s.timestamp_us) * 1e-6f;
                Scalar sigma_m = s.sigma_m;
                if (!(sigma_m > 0.05f)) sigma_m = 0.05f;
                const Scalar w = (1.0f / (1.0f + age_s_v / 0.030f)) / (sigma_m * sigma_m);
                const PosVector3D g = tdoa_estimator::tdoaPairGradient(
                    res.solve.position, sc.anchors[s.anchor_a], sc.anchors[s.anchor_b]);
                info.addOuter(g, w);
                matcher_snap.pair_weight[pairIndex(s.anchor_a, s.anchor_b)] = w;
            }
            std::copy(info.m, info.m + 6, matcher_snap.info);
        }
    };

    // Firmware wake handler for the window pipeline: solve only when the
    // cadence has elapsed, otherwise re-arm the timeout to the remainder.
    auto wakeWindow = [&](uint64_t now_us) {
        last_wake_w_us = now_us;
        if (last_solve_w_us != 0 && (now_us - last_solve_w_us) < kWindowCadenceUs) {
            wake_timeout_w_us = std::min<uint64_t>(
                kWindowCadenceUs - (now_us - last_solve_w_us), kWatchdogUs);
            if (wake_timeout_w_us < 1000) wake_timeout_w_us = 1000;
            return;
        }
        wake_timeout_w_us = kWindowCadenceUs;
        last_solve_w_us = now_us;
        runWindowConsumer(now_us);
    };

    // Event loop over TDMA slots.
    for (uint64_t slot_start = 0; slot_start < duration_us; slot_start += kSlotUs) {
        const uint8_t anchor = static_cast<uint8_t>((slot_start / kSlotUs) % kNumAnchors);
        const uint64_t rx_us = slot_start + kSlotUs / 2;
        const double t_s = static_cast<double>(rx_us) * 1e-6;

        bool produced = false;
        if (uni(rng) >= sc.packet_loss) {
            // Matcher policy: which other anchor does this packet pair with?
            int best = -1;
            if (sc.matcher == MatcherPolicy::YOUNGEST) {
                // Most recently heard other anchor — with clean round-robin
                // TDMA this is nearly always the previous slot's anchor, so
                // only the 8 "ring" pairs are ever produced.
                for (uint8_t m = 0; m < kNumAnchors; ++m) {
                    if (m == anchor || !rx_seen[m]) continue;
                    if (rx_us - last_rx_us[m] > 50000) continue;
                    if (best < 0 || last_rx_us[m] > last_rx_us[best]) best = m;
                }
            } else {
                uint8_t eligible[kNumAnchors];
                uint8_t eligible_count = 0;
                for (uint8_t m = 0; m < kNumAnchors; ++m) {
                    if (m == anchor || !rx_seen[m]) continue;
                    if (rx_us - last_rx_us[m] > 50000) continue;
                    eligible[eligible_count++] = m;
                }
                const bool snap_usable = sc.matcher == MatcherPolicy::GEOMETRIC
                    && matcher_snap.valid
                    && (rx_us - matcher_snap.t_us) <= 500000;
                if (snap_usable && eligible_count > 0) {
                    // GEOMETRIC: E-optimal refresh gain against the published
                    // window information (shared scorer = firmware code).
                    Scalar best_score = -std::numeric_limits<Scalar>::max();
                    for (uint8_t e = 0; e < eligible_count; ++e) {
                        const uint8_t m = eligible[e];
                        const Scalar score = tdoa_estimator::matcherScorePair(
                            matcher_snap.info, matcher_snap.tag,
                            sc.anchors[anchor], sc.anchors[m],
                            matcher_snap.pair_weight[pairIndex(anchor, m)],
                            tdoa_estimator::kMatcherFullRowWeight,
                            sc.geo_trace_blend, sc.geo_refresh_discount);
                        if (score > best_score) {
                            best_score = score;
                            best = m;
                        }
                    }
                } else if (eligible_count > 0) {
                    // RANDOM (also the GEOMETRIC cold-start/stale fallback):
                    // rotating pick among all eligible remote candidates.
                    best = eligible[static_cast<size_t>(uni(rng) * eligible_count)
                                    % eligible_count];
                }
            }
            rx_seen[anchor] = true;
            last_rx_us[anchor] = rx_us;

            if (best >= 0 && uni(rng) >= sc.unreliable_drop) {
                const uint8_t other = static_cast<uint8_t>(best);
                // distanceDiff = d(current) - d(other); canonical slot stores
                // d(b) - d(a) for a < b (see estimatorCallback).
                const Scalar dcur = trueDistance(anchor, t_s);
                const Scalar doth = trueDistance(other, t_s);
                Scalar diff = dcur - doth
                    + static_cast<Scalar>(gauss(rng)) * sc.noise_sigma_m;
                uint8_t a = other, b = anchor;
                if (a > b) {
                    std::swap(a, b);
                    diff = -diff;
                }
                const uint8_t idx = pairIndex(a, b);
                // Pair-persistent bias attaches to the canonical direction.
                diff += pair_bias[idx];
                for (auto* slots : {&slots_current, &slots_window}) {
                    auto& slot = (*slots)[idx];
                    const bool was_fresh = slot.fresh;
                    slot.tdoa = diff;
                    slot.timestamp_us = rx_us;
                    slot.anchor_a = a;
                    slot.anchor_b = b;
                    slot.sigma_m = 0.15f;
                    slot.fresh = true;
                    if (!was_fresh) {
                        if (slots == &slots_current) fresh_count++;
                        else fresh_count_w++;
                    }
                }
                produced = true;
            }
        }

        // Producer notify logic (both pipelines share the same mechanism).
        if (produced && fresh_count >= kMinFreshForNotify
            && (rx_us - last_notify_us) >= kNotifyDebounceUs) {
            last_notify_us = rx_us;
            notify_pending = true;
        }
        if (produced && fresh_count_w >= kMinFreshForNotify
            && (rx_us - last_notify_w_us) >= kNotifyDebounceUs) {
            last_notify_w_us = rx_us;
            notify_pending_w = true;
        }

        // Current-pipeline consumer wakes: notification or watchdog.
        if (notify_pending) {
            notify_pending = false;
            runCurrentConsumer(rx_us);
        } else if (rx_us - last_wake_us >= kWatchdogUs) {
            runCurrentConsumer(rx_us);
        }

        // Window-pipeline consumer wakes: notification or the dynamic timeout
        // re-armed by the previous wake (mirrors the firmware task).
        if (notify_pending_w) {
            notify_pending_w = false;
            wakeWindow(rx_us);
        } else if (rx_us - last_wake_w_us >= wake_timeout_w_us) {
            wakeWindow(rx_us);
        }
    }

    if (window_solve_count > 0) {
        result.window_mean_rows =
            static_cast<double>(window_rows_sum) / static_cast<double>(window_solve_count);
        result.window_mean_distinct_pairs = result.window_mean_rows;
        result.window_cross_plane_frac = window_rows_sum > 0
            ? static_cast<double>(window_cross_sum) / static_cast<double>(window_rows_sum)
            : 0.0;
    }

    if (verbose) {
        auto print = [&](const char* name, const Metrics& m) {
            std::printf("  %-8s rate=%5.1f Hz  rms3d=%5.3f  rmsXY=%5.3f  rmsZ=%5.3f  "
                        "p95=%5.3f  maxGap=%6.0f ms  n=%zu\n",
                        name, m.rateHz(), Metrics::rms(m.errors3d),
                        Metrics::rms(m.errorsXY), Metrics::rms(m.errorsZ),
                        Metrics::percentile(m.errors3d, 0.95), m.maxGapMs(),
                        m.errors3d.size());
        };
        std::printf("Scenario: %s\n", sc.name.c_str());
        print("current", result.current);
        print("window", result.window);
    }
    return result;
}

// ---------------------------------------------------------------------------
// Scenarios
// ---------------------------------------------------------------------------

Scenario nominalStatic()
{
    Scenario sc;
    sc.name = "nominal-static";
    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    sc.trajectory = [](double) { PosVector3D p; p << 4.0f, 3.0f, 1.2f; return p; };
    sc.seed = 101;
    return sc;
}

Scenario movingCircle()
{
    Scenario sc;
    sc.name = "moving-circle";
    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    sc.trajectory = [](double t) {
        PosVector3D p;
        p << 5.0f + 2.5f * std::cos(0.7 * t),
             4.0f + 2.5f * std::sin(0.7 * t),
             1.5f + 0.5f * std::sin(0.3 * t);
        return p;
    };
    sc.seed = 202;
    return sc;
}

Scenario nlosBursts()
{
    Scenario sc;
    sc.name = "nlos-bursts";
    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    sc.trajectory = [](double) { PosVector3D p; p << 6.0f, 5.0f, 1.0f; return p; };
    // Rotating NLOS: every 6 s a different anchor is biased for 3 s.
    for (int i = 0; i < 9; ++i) {
        NlosEvent ev;
        ev.anchor = static_cast<uint8_t>(i % kNumAnchors);
        ev.start_s = 4.0 + 6.0 * i;
        ev.end_s = ev.start_s + 3.0;
        ev.bias_m = 0.9f;
        sc.nlos.push_back(ev);
    }
    sc.seed = 303;
    return sc;
}

Scenario lowPlaneSeparation()
{
    Scenario sc;
    sc.name = "low-plane-sep";
    // 0.25m separation: below the 0.5m firmware gate -> current pipeline
    // rejects every batch even though XY is perfectly solvable.
    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 0.55f);
    sc.trajectory = [](double) { PosVector3D p; p << 4.0f, 3.0f, 1.0f; return p; };
    sc.seed = 404;
    return sc;
}

Scenario highLoss()
{
    Scenario sc;
    sc.name = "high-loss";
    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    sc.trajectory = [](double t) {
        PosVector3D p;
        p << 5.0f + 2.0f * std::cos(0.5 * t),
             4.0f + 2.0f * std::sin(0.5 * t),
             1.2f;
        return p;
    };
    sc.packet_loss = 0.30;
    sc.unreliable_drop = 0.15;
    sc.seed = 505;
    return sc;
}

Scenario nlosMoving()
{
    Scenario sc;
    sc.name = "nlos-moving";
    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    sc.trajectory = [](double t) {
        PosVector3D p;
        p << 5.0f + 2.5f * std::cos(0.7 * t),
             4.0f + 2.5f * std::sin(0.7 * t),
             1.5f + 0.5f * std::sin(0.3 * t);
        return p;
    };
    for (int i = 0; i < 9; ++i) {
        NlosEvent ev;
        ev.anchor = static_cast<uint8_t>(i % kNumAnchors);
        ev.start_s = 4.0 + 6.0 * i;
        ev.end_s = ev.start_s + 3.0;
        ev.bias_m = 0.9f;
        sc.nlos.push_back(ev);
    }
    sc.seed = 606;
    return sc;
}

// ---------------------------------------------------------------------------
// NLOS / latency analysis helpers
// ---------------------------------------------------------------------------

// A fix is classified in-burst if any NLOS event is active, or ended less
// than `tail_s` earlier (contaminated measurements linger in both pipelines:
// window age 150ms for the new one, batch span 120ms / stale 350ms for the
// current one).
bool inBurst(const Scenario& sc, double t_s, double tail_s = 0.35)
{
    for (const auto& ev : sc.nlos) {
        if (t_s >= ev.start_s && t_s < ev.end_s + tail_s) {
            return true;
        }
    }
    return false;
}

struct BurstStats {
    double rate_hz = 0.0;
    double rms3d = 0.0;
    double p95 = 0.0;
    double frac_bad = 0.0;       // fraction of emits with 3D error > threshold
    double bad_per_s = 0.0;      // absolute bad-fix rate
    double mean_sigma = 0.0;     // mean reported 1-sigma (m), -1 if none reported
};

BurstStats analyzeSlice(const Metrics& m, const Scenario& sc, bool want_burst,
                        double bad_threshold_m)
{
    BurstStats st;
    std::vector<double> errs;
    size_t bad = 0;
    double sigma_sum = 0.0;
    size_t sigma_count = 0;
    for (const auto& r : m.records) {
        if (inBurst(sc, r.t_s) != want_burst) continue;
        const double err = (r.est - sc.trajectory(r.t_s)).norm();
        errs.push_back(err);
        if (err > bad_threshold_m) ++bad;
        if (r.sigma_reported_m >= 0.0) {
            sigma_sum += r.sigma_reported_m;
            ++sigma_count;
        }
    }

    // Time spent in the requested slice (post-warmup).
    double slice_s = 0.0;
    const double dt = 0.01;
    for (double t = m.warmup_s; t < m.duration_s; t += dt) {
        if (inBurst(sc, t) == want_burst) slice_s += dt;
    }

    st.rate_hz = slice_s > 0 ? static_cast<double>(errs.size()) / slice_s : 0.0;
    st.rms3d = Metrics::rms(errs);
    st.p95 = Metrics::percentile(errs, 0.95);
    st.frac_bad = errs.empty() ? 0.0 : static_cast<double>(bad) / static_cast<double>(errs.size());
    st.bad_per_s = slice_s > 0 ? static_cast<double>(bad) / slice_s : 0.0;
    st.mean_sigma = sigma_count > 0 ? sigma_sum / static_cast<double>(sigma_count) : -1.0;
    return st;
}

// Effective estimator lag: the time shift tau minimizing RMS between the
// emitted estimate at t and ground truth at t - tau. Positive = estimate lags.
double estimateLagMs(const Metrics& m, const Scenario& sc)
{
    if (m.records.size() < 50) return -1.0;
    double best_tau = 0.0;
    double best_rms = std::numeric_limits<double>::max();
    for (double tau = -0.10; tau <= 0.40; tau += 0.005) {
        double sse = 0.0;
        for (const auto& r : m.records) {
            sse += (r.est - sc.trajectory(r.t_s - tau)).squaredNorm();
        }
        const double rms = std::sqrt(sse / static_cast<double>(m.records.size()));
        if (rms < best_rms) {
            best_rms = rms;
            best_tau = tau;
        }
    }
    return best_tau * 1000.0;
}

// Mean error projection onto the velocity direction. Negative = trailing the
// true position (lag); expressed in ms of travel at the local speed.
double alongTrackLagMs(const Metrics& m, const Scenario& sc)
{
    double sum_ms = 0.0;
    size_t count = 0;
    for (const auto& r : m.records) {
        const double h = 0.02;
        const PosVector3D v = (sc.trajectory(r.t_s + h) - sc.trajectory(r.t_s - h))
            / static_cast<Scalar>(2.0 * h);
        const double speed = v.norm();
        if (speed < 0.2) continue;
        const PosVector3D err = r.est - sc.trajectory(r.t_s);
        const double along = err.dot(v) / speed;  // meters, negative = behind
        sum_ms += -along / speed * 1000.0;        // positive = lag
        ++count;
    }
    return count > 0 ? sum_ms / static_cast<double>(count) : -1.0;
}

// ---------------------------------------------------------------------------
// ArduPilot-like EKF consumer
// ---------------------------------------------------------------------------
// Per-axis constant-velocity Kalman filter fusing the emitted fixes exactly
// the way EKF3 fuses external-nav position: assuming INDEPENDENT measurement
// noise. If our fixes are time-correlated, the filter over-counts information
// and chases the correlated error - this is what the pilot feels.

struct EkfAxis {
    double p = 0, v = 0;
    double P00 = 25, P01 = 0, P11 = 25;

    void predict(double dt, double q)
    {
        p += v * dt;
        const double dt2 = dt * dt, dt3 = dt2 * dt;
        P00 += 2 * dt * P01 + dt2 * P11 + q * dt3 / 3.0;
        P01 += dt * P11 + q * dt2 / 2.0;
        P11 += q * dt;
    }

    // Returns normalized innovation squared (NIS) for consistency checks.
    double update(double z, double R)
    {
        const double y = z - p;
        const double S = P00 + R;
        const double K0 = P00 / S, K1 = P01 / S;
        p += K0 * y;
        v += K1 * y;
        const double P00n = (1 - K0) * P00;
        const double P01n = (1 - K0) * P01;
        const double P11n = P11 - K1 * P01;
        P00 = P00n; P01 = P01n; P11 = P11n;
        return y * y / S;
    }
};

struct EkfStudyResult {
    double pos3d_rms = 0, posZ_rms = 0;
    double vel3d_rms = 0, velZ_rms = 0;
    double nis_mean = 0;
    double rate_hz = 0;
};

enum class EkfNoiseMode {
    FIXED,            // ArduPilot param-style fixed noise (ignores our covariance)
    REPORTED,         // uses our reported per-fix sigma
    REPORTED_SCALED,  // reported sigma inflated by a correlation factor
    ARDUPILOT,        // exact AVCopter-4.6 path: posErr = cbrt(varx^2+vary^2+varz^2),
                      // floored at VISO_POS_M_NSE; corr_scale multiplies the
                      // variances (models firmware-side covariance scaling) and
                      // var_cap models the firmware's report_var_max clamp.
};

EkfStudyResult runEkfConsumer(const Metrics& m, const Scenario& sc,
                              EkfNoiseMode mode, double fixed_sigma_m,
                              double corr_scale, double var_cap = 25.0)
{
    EkfStudyResult out;
    if (m.records.size() < 50) return out;

    EkfAxis axis[3];
    // Hover-ish process noise (accel PSD, m^2/s^3) - EKF3 external-nav scale.
    const double q = 3.0;

    double prev_t = m.records.front().t_s;
    // Initialize at first fix.
    for (int a = 0; a < 3; ++a) {
        axis[a].p = m.records.front().est(a);
    }

    std::vector<double> e3d, eZ, ev3d, evZ;
    double nis_sum = 0;
    size_t nis_n = 0;

    for (size_t i = 1; i < m.records.size(); ++i) {
        const auto& r = m.records[i];
        const double dt = r.t_s - prev_t;
        if (dt <= 0 || dt > 1.0) { prev_t = r.t_s; continue; }
        prev_t = r.t_s;

        double sigma = fixed_sigma_m;
        if (mode == EkfNoiseMode::ARDUPILOT && r.var_reported[0] > 0) {
            double s2 = 0;
            for (int a = 0; a < 3; ++a) {
                const double v = std::min(r.var_reported[a] * corr_scale, var_cap);
                s2 += v * v;
            }
            // GCS_Common.cpp: posErr = cbrtf(sq(cov[0])+sq(cov[6])+sq(cov[11]));
            // AP_VisualOdom_MAV.cpp: constrained to >= VISO_POS_M_NSE.
            sigma = std::max(std::cbrt(s2), fixed_sigma_m);
        } else if (mode != EkfNoiseMode::FIXED && r.sigma_reported_m > 0) {
            sigma = r.sigma_reported_m;
            if (mode == EkfNoiseMode::REPORTED_SCALED) sigma *= corr_scale;
        }
        const double R = sigma * sigma;

        for (int a = 0; a < 3; ++a) {
            axis[a].predict(dt, q);
            nis_sum += axis[a].update(r.est(a), R);
            ++nis_n;
        }

        // Evaluate against truth (skip the filter's own settling: 3s).
        if (r.t_s < m.warmup_s + 3.0) continue;
        const PosVector3D truth = sc.trajectory(r.t_s);
        const double h = 0.02;
        const PosVector3D vtruth =
            (sc.trajectory(r.t_s + h) - sc.trajectory(r.t_s - h)) / static_cast<Scalar>(2 * h);
        const double ex = axis[0].p - truth(0), ey = axis[1].p - truth(1), ez = axis[2].p - truth(2);
        const double vx = axis[0].v - vtruth(0), vy = axis[1].v - vtruth(1), vz = axis[2].v - vtruth(2);
        e3d.push_back(std::sqrt(ex * ex + ey * ey + ez * ez));
        eZ.push_back(std::fabs(ez));
        ev3d.push_back(std::sqrt(vx * vx + vy * vy + vz * vz));
        evZ.push_back(std::fabs(vz));
    }

    out.pos3d_rms = Metrics::rms(e3d);
    out.posZ_rms = Metrics::rms(eZ);
    out.vel3d_rms = Metrics::rms(ev3d);
    out.velZ_rms = Metrics::rms(evZ);
    out.nis_mean = nis_n > 0 ? nis_sum / static_cast<double>(nis_n) : 0;
    out.rate_hz = m.rateHz();
    return out;
}

} // namespace

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

TEST(TDoAWindowSim, NominalStatic)
{
    const SimResult r = runScenario(nominalStatic(), true);
    EXPECT_GE(r.window.rateHz(), 45.0);
    EXPECT_GE(r.window.rateHz(), r.current.rateHz());
    EXPECT_LE(Metrics::rms(r.window.errors3d), 0.30);
    EXPECT_LE(Metrics::rms(r.window.errorsXY), 0.15);
    EXPECT_LE(r.window.maxGapMs(), 100.0);
}

TEST(TDoAWindowSim, MovingCircle)
{
    const SimResult r = runScenario(movingCircle(), true);
    EXPECT_GE(r.window.rateHz(), 45.0);
    EXPECT_GE(r.window.rateHz(), r.current.rateHz());
    // Accuracy at least comparable to the current pipeline.
    EXPECT_LE(Metrics::rms(r.window.errors3d),
              Metrics::rms(r.current.errors3d) * 1.5 + 0.05);
}

TEST(TDoAWindowSim, NlosBursts)
{
    const SimResult r = runScenario(nlosBursts(), true);
    EXPECT_GE(r.window.rateHz(), 45.0);
    EXPECT_LE(Metrics::rms(r.window.errors3d),
              Metrics::rms(r.current.errors3d) * 1.5 + 0.05);
    EXPECT_LE(r.window.maxGapMs(), 200.0);
}

TEST(TDoAWindowSim, LowPlaneSeparationStillEmits)
{
    const SimResult r = runScenario(lowPlaneSeparation(), true);
    // The current pipeline's plane-separation gate blocks everything here.
    // The window estimator must keep emitting with good XY (Z is genuinely
    // weak; the honest covariance is expected to reflect that).
    EXPECT_GE(r.window.rateHz(), 45.0);
    EXPECT_LE(Metrics::rms(r.window.errorsXY), 0.25);
}

TEST(TDoAWindowSim, HighLoss)
{
    const SimResult r = runScenario(highLoss(), true);
    EXPECT_GE(r.window.rateHz(), 40.0);
    EXPECT_GE(r.window.rateHz(), r.current.rateHz());
    EXPECT_LE(Metrics::rms(r.window.errors3d),
              Metrics::rms(r.current.errors3d) * 1.5 + 0.05);
}

// --- Unit-level checks for the window estimator itself ---

TEST(WindowEstimator, InsufficientRowsIsInvalidNotCrash)
{
    tdoa_estimator::WindowEstimatorState state;
    const auto res = tdoa_estimator::estimateWindow3D(nullptr, 0, 1000, state, {});
    EXPECT_FALSE(res.solve.valid);
    EXPECT_FALSE(state.has_prior);
}

TEST(WindowEstimator, ColdStartConvergesFromCentroid)
{
    const auto anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    PosVector3D truth;
    truth << 3.0f, 5.0f, 1.4f;

    tdoa_estimator::RobustTdoaRow rows[16];
    size_t n = 0;
    for (uint8_t a = 0; a < kNumAnchors && n < 16; ++a) {
        const uint8_t b = static_cast<uint8_t>((a + 1) % kNumAnchors);
        auto& r = rows[n++];
        r.anchor_a = std::min(a, b);
        r.anchor_b = std::max(a, b);
        r.anchor_a_pos = anchors[r.anchor_a];
        r.anchor_b_pos = anchors[r.anchor_b];
        r.tdoa = (truth - anchors[r.anchor_a]).norm() - (truth - anchors[r.anchor_b]).norm();
        r.age_us = 1000;
        r.nominal_sigma_m = 0.15f;
    }

    tdoa_estimator::WindowEstimatorState state;
    const auto res = tdoa_estimator::estimateWindow3D(rows, n, 1000000, state, {});
    ASSERT_TRUE(res.solve.valid);
    EXPECT_TRUE(state.has_prior);
    EXPECT_LE((res.solve.position - truth).norm(), 0.05f);
    EXPECT_TRUE(res.solve.covarianceValid);
}

TEST(WindowEstimator, SingleOutlierRowIsDownweighted)
{
    const auto anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
    PosVector3D truth;
    truth << 5.0f, 4.0f, 1.2f;

    tdoa_estimator::RobustTdoaRow rows[12];
    size_t n = 0;
    for (uint8_t a = 0; a < kNumAnchors && n < 12; ++a) {
        const uint8_t b = static_cast<uint8_t>((a + 1) % kNumAnchors);
        auto& r = rows[n++];
        r.anchor_a = std::min(a, b);
        r.anchor_b = std::max(a, b);
        r.anchor_a_pos = anchors[r.anchor_a];
        r.anchor_b_pos = anchors[r.anchor_b];
        r.tdoa = (truth - anchors[r.anchor_a]).norm() - (truth - anchors[r.anchor_b]).norm();
        r.age_us = 1000;
        r.nominal_sigma_m = 0.15f;
    }
    rows[0].tdoa += 1.5f;  // gross NLOS outlier

    tdoa_estimator::WindowEstimatorState state;
    const auto res = tdoa_estimator::estimateWindow3D(rows, n, 1000000, state, {});
    ASSERT_TRUE(res.solve.valid);
    EXPECT_LE((res.solve.position - truth).norm(), 0.25f);
}

TEST(TDoAWindowSim, NlosLeakageAnalysis)
{
    constexpr double kBadFixM = 0.5;
    for (const auto& sc : {nlosBursts(), nlosMoving()}) {
        const SimResult r = runScenario(sc);
        std::printf("NLOS analysis: %s (bad fix = 3D err > %.1f m)\n", sc.name.c_str(), kBadFixM);
        struct Row { const char* name; const Metrics* m; };
        const Row rows[] = {{"current", &r.current}, {"window", &r.window}};
        for (const auto& row : rows) {
            const BurstStats burst = analyzeSlice(*row.m, sc, true, kBadFixM);
            const BurstStats clear = analyzeSlice(*row.m, sc, false, kBadFixM);
            std::printf("  %-8s burst: rate=%5.1fHz rms=%5.3f p95=%5.3f bad=%4.1f%% (%.2f/s) sigma=%5.3f"
                        " | clear: rate=%5.1fHz rms=%5.3f bad=%4.1f%% sigma=%5.3f\n",
                        row.name,
                        burst.rate_hz, burst.rms3d, burst.p95, burst.frac_bad * 100.0,
                        burst.bad_per_s, burst.mean_sigma,
                        clear.rate_hz, clear.rms3d, clear.frac_bad * 100.0, clear.mean_sigma);
        }

        const BurstStats cur_burst = analyzeSlice(r.current, sc, true, kBadFixM);
        const BurstStats win_burst = analyzeSlice(r.window, sc, true, kBadFixM);
        const BurstStats win_clear = analyzeSlice(r.window, sc, false, kBadFixM);

        // The window estimator must not let through a higher *fraction* of
        // contaminated fixes than the current pipeline does.
        EXPECT_LE(win_burst.frac_bad, cur_burst.frac_bad + 0.02) << sc.name;
        // In-burst accuracy at least as good.
        EXPECT_LE(win_burst.rms3d, cur_burst.rms3d * 1.1 + 0.02) << sc.name;
        // Honest covariance: reported sigma must inflate during bursts so the
        // downstream EKF can de-weight contaminated fixes.
        EXPECT_GE(win_burst.mean_sigma, win_clear.mean_sigma * 1.5) << sc.name;
    }
}

TEST(TDoAWindowSim, LatencyAnalysis)
{
    for (const auto& sc : {movingCircle(), highLoss()}) {
        const SimResult r = runScenario(sc);
        const double cur_lag = estimateLagMs(r.current, sc);
        const double win_lag = estimateLagMs(r.window, sc);
        const double cur_along = alongTrackLagMs(r.current, sc);
        const double win_along = alongTrackLagMs(r.window, sc);
        std::printf("Latency: %-14s current: shift-lag=%5.1fms along-track=%5.1fms | "
                    "window: shift-lag=%5.1fms along-track=%5.1fms\n",
                    sc.name.c_str(), cur_lag, cur_along, win_lag, win_along);
        // The sliding window must not add lag relative to the batch pipeline.
        EXPECT_LE(win_lag, cur_lag + 10.0) << sc.name;
        EXPECT_LE(win_along, cur_along + 10.0) << sc.name;
        // And absolute lag must stay small relative to one solve period.
        EXPECT_LE(win_lag, 60.0) << sc.name;
    }
}

// YOUNGEST vs RANDOM matcher policy: does YOUNGEST starve pair diversity and
// hurt Z? (Field hypothesis from flight testing.)
TEST(TDoAWindowSim, MatcherPolicyPairStarvation)
{
    struct Geometry {
        const char* name;
        Scalar z_low, z_high;
    };
    const Geometry geoms[] = {
        {"plane-sep-2.5m", 0.3f, 2.8f},
        {"plane-sep-1.2m", 0.3f, 1.5f},
    };

    for (const auto& g : geoms) {
        for (const bool biased : {false, true}) {
            for (const bool moving : {false, true}) {
                constexpr int kPolicies = 3;
                const char* names[kPolicies] = {"YOUNGEST", "RANDOM", "GEOMETRIC"};
                double rmsZ[kPolicies] = {}, rmsXY[kPolicies] = {},
                       rows[kPolicies] = {}, cross[kPolicies] = {};
                for (int p = 0; p < kPolicies; ++p) {
                    Scenario sc;
                    sc.name = std::string(g.name) + (moving ? "/moving" : "/static");
                    sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, g.z_low, g.z_high);
                    if (moving) {
                        sc.trajectory = [](double t) {
                            PosVector3D q;
                            q << 5.0f + 2.5f * std::cos(0.7 * t),
                                 4.0f + 2.5f * std::sin(0.7 * t),
                                 1.2f + 0.4f * std::sin(0.3 * t);
                            return q;
                        };
                    } else {
                        sc.trajectory = [](double) { PosVector3D q; q << 4.0f, 3.0f, 1.2f; return q; };
                    }
                    if (biased) {
                        // Pair-persistent multipath / antenna-delay residuals.
                        sc.anchor_bias_sigma_m = 0.03f;
                        sc.pair_bias_sigma_m = 0.03f;
                    }
                    sc.matcher = static_cast<MatcherPolicy>(p);
                    sc.seed = 707;  // identical stream/bias draws per policy
                    const SimResult r = runScenario(sc);
                    rmsZ[p] = Metrics::rms(r.window.errorsZ);
                    rmsXY[p] = Metrics::rms(r.window.errorsXY);
                    rows[p] = r.window_mean_rows;
                    cross[p] = r.window_cross_plane_frac;
                }
                std::printf("Matcher %-14s %-6s %-6s |", g.name,
                            biased ? "biased" : "white", moving ? "moving" : "static");
                for (int p = 0; p < kPolicies; ++p) {
                    std::printf(" %s: rows=%4.1f cross=%4.1f%% rmsZ=%5.3f rmsXY=%5.3f |",
                                names[p], rows[p], cross[p] * 100.0, rmsZ[p], rmsXY[p]);
                }
                std::printf("\n");

                // RANDOM must populate far more distinct pairs than YOUNGEST...
                EXPECT_GE(rows[1], rows[0] * 1.5);
                // ...and must not degrade accuracy.
                EXPECT_LE(rmsZ[1], rmsZ[0] * 1.05 + 0.01);
                EXPECT_LE(rmsXY[1], rmsXY[0] * 1.15 + 0.01);
                // GEOMETRIC must beat RANDOM on the weak axis and not lose XY.
                EXPECT_LE(rmsZ[2], rmsZ[1] * 1.02 + 0.005);
                EXPECT_LE(rmsXY[2], rmsXY[1] * 1.10 + 0.01);
            }
        }
    }
}

// Does ArduPilot actually benefit from 50Hz of window-correlated fixes?
// Field hypothesis after A/B flights: the old batch pipeline (independent
// ~30Hz fixes) flies smoother than the 50Hz sliding window, especially Z.
// This study feeds an EKF3-style constant-velocity filter with fixes from
// different cadence/window/noise configurations of the SAME measurement
// stream and measures the filter's state quality (velocity error = what the
// controller feels).
TEST(TDoAWindowSim, EkfConsumerStudy)
{
    auto makeScenario = [](uint64_t cadence_us) {
        Scenario sc;
        sc.name = "ekf-study";
        sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 2.8f);
        // Circuit with altitude excursions ABOVE the upper anchor plane
        // (z up to 3.7m vs planes at 0.3/2.8m): outside the vertical envelope
        // the Z information genuinely collapses - this is where the reported
        // covariance and its clamps actually matter.
        sc.trajectory = [](double t) {
            PosVector3D q;
            q << 5.0f + 2.5f * std::cos(0.7 * t),
                 4.0f + 2.5f * std::sin(0.7 * t),
                 2.0f + 1.7f * std::sin(0.25 * t);
            return q;
        };
        sc.anchor_bias_sigma_m = 0.03f;
        sc.pair_bias_sigma_m = 0.03f;
        // Rotating NLOS bursts, as in the field.
        for (int i = 0; i < 9; ++i) {
            NlosEvent ev;
            ev.anchor = static_cast<uint8_t>(i % kNumAnchors);
            ev.start_s = 4.0 + 6.0 * i;
            ev.end_s = ev.start_s + 3.0;
            ev.bias_m = 0.9f;
            sc.nlos.push_back(ev);
        }
        sc.matcher = MatcherPolicy::GEOMETRIC;
        sc.window_cadence_us = cadence_us;
        sc.seed = 909;
        return sc;
    };

    struct Config {
        const char* name;
        uint64_t cadence_us;
        uint32_t window_age_us;
        EkfNoiseMode mode;
        double fixed_sigma;
        double corr_scale;
        bool legacy_cov;  // pre-fix firmware covariance (no scaling, cap 25)
        bool use_batch;   // evaluate the old batch pipeline from the same run
    };
    const double kFixedSigma = 0.2;  // ArduPilot VISO_POS_M_NSE default
    const Config configs[] = {
        {"batch ~30Hz indep     R=fix ", 20000, 150000, EkfNoiseMode::FIXED, kFixedSigma, 1.0, true, true},
        {"win 150/20ms (50Hz)   R=fix ", 20000, 150000, EkfNoiseMode::FIXED, kFixedSigma, 1.0, true, false},
        {"win 150/40ms (25Hz)   R=fix ", 40000, 150000, EkfNoiseMode::FIXED, kFixedSigma, 1.0, true, false},
        {"win 250/50ms (20Hz)   R=fix ", 50000, 250000, EkfNoiseMode::FIXED, kFixedSigma, 1.0, true, false},
        {"win 150/20ms R=fix*sqrt7.5  ", 20000, 150000, EkfNoiseMode::FIXED, kFixedSigma * 2.74, 1.0, true, false},
        // Exact AVCopter-4.6 posErr path (message covariance + VISO floor):
        {"AP4.6 pre-fix cov (AS FLOWN)", 20000, 150000, EkfNoiseMode::ARDUPILOT, kFixedSigma, 1.0, true, false},
        {"AP4.6 new cov (scaled,cap4) ", 20000, 150000, EkfNoiseMode::ARDUPILOT, kFixedSigma, 1.0, false, false},
        {"AP4.6 new cov + VISO=0.55   ", 20000, 150000, EkfNoiseMode::ARDUPILOT, kFixedSigma * 2.74, 1.0, false, false},
    };

    double batch_velZ = 0, flown_velZ = 0, newcov_velZ = 0, best_velZ = 1e9;
    for (const auto& c : configs) {
        Scenario sc = makeScenario(c.cadence_us);
        sc.cov_reuse_scaling = !c.legacy_cov;
        sc.cov_var_max = c.legacy_cov ? 25.0f : 4.0f;
        tdoa_estimator::WindowEstimatorOptions opts;
        opts.window_max_age_us = c.window_age_us;
        const SimResult r = runScenario(sc, false, opts);
        const Metrics& m = c.use_batch ? r.current : r.window;
        const EkfStudyResult e = runEkfConsumer(m, sc, c.mode, c.fixed_sigma, c.corr_scale);
        std::printf("EKF %-28s rate=%5.1fHz | ekfPos3d=%5.3f ekfPosZ=%5.3f | "
                    "ekfVel3d=%5.3f ekfVelZ=%5.3f | NIS=%5.2f\n",
                    c.name, e.rate_hz, e.pos3d_rms, e.posZ_rms,
                    e.vel3d_rms, e.velZ_rms, e.nis_mean);
        if (c.use_batch) batch_velZ = e.velZ_rms;
        if (std::string(c.name).find("AS FLOWN") != std::string::npos) flown_velZ = e.velZ_rms;
        if (std::string(c.name).find("new cov (scaled") != std::string::npos) newcov_velZ = e.velZ_rms;
        if (!c.use_batch) best_velZ = std::min(best_velZ, e.velZ_rms);
    }
    // The corrected covariance must improve on the as-flown configuration...
    EXPECT_LE(newcov_velZ, flown_velZ * 1.01);
    // ...and the study must produce a window configuration at least as good
    // as the old batch pipeline for the EKF.
    EXPECT_LE(best_velZ, batch_velZ * 1.05 + 0.005);
}

// Corner-of-cell / asymmetric geometry: adaptive targeting should show its
// largest edge here (the information matrix is anisotropic in XY too).
TEST(TDoAWindowSim, MatcherPolicyCornerGeometry)
{
    constexpr int kPolicies = 3;
    const char* names[kPolicies] = {"YOUNGEST", "RANDOM", "GEOMETRIC"};
    double rmsZ[kPolicies] = {}, rms3d[kPolicies] = {};
    for (int p = 0; p < kPolicies; ++p) {
        Scenario sc;
        sc.name = "corner";
        sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, 1.5f);
        // Slow loiter near a corner, outside the sweet spot.
        sc.trajectory = [](double t) {
            PosVector3D q;
            q << 1.2f + 0.4f * std::cos(0.4 * t),
                 1.2f + 0.4f * std::sin(0.4 * t),
                 1.0f;
            return q;
        };
        sc.anchor_bias_sigma_m = 0.03f;
        sc.pair_bias_sigma_m = 0.03f;
        sc.matcher = static_cast<MatcherPolicy>(p);
        sc.seed = 808;
        const SimResult r = runScenario(sc);
        rmsZ[p] = Metrics::rms(r.window.errorsZ);
        rms3d[p] = Metrics::rms(r.window.errors3d);
        std::printf("Corner %s: rows=%4.1f cross=%4.1f%% rms3d=%5.3f rmsZ=%5.3f rmsXY=%5.3f\n",
                    names[p], r.window_mean_rows, r.window_cross_plane_frac * 100.0,
                    rms3d[p], rmsZ[p], Metrics::rms(r.window.errorsXY));
    }
    EXPECT_LE(rms3d[2], rms3d[1] * 1.02 + 0.005);   // GEOMETRIC >= RANDOM
    EXPECT_LE(rms3d[1], rms3d[0]);                   // RANDOM >= YOUNGEST
}

// Matcher tuning sweep — not part of the regular suite.
TEST(TDoAWindowSim, DISABLED_MatcherTuningSweep)
{
    struct Cfg { Scalar blend, discount; };
    const Cfg cfgs[] = {{0.05f, 1.0f}, {0.05f, 1.3f}, {0.05f, 1.6f},
                        {0.15f, 1.3f}, {0.30f, 1.3f}, {0.15f, 1.6f}};
    for (const auto& c : cfgs) {
        std::printf("=== blend=%.2f discount=%.1f ===\n", c.blend, c.discount);
        for (const bool biased : {false, true}) {
            for (const Scalar z_high : {2.8f, 1.5f}) {
                Scenario sc;
                sc.anchors = makeTwoPlaneAnchors(10.0f, 8.0f, 0.3f, z_high);
                sc.trajectory = [](double) { PosVector3D q; q << 4.0f, 3.0f, 1.2f; return q; };
                if (biased) { sc.anchor_bias_sigma_m = 0.03f; sc.pair_bias_sigma_m = 0.03f; }
                sc.matcher = MatcherPolicy::GEOMETRIC;
                sc.geo_trace_blend = c.blend;
                sc.geo_refresh_discount = c.discount;
                sc.duration_s = 40.0;
                sc.seed = 707;
                const SimResult r = runScenario(sc);
                std::printf("  sep=%.1f %-6s rows=%4.1f cross=%4.1f%% rmsZ=%5.3f rmsXY=%5.3f\n",
                            z_high - 0.3f, biased ? "biased" : "white",
                            r.window_mean_rows, r.window_cross_plane_frac * 100.0,
                            Metrics::rms(r.window.errorsZ), Metrics::rms(r.window.errorsXY));
            }
        }
    }
}

// Parameter sweep for tuning — not part of the regular suite.
TEST(TDoAWindowSim, DISABLED_ParamSweep)
{
    struct Variant {
        const char* name;
        uint32_t window_us;
        uint32_t half_life_us;
        Scalar huber_k;
        uint8_t irls;
    };
    const Variant variants[] = {
        {"base 250/60 k1.5 i1", 250000, 60000, 1.5f, 1},
        {"fast 150/30 k1.5 i1", 150000, 30000, 1.5f, 1},
        {"fast 150/30 k1.0 i2", 150000, 30000, 1.0f, 2},
        {"fast 120/25 k1.0 i2", 120000, 25000, 1.0f, 2},
        {"tight 100/20 k1.0 i2", 100000, 20000, 1.0f, 2},
    };
    for (const auto& v : variants) {
        tdoa_estimator::WindowEstimatorOptions o;
        o.window_max_age_us = v.window_us;
        o.age_half_life_us = v.half_life_us;
        o.huber_k = v.huber_k;
        o.irls_passes = v.irls;
        std::printf("=== %s ===\n", v.name);
        for (const auto& sc : {nominalStatic(), movingCircle(), nlosBursts(),
                               lowPlaneSeparation(), highLoss()}) {
            const SimResult r = runScenario(sc, false, o);
            const auto& m = r.window;
            std::printf("  %-16s rate=%5.1f rms3d=%5.3f rmsXY=%5.3f rmsZ=%5.3f p95=%5.3f\n",
                        sc.name.c_str(), m.rateHz(), Metrics::rms(m.errors3d),
                        Metrics::rms(m.errorsXY), Metrics::rms(m.errorsZ),
                        Metrics::percentile(m.errors3d, 0.95));
        }
    }
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
