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

struct Scenario {
    std::string name;
    std::array<PosVector3D, kNumAnchors> anchors;
    std::function<PosVector3D(double)> trajectory;
    double duration_s = 60.0;
    double packet_loss = 0.10;      // P(anchor packet not received)
    double unreliable_drop = 0.10;  // P(TDoA dropped upstream: clock corr unreliable)
    Scalar noise_sigma_m = 0.08f;   // per-row TDoA noise
    std::vector<NlosEvent> nlos;
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
                 double sigma_reported_m = -1.0)
    {
        emit_times_s.push_back(t_s);
        if (t_s < warmup_s) return;
        const PosVector3D d = est - truth;
        errors3d.push_back(d.norm());
        errorsXY.push_back(d.head<2>().norm());
        errorsZ.push_back(std::fabs(d(2)));
        records.push_back(EmitRecord{t_s, est, sigma_reported_m});
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
    constexpr uint64_t kWindowCadenceUs = 20000;
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

    auto trueDistance = [&](uint8_t anchor, double t_s) -> Scalar {
        const PosVector3D tag = sc.trajectory(t_s);
        return (tag - sc.anchors[anchor]).norm() + nlosBias(sc, anchor, t_s);
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

    auto runWindowConsumer = [&](uint64_t now_us) {
        tdoa::MeasurementSlot snapshot[kNumPairs];
        const auto snap = tdoa::SnapshotWindowMeasurements(
            slots_window, configured, now_us, kStaleThresholdUs,
            win_opts.window_max_age_us, snapshot, kNumPairs);
        fresh_count_w -= std::min<size_t>(fresh_count_w, snap.consumed + snap.expired);

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
            result.window.addEmit(t_s, res.solve.position, sc.trajectory(t_s), sigma);
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
            // Youngest other anchor heard recently (matches YOUNGEST policy).
            int best = -1;
            for (uint8_t m = 0; m < kNumAnchors; ++m) {
                if (m == anchor || !rx_seen[m]) continue;
                if (rx_us - last_rx_us[m] > 50000) continue;
                if (best < 0 || last_rx_us[m] > last_rx_us[best]) best = m;
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
