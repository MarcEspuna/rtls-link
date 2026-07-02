#pragma once

#include "tdoa_newton_raphson.hpp"
#include "tdoa_robust_estimator.hpp"

#include <cstddef>
#include <cstdint>

namespace tdoa_estimator {

// Sliding-window MAP estimator.
//
// Design goals (vs. the batch robust estimator):
//  - Never reject a solvable window: weak geometry is handled by a MAP prior
//    toward the previous fix instead of hard gates, and honesty is expressed
//    through the reported covariance instead of silence.
//  - Fixed output cadence: the caller solves on a timer over the freshest
//    measurement per pair; measurements are not consumed, so the output rate
//    is decoupled from batch accumulation.
//  - Per-row robustness: age/sigma information weighting plus Huber IRLS,
//    so a single NLOS row cannot veto the fix.
struct WindowEstimatorOptions {
    uint8_t min_rows = 4;               // absolute floor for a data solve
    uint8_t min_unique_anchors = 4;     // below this 3D is fundamentally unsolvable
    uint8_t max_iterations = 10;
    uint8_t irls_passes = 2;            // Huber reweight+resolve passes
    Scalar convergence_threshold = 1e-3f;
    Scalar catastrophic_rmse_m = 3.0f;  // only reject a fix when residuals are hopeless
    uint32_t window_max_age_us = 150000; // rows older than this are ignored
    uint32_t age_half_life_us = 30000;   // age decay half-life for row weights
    Scalar sigma_floor_m = 0.05f;        // floor for per-row sigma in info weights
    Scalar huber_k = 1.0f;
    Scalar min_residual_scale_m = 0.05f;
    // MAP prior toward the previous fix. Prior sigma grows with time since the
    // last accepted fix so a stale prior fades away naturally.
    Scalar prior_sigma_floor_m = 0.3f;
    Scalar prior_vel_m_s = 3.0f;
    Scalar prior_sigma_max_m = 10.0f;
    // Reported covariance clamp: axes the data cannot observe saturate at this
    // variance instead of going unbounded (or being gated away).
    Scalar report_var_max_m2 = 25.0f;
    Scalar report_var_min_m2 = 1e-4f;
    // Velocity compensation: roll each measurement forward to solve time using
    // an EMA velocity from consecutive fixes (removes age-induced lag when the
    // tag moves). Disabled automatically while no stable velocity is available.
    bool velocity_compensation = true;
    Scalar max_velocity_m_s = 5.0f;
    Scalar velocity_ema_alpha = 0.15f;
    // After this many consecutive catastrophic solves the prior is dropped and
    // the estimator cold-starts from the anchor centroid.
    uint8_t max_consecutive_bad = 10;
};

struct WindowEstimatorState {
    bool has_prior = false;
    PosVector3D prior_position = PosVector3D::Zero();
    uint64_t prior_time_us = 0;
    uint8_t consecutive_bad = 0;
    bool has_velocity = false;
    PosVector3D velocity = PosVector3D::Zero();
};

struct WindowEstimatorResult {
    SolverResult solve;          // position, rmse (m), data-only covariance
    uint8_t used_rows = 0;
    uint8_t unique_anchors = 0;
    bool prior_used = false;
    Scalar prior_sigma_m = 0.0f;
    Scalar residual_scale_m = 0.0f;
};

// Solve one window. `rows` is the current freshest-per-pair window (already
// age-filtered slots are fine; rows older than window_max_age_us are skipped
// here as well). Updates `state` with the new prior on success.
WindowEstimatorResult estimateWindow3D(const RobustTdoaRow* rows,
                                       size_t row_count,
                                       uint64_t now_us,
                                       WindowEstimatorState& state,
                                       const WindowEstimatorOptions& options = {});

} // namespace tdoa_estimator
