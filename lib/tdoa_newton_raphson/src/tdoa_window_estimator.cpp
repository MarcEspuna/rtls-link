#include "tdoa_window_estimator.hpp"

#include <Eigen/QR>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>

namespace tdoa_estimator {
namespace {

// Augmented system: up to kMaxCapacity data rows + 3 prior rows + 3 LM rows.
using AugMatrix = Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 0, kMaxCapacity + 6, 3>;
using AugVector = Eigen::Matrix<Scalar, Eigen::Dynamic, 1, 0, kMaxCapacity + 6, 1>;

constexpr Scalar kDistanceFloor = static_cast<Scalar>(1e-4);
constexpr Scalar kLMInitialFactor = static_cast<Scalar>(1e-3);
constexpr Scalar kLMShrinkFactor = static_cast<Scalar>(0.5);
constexpr Scalar kLMGrowFactor = static_cast<Scalar>(4.0);
constexpr Scalar kLMMaxLambda = static_cast<Scalar>(1e6);
constexpr Scalar kLMMinLambda = static_cast<Scalar>(1e-12);

struct WindowRow {
    PosVector3D pos_a = PosVector3D::Zero();
    PosVector3D pos_b = PosVector3D::Zero();
    Scalar tdoa = 0.0f;          // d(a) - d(b) convention (solver residual dL - dR - tdoa)
    Scalar age_s = 0.0f;         // measurement age at solve time
    Scalar info_weight = 0.0f;   // age-decayed 1/sigma^2 (1/m^2)
    Scalar robust_weight = 1.0f; // Huber factor, unitless
};

Scalar ageWeight(uint32_t age_us, uint32_t half_life_us)
{
    if (half_life_us == 0) {
        return Scalar(1);
    }
    return Scalar(1) / (Scalar(1) + static_cast<Scalar>(age_us) / static_cast<Scalar>(half_life_us));
}

void computeResiduals(const WindowRow* rows, int n, const PosVector3D& pos, Scalar* residuals)
{
    for (int i = 0; i < n; ++i) {
        Scalar dA = (rows[i].pos_a - pos).norm();
        Scalar dB = (rows[i].pos_b - pos).norm();
        if (dA < kDistanceFloor) dA = kDistanceFloor;
        if (dB < kDistanceFloor) dB = kDistanceFloor;
        residuals[i] = (dA - dB) - rows[i].tdoa;
    }
}

void buildJacobianRow(const WindowRow& row, const PosVector3D& pos, Eigen::Matrix<Scalar, 1, 3>& J)
{
    PosVector3D diffA = pos - row.pos_a;
    PosVector3D diffB = pos - row.pos_b;
    Scalar dA = diffA.norm();
    Scalar dB = diffB.norm();
    if (dA < kDistanceFloor) dA = kDistanceFloor;
    if (dB < kDistanceFloor) dB = kDistanceFloor;
    J = (diffA / dA - diffB / dB).transpose();
}

Scalar totalRowWeight(const WindowRow& row)
{
    return row.info_weight * row.robust_weight;
}

// MAP cost: sum_i w_i r_i^2 + prior_info * ||x - x_prior||^2
Scalar mapCost(const WindowRow* rows, int n, const Scalar* residuals,
               const PosVector3D& pos, const PosVector3D& prior_pos, Scalar prior_info)
{
    Scalar cost = Scalar(0);
    for (int i = 0; i < n; ++i) {
        cost += totalRowWeight(rows[i]) * residuals[i] * residuals[i];
    }
    cost += prior_info * (pos - prior_pos).squaredNorm();
    return cost;
}

struct MapSolveOutcome {
    bool converged = false;
    int iterations = 0;
};

// Levenberg-Marquardt on the MAP cost. Augmented-QR step keeps float stable.
MapSolveOutcome solveMapLM(const WindowRow* rows,
                           int n,
                           const PosVector3D& prior_pos,
                           Scalar prior_info,
                           int max_iterations,
                           Scalar convergence_threshold,
                           PosVector3D& pos)
{
    MapSolveOutcome outcome;
    const Scalar prior_info_sqrt = std::sqrt(prior_info);

    Scalar residuals[kMaxCapacity];
    Scalar trial_residuals[kMaxCapacity];
    computeResiduals(rows, n, pos, residuals);
    Scalar prev_cost = mapCost(rows, n, residuals, pos, prior_pos, prior_info);

    AugMatrix jaug;
    AugVector raug;
    Eigen::Matrix<Scalar, 1, 3> jrow;

    // Lambda scaled from the data information so damping is proportionate.
    Scalar info_trace = prior_info * Scalar(3);
    for (int i = 0; i < n; ++i) {
        buildJacobianRow(rows[i], pos, jrow);
        info_trace += totalRowWeight(rows[i]) * jrow.squaredNorm();
    }
    Scalar lambda = kLMInitialFactor * info_trace / Scalar(3);
    if (lambda < kLMMinLambda) lambda = kLMMinLambda;

    for (int iter = 0; iter < max_iterations; ++iter) {
        outcome.iterations++;

        jaug.resize(n + 6, 3);
        raug.resize(n + 6);
        for (int i = 0; i < n; ++i) {
            buildJacobianRow(rows[i], pos, jrow);
            const Scalar scale = std::sqrt(totalRowWeight(rows[i]));
            jaug.row(i) = jrow * scale;
            raug(i) = residuals[i] * scale;
        }
        jaug.template block<3, 3>(n, 0) =
            Eigen::Matrix<Scalar, 3, 3>::Identity() * prior_info_sqrt;
        raug.template segment<3>(n) = (pos - prior_pos) * prior_info_sqrt;
        jaug.template block<3, 3>(n + 3, 0) =
            Eigen::Matrix<Scalar, 3, 3>::Identity() * std::sqrt(lambda);
        raug.template segment<3>(n + 3).setZero();

        const PosVector3D delta = jaug.householderQr().solve(raug);
        const PosVector3D pos_trial = pos - delta;

        computeResiduals(rows, n, pos_trial, trial_residuals);
        const Scalar trial_cost =
            mapCost(rows, n, trial_residuals, pos_trial, prior_pos, prior_info);

        if (trial_cost < prev_cost) {
            pos = pos_trial;
            std::copy(trial_residuals, trial_residuals + n, residuals);

            const Scalar step_norm = delta.norm();
            const Scalar rel_improve = (prev_cost > Scalar(0))
                ? (prev_cost - trial_cost) / prev_cost
                : Scalar(0);
            prev_cost = trial_cost;

            lambda *= kLMShrinkFactor;
            if (lambda < kLMMinLambda) lambda = kLMMinLambda;

            if (step_norm < convergence_threshold || rel_improve < convergence_threshold) {
                outcome.converged = true;
                break;
            }
        } else {
            lambda *= kLMGrowFactor;
            if (lambda > kLMMaxLambda) {
                break;
            }
        }
    }
    return outcome;
}

Scalar medianAbs(Scalar* values, int count)
{
    if (count == 0) {
        return Scalar(0);
    }
    for (int i = 0; i < count; ++i) {
        values[i] = std::fabs(values[i]);
    }
    std::sort(values, values + count);
    return values[count / 2];
}

// Data-only covariance with eigenvalue clamping: unobservable axes saturate at
// report_var_max_m2 instead of exploding — honest but bounded.
bool computeClampedCovariance(const WindowRow* rows,
                              int n,
                              const PosVector3D& pos,
                              const Scalar* residuals,
                              const WindowEstimatorOptions& options,
                              CovMatrix3D& out)
{
    Eigen::Matrix<double, 3, 3> info = Eigen::Matrix<double, 3, 3>::Zero();
    double weighted_sse = 0.0;
    double weight_sum = 0.0;
    Eigen::Matrix<Scalar, 1, 3> jrow;
    for (int i = 0; i < n; ++i) {
        buildJacobianRow(rows[i], pos, jrow);
        const double w = static_cast<double>(totalRowWeight(rows[i]));
        const Eigen::Matrix<double, 1, 3> jd = jrow.cast<double>();
        info += w * jd.transpose() * jd;
        weighted_sse += w * static_cast<double>(residuals[i]) * static_cast<double>(residuals[i]);
        weight_sum += w;
    }

    if (weight_sum <= 0.0) {
        out = CovMatrix3D::Identity() * static_cast<double>(options.report_var_max_m2);
        return false;
    }

    // Unit-weight variance factor: weights are 1/sigma^2, so a perfect noise
    // model gives ~1. Floor keeps a lucky window from reporting overconfidence.
    const int dof = std::max(1, n - 3);
    double variance_factor = weighted_sse / static_cast<double>(dof);
    variance_factor = std::max(variance_factor, 0.25);

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 3, 3>> es(info);
    if (es.info() != Eigen::Success) {
        out = CovMatrix3D::Identity() * static_cast<double>(options.report_var_max_m2);
        return false;
    }

    const double info_floor = variance_factor / static_cast<double>(options.report_var_max_m2);
    const double info_ceil = variance_factor / static_cast<double>(options.report_var_min_m2);
    Eigen::Matrix<double, 3, 1> inv_eigs;
    for (int i = 0; i < 3; ++i) {
        double lam = es.eigenvalues()(i);
        if (lam < info_floor) lam = info_floor;
        if (lam > info_ceil) lam = info_ceil;
        inv_eigs(i) = variance_factor / lam;
    }
    out = es.eigenvectors() * inv_eigs.asDiagonal() * es.eigenvectors().transpose();
    out = (out + out.transpose()) / 2.0;
    return true;
}

} // namespace

WindowEstimatorResult estimateWindow3D(const RobustTdoaRow* rows,
                                       size_t row_count,
                                       uint64_t now_us,
                                       WindowEstimatorState& state,
                                       const WindowEstimatorOptions& options)
{
    WindowEstimatorResult result;
    result.solve.position = state.prior_position;
    result.solve.valid = false;
    result.solve.converged = false;
    result.solve.iterations = 0;
    result.solve.rmse = std::numeric_limits<Scalar>::infinity();
    result.solve.covarianceValid = false;
    result.solve.positionCovariance =
        CovMatrix3D::Identity() * static_cast<double>(options.report_var_max_m2);

    if (rows == nullptr) {
        return result;
    }

    // --- Window selection: recent, finite rows only ---
    WindowRow window[kMaxCapacity];
    int n = 0;
    uint32_t anchor_mask = 0;
    for (size_t i = 0; i < row_count && n < static_cast<int>(kMaxCapacity); ++i) {
        const RobustTdoaRow& row = rows[i];
        if (row.age_us > options.window_max_age_us) {
            continue;
        }
        if (!std::isfinite(static_cast<double>(row.tdoa))) {
            continue;
        }
        WindowRow& w = window[n];
        w.pos_a = row.anchor_a_pos;
        w.pos_b = row.anchor_b_pos;
        w.tdoa = row.tdoa;
        w.age_s = static_cast<Scalar>(row.age_us) * Scalar(1e-6);
        Scalar sigma = row.nominal_sigma_m;
        if (!std::isfinite(static_cast<double>(sigma)) || sigma < options.sigma_floor_m) {
            sigma = options.sigma_floor_m;
        }
        Scalar health = std::isfinite(static_cast<double>(row.health)) ? row.health : Scalar(1);
        if (health < Scalar(0.05)) health = Scalar(0.05);
        if (health > Scalar(1)) health = Scalar(1);
        w.info_weight = ageWeight(row.age_us, options.age_half_life_us) * health
            / (sigma * sigma);
        w.robust_weight = Scalar(1);
        if (row.anchor_a < 32) anchor_mask |= (1u << row.anchor_a);
        if (row.anchor_b < 32) anchor_mask |= (1u << row.anchor_b);
        ++n;
    }

    uint8_t unique_anchors = 0;
    for (uint32_t m = anchor_mask; m != 0; m >>= 1u) {
        unique_anchors += static_cast<uint8_t>(m & 1u);
    }
    result.used_rows = static_cast<uint8_t>(n);
    result.unique_anchors = unique_anchors;

    if (n < options.min_rows || unique_anchors < options.min_unique_anchors) {
        return result;
    }

    // --- Prior ---
    PosVector3D initial = PosVector3D::Zero();
    Scalar prior_sigma = options.prior_sigma_max_m;
    if (state.has_prior) {
        const uint64_t dt_us = now_us >= state.prior_time_us ? now_us - state.prior_time_us : 0;
        const Scalar dt_s = static_cast<Scalar>(dt_us) * Scalar(1e-6);
        prior_sigma = options.prior_sigma_floor_m + options.prior_vel_m_s * dt_s;
        if (prior_sigma > options.prior_sigma_max_m) {
            prior_sigma = options.prior_sigma_max_m;
        }
        initial = state.prior_position;
        result.prior_used = true;
    } else {
        for (int i = 0; i < n; ++i) {
            initial += window[i].pos_a + window[i].pos_b;
        }
        initial /= static_cast<Scalar>(2 * n);
    }
    result.prior_sigma_m = prior_sigma;
    const PosVector3D prior_pos = state.has_prior ? state.prior_position : initial;
    const Scalar prior_info = Scalar(1) / (prior_sigma * prior_sigma);

    // --- Velocity compensation ---
    // A measurement of age `a` observed h(x(t-a)) ≈ h(x(t)) - ∇h·v·a, so roll
    // it forward: tdoa += ∇h·v·a. Removes the lag that age-weighted reuse
    // would otherwise introduce for a moving tag.
    if (options.velocity_compensation && state.has_velocity) {
        Eigen::Matrix<Scalar, 1, 3> grad;
        for (int i = 0; i < n; ++i) {
            if (window[i].age_s <= Scalar(0)) {
                continue;
            }
            buildJacobianRow(window[i], initial, grad);
            window[i].tdoa += grad.dot(state.velocity) * window[i].age_s;
        }
    }

    // --- MAP solve + Huber IRLS ---
    PosVector3D pos = initial;
    MapSolveOutcome outcome = solveMapLM(window, n, prior_pos, prior_info,
                                         options.max_iterations,
                                         options.convergence_threshold, pos);

    Scalar residuals[kMaxCapacity];
    Scalar scratch[kMaxCapacity];
    for (uint8_t pass = 0; pass < options.irls_passes; ++pass) {
        computeResiduals(window, n, pos, residuals);
        std::copy(residuals, residuals + n, scratch);
        const Scalar mad_scale = Scalar(1.4826) * medianAbs(scratch, n);
        const Scalar scale = std::max(options.min_residual_scale_m, mad_scale);
        result.residual_scale_m = scale;
        const Scalar delta = options.huber_k * scale;

        bool changed = false;
        for (int i = 0; i < n; ++i) {
            const Scalar abs_r = std::fabs(residuals[i]);
            Scalar rw = Scalar(1);
            if (abs_r > delta && abs_r > Scalar(1e-6)) {
                rw = delta / abs_r;
            }
            if (std::fabs(rw - window[i].robust_weight) > Scalar(1e-3)) {
                changed = true;
            }
            window[i].robust_weight = rw;
        }
        if (!changed) {
            break;
        }
        const MapSolveOutcome repass = solveMapLM(window, n, prior_pos, prior_info,
                                                  options.max_iterations,
                                                  options.convergence_threshold, pos);
        outcome.iterations += repass.iterations;
        outcome.converged = repass.converged;
    }

    computeResiduals(window, n, pos, residuals);

    // Robust-weighted RMSE in meters (info weights excluded so units stay m).
    Scalar sse = Scalar(0);
    Scalar wsum = Scalar(0);
    for (int i = 0; i < n; ++i) {
        sse += window[i].robust_weight * residuals[i] * residuals[i];
        wsum += window[i].robust_weight;
    }
    const Scalar rmse = wsum > Scalar(0)
        ? std::sqrt(sse / wsum)
        : std::numeric_limits<Scalar>::infinity();

    result.solve.position = pos;
    result.solve.rmse = rmse;
    result.solve.iterations = outcome.iterations;
    result.solve.converged = outcome.converged;

    const bool finite = std::isfinite(static_cast<double>(pos(0)))
        && std::isfinite(static_cast<double>(pos(1)))
        && std::isfinite(static_cast<double>(pos(2)))
        && std::isfinite(static_cast<double>(rmse));
    result.solve.valid = finite && rmse <= options.catastrophic_rmse_m;

    if (result.solve.valid) {
        result.solve.covarianceValid = computeClampedCovariance(
            window, n, pos, residuals, options, result.solve.positionCovariance);

        // EMA velocity from consecutive fixes, clamped to a plausible speed.
        if (state.has_prior && now_us > state.prior_time_us) {
            const Scalar dt_s =
                static_cast<Scalar>(now_us - state.prior_time_us) * Scalar(1e-6);
            if (dt_s > Scalar(1e-3) && dt_s < Scalar(0.25)) {
                PosVector3D v_new = (pos - state.prior_position) / dt_s;
                const Scalar speed = v_new.norm();
                if (speed > options.max_velocity_m_s) {
                    v_new *= options.max_velocity_m_s / speed;
                }
                if (state.has_velocity) {
                    state.velocity = state.velocity * (Scalar(1) - options.velocity_ema_alpha)
                        + v_new * options.velocity_ema_alpha;
                } else {
                    state.velocity = v_new * options.velocity_ema_alpha;
                }
                state.has_velocity = true;
            } else {
                state.has_velocity = false;
                state.velocity = PosVector3D::Zero();
            }
        }

        state.has_prior = true;
        state.prior_position = pos;
        state.prior_time_us = now_us;
        state.consecutive_bad = 0;
    } else {
        if (state.consecutive_bad < UINT8_MAX) {
            state.consecutive_bad++;
        }
        if (state.consecutive_bad >= options.max_consecutive_bad) {
            // Divergence watchdog: drop the prior so the next solve cold-starts.
            state.has_prior = false;
            state.consecutive_bad = 0;
            state.has_velocity = false;
            state.velocity = PosVector3D::Zero();
        }
    }

    return result;
}

} // namespace tdoa_estimator
