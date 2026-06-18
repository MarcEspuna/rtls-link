#include "tdoa_robust_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tdoa_estimator {
namespace {

struct RowScore {
    uint8_t index = 0;
    Scalar score = 0.0f;
};

struct RowGeometry3D {
    PosVector3D gradient = PosVector3D::Zero();
    Scalar weight = 0.0f;
    uint8_t anchor_mask = 0;
    bool valid = false;
};

struct GeometryInfo3D {
    Scalar xx = 0.0f;
    Scalar xy = 0.0f;
    Scalar xz = 0.0f;
    Scalar yy = 0.0f;
    Scalar yz = 0.0f;
    Scalar zz = 0.0f;
};

Scalar clampScalar(Scalar value, Scalar lo, Scalar hi)
{
    if (!std::isfinite(static_cast<double>(value))) {
        return lo;
    }
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

Scalar ageWeight(uint32_t age_us, const RobustEstimatorOptions& options)
{
    if (options.age_half_life_us == 0) {
        return Scalar(1);
    }
    const Scalar age = static_cast<Scalar>(age_us);
    const Scalar half_life = static_cast<Scalar>(options.age_half_life_us);
    return Scalar(1) / (Scalar(1) + (age / half_life));
}

Scalar sigmaWeight(Scalar sigma_m, const RobustEstimatorOptions& options)
{
    const Scalar reference = options.reference_sigma_m > Scalar(1e-6)
        ? options.reference_sigma_m
        : Scalar(0.15);
    if (!std::isfinite(static_cast<double>(sigma_m)) || sigma_m <= Scalar(1e-6)) {
        return Scalar(1);
    }
    if (sigma_m <= reference) {
        return Scalar(1);
    }
    const Scalar ratio = reference / sigma_m;
    return ratio * ratio;
}

uint8_t rowAnchorMask(const RobustTdoaRow& row)
{
    uint8_t mask = 0;
    if (row.anchor_a < 8) mask |= static_cast<uint8_t>(1u << row.anchor_a);
    if (row.anchor_b < 8) mask |= static_cast<uint8_t>(1u << row.anchor_b);
    return mask;
}

RowGeometry3D makeRowGeometry(const RobustTdoaRow& row,
                              const PosVector3D& initial_position,
                              Scalar weight)
{
    RowGeometry3D geometry;
    geometry.weight = weight;
    geometry.anchor_mask = rowAnchorMask(row);

    PosVector3D diff_a = initial_position - row.anchor_a_pos;
    PosVector3D diff_b = initial_position - row.anchor_b_pos;
    Scalar da = diff_a.norm();
    Scalar db = diff_b.norm();
    if (da < Scalar(1e-4)) da = Scalar(1e-4);
    if (db < Scalar(1e-4)) db = Scalar(1e-4);

    geometry.gradient = (diff_a / da) - (diff_b / db);
    geometry.valid = std::isfinite(static_cast<double>(geometry.gradient(0)))
        && std::isfinite(static_cast<double>(geometry.gradient(1)))
        && std::isfinite(static_cast<double>(geometry.gradient(2)))
        && weight > Scalar(0);
    return geometry;
}

Scalar rowGeometryScore(const RowGeometry3D& geometry)
{
    if (!geometry.valid) {
        return Scalar(0);
    }
    const Scalar information = geometry.gradient.squaredNorm();
    const Scalar vertical = std::fabs(geometry.gradient(2));
    return information * (Scalar(1) + Scalar(0.5) * vertical);
}

Scalar baseWeightForRow(const RobustTdoaRow& row, const RobustEstimatorOptions& options)
{
    const Scalar health = clampScalar(row.health, Scalar(0), Scalar(1));
    const Scalar weight = ageWeight(row.age_us, options)
        * sigmaWeight(row.nominal_sigma_m, options)
        * health;
    return clampScalar(weight, options.min_weight, Scalar(1));
}

void addGeometry(GeometryInfo3D& info, const RowGeometry3D& geometry)
{
    if (!geometry.valid) {
        return;
    }
    const Scalar w = geometry.weight;
    const Scalar gx = geometry.gradient(0);
    const Scalar gy = geometry.gradient(1);
    const Scalar gz = geometry.gradient(2);
    info.xx += w * gx * gx;
    info.xy += w * gx * gy;
    info.xz += w * gx * gz;
    info.yy += w * gy * gy;
    info.yz += w * gy * gz;
    info.zz += w * gz * gz;
}

Scalar geometryTrace(const GeometryInfo3D& info)
{
    return info.xx + info.yy + info.zz;
}

Scalar geometryDeterminant(const GeometryInfo3D& info)
{
    return info.xx * ((info.yy * info.zz) - (info.yz * info.yz))
        - info.xy * ((info.xy * info.zz) - (info.yz * info.xz))
        + info.xz * ((info.xy * info.yz) - (info.yy * info.xz));
}

Scalar geometryDeterminantRatio(const GeometryInfo3D& info)
{
    const Scalar trace = geometryTrace(info);
    if (trace <= Scalar(1.0e-6f) || !std::isfinite(static_cast<double>(trace))) {
        return Scalar(0);
    }
    const Scalar determinant = geometryDeterminant(info);
    if (!std::isfinite(static_cast<double>(determinant)) || determinant <= Scalar(0)) {
        return Scalar(0);
    }
    return determinant / (trace * trace * trace);
}

Scalar geometryQualityScore(const GeometryInfo3D& info)
{
    const Scalar trace = geometryTrace(info);
    if (trace <= Scalar(1.0e-6f) || !std::isfinite(static_cast<double>(trace))) {
        return Scalar(0);
    }
    const Scalar axis_balance = std::min(info.xx, std::min(info.yy, info.zz)) / trace;
    const Scalar z_share = info.zz / trace;
    return (Scalar(16) * geometryDeterminantRatio(info))
        + axis_balance
        + (Scalar(0.25f) * z_share);
}

bool geometryAcceptable(const GeometryInfo3D& info,
                        const RobustEstimatorOptions& options)
{
    return info.xx >= options.min_geometry_axis_information
        && info.yy >= options.min_geometry_axis_information
        && info.zz >= options.min_geometry_axis_information
        && geometryDeterminantRatio(info) >= options.min_geometry_determinant_ratio;
}

GeometryInfo3D selectedGeometryInfo(const RobustTdoaRow* rows,
                                    const RobustEstimatorResult& result,
                                    const PosVector3D& initial_position,
                                    const Scalar* source_weights)
{
    GeometryInfo3D info;
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        const uint8_t source_index = result.selected_indices[i];
        const RowGeometry3D geometry =
            makeRowGeometry(rows[source_index], initial_position, source_weights[source_index]);
        addGeometry(info, geometry);
    }
    return info;
}

uint8_t popcount8(uint8_t value)
{
    uint8_t count = 0;
    while (value != 0) {
        count += value & 1u;
        value >>= 1u;
    }
    return count;
}

bool containsIndex(const RobustEstimatorResult& result, uint8_t index)
{
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        if (result.selected_indices[i] == index) {
            return true;
        }
    }
    return false;
}

void addSelected(RobustEstimatorResult& result, uint8_t index)
{
    if (result.selected_rows >= kMaxCapacity || containsIndex(result, index)) {
        return;
    }
    result.selected_indices[result.selected_rows++] = index;
}

uint8_t selectedAnchorMask(const RobustEstimatorResult& result, const RobustTdoaRow* rows)
{
    uint8_t mask = 0;
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        const RobustTdoaRow& row = rows[result.selected_indices[i]];
        if (row.anchor_a < 8) mask |= static_cast<uint8_t>(1u << row.anchor_a);
        if (row.anchor_b < 8) mask |= static_cast<uint8_t>(1u << row.anchor_b);
    }
    return mask;
}

void selectRows(const RobustTdoaRow* rows,
                size_t row_count,
                const PosVector3D& initial_position,
                const RobustEstimatorOptions& options,
                RobustEstimatorResult& result)
{
    const uint8_t capped_count = static_cast<uint8_t>(std::min(row_count, kMaxCapacity));
    result.input_rows = capped_count;

    RowScore scores[kMaxCapacity] = {};
    RowGeometry3D geometries[kMaxCapacity] = {};
    for (uint8_t i = 0; i < capped_count; i++) {
        const Scalar weight = baseWeightForRow(rows[i], options);
        result.base_weights[i] = weight;
        geometries[i] = makeRowGeometry(rows[i], initial_position, weight);
        scores[i].index = i;
        scores[i].score = weight * rowGeometryScore(geometries[i]);
    }

    std::sort(scores, scores + capped_count, [](const RowScore& a, const RowScore& b) {
        if (a.score == b.score) {
            return a.index < b.index;
        }
        return a.score > b.score;
    });

    const uint8_t max_rows = std::min<uint8_t>(
        options.max_selected_rows == 0 ? capped_count : options.max_selected_rows,
        capped_count);

    if (!options.enable_pair_selection || capped_count <= max_rows) {
        for (uint8_t i = 0; i < capped_count; i++) {
            result.selected_indices[result.selected_rows++] = i;
        }
        result.unique_anchors = popcount8(selectedAnchorMask(result, rows));
        return;
    }

    result.pair_selection_used = true;

    uint8_t anchor_mask = 0;
    GeometryInfo3D selected_info;
    while (result.selected_rows < max_rows) {
        int best_index = -1;
        Scalar best_score = -std::numeric_limits<Scalar>::infinity();
        const uint8_t current_unique = popcount8(anchor_mask);

        for (uint8_t s = 0; s < capped_count; s++) {
            const uint8_t idx = scores[s].index;
            if (containsIndex(result, idx) || !geometries[idx].valid) {
                continue;
            }

            GeometryInfo3D trial_info = selected_info;
            addGeometry(trial_info, geometries[idx]);

            const uint8_t next_mask = static_cast<uint8_t>(anchor_mask | geometries[idx].anchor_mask);
            const uint8_t next_unique = popcount8(next_mask);
            const uint8_t added_anchors = next_unique > current_unique
                ? static_cast<uint8_t>(next_unique - current_unique)
                : 0;

            Scalar candidate_score = geometryQualityScore(trial_info);
            if (current_unique < options.min_unique_anchors && added_anchors > 0) {
                candidate_score += Scalar(10) + Scalar(2) * static_cast<Scalar>(added_anchors);
            }
            if (result.selected_rows < options.min_rows) {
                candidate_score += Scalar(1);
            }
            candidate_score += Scalar(0.001f) * scores[s].score;

            if (best_index < 0 || candidate_score > best_score) {
                best_index = idx;
                best_score = candidate_score;
            }
        }

        if (best_index < 0) {
            break;
        }
        addSelected(result, static_cast<uint8_t>(best_index));
        addGeometry(selected_info, geometries[best_index]);
        anchor_mask = static_cast<uint8_t>(anchor_mask | geometries[best_index].anchor_mask);
    }

    result.unique_anchors = popcount8(selectedAnchorMask(result, rows));
}

Scalar medianAbsResidual(const Scalar* residuals, uint8_t count)
{
    if (count == 0) {
        return Scalar(0);
    }
    Scalar values[kMaxCapacity] = {};
    for (uint8_t i = 0; i < count; i++) {
        values[i] = std::fabs(residuals[i]);
    }
    std::sort(values, values + count);
    return values[count / 2];
}

void buildSolveMatrices(const RobustTdoaRow* rows,
                        const RobustEstimatorResult& result,
                        PosMatrix& L,
                        PosMatrix& R,
                        DynVector& doas,
                        DynVector& weights,
                        const Scalar* source_weights)
{
    const int n = result.selected_rows;
    L.resize(n, 3);
    R.resize(n, 3);
    doas.resize(n);
    weights.resize(n);

    for (int i = 0; i < n; i++) {
        const uint8_t source_index = result.selected_indices[i];
        const RobustTdoaRow& row = rows[source_index];
        L.row(i) = row.anchor_a_pos.transpose();
        R.row(i) = row.anchor_b_pos.transpose();
        doas(i) = row.tdoa;
        weights(i) = source_weights[source_index];
    }
}

void applyFinalRmseThreshold(RobustEstimatorResult& result, Scalar rmse_threshold)
{
    if (!std::isfinite(static_cast<double>(result.solve.rmse))
        || result.solve.rmse > rmse_threshold) {
        result.solve.valid = false;
    }
}

SolverResult withFinalRmseThreshold(SolverResult solve, Scalar rmse_threshold)
{
    if (!std::isfinite(static_cast<double>(solve.rmse))
        || solve.rmse > rmse_threshold) {
        solve.valid = false;
    }
    return solve;
}

} // namespace

RobustEstimatorResult estimateRobust3D(const RobustTdoaRow* rows,
                                       size_t row_count,
                                       const PosVector3D& initial_position,
                                       const RobustEstimatorOptions& options)
{
    RobustEstimatorResult result;
    result.solve.position = initial_position;
    result.solve.valid = false;
    result.solve.converged = false;
    result.solve.iterations = 0;
    result.solve.rmse = std::numeric_limits<Scalar>::infinity();
    result.solve.covarianceValid = false;
    result.solve.positionCovariance = CovMatrix3D::Identity();

    if (rows == nullptr || row_count < options.min_rows) {
        return result;
    }

    selectRows(rows, row_count, initial_position, options, result);
    if (result.selected_rows < options.min_rows
        || result.unique_anchors < options.min_unique_anchors) {
        return result;
    }
    if (!geometryAcceptable(
            selectedGeometryInfo(rows, result, initial_position, result.base_weights),
            options)) {
        return result;
    }

    PosMatrix L;
    PosMatrix R;
    DynVector doas;
    DynVector weights;
    buildSolveMatrices(rows, result, L, R, doas, weights, result.base_weights);

    const Scalar first_pass_rmse_threshold = options.enable_robust_pass
        ? std::numeric_limits<Scalar>::max()
        : options.rmse_threshold;
    SolverResult first = newtonRaphsonWeighted(
        L, R, doas, weights, initial_position,
        options.max_iterations,
        options.convergence_threshold,
        first_pass_rmse_threshold);
    result.solve = first;

    DynVector solve_residuals;
    computeResiduals3D(L, R, doas, first.position, solve_residuals);
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        const uint8_t source_index = result.selected_indices[i];
        result.residuals[source_index] = solve_residuals(i);
        result.final_weights[source_index] = result.base_weights[source_index];
    }

    if (!options.enable_robust_pass || !first.converged) {
        applyFinalRmseThreshold(result, options.rmse_threshold);
        return result;
    }

    Scalar selected_residuals[kMaxCapacity] = {};
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        selected_residuals[i] = solve_residuals(i);
    }

    const Scalar mad_scale = Scalar(1.4826) * medianAbsResidual(selected_residuals, result.selected_rows);
    result.residual_scale_m = std::max(options.min_residual_scale_m, mad_scale);
    const Scalar delta = options.huber_k * result.residual_scale_m;

    bool changed = false;
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        const uint8_t source_index = result.selected_indices[i];
        const Scalar abs_residual = std::fabs(solve_residuals(i));
        Scalar robust_weight = Scalar(1);
        if (abs_residual > delta && abs_residual > Scalar(1e-6)) {
            robust_weight = delta / abs_residual;
        }
        const Scalar next_weight = clampScalar(
            result.base_weights[source_index] * robust_weight,
            options.min_weight,
            Scalar(1));
        changed = changed || std::fabs(next_weight - result.base_weights[source_index]) > Scalar(1e-4);
        result.final_weights[source_index] = next_weight;
    }

    if (!changed) {
        applyFinalRmseThreshold(result, options.rmse_threshold);
        return result;
    }

    result.robust_pass_used = true;
    if (!geometryAcceptable(
            selectedGeometryInfo(rows, result, first.position, result.final_weights),
            options)) {
        const SolverResult thresholded_first = withFinalRmseThreshold(first, options.rmse_threshold);
        if (thresholded_first.valid) {
            result.solve = thresholded_first;
        } else {
            result.solve.valid = false;
        }
        return result;
    }

    buildSolveMatrices(rows, result, L, R, doas, weights, result.final_weights);
    result.solve = newtonRaphsonWeighted(
        L, R, doas, weights, first.position,
        options.max_iterations,
        options.convergence_threshold,
        options.rmse_threshold);
    if (!result.solve.valid) {
        const SolverResult thresholded_first = withFinalRmseThreshold(first, options.rmse_threshold);
        if (thresholded_first.valid) {
            result.solve = thresholded_first;
        }
    }

    computeResiduals3D(L, R, doas, result.solve.position, solve_residuals);
    for (uint8_t i = 0; i < result.selected_rows; i++) {
        const uint8_t source_index = result.selected_indices[i];
        result.residuals[source_index] = solve_residuals(i);
    }

    return result;
}

} // namespace tdoa_estimator
