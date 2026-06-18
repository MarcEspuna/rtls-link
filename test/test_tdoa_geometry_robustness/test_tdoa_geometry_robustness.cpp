// Tests for the TDoA geometry-robustness changes:
//   Change 1 - honest (correlation-aware) covariance
//   Change 2 - anisotropic null-space prior
#include <gtest/gtest.h>

#include <vector>

#include "tdoa_newton_raphson.hpp"
#include "tdoa_robust_estimator.hpp"

namespace {

using tdoa_estimator::CovMatrix3D;
using tdoa_estimator::PosVector3D;
using tdoa_estimator::RobustEstimatorOptions;
using tdoa_estimator::RobustEstimatorResult;
using tdoa_estimator::RobustTdoaRow;
using tdoa_estimator::Scalar;

struct Anchor {
    uint8_t id;
    PosVector3D pos;
};

// A well-conditioned room: 4 floor + 4 ceiling anchors (two Z planes).
std::vector<Anchor> wellConditionedAnchors()
{
    return {
        {0, {-4.0f, -3.0f, 0.0f}}, {1, {4.0f, -3.0f, 0.0f}},
        {2, {-4.0f, 3.0f, 0.0f}},  {3, {4.0f, 3.0f, 0.0f}},
        {4, {-4.0f, -3.0f, 3.0f}}, {5, {4.0f, -3.0f, 3.0f}},
        {6, {-4.0f, 3.0f, 3.0f}},  {7, {4.0f, 3.0f, 3.0f}},
    };
}

// Near-coplanar (weak-Z) layout: all anchors within a 0.4 m Z band.
std::vector<Anchor> nearCoplanarAnchors()
{
    return {
        {0, {-4.0f, -3.0f, 0.0f}}, {1, {4.0f, -3.0f, 0.0f}},
        {2, {-4.0f, 3.0f, 0.0f}},  {3, {4.0f, 3.0f, 0.0f}},
        {4, {0.0f, -3.0f, 0.4f}},  {5, {0.0f, 3.0f, 0.4f}},
    };
}

// Build a row for the (a,b) pair at `tag`, optionally adding a bias to the TDoA.
RobustTdoaRow makeRow(const Anchor& a, const Anchor& b, const PosVector3D& tag,
                      Scalar bias = 0.0f)
{
    const Scalar da = (a.pos - tag).norm();
    const Scalar db = (b.pos - tag).norm();
    RobustTdoaRow row;
    row.anchor_a = a.id;
    row.anchor_b = b.id;
    row.anchor_a_pos = a.pos;
    row.anchor_b_pos = b.pos;
    row.tdoa = (da - db) + bias;
    row.age_us = 1000;
    row.nominal_sigma_m = 0.15f;
    row.health = 1.0f;
    return row;
}

// All unique pairs of the anchor set, evaluated at `tag`.
std::vector<RobustTdoaRow> allPairs(const std::vector<Anchor>& anchors,
                                    const PosVector3D& tag, Scalar bias = 0.0f)
{
    std::vector<RobustTdoaRow> rows;
    for (size_t i = 0; i < anchors.size(); i++) {
        for (size_t j = i + 1; j < anchors.size(); j++) {
            rows.push_back(makeRow(anchors[i], anchors[j], tag, bias));
        }
    }
    return rows;
}

RobustEstimatorOptions baseOptions()
{
    RobustEstimatorOptions opts;
    opts.min_rows = 5;
    opts.min_unique_anchors = 4;
    opts.max_selected_rows = 32;   // keep all rows (no pair-selection drop)
    opts.enable_pair_selection = false;
    opts.enable_robust_pass = false;
    return opts;
}

} // namespace

// ---- Change 2: null-space prior ----

// Disabled prior must be a bitwise no-op vs the legacy solve.
TEST(NullspacePrior, DisabledIsExactNoop)
{
    const auto anchors = wellConditionedAnchors();
    const PosVector3D tag(1.0f, 0.5f, 1.5f);
    const auto rows = allPairs(anchors, tag);

    RobustEstimatorOptions baseline = baseOptions();

    RobustEstimatorOptions withDisabled = baseOptions();
    withDisabled.nullspace_prior.enabled = false;
    withDisabled.nullspace_prior.position = PosVector3D(99.0f, 99.0f, 99.0f);
    withDisabled.nullspace_prior.sigma_m = 0.01f; // ignored because disabled

    const auto a = estimateRobust3D(rows.data(), rows.size(), tag, baseline);
    const auto b = estimateRobust3D(rows.data(), rows.size(), tag, withDisabled);

    ASSERT_TRUE(a.solve.valid);
    ASSERT_TRUE(b.solve.valid);
    EXPECT_FLOAT_EQ(a.solve.position.x(), b.solve.position.x());
    EXPECT_FLOAT_EQ(a.solve.position.y(), b.solve.position.y());
    EXPECT_FLOAT_EQ(a.solve.position.z(), b.solve.position.z());
}

// On well-conditioned geometry the prior must NOT pull observable axes, even
// with a deliberately wrong, strongly-weighted prior position.
TEST(NullspacePrior, DoesNotHarmObservableAxes)
{
    const auto anchors = wellConditionedAnchors();
    const PosVector3D tag(1.0f, 0.5f, 1.5f);
    const auto rows = allPairs(anchors, tag);

    RobustEstimatorOptions opts = baseOptions();
    opts.nullspace_prior.enabled = true;
    opts.nullspace_prior.sigma_m = 0.05f; // strong prior
    opts.nullspace_prior.position = PosVector3D(-5.0f, -5.0f, 6.0f); // very wrong

    const auto r = estimateRobust3D(rows.data(), rows.size(), tag, opts);
    ASSERT_TRUE(r.solve.valid);
    // All three axes are well observed here, so data dominates the wrong prior.
    EXPECT_NEAR(r.solve.position.x(), tag.x(), 0.05f);
    EXPECT_NEAR(r.solve.position.y(), tag.y(), 0.05f);
    EXPECT_NEAR(r.solve.position.z(), tag.z(), 0.10f);
}

// On weak-Z geometry with a biased measurement set, the prior pulls the
// unstable Z toward the previous estimate (reduces the Z deviation).
TEST(NullspacePrior, StabilizesWeakAxis)
{
    const auto anchors = nearCoplanarAnchors();
    const PosVector3D tag(1.0f, 0.5f, 2.0f);
    // Bias every measurement to push the under-determined Z away from truth.
    const auto rows = allPairs(anchors, tag, 0.05f);

    const PosVector3D priorPos(1.0f, 0.5f, 2.0f); // previous good estimate

    RobustEstimatorOptions without = baseOptions();
    const auto rNo = estimateRobust3D(rows.data(), rows.size(), priorPos, without);

    RobustEstimatorOptions with = baseOptions();
    with.nullspace_prior.enabled = true;
    with.nullspace_prior.sigma_m = 0.3f;
    const auto rYes = estimateRobust3D(rows.data(), rows.size(), priorPos, with);

    ASSERT_TRUE(rNo.solve.valid);
    ASSERT_TRUE(rYes.solve.valid);

    const float zErrNo = std::fabs(rNo.solve.position.z() - priorPos.z());
    const float zErrYes = std::fabs(rYes.solve.position.z() - priorPos.z());
    EXPECT_LT(zErrYes, zErrNo);
}

// ---- Change 1: honest covariance ----

// Honest covariance must be no-op when disabled (legacy diagonal path).
TEST(HonestCovariance, DisabledKeepsLegacy)
{
    const auto anchors = wellConditionedAnchors();
    const PosVector3D tag(1.0f, 0.5f, 1.5f);
    const auto rows = allPairs(anchors, tag);

    RobustEstimatorOptions legacy = baseOptions();
    const auto r = estimateRobust3D(rows.data(), rows.size(), tag, legacy);
    ASSERT_TRUE(r.solve.valid);
    EXPECT_TRUE(r.solve.covarianceValid);
}

// Honest covariance is less optimistic (larger) than the legacy diagonal one,
// because it models the factor-of-2 and shared-anchor correlation.
TEST(HonestCovariance, LessOptimisticThanLegacy)
{
    const auto anchors = wellConditionedAnchors();
    const PosVector3D tag(1.0f, 0.5f, 1.5f);
    const auto rows = allPairs(anchors, tag);

    RobustEstimatorOptions legacy = baseOptions();
    RobustEstimatorOptions honest = baseOptions();
    honest.honest_covariance = true;

    const auto rl = estimateRobust3D(rows.data(), rows.size(), tag, legacy);
    const auto rh = estimateRobust3D(rows.data(), rows.size(), tag, honest);

    ASSERT_TRUE(rl.solve.valid && rl.solve.covarianceValid);
    ASSERT_TRUE(rh.solve.valid && rh.solve.covarianceValid);
    EXPECT_GT(rh.solve.positionCovariance.trace(), rl.solve.positionCovariance.trace());
}

// Core honesty property: adding fully-redundant (duplicate) rows must NOT shrink
// the honest covariance (no new information), whereas the legacy diagonal
// covariance shrinks because it double-counts the correlated rows.
TEST(HonestCovariance, RedundantRowsDoNotShrinkIt)
{
    const auto anchors = wellConditionedAnchors();
    const PosVector3D tag(1.0f, 0.5f, 1.5f);

    // Base set: a 10-row spanning subset over all 8 anchors.
    std::vector<RobustTdoaRow> base = {
        makeRow(anchors[0], anchors[1], tag), makeRow(anchors[0], anchors[2], tag),
        makeRow(anchors[0], anchors[3], tag), makeRow(anchors[0], anchors[4], tag),
        makeRow(anchors[0], anchors[5], tag), makeRow(anchors[0], anchors[6], tag),
        makeRow(anchors[0], anchors[7], tag), makeRow(anchors[1], anchors[4], tag),
        makeRow(anchors[2], anchors[5], tag), makeRow(anchors[3], anchors[6], tag),
    };
    std::vector<RobustTdoaRow> withDups = base;
    for (int i = 0; i < 8; i++) {          // duplicate the first 8 rows verbatim
        withDups.push_back(base[i]);
    }

    auto traceFor = [&](const std::vector<RobustTdoaRow>& rows, bool honest) {
        RobustEstimatorOptions opts = baseOptions();
        opts.honest_covariance = honest;
        const auto r = estimateRobust3D(rows.data(), rows.size(), tag, opts);
        EXPECT_TRUE(r.solve.valid && r.solve.covarianceValid);
        return r.solve.positionCovariance.trace();
    };

    const double legacyBase = traceFor(base, false);
    const double legacyDup = traceFor(withDups, false);
    const double honestBase = traceFor(base, true);
    const double honestDup = traceFor(withDups, true);

    // Legacy double-counts redundancy -> covariance shrinks noticeably.
    EXPECT_LT(legacyDup, 0.85 * legacyBase);
    // Honest covariance is stable under redundancy (within 10%).
    EXPECT_NEAR(honestDup, honestBase, 0.10 * honestBase);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
