#include "tdoa_newton_raphson.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct Pair {
    int left;
    int right;
};

struct Stats {
    double meanUs = 0.0;
    double p50Us = 0.0;
    double p95Us = 0.0;
    double p99Us = 0.0;
    double maxUs = 0.0;
};

std::vector<Eigen::Vector3d> anchors3d(double w, double d, double h)
{
    return {
        {0.0, 0.0, 0.0},
        {w, 0.0, 0.0},
        {w, d, 0.0},
        {0.0, d, 0.0},
        {0.0, 0.0, h},
        {w, 0.0, h},
        {w, d, h},
        {0.0, d, h},
    };
}

std::vector<Pair> allPairs(int count)
{
    std::vector<Pair> out;
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            out.push_back({i, j});
        }
    }
    return out;
}

std::vector<Pair> sparsePairs8()
{
    return {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };
}

Eigen::Vector3d randomPoint(std::mt19937_64& rng)
{
    std::uniform_real_distribution<double> xy(0.35, 4.65);
    std::uniform_real_distribution<double> z(0.45, 3.45);
    return {xy(rng), xy(rng), z(rng)};
}

double tdoaValue(const Eigen::Vector3d& point, const Eigen::Vector3d& left, const Eigen::Vector3d& right)
{
    return (point - left).norm() - (point - right).norm();
}

void fillMatrices(
    const std::vector<Eigen::Vector3d>& anchors,
    const std::vector<Pair>& pairs,
    const Eigen::Vector3d& truth,
    std::mt19937_64& rng,
    tdoa_estimator::PosMatrix& left,
    tdoa_estimator::PosMatrix& right,
    tdoa_estimator::DynVector& doas)
{
    std::normal_distribution<double> noise(0.0, 0.015);
    const int n = static_cast<int>(pairs.size());
    left.resize(n, 3);
    right.resize(n, 3);
    doas.resize(n);
    for (int i = 0; i < n; ++i) {
        left.row(i) = anchors[pairs[i].left].transpose();
        right.row(i) = anchors[pairs[i].right].transpose();
        doas(i) = tdoaValue(truth, anchors[pairs[i].left], anchors[pairs[i].right]) + noise(rng);
    }
}

tdoa_estimator::DynVector residuals(
    const tdoa_estimator::PosMatrix& left,
    const tdoa_estimator::PosMatrix& right,
    const tdoa_estimator::DynVector& doas,
    const tdoa_estimator::DynVector& pos)
{
    const int n = left.rows();
    tdoa_estimator::DynVector out(n);
    for (int i = 0; i < n; ++i) {
        out(i) = (left.row(i).transpose() - pos).norm()
               - (right.row(i).transpose() - pos).norm()
               - doas(i);
    }
    return out;
}

tdoa_estimator::DynVector step3d(
    const tdoa_estimator::PosMatrix& left,
    const tdoa_estimator::PosMatrix& right,
    const tdoa_estimator::DynVector& res,
    const tdoa_estimator::DynVector& pos)
{
    const int n = left.rows();
    tdoa_estimator::PosMatrix j(n, 3);
    for (int i = 0; i < n; ++i) {
        double dl = (left.row(i).transpose() - pos).norm();
        double dr = (right.row(i).transpose() - pos).norm();
        if (dl < 1e-6) {
            dl = 1e-6;
        }
        if (dr < 1e-6) {
            dr = 1e-6;
        }
        j.row(i) = (pos - left.row(i).transpose()).transpose() / dl
                 - (pos - right.row(i).transpose()).transpose() / dr;
    }
    return j.completeOrthogonalDecomposition().solve(res);
}

tdoa_estimator::DynVector solve3dNoCov(
    const tdoa_estimator::PosMatrix& left,
    const tdoa_estimator::PosMatrix& right,
    const tdoa_estimator::DynVector& doas,
    tdoa_estimator::DynVector pos,
    int maxIterations,
    int& iterations)
{
    iterations = 0;
    for (int i = 0; i < maxIterations; ++i) {
        ++iterations;
        const auto res = residuals(left, right, doas, pos);
        const auto step = step3d(left, right, res, pos);
        pos = pos - step;
        if (step.norm() < 1e-4) {
            break;
        }
    }
    return pos;
}

tdoa_estimator::CovMatrix3D computeCovariance3d(
    const tdoa_estimator::PosMatrix& left,
    const tdoa_estimator::PosMatrix& right,
    const tdoa_estimator::DynVector& pos,
    double measurementVariance)
{
    const int n = left.rows();
    tdoa_estimator::PosMatrix j(n, 3);
    for (int i = 0; i < n; ++i) {
        double dl = (left.row(i).transpose() - pos).norm();
        double dr = (right.row(i).transpose() - pos).norm();
        if (dl < 1e-6) {
            dl = 1e-6;
        }
        if (dr < 1e-6) {
            dr = 1e-6;
        }
        j.row(i) = (pos - left.row(i).transpose()).transpose() / dl
                 - (pos - right.row(i).transpose()).transpose() / dr;
    }

    tdoa_estimator::CovMatrix3D jtj = j.transpose() * j;
    Eigen::LDLT<tdoa_estimator::CovMatrix3D> ldlt(jtj);
    auto cov = ldlt.solve(tdoa_estimator::CovMatrix3D::Identity()) * measurementVariance;
    return (cov + cov.transpose()) / 2.0;
}

Stats summarize(std::vector<double> values)
{
    std::sort(values.begin(), values.end());
    auto pct = [&](double p) {
        const double idx = (values.size() - 1) * p;
        const auto lo = static_cast<size_t>(std::floor(idx));
        const auto hi = static_cast<size_t>(std::ceil(idx));
        if (lo == hi) {
            return values[lo];
        }
        const double frac = idx - lo;
        return values[lo] * (1.0 - frac) + values[hi] * frac;
    };

    Stats s;
    s.meanUs = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    s.p50Us = pct(0.50);
    s.p95Us = pct(0.95);
    s.p99Us = pct(0.99);
    s.maxUs = values.back();
    return s;
}

void printStats(const std::string& label, const Stats& s)
{
    std::cout << std::left << std::setw(24) << label
              << " mean=" << std::right << std::fixed << std::setprecision(3) << std::setw(8) << s.meanUs << " us"
              << " p50=" << std::setw(8) << s.p50Us << " us"
              << " p95=" << std::setw(8) << s.p95Us << " us"
              << " p99=" << std::setw(8) << s.p99Us << " us"
              << " max=" << std::setw(8) << s.maxUs << " us\n";
}

void runCase(const std::string& name, const std::vector<Pair>& pairs, int samples)
{
    std::mt19937_64 rng(0x434f5655ULL);
    const auto anchors = anchors3d(5.0, 5.0, 4.0);
    tdoa_estimator::PosMatrix left;
    tdoa_estimator::PosMatrix right;
    tdoa_estimator::DynVector doas;
    tdoa_estimator::DynVector initial(3);
    initial << 2.5, 2.5, 1.8;

    std::vector<double> solveTimes;
    std::vector<double> covarianceTimes;
    solveTimes.reserve(samples);
    covarianceTimes.reserve(samples);

    int totalIterations = 0;
    for (int i = 0; i < samples + 500; ++i) {
        const auto truth = randomPoint(rng);
        fillMatrices(anchors, pairs, truth, rng, left, right, doas);

        int iterations = 0;
        const auto t0 = Clock::now();
        auto pos = solve3dNoCov(left, right, doas, initial, 10, iterations);
        const auto t1 = Clock::now();

        const auto res = residuals(left, right, doas, pos);
        const double variance = std::max(0.0001, res.squaredNorm() / std::max(1, static_cast<int>(doas.size()) - 3));

        const auto t2 = Clock::now();
        volatile auto cov = computeCovariance3d(left, right, pos, variance);
        (void)cov;
        const auto t3 = Clock::now();

        if (i >= 500) {
            solveTimes.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
            covarianceTimes.push_back(std::chrono::duration<double, std::micro>(t3 - t2).count());
            totalIterations += iterations;
        }
    }

    std::cout << "\n# " << name << " measurements=" << pairs.size()
              << " avg_iterations=" << (static_cast<double>(totalIterations) / samples) << "\n";
    printStats("solve_no_cov", summarize(solveTimes));
    printStats("covariance_only", summarize(covarianceTimes));
}

} // namespace

int main(int argc, char** argv)
{
    int samples = 20000;
    if (argc > 1) {
        samples = std::max(100, std::atoi(argv[1]));
    }

    std::cout << "TDoA covariance/Jacobian overhead benchmark\n";
    std::cout << "samples=" << samples << "\n";
    runCase("8 anchors all pairs", allPairs(8), samples);
    runCase("8 anchors sparse 12", sparsePairs8(), samples);
    return 0;
}

