#include "tdoa_newton_raphson.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#ifdef TRACK_MALLOC
extern "C" void* __real_malloc(size_t size);
extern "C" void __real_free(void* ptr);
extern "C" void* __real_calloc(size_t nmemb, size_t size);
extern "C" void* __real_realloc(void* ptr, size_t size);

static std::atomic<uint64_t> g_mallocCalls{0};
static std::atomic<uint64_t> g_mallocBytes{0};

extern "C" void* __wrap_malloc(size_t size)
{
    g_mallocCalls.fetch_add(1, std::memory_order_relaxed);
    g_mallocBytes.fetch_add(static_cast<uint64_t>(size), std::memory_order_relaxed);
    return __real_malloc(size);
}

extern "C" void __wrap_free(void* ptr)
{
    __real_free(ptr);
}

extern "C" void* __wrap_calloc(size_t nmemb, size_t size)
{
    g_mallocCalls.fetch_add(1, std::memory_order_relaxed);
    g_mallocBytes.fetch_add(static_cast<uint64_t>(nmemb) * static_cast<uint64_t>(size), std::memory_order_relaxed);
    return __real_calloc(nmemb, size);
}

extern "C" void* __wrap_realloc(void* ptr, size_t size)
{
    g_mallocCalls.fetch_add(1, std::memory_order_relaxed);
    g_mallocBytes.fetch_add(static_cast<uint64_t>(size), std::memory_order_relaxed);
    return __real_realloc(ptr, size);
}
#endif

namespace {

using Clock = std::chrono::steady_clock;

struct Pair {
    int left;
    int right;
};

struct Scenario {
    std::string name;
    std::vector<Eigen::Vector3d> anchors;
    std::vector<Pair> pairs;
    bool solve3d;
    int maxIterations;
    double convergenceThreshold;
};

struct TimingStats {
    double meanUs = 0.0;
    double p50Us = 0.0;
    double p95Us = 0.0;
    double p99Us = 0.0;
    double maxUs = 0.0;
    double meanIterations = 0.0;
    double meanMallocCalls = 0.0;
    double meanMallocBytes = 0.0;
    int validCount = 0;
    int convergedCount = 0;
};

std::vector<Eigen::Vector3d> make2DAnchors(double w, double d)
{
    return {
        {0.0, 0.0, 0.0},
        {w, 0.0, 0.0},
        {w, d, 0.0},
        {0.0, d, 0.0},
    };
}

std::vector<Eigen::Vector3d> make6Anchors(double w, double d)
{
    return {
        {0.0, 0.0, 0.0},
        {w, 0.0, 0.0},
        {w, d, 0.0},
        {0.0, d, 0.0},
        {w * 0.5, 0.0, 0.0},
        {w * 0.5, d, 0.0},
    };
}

std::vector<Eigen::Vector3d> make3DAnchors(double w, double d, double h)
{
    auto lower = make2DAnchors(w, d);
    auto anchors = lower;
    for (auto top : lower) {
        top.z() = h;
        anchors.push_back(top);
    }
    return anchors;
}

std::vector<Pair> allPairs(int count)
{
    std::vector<Pair> pairs;
    for (int i = 0; i < count; ++i) {
        for (int j = i + 1; j < count; ++j) {
            pairs.push_back({i, j});
        }
    }
    return pairs;
}

std::vector<Pair> sparsePairs8()
{
    return {
        {0, 1}, {1, 2}, {2, 3}, {3, 0},
        {4, 5}, {5, 6}, {6, 7}, {7, 4},
        {0, 4}, {1, 5}, {2, 6}, {3, 7},
    };
}

Eigen::Vector3d randomPoint(std::mt19937_64& rng, bool solve3d)
{
    std::uniform_real_distribution<double> xy(0.35, 4.65);
    if (!solve3d) {
        return {xy(rng), xy(rng), 0.0};
    }

    std::uniform_real_distribution<double> z(0.45, 3.45);
    return {xy(rng), xy(rng), z(rng)};
}

double tdoaValue(const Eigen::Vector3d& p, const Eigen::Vector3d& left, const Eigen::Vector3d& right)
{
    return (p - left).norm() - (p - right).norm();
}

void fillMatrices(
    const std::vector<Eigen::Vector3d>& anchors,
    const std::vector<Pair>& pairs,
    const Eigen::Vector3d& truth,
    std::mt19937_64& rng,
    double noiseSigmaM,
    tdoa_estimator::PosMatrix& left,
    tdoa_estimator::PosMatrix& right,
    tdoa_estimator::DynVector& doas)
{
    std::normal_distribution<double> noise(0.0, noiseSigmaM);
    const int n = static_cast<int>(pairs.size());
    left.resize(n, 3);
    right.resize(n, 3);
    doas.resize(n);

    for (int i = 0; i < n; ++i) {
        const auto& pair = pairs[i];
        left.row(i) = anchors[pair.left].transpose();
        right.row(i) = anchors[pair.right].transpose();
        doas(i) = tdoaValue(truth, anchors[pair.left], anchors[pair.right]) + noise(rng);
    }
}

double percentile(std::vector<double> sorted, double p)
{
    if (sorted.empty()) {
        return 0.0;
    }

    std::sort(sorted.begin(), sorted.end());
    const double idx = (static_cast<double>(sorted.size()) - 1.0) * p;
    const auto lo = static_cast<size_t>(std::floor(idx));
    const auto hi = static_cast<size_t>(std::ceil(idx));
    if (lo == hi) {
        return sorted[lo];
    }
    const double frac = idx - static_cast<double>(lo);
    return sorted[lo] * (1.0 - frac) + sorted[hi] * frac;
}

TimingStats runScenario(const Scenario& scenario, int warmup, int samples)
{
    std::mt19937_64 rng(0x54444f41ULL);
    std::vector<double> timingsUs;
    timingsUs.reserve(samples);

    int totalIterations = 0;
    int validCount = 0;
    int convergedCount = 0;
    uint64_t totalMallocCalls = 0;
    uint64_t totalMallocBytes = 0;

    tdoa_estimator::PosMatrix left;
    tdoa_estimator::PosMatrix right;
    tdoa_estimator::DynVector doas;
    tdoa_estimator::DynVector initial3d(3);
    initial3d << 2.5, 2.5, scenario.solve3d ? 1.8 : 0.0;
    tdoa_estimator::PosVector2D initial2d;
    initial2d << 2.5, 2.5;

    const int totalRuns = warmup + samples;
    for (int run = 0; run < totalRuns; ++run) {
        const Eigen::Vector3d truth = randomPoint(rng, scenario.solve3d);
        fillMatrices(scenario.anchors, scenario.pairs, truth, rng, 0.015, left, right, doas);

#ifdef TRACK_MALLOC
        const uint64_t mallocCallsBefore = g_mallocCalls.load(std::memory_order_relaxed);
        const uint64_t mallocBytesBefore = g_mallocBytes.load(std::memory_order_relaxed);
#endif
        const auto start = Clock::now();
        int iterations = 0;
        bool valid = false;
        bool converged = false;

        if (scenario.solve3d) {
            auto result = tdoa_estimator::newtonRaphson(
                left,
                right,
                doas,
                initial3d,
                scenario.maxIterations,
                scenario.convergenceThreshold,
                0.8);
            iterations = result.iterations;
            valid = result.valid;
            converged = result.converged;
        } else {
            auto result = tdoa_estimator::newtonRaphson2D(
                left,
                right,
                doas,
                initial2d,
                0.0,
                scenario.maxIterations,
                scenario.convergenceThreshold,
                0.8);
            iterations = result.iterations;
            valid = result.valid;
            converged = result.converged;
        }

        const auto stop = Clock::now();
#ifdef TRACK_MALLOC
        const uint64_t mallocCallsAfter = g_mallocCalls.load(std::memory_order_relaxed);
        const uint64_t mallocBytesAfter = g_mallocBytes.load(std::memory_order_relaxed);
#endif

        if (run >= warmup) {
            const double us = std::chrono::duration<double, std::micro>(stop - start).count();
            timingsUs.push_back(us);
            totalIterations += iterations;
            validCount += valid ? 1 : 0;
            convergedCount += converged ? 1 : 0;
#ifdef TRACK_MALLOC
            totalMallocCalls += (mallocCallsAfter - mallocCallsBefore);
            totalMallocBytes += (mallocBytesAfter - mallocBytesBefore);
#endif
        }
    }

    TimingStats stats;
    stats.meanUs = std::accumulate(timingsUs.begin(), timingsUs.end(), 0.0) / timingsUs.size();
    stats.p50Us = percentile(timingsUs, 0.50);
    stats.p95Us = percentile(timingsUs, 0.95);
    stats.p99Us = percentile(timingsUs, 0.99);
    stats.maxUs = *std::max_element(timingsUs.begin(), timingsUs.end());
    stats.meanIterations = static_cast<double>(totalIterations) / static_cast<double>(samples);
#ifdef TRACK_MALLOC
    stats.meanMallocCalls = static_cast<double>(totalMallocCalls) / static_cast<double>(samples);
    stats.meanMallocBytes = static_cast<double>(totalMallocBytes) / static_cast<double>(samples);
#endif
    stats.validCount = validCount;
    stats.convergedCount = convergedCount;
    return stats;
}

void printResult(const Scenario& scenario, const TimingStats& stats, int samples)
{
    std::cout << std::left << std::setw(26) << scenario.name
              << " meas=" << std::right << std::setw(2) << scenario.pairs.size()
              << " dim=" << (scenario.solve3d ? 3 : 2)
              << " mean=" << std::fixed << std::setprecision(2) << std::setw(8) << stats.meanUs << " us"
              << " p50=" << std::setw(8) << stats.p50Us << " us"
              << " p95=" << std::setw(8) << stats.p95Us << " us"
              << " p99=" << std::setw(8) << stats.p99Us << " us"
              << " max=" << std::setw(8) << stats.maxUs << " us"
              << " iter=" << std::setprecision(2) << std::setw(5) << stats.meanIterations
#ifdef TRACK_MALLOC
              << " mallocs=" << std::setw(6) << stats.meanMallocCalls
              << " bytes=" << std::setw(8) << stats.meanMallocBytes
#endif
              << " valid=" << stats.validCount << "/" << samples
              << " conv=" << stats.convergedCount << "/" << samples
              << "\n";
}

} // namespace

int main(int argc, char** argv)
{
    int samples = 20000;
    int warmup = 1000;
    if (argc >= 2) {
        samples = std::max(100, std::atoi(argv[1]));
    }

    const auto anchors4 = make2DAnchors(5.0, 5.0);
    const auto anchors6 = make6Anchors(5.0, 5.0);
    const auto anchors8 = make3DAnchors(5.0, 5.0, 4.0);

    const std::vector<Scenario> scenarios = {
        {"4 anchors 2D all-pairs", anchors4, allPairs(4), false, 20, 1e-4},
        {"4 anchors 3D all-pairs", anchors4, allPairs(4), true, 20, 1e-4},
        {"6 anchors 2D all-pairs", anchors6, allPairs(6), false, 20, 1e-4},
        {"6 anchors 3D all-pairs", anchors6, allPairs(6), true, 20, 1e-4},
        {"8 anchors 3D sparse", anchors8, sparsePairs8(), true, 20, 1e-4},
        {"8 anchors 3D all-pairs", anchors8, allPairs(8), true, 20, 1e-4},
        {"8 anchors 3D forced-10", anchors8, allPairs(8), true, 10, 0.0},
        {"8 anchors 3D forced-20", anchors8, allPairs(8), true, 20, 0.0},
    };

    std::cout << "TDoA Eigen solver benchmark\n";
    std::cout << "samples=" << samples << " warmup=" << warmup << "\n\n";

    for (const auto& scenario : scenarios) {
        const TimingStats stats = runScenario(scenario, warmup, samples);
        printResult(scenario, stats, samples);
    }

    return 0;
}
