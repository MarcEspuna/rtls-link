#pragma once

#include <cmath>
#include <stdint.h>

namespace Utils {

struct RunningStats {
    uint32_t count = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double minValue = 0.0;
    double maxValue = 0.0;
};

inline void UpdateRunningStats(RunningStats& stats, double value)
{
    stats.count++;
    if (stats.count == 1) {
        stats.mean = value;
        stats.m2 = 0.0;
        stats.minValue = value;
        stats.maxValue = value;
        return;
    }

    if (value < stats.minValue) {
        stats.minValue = value;
    }
    if (value > stats.maxValue) {
        stats.maxValue = value;
    }

    const double delta = value - stats.mean;
    stats.mean += delta / static_cast<double>(stats.count);
    const double delta2 = value - stats.mean;
    stats.m2 += delta * delta2;
}

inline double RunningStatsStd(const RunningStats& stats)
{
    if (stats.count <= 1) {
        return 0.0;
    }
    return std::sqrt(stats.m2 / static_cast<double>(stats.count));
}

template <typename JsonBuilder>
inline void AppendRunningStatsJson(JsonBuilder& out, const RunningStats& stats, uint8_t decimals)
{
    out.Append("{\"count\":");
    out.AppendUnsigned(stats.count);
    out.Append(",\"mean\":");
    out.AppendDouble(stats.mean, decimals);
    out.Append(",\"std\":");
    out.AppendDouble(RunningStatsStd(stats), decimals);
    out.Append(",\"min\":");
    out.AppendDouble(stats.count == 0 ? 0.0 : stats.minValue, decimals);
    out.Append(",\"max\":");
    out.AppendDouble(stats.count == 0 ? 0.0 : stats.maxValue, decimals);
    out.Append("}");
}

} // namespace Utils
