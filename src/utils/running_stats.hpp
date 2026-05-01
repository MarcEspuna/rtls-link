#pragma once

#include <Arduino.h>

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

inline void AppendRunningStatsJson(String& out, const RunningStats& stats, uint8_t decimals)
{
    const unsigned int places = static_cast<unsigned int>(decimals);
    out += "{\"count\":";
    out += String(stats.count);
    out += ",\"mean\":";
    out += String(stats.mean, places);
    out += ",\"std\":";
    out += String(RunningStatsStd(stats), places);
    out += ",\"min\":";
    out += String(stats.count == 0 ? 0.0 : stats.minValue, places);
    out += ",\"max\":";
    out += String(stats.count == 0 ? 0.0 : stats.maxValue, places);
    out += "}";
}

} // namespace Utils
