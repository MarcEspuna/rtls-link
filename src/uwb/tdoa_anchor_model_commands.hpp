#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include <Arduino.h>

namespace TDoAAnchorModelCommands {
    void Reset();
    bool StartCollection();
    bool Lock();
    String StatusJson();
    String CollectStatusJson();
    String ExportJson();
    String EstimatorStatsJson();
    void ResetEstimatorStats();
}

#endif // USE_UWB_MODE_TDOA_TAG
