#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include <Arduino.h>
#include "protocol/rtls_binary_protocol.hpp"

namespace TDoAAnchorModelCommands {
    void Reset();
    bool StartCollection();
    bool Lock();
    String StatusJson();
    String CollectStatusJson();
    String ExportJson();
    String EstimatorStatsJson();
    void AppendBinaryStatus(rtls::protocol::BinaryFrameBuilder<2048>& outFrame, uint8_t view);
    void ResetEstimatorStats();
}

#endif // USE_UWB_MODE_TDOA_TAG
