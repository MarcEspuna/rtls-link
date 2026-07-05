#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include <Arduino.h>

#include "protocol/rtls_binary_protocol.hpp"

namespace TDoAPositionEstimatorCommands {
    String StatusJson();
    void AppendBinaryStatus(rtls::protocol::BinaryFrameBuilder<2048>& outFrame);
    void ResetStats();
}

#ifdef USE_DYNAMIC_ANCHOR_POSITIONS
namespace TDoADynamicAnchorCommands {
    // Diagnostic dump of the dynamic anchor position calculator: per-pair
    // accumulation counts / readiness / distances, and overall state.
    String StatusJson();
}
#endif

#endif // USE_UWB_MODE_TDOA_TAG
