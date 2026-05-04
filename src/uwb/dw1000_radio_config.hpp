#pragma once

#include <stdint.h>

#include "uwb_params.hpp"

extern "C" {
    #include "libdw1000.h"
}

namespace dw1000_radio {

const uint8_t* ModeByIndex(uint8_t index);
bool Is64MHzPrfMode(uint8_t index);
void ApplyTxPower(dwDevice_t* dev, uint8_t powerLevel, uint8_t smartPowerEnable);
void ApplyTdoaRadioParams(dwDevice_t* dev, const UWBParams& params);

} // namespace dw1000_radio
