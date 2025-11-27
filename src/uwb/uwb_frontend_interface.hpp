#pragma once

#include <stdint.h>
#include "uwb_params.hpp"

class IUWBFrontend {
public:
    virtual ~IUWBFrontend() = default;

    virtual void UpdateAntennaDelay(uint16_t delay) = 0;
    virtual void UpdateMode(UWBMode mode) = 0;
};
