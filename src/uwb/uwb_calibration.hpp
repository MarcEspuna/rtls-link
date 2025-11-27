#pragma once

#include "uwb_backend.hpp"

class UWBCalibration : public UWBBackend {
public:
    UWBCalibration(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, float calibrationDistance);

    void Update() override;

private:

};
