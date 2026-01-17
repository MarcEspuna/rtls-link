#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TWR_ANCHOR

#include <Arduino.h>
#include <SPI.h>


#include "uwb_backend.hpp"


class UWBAnchor : public UWBBackend {
public:
    UWBAnchor(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay);

    void Update() override;

private:

};

#endif // USE_UWB_MODE_TWR_ANCHOR