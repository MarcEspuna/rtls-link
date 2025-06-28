#pragma once

#include <Arduino.h>
#include <SPI.h>


#include "uwb_backend.hpp"


class UWBAnchor : public UWBBackend {
public:
    UWBAnchor(UWBFront& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay);

    void Update() override;

private:

};