#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "bsp/board.hpp"

#include "uwb_params.hpp"
#include "uwb_config.hpp"

class UWBFront;

class UWBBackend {
public:
    UWBBackend(UWBFront& front, const bsp::UWBConfig& spi_config);
    virtual ~UWBBackend() = default;

    virtual void Update() = 0;

    virtual bool Start() { return false; }

    virtual uint32_t GetNumberOfConnectedDevices();

    // Methods for backend implementation to publish to frontend
    void UpdateAntennaDelay(uint16_t delay);
    void UpdateMode(UWBMode mode);

protected:
    UWBFront& m_Front;
};