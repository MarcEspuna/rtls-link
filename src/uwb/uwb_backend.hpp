#pragma once

#include <Arduino.h>
#include <SPI.h>

#include "bsp/board.hpp"

#include "uwb_params.hpp"
#include "uwb_config.hpp"

#include "uwb_frontend_interface.hpp"

class UWBBackend {
public:
    UWBBackend(IUWBFrontend& front, const bsp::UWBConfig& spi_config);
    virtual ~UWBBackend() = default;

    virtual void Update() = 0;

    virtual bool Start() { return false; }

    virtual uint32_t GetNumberOfConnectedDevices();

    // Methods for backend implementation to publish to frontend
    void UpdateAntennaDelay(uint16_t delay);
    void UpdateMode(UWBMode mode);

protected:
    IUWBFrontend& m_Front;
};