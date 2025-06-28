#pragma once

#include <Arduino.h>
#include <SPI.h>

extern "C" {
    #include "libdw1000.h"
}

#include "bsp/board.hpp"

namespace libDw1000 {

    enum IsrFlags : uint8_t {
        RX_DONE = 1 << 0,
        TX_DONE = 1 << 1,
        RX_TIMEOUT = 1 << 2,
        RX_FAILED = 1 << 3,
    };

    /**
     * @brief User data struct for the libdw1000 library
     * @note Must be setup before calling dwInit
     */
    struct DwData {
        const SPISettings settings[2] = {
            SPISettings(bsp::kBoardConfig.uwb.slow_spi_clk_hz, MSBFIRST, SPI_MODE0),
            SPISettings(bsp::kBoardConfig.uwb.fast_spi_clk_hz, MSBFIRST, SPI_MODE0)
        };
        uint8_t current_spi_idx = 0;
        uint8_t rst_pin;
        uint8_t cs_pin;
        uint8_t interrupt_flags;
    };

    DwData* GetUserData(dwDevice_t* dev);

    void SpiRead(dwDevice_t* dev, const void *header, size_t headerLength, void* data, size_t dataLength);


    void SpiWrite(dwDevice_t* dev, const void *header, size_t headerLength, const void* data, size_t dataLength);


    void SpiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed);


    void DelayMs(dwDevice_t* dev, unsigned int ms);


    void Reset(dwDevice_t* dev);
}


