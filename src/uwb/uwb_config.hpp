#pragma once

#include <Arduino.h>

struct UWBBackendSPI {
    uint8_t mosiPin;
    uint8_t misoPin;
    uint8_t sckPin;
    uint8_t csPin;
    uint8_t rstPin;
    uint8_t irqPin;
};



