#include <SPI.h>

#include "bsp/board.hpp"

#include "uwb_backend.hpp"

UWBBackend::UWBBackend(const bsp::UWBConfig& uwb_config)
{
    // Wait for 100ms to make sure serial has transmitted all the data previously written.
    vTaskDelay(pdMS_TO_TICKS(1000));
    Serial.begin(115200);

    const bsp::UWBPinout& pinout = uwb_config.pins; 
    SPI.begin(pinout.spi_clk_pin, pinout.spi_miso_pin, pinout.spi_mosi_pin);
    pinMode(pinout.spi_cs_pin, OUTPUT);
    digitalWrite(pinout.spi_cs_pin, HIGH);
    pinMode(pinout.reset_pin, INPUT);           // DW1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
    pinMode(pinout.int_pin, INPUT);
}
