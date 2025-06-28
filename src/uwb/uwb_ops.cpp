#include "Arduino.h"
#include "SPI.h"



#include "uwb_ops.hpp"

#include <HardwareSerial.h>

namespace libDw1000 {

DwData* GetUserData(dwDevice_t* dev)
{
    return reinterpret_cast<DwData*>(dev->userdata);
}

void SpiRead(dwDevice_t* dev, const void *header, size_t headerLength, void* data, size_t dataLength)
{
    DwData* dw_data = GetUserData(dev);
    SPISettings curr_settings = dw_data->settings[dw_data->current_spi_idx];
    SPI.beginTransaction(curr_settings);
    digitalWrite(dw_data->cs_pin, LOW);
    for (size_t i = 0; i < headerLength; i++)
    {
        SPI.transfer(((uint8_t*)header)[i]);
    }
    for (size_t i = 0; i < dataLength; i++)
    {
        ((uint8_t*)data)[i] = SPI.transfer(0);
    }
    delayMicroseconds(5);
    digitalWrite(dw_data->cs_pin, HIGH);
    SPI.endTransaction();
}


void SpiWrite(dwDevice_t* dev, const void *header, size_t headerLength, const void* data, size_t dataLength)
{
    DwData* dw_data = GetUserData(dev);
    SPISettings curr_settings = dw_data->settings[dw_data->current_spi_idx];
    SPI.beginTransaction(curr_settings);
    digitalWrite(dw_data->cs_pin, LOW);
    for (size_t i = 0; i < headerLength; i++)
    {
        SPI.transfer(((uint8_t*)header)[i]);
    }
    for (size_t i = 0; i < dataLength; i++)
    {
        SPI.transfer(((uint8_t*)data)[i]);
    }
    delayMicroseconds(5);
    digitalWrite(dw_data->cs_pin, HIGH);
    SPI.endTransaction();
}


void SpiSetSpeed(dwDevice_t* dev, dwSpiSpeed_t speed)
{
    DwData* dw_data = GetUserData(dev);
    dw_data->current_spi_idx = static_cast<uint8_t>(speed);
    DelayMs(dev, 2);
}


void DelayMs(dwDevice_t* dev, unsigned int ms)
{
    delay(ms);
}


void Reset(dwDevice_t* dev)
{
    DwData* dw_data = GetUserData(dev);
    pinMode(dw_data->rst_pin, INPUT);                   
    
    // dw1000 data sheet v2.08 §5.6.1 page 20, the RSTn pin should not be driven high but left floating.
    pinMode(dw_data->rst_pin, INPUT);        // Reset should never be driven high
    digitalWrite(dw_data->rst_pin, LOW);
    pinMode(dw_data->rst_pin, OUTPUT);
    digitalWrite(dw_data->rst_pin, LOW);
    delay(2); // dw1000 data sheet v2.08 §5.6.1 page 20: nominal 50ns, to be safe take more time
    pinMode(dw_data->rst_pin, INPUT);        // Reset should never be driven high
    delay(10); // dwm1000 data sheet v1.2 page 5: nominal 3 ms, to be safe take more time

    // force into idle mode (although it should be already after reset)
    // idle();
}

}   // namespace libDw1000