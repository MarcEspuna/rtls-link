#pragma once

#include <stdint.h>
#include <stddef.h>

#include <Arduino.h>
#include <HardwareSerial.h>

#include "serial_comm_interface.hpp"

class UartComm : public ISerialComm {
public:
    UartComm(HardwareSerial& serial);
    virtual bool send_bytes(const uint8_t* data, size_t length) override;
    virtual uint64_t get_time_us() override;

private:
    HardwareSerial& serial_;
};