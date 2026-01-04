#pragma once

#include <Arduino.h>
#include <common/mavlink.h>
#include <functional>
#include <cstdint>

class RangefinderSensor {
public:
    explicit RangefinderSensor(HardwareSerial& serial);
    ~RangefinderSensor() = default;

    // Prevent copying
    RangefinderSensor(const RangefinderSensor&) = delete;
    RangefinderSensor& operator=(const RangefinderSensor&) = delete;

    // Initialize the serial port
    void init(uint32_t baud_rate, uint8_t rx_pin, uint8_t tx_pin);

    // Process incoming bytes - call this from Update loop
    void process_received_bytes();

    // Set callback for distance updates
    // Callback receives: distance_cm, timestamp_ms
    void set_distance_callback(std::function<void(uint16_t, uint64_t)> callback);

    // Get last known distance (cm) and timestamp
    uint16_t get_last_distance_cm() const { return last_distance_cm_; }
    uint64_t get_last_timestamp_ms() const { return last_timestamp_ms_; }
    bool has_valid_data() const { return has_received_data_; }

private:
    HardwareSerial& serial_;
    std::function<void(uint16_t, uint64_t)> distance_callback_;

    uint16_t last_distance_cm_ = 0;
    uint64_t last_timestamp_ms_ = 0;
    bool has_received_data_ = false;

    mavlink_status_t mavlink_status_{};  // Parser state for this channel
};
