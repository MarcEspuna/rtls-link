#include "rangefinder_sensor.hpp"

#include <Arduino.h>
#include <cstring>

RangefinderSensor::RangefinderSensor(HardwareSerial& serial)
    : serial_(serial), distance_callback_(nullptr)
{
    memset(&mavlink_status_, 0, sizeof(mavlink_status_));
}

void RangefinderSensor::init(uint32_t baud_rate, uint8_t rx_pin, uint8_t tx_pin) {
    serial_.begin(baud_rate, SERIAL_8N1, rx_pin, tx_pin);
    printf("[RangefinderSensor] Initialized on RX:%d TX:%d @ %lu baud\n",
           rx_pin, tx_pin, baud_rate);
}

void RangefinderSensor::set_distance_callback(std::function<void(mavlink_distance_sensor_t, uint64_t, uint8_t, uint8_t)> callback) {
    distance_callback_ = callback;
}

void RangefinderSensor::process_received_bytes() {
    mavlink_message_t msg;

    while (serial_.available()) {
        uint8_t c = serial_.read();

        // Use MAVLINK_COMM_1 to avoid conflict with LocalPositionSensor on MAVLINK_COMM_0
        if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &mavlink_status_)) {
            // Successfully parsed a message
            if (msg.msgid == MAVLINK_MSG_ID_DISTANCE_SENSOR) {
                mavlink_distance_sensor_t distance_sensor;
                mavlink_msg_distance_sensor_decode(&msg, &distance_sensor);

                last_distance_cm_ = distance_sensor.current_distance;
                last_timestamp_ms_ = millis();
                has_received_data_ = true;

                // Invoke callback with full struct (by value) and source IDs if set
                if (distance_callback_) {
                    distance_callback_(distance_sensor, last_timestamp_ms_, msg.sysid, msg.compid);
                }
            }
        }
    }
}
