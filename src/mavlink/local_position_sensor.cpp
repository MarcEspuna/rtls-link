#include "local_position_sensor.hpp"

#include <common/mavlink.h> // Make sure MAVLink headers are included

#include <array>            // For std::array
#include <cstring>          // For std::memcpy, std::memset
#include <cstdio>           // For printf
#include <cmath>            // For NAN

// Helper to convert floating point degrees to integer degE7 (degrees * 10^7)
static int32_t deg_to_degE7(double deg) {
    return static_cast<int32_t>(deg * 1e7);
}

// Helper to convert meters to millimeters
static int32_t meters_to_mm(float meters) {
    return static_cast<int32_t>(meters * 1000.0f);
}

LocalPositionSensor::LocalPositionSensor(ISerialComm& comm_interface, uint8_t system_id, uint8_t component_id) :
    comm_interface_(comm_interface),
    system_id_(system_id),
    component_id_(component_id),
    heartbeat_callback_(nullptr)
{}

// Add method to set heartbeat callback
void LocalPositionSensor::set_heartbeat_callback(std::function<void(uint8_t, uint8_t)> callback) {
    heartbeat_callback_ = callback;
}

// --- Message Sending ---

bool LocalPositionSensor::send_message(const mavlink_message_t& msg) {
    // Create buffer, pack message, and send using comm_interface_
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> send_buf;
    uint16_t len = mavlink_msg_to_send_buffer(send_buf.data(), &msg);
    if (len > 0) {
        return comm_interface_.send_bytes(send_buf.data(), len);
    }
    return false;
}


bool LocalPositionSensor::send_vision_position_estimate(
    float x, float y, float z,
    float roll, float pitch, float yaw,
    const std::array<float, VISION_POSITION_COVARIANCE_SIZE>* covariance,
    uint8_t reset_counter)
{
    mavlink_message_t msg;
    mavlink_vision_position_estimate_t vpe;

    vpe.usec = comm_interface_.get_time_us();
    vpe.x = x;
    vpe.y = y;
    vpe.z = z;
    vpe.roll = roll;
    vpe.pitch = pitch;
    vpe.yaw = yaw;
    vpe.reset_counter = reset_counter;

    if (covariance) {
        // MAVLink expects float[21] for covariance - copy from the std::array
        std::memcpy(vpe.covariance, covariance->data(), sizeof(float) * VISION_POSITION_COVARIANCE_SIZE);
    } else {
        // Fill with NaN if no covariance provided
        for(size_t i=0; i < MAVLINK_MSG_VISION_POSITION_ESTIMATE_FIELD_COVARIANCE_LEN; ++i) {
            vpe.covariance[i] = NAN;
        }
    }

    mavlink_msg_vision_position_estimate_pack(
        system_id_, component_id_, &msg,
        vpe.usec, vpe.x, vpe.y, vpe.z,
        vpe.roll, vpe.pitch, vpe.yaw,
        vpe.covariance, vpe.reset_counter);

    return send_message(msg);
}


bool LocalPositionSensor::send_heartbeat() {
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;

    heartbeat.type = mavlink_type_;
    heartbeat.autopilot = mavlink_autopilot_;
    heartbeat.base_mode = mavlink_base_mode_;
    heartbeat.custom_mode = mavlink_custom_mode_;
    heartbeat.system_status = mavlink_system_status_;
    heartbeat.mavlink_version = 3; // MAVLink protocol version

    mavlink_msg_heartbeat_pack(
        system_id_, component_id_, &msg,
        heartbeat.type, heartbeat.autopilot, heartbeat.base_mode,
        heartbeat.custom_mode, heartbeat.system_status);

    return send_message(msg);
}

bool LocalPositionSensor::send_set_gps_global_origin(
    double latitude, 
    double longitude, 
    float altitude,
    uint8_t target_system,
    uint64_t timestamp_us)
{
    mavlink_message_t msg;
    
    // Use current time if timestamp not provided
    if (timestamp_us == 0) {
        timestamp_us = comm_interface_.get_time_us();
    }
    
    // Convert degrees to degE7 (degrees * 10^7) and meters to millimeters
    int32_t lat_degE7 = deg_to_degE7(latitude);
    int32_t lon_degE7 = deg_to_degE7(longitude);
    int32_t alt_mm = meters_to_mm(altitude);
    
    // Pack the SET_GPS_GLOBAL_ORIGIN message
    mavlink_msg_set_gps_global_origin_pack(
        system_id_, component_id_, &msg,
        target_system, lat_degE7, lon_degE7, alt_mm, timestamp_us);
    
    return send_message(msg);
}

// --- Message Receiving ---

void LocalPositionSensor::process_received_bytes(const uint8_t* buffer, size_t length) {
    mavlink_message_t msg;
    mavlink_status_t status;

    for (size_t i = 0; i < length; ++i) {
        // Try to parse the byte
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &msg, &status)) {
            // Packet successfully received and parsed: msg contains the message
            // printf("Received MAVLink message with ID: %d from SysID: %d CompID: %d\n", 
            //        msg.msgid, msg.sysid, msg.compid);

            // Handle specific messages, e.g., heartbeat from ArduPilot
            switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                    
                    // Print detailed heartbeat info
                    // printf("  Heartbeat received from system %d, component %d\n", 
                    //        (int)msg.sysid, (int)msg.compid);
                    // printf("  Type: %d, Autopilot: %d, System Status: %d\n", 
                    //        (int)heartbeat.type, (int)heartbeat.autopilot, 
                    //        (int)heartbeat.system_status);
                    
                    // Instead of using dynamic_cast, use the callback if set
                    if (heartbeat_callback_) {
                        heartbeat_callback_(msg.sysid, msg.compid);
                    }
                    break;
                }
                // Add cases for other messages you might want to handle
                default:
                    break;
            }
        }
    }
    // Note: mavlink_parse_char maintains state between calls for incomplete packets.
}


// --- Configuration ---

void LocalPositionSensor::set_system_id(uint8_t system_id) {
    system_id_ = system_id;
}

uint8_t LocalPositionSensor::get_system_id() const {
    return system_id_;
}

void LocalPositionSensor::set_component_id(uint8_t component_id) {
    component_id_ = component_id;
}

uint8_t LocalPositionSensor::get_component_id() const {
    return component_id_;
} 