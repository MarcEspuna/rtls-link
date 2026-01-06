#ifndef LOCAL_POSITION_SENSOR_HPP
#define LOCAL_POSITION_SENSOR_HPP

#include "serial_comm_interface.hpp"
#include <cstdint>
#include <chrono> // For time handling
#include <array>  // For std::array
#include <functional> // For std::function
#include <mutex>  // For thread-safe MAVLink TX

// Forward declare MAVLink message struct if needed, or include directly
// Including directly is simpler if the library always needs it.
#include <common/mavlink.h> // Assuming common dialect for now

// Define the covariance array size based on MAVLink spec
constexpr size_t VISION_POSITION_COVARIANCE_SIZE = 21; // 21 elements for the covariance matrix

class LocalPositionSensor {
public:
    /**
     * @brief Constructor for the LocalPositionSensor.
     * @param comm_interface Reference to a concrete implementation of ISerialComm.
     * @param system_id The MAVLink System ID for this sensor.
     * @param component_id The MAVLink Component ID for this sensor.
     */
    LocalPositionSensor(ISerialComm& comm_interface, uint8_t system_id = 2, uint8_t component_id = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY);
    ~LocalPositionSensor() = default;

    // Prevent copying and assignment
    LocalPositionSensor(const LocalPositionSensor&) = delete;
    LocalPositionSensor& operator=(const LocalPositionSensor&) = delete;

    /**
     * @brief Sends a VISION_POSITION_ESTIMATE message.
     * @param timestamp_us Timestamp of the estimate in microseconds (usec). Usually from `std::chrono` or sensor hardware.
     * @param x Local X position (NED frame if possible, meters).
     * @param y Local Y position (NED frame if possible, meters).
     * @param z Local Z position (NED frame if possible, meters).
     * @param roll Roll angle (radians).
     * @param pitch Pitch angle (radians).
     * @param yaw Yaw angle (radians).
     * @param covariance Optional reference to covariance matrix (roll-major, 21 elements: x, y, z, roll, pitch, yaw).
     * @param reset_counter Optional counter indicating position estimate resets.
     */
    bool send_vision_position_estimate(
        float x, float y, float z,
        float roll, float pitch, float yaw,
        const std::array<float, VISION_POSITION_COVARIANCE_SIZE>* covariance = nullptr,
        uint8_t reset_counter = 0);
    
    /**
     * @brief Sends a SET_GPS_GLOBAL_ORIGIN message.
     * This message sets the global position of the origin of the local coordinate system.
     * Note: This is required when using vision-based position estimation.
     * 
     * Note: The newer SET_GLOBAL_ORIGIN message is not available in this version
     * of the MAVLink library, so we use this one. It provides equivalent functionality.
     * 
     * @param latitude Latitude (degrees, floating point)
     * @param longitude Longitude (degrees, floating point)
     * @param altitude Altitude (meters, floating point, AMSL)
     * @param target_system System ID of the target system (usually the autopilot)
     * @param timestamp_us Optional timestamp in microseconds (defaults to current time)
     * @return bool True if message was successfully sent
     */
    bool send_set_gps_global_origin(
        double latitude, 
        double longitude, 
        float altitude,
        uint8_t target_system,
        uint64_t timestamp_us = 0);

    /**
     * @brief Sends a HEARTBEAT message.
     * Should be called periodically (e.g., once per second).
     */
    bool send_heartbeat();

    /**
     * @brief Sends a DISTANCE_SENSOR message (for rangefinder forwarding).
     * Uses copy-modify-encode pattern to preserve all fields from source.
     *
     * @param source_msg The original DISTANCE_SENSOR message received from rangefinder
     * @param override_sensor_id Sensor ID to use (0xFF = preserve source value)
     * @param override_orientation Orientation to use (0xFF = preserve source value)
     * @param source_sysid Source message system ID (used if preserve_source_ids is true)
     * @param source_compid Source message component ID (used if preserve_source_ids is true)
     * @param preserve_source_ids If true, use source sysid/compid instead of this device's
     * @return bool True if message was successfully sent
     */
    bool send_distance_sensor(
        const mavlink_distance_sensor_t& source_msg,
        uint8_t override_sensor_id = 0xFF,
        uint8_t override_orientation = 0xFF,
        uint8_t source_sysid = 0,
        uint8_t source_compid = 0,
        bool preserve_source_ids = false);

    /**
     * @brief Processes received bytes to potentially parse MAVLink messages (e.g., HEARTBEAT from ArduPilot).
     * To be implemented later if needed.
     * @param buffer Pointer to the received data buffer.
     * @param length Number of bytes received.
     */
    void process_received_bytes(const uint8_t* buffer, size_t length);

    /**
     * @brief Set a callback function to be called when a heartbeat is received
     * @param callback Function that takes system ID and component ID
     */
    void set_heartbeat_callback(std::function<void(uint8_t, uint8_t)> callback);

    // --- Configuration ---
    void set_system_id(uint8_t system_id);
    uint8_t get_system_id() const;
    void set_component_id(uint8_t component_id);
    uint8_t get_component_id() const;

private:
    ISerialComm& comm_interface_;
    uint8_t system_id_;
    uint8_t component_id_;
    std::function<void(uint8_t, uint8_t)> heartbeat_callback_;
    mutable std::mutex tx_mutex_;  // Protects send_message() for thread safety

    // Internal helper to pack and send a message (thread-safe)
    bool send_message(const mavlink_message_t& msg);

    // Internal state for heartbeat
    uint8_t mavlink_type_ = MAV_TYPE_GENERIC; // Type of this MAV system
    uint8_t mavlink_autopilot_ = MAV_AUTOPILOT_INVALID; // No autopilot on this system
    uint8_t mavlink_base_mode_ = 0; // Not used for sensor usually
    uint32_t mavlink_custom_mode_ = 0; // No custom mode
    uint8_t mavlink_system_status_ = MAV_STATE_ACTIVE; // System is active
};

#endif // LOCAL_POSITION_SENSOR_HPP 