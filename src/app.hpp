#pragma once

/**
 * Scheduling all the tasks
 * 
 */
#include "front.hpp"
#include "scheduler.hpp"

#include "mavlink/local_position_sensor.hpp"
#include "mavlink/uart_comm.hpp"

#include <etl/delegate.h>
#include <cmath> // For sin and cos functions

// Define a Vector3f struct for 3D coordinates
struct Vector3f {
    float x;
    float y;
    float z;
};

class App {
public:

    App();
    void Init();

    void Update();

    // Should be called periodically for beacon protocool
    void AddAnchorEcho();

    // Task that blinks LED based on current status. Called at 10Hz rate
    void StatusLedTask();
    
    // Setup the anchors to echo
    static void AnchorsToEcho(const etl::array<double, 12>& anchor_locations);

    static void SendSample(float x_m, float y_m, float z_m, uint16_t error);
    static void SendRangeSample(uint8_t id, float range);

    static void Start();
    static void Stop();
    
    static HardwareSerial& GetArdupilotSerial();
    static App& GetInstance();

    // Helper function to correct for yaw orientation
    static Vector3f correct_for_orient_yaw(float x, float y, float z);

private:
    UartComm uart_comm_;
    LocalPositionSensor local_position_sensor_;

    uint64_t last_sample_timestamp_ms_;
    uint64_t device_unhealthy_timestamp_ms_;
    uint64_t last_heartbeat_timestamp_ms_;
    uint64_t last_heartbeat_received_timestamp_ms_;
    bool is_origin_position_sent_ = false;
    bool is_origin_set_ = false;
private:
    static constexpr uint64_t kSendOriginPositionAfterMs = 16000;    // After 8 seconds healthy device, send global origin position
    static constexpr uint64_t kDeviceHealtyMinDurationMs = 100;     // If no packet sent for more than 100ms, consider device as unhealthy

    // HARDCODE FOR NOW
    static constexpr double kOriginLatitude = 41.5032932355418;
    static constexpr double kOriginLongitude = 2.16679606586695;
    static constexpr float kOriginAltitude = 72.14;

    // Rotation degrees to NED frame
    static constexpr float kRotationDegrees = 40.0f;

    static constexpr uint8_t kSystemId = 199;
    static constexpr uint8_t kComponentId = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;

    static constexpr uint64_t kHeartbeatIntervalMs = 1000;

    static constexpr uint64_t kHeartbeatRcvTimeoutMs = 3000;
};

extern App app;