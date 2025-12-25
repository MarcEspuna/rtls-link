#include "app.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <etl/delegate.h>

#include <etl/queue.h>

#include <utils/utils.hpp>

#include "bsp/board.hpp"
#include "mavlink/local_position_sensor.hpp"
#include "bcn_konex/beacon_protocool.hpp"
#include "mavlink/uart_comm.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

#define BEACON_PROTOCOL_ENABLED 0
#define MAVLINK_PROTOCOL_ENABLED 1

#if BEACON_PROTOCOL_ENABLED && MAVLINK_PROTOCOL_ENABLED
#error "Beacon protocol and MAVLink cannot be enabled at the same time"
#endif

App::App()
  : uart_comm_(App::GetArdupilotSerial()), local_position_sensor_(uart_comm_, kSystemId, kComponentId)
{}

// For the future maybe we should wait for reception of heartbeats before starting sending samples
// to avoid transmition before the copter is ready
void App::Init()
{
  printf("------ Initializing the application ------\n");
  
  bcn_konex::BeaconProtocol::Init();

  Serial1.begin(921600, SERIAL_8N1, bsp::kBoardConfig.uwb_data_uart.rx_pin, bsp::kBoardConfig.uwb_data_uart.tx_pin);

  local_position_sensor_.set_heartbeat_callback([this](uint8_t system_id, uint8_t component_id) {
    // printf("Heartbeat received from system %d, component %d\n", system_id, component_id);
    last_heartbeat_received_timestamp_ms_ = millis();
    // Future implementation:
    // If all of a sudden we stop receiving heartbeats maybe we should retry to send the origin position
    // maybe the copter has been reset
  });

  // Initialize timestamp
  device_unhealthy_timestamp_ms_ = millis();

#if STATUS_TASK == ENABLE
  printf("------ Status task enabled ------\n");
  pinMode(bsp::kBoardConfig.led_pin, OUTPUT);       // Set the LED pin as output
  digitalWrite(bsp::kBoardConfig.led_pin, LOW);     // Turn off the LED
#endif

  printf("------ Application initialized ------\n");
}


void App::Update()
{
#if MAVLINK_PROTOCOL_ENABLED
  // App health stats (static to persist across calls)
  static uint32_t app_stats_healthy_cycles = 0;
  static uint32_t app_stats_unhealthy_cycles = 0;
  static uint64_t app_stats_last_log_ms = 0;
  static constexpr uint64_t APP_STATS_LOG_INTERVAL_MS = 1000;

  uint8_t buffer[1024];
  uint32_t buffer_index = 0;
  uint64_t now_ms = millis();
  // ********** SENDING **********

  // Listen and send heartbeat
  if (now_ms - last_heartbeat_timestamp_ms_ > kHeartbeatIntervalMs) {
    local_position_sensor_.send_heartbeat();
    last_heartbeat_timestamp_ms_ = now_ms;
  }

  // Check if device is healthy
  if (now_ms - last_sample_timestamp_ms_ > kDeviceHealtyMinDurationMs) {
    // Device is unhealthy
    device_unhealthy_timestamp_ms_ = now_ms;
    app_stats_unhealthy_cycles++;
  } else {
    // Device is healthy
    app_stats_healthy_cycles++;
    uint64_t time_since_unhealthy = now_ms - device_unhealthy_timestamp_ms_;
    uint64_t time_since_rcv_heartbeat = now_ms - last_heartbeat_received_timestamp_ms_; 
    if (time_since_unhealthy > kSendOriginPositionAfterMs 
        && !is_origin_position_sent_ 
        && time_since_rcv_heartbeat < kHeartbeatRcvTimeoutMs) {
      uint8_t target_system_id = Front::uwbLittleFSFront.GetParams().mavlinkTargetSystemId;
      printf("Sending origin position to system %d\n", target_system_id);
      local_position_sensor_.send_set_gps_global_origin(
          Front::uwbLittleFSFront.GetParams().originLat, 
          Front::uwbLittleFSFront.GetParams().originLon, 
          Front::uwbLittleFSFront.GetParams().originAlt, 
          target_system_id, micros());
      
      time_since_unhealthy = now_ms;
      is_origin_position_sent_ = true;
    }
  }

  // Periodic App health log (every 1 second)
  if (now_ms - app_stats_last_log_ms >= APP_STATS_LOG_INTERVAL_MS) {
    uint64_t time_since_unhealthy = now_ms - device_unhealthy_timestamp_ms_;
    uint64_t time_since_heartbeat = now_ms - last_heartbeat_received_timestamp_ms_;
    printf("[App Stats] H:%u U:%u | OriginSent:%d UnhealthyAge:%llums HB_Age:%llums\n",
           app_stats_healthy_cycles, app_stats_unhealthy_cycles,
           is_origin_position_sent_ ? 1 : 0,
           time_since_unhealthy, time_since_heartbeat);
    app_stats_healthy_cycles = 0;
    app_stats_unhealthy_cycles = 0;
    app_stats_last_log_ms = now_ms;
  }

  // ********** RECEIVING **********
  // For now we are only receiving heartbeat messages
  // Read the buffer
  while (Serial1.available() && buffer_index < sizeof(buffer)) {
    uint8_t c = Serial1.read();
    buffer[buffer_index++] = c;
  }

  // Process the buffer
  local_position_sensor_.process_received_bytes(buffer, buffer_index);

#endif
#if BEACON_PROTOCOL_ENABLED
  AddAnchorEcho();
#endif
}

/**
 *  @brief Run the application task
 * 
 *  @todo In the future we should take into account timestamps so we don't sand too old data.
 * 
 */
void App::AddAnchorEcho()
{
  bcn_konex::BeaconProtocol::SendAddAnchorEcho();
}

void App::StatusLedTask()
{
#if STATUS_TASK == ENABLE
  static uint32_t i = 0;

  // It will blink to the number of connected anchors
  if (Front::uwbLittleFSFront.GetConnectedDevices() > i) {
    digitalWrite(bsp::kBoardConfig.led_pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));  // Use FreeRTOS delay instead of Arduino delay()
    digitalWrite(bsp::kBoardConfig.led_pin, LOW);
  }

  // Reset i every 10 cycles
  i++;
  i = i % 10;
#endif
}


// --- Only used for TWR (legacy)---
void App::AnchorsToEcho(const etl::array<double, 12>& anchor_locations)
{
  #if BEACON_PROTOCOL_ENABLED
  bcn_konex::BeaconProtocol::SetAnchorsToEcho(anchor_locations);
  #endif
}

// Helper function to correct for yaw orientation
// Note: This function works correctly with negative values of kRotationDegrees
// because the standard 2D rotation matrix is used:
// [cos(θ) -sin(θ)]
// [sin(θ)  cos(θ)]
// When θ is negative, the rotation is clockwise instead of counterclockwise,
// which is mathematically valid and works as expected.
Vector3f App::correct_for_orient_yaw(float x, float y, float z) {
  Vector3f result = {x, y, z};
  
  // Exit immediately if no correction needed (optimization)
  float rotationDegrees = Front::uwbLittleFSFront.GetParams().rotationDegrees;
  if (rotationDegrees == 0.0f) {
    return result;
  }

  // Calculate rotation constants
  float orient_yaw_rad = rotationDegrees * M_PI / 180.0f; // Convert degrees to radians
  float orient_cos_yaw = cosf(orient_yaw_rad);
  float orient_sin_yaw = sinf(orient_yaw_rad);

  // Rotate x,y by -orient_yaw (apply 2D rotation matrix)
  result.x = x * orient_cos_yaw - y * orient_sin_yaw;
  result.y = x * orient_sin_yaw + y * orient_cos_yaw;
  // z stays the same
  
  return result;
}

void App::SendSample(float x_m, float y_m, float z_m, uint16_t error)
{
  #if BEACON_PROTOCOL_ENABLED
  bcn_konex::BeaconProtocol::SendSample(x_m, y_m, z_m, error);
  #endif

  #if MAVLINK_PROTOCOL_ENABLED
  // Apply coordinate system rotation to correct for beacon system orientation
  Vector3f rotated_vector = correct_for_orient_yaw(x_m, y_m, z_m);

  // Check if we have received a heartbeat recently
  if (millis() - app.last_heartbeat_received_timestamp_ms_ < kHeartbeatRcvTimeoutMs) {
    // Send the rotated coordinates
    app.local_position_sensor_.send_vision_position_estimate(rotated_vector.x, rotated_vector.y, rotated_vector.z, 0, 0, 0);
    app.last_sample_timestamp_ms_ = millis();
  }
  #endif
}

void App::SendRangeSample(uint8_t id, float range)
{
  #if BEACON_PROTOCOL_ENABLED
  bcn_konex::BeaconProtocol::SendRangeSample(id, range);
  #endif
}
// --- End of legacy TWR ---

void App::Start()
{
  // For now the start will do nothing
}

HardwareSerial& App::GetArdupilotSerial()
{
    return Serial1;
}