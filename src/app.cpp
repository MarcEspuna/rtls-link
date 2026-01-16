#include "config/features.hpp"  // MUST be first project include

#include "app.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <etl/delegate.h>
#include <etl/queue.h>

#include <utils/utils.hpp>

#include "bsp/board.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"
#include "uwb/uwb_params.hpp"

#ifdef USE_MAVLINK
#include "mavlink/local_position_sensor.hpp"
#include "mavlink/uart_comm.hpp"
#endif

#ifdef HAS_RANGEFINDER
#include "mavlink/rangefinder_sensor.hpp"
#endif

#ifdef USE_BEACON_PROTOCOL
#include "bcn_konex/beacon_protocool.hpp"
#endif

#ifdef USE_MAVLINK
App::App()
  : uart_comm_(App::GetArdupilotSerial()), local_position_sensor_(uart_comm_, kSystemId, kComponentId)
{}
#else
App::App()
{}
#endif

// For the future maybe we should wait for reception of heartbeats before starting sending samples
// to avoid transmition before the copter is ready
void App::Init()
{
  printf("------ Initializing the application ------\n");

#ifdef USE_BEACON_PROTOCOL
  bcn_konex::BeaconProtocol::Init();
#endif

#ifdef USE_MAVLINK
  Serial1.begin(921600, SERIAL_8N1, bsp::kBoardConfig.uwb_data_uart.rx_pin, bsp::kBoardConfig.uwb_data_uart.tx_pin);

  local_position_sensor_.set_heartbeat_callback([this](uint8_t system_id, uint8_t component_id) {
    last_heartbeat_received_timestamp_ms_ = millis();
  });
#endif

  // Initialize timestamp
  device_unhealthy_timestamp_ms_ = millis();

  // Initialize mutex for rate statistics thread safety
  rate_stats_mutex_ = xSemaphoreCreateMutex();

#if defined(USE_STATUS_LED_TASK) && defined(BOARD_HAS_LED)
  printf("------ Status task enabled ------\n");
  pinMode(bsp::kBoardConfig.led_pin, OUTPUT);
  digitalWrite(bsp::kBoardConfig.led_pin, LOW);
#endif

#ifdef HAS_RANGEFINDER
  // Initialize rangefinder sensor (ESP32S3 only)
  if (bsp::kBoardConfig.rangefinder_uart.rx_pin < 64) {
    // Use Serial0 for UART0 (Serial is USB CDC on ESP32S3)
    rangefinder_sensor_ = new RangefinderSensor(Serial0);
    rangefinder_sensor_->init(
        115200,
        bsp::kBoardConfig.rangefinder_uart.rx_pin,
        bsp::kBoardConfig.rangefinder_uart.tx_pin
    );

    rangefinder_sensor_->set_distance_callback(
        [this](mavlink_distance_sensor_t distance_msg, uint64_t timestamp_ms,
               uint8_t src_sysid, uint8_t src_compid) {
            last_rangefinder_distance_cm_ = distance_msg.current_distance;
            last_rangefinder_timestamp_ms_ = timestamp_ms;
            rangefinder_ever_received_ = true;

            // Forward to ArduPilot if enabled
            const auto& params = Front::uwbLittleFSFront.GetParams();
            if (params.rfForwardEnable) {
                bool success = local_position_sensor_.send_distance_sensor(
                    distance_msg,
                    params.rfForwardSensorId,
                    params.rfForwardOrientation,
                    src_sysid,
                    src_compid,
                    params.rfForwardPreserveSrcIds != 0);

                // Track and log failures
                if (!success) {
                    rf_forward_fail_count_++;
                    if (rf_forward_fail_count_ == kRfForwardFailLogThreshold) {
                        printf("[Rangefinder] Forward failed %lu consecutive times\n",
                               static_cast<unsigned long>(rf_forward_fail_count_));
                    }
                } else {
                    rf_forward_fail_count_ = 0;
                }
            }

            // Time-limited logging (once per second max)
            if (timestamp_ms - last_rangefinder_log_ms_ >= kRangefinderLogIntervalMs) {
                printf("[Rangefinder] Distance: %u cm (%.2f m) [fwd=%d]\n",
                       distance_msg.current_distance,
                       static_cast<float>(distance_msg.current_distance) / 100.0f,
                       params.rfForwardEnable ? 1 : 0);
                last_rangefinder_log_ms_ = timestamp_ms;
            }
        }
    );

    printf("[App] Rangefinder sensor initialized\n");
  }
#endif // HAS_RANGEFINDER

  printf("------ Application initialized ------\n");
}


void App::Update()
{
#ifdef USE_MAVLINK
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

#ifdef HAS_RANGEFINDER
  // Process rangefinder MAVLink messages
  if (rangefinder_sensor_) {
    rangefinder_sensor_->process_received_bytes();
  }
#endif

  // For now we are only receiving heartbeat messages
  // Read the buffer
  while (Serial1.available() && buffer_index < sizeof(buffer)) {
    uint8_t c = Serial1.read();
    buffer[buffer_index++] = c;
  }

  // Process the buffer
  local_position_sensor_.process_received_bytes(buffer, buffer_index);

#endif // USE_MAVLINK

#ifdef USE_BEACON_PROTOCOL
  AddAnchorEcho();
#endif
}

#ifdef USE_BEACON_PROTOCOL
/**
 *  @brief Run the application task for beacon protocol
 */
void App::AddAnchorEcho()
{
  bcn_konex::BeaconProtocol::SendAddAnchorEcho();
}
#endif

#if defined(USE_STATUS_LED_TASK) && defined(BOARD_HAS_LED)
void App::StatusLedTask()
{
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
}
#endif

#ifdef USE_BEACON_PROTOCOL
// --- Only used for TWR (legacy)---
void App::AnchorsToEcho(const etl::array<double, 12>& anchor_locations)
{
  bcn_konex::BeaconProtocol::SetAnchorsToEcho(anchor_locations);
}
#endif

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

#ifdef HAS_RANGEFINDER
float App::GetRangefinderZ() {
  ZCalcMode mode = Front::uwbLittleFSFront.GetParams().zCalcMode;

  if (mode == ZCalcMode::RANGEFINDER) {
    // Check if we have ever received data and it's not stale
    if (app.rangefinder_ever_received_) {
      uint64_t age_ms = millis() - app.last_rangefinder_timestamp_ms_;
      if (age_ms <= kRangefinderStaleTimeoutMs) {
        // Convert cm to meters and negate for NED frame
        // Rangefinder measures positive distance to ground, but in NED
        // Z is positive downward, so altitude above ground is negative
        return -static_cast<float>(app.last_rangefinder_distance_cm_) / 100.0f;
      }
    }
    // Never received or stale - return NAN to indicate unavailable
    return NAN;
  }

  // Return NAN to indicate "use original Z from TDoA"
  return NAN;
}

bool App::IsRangefinderEnabled() {
  return Front::uwbLittleFSFront.GetParams().zCalcMode == ZCalcMode::RANGEFINDER;
}

bool App::IsRangefinderHealthy() {
  if (!app.rangefinder_ever_received_) {
    return false;
  }
  return (millis() - app.last_rangefinder_timestamp_ms_) <= kRangefinderStaleTimeoutMs;
}
#endif // HAS_RANGEFINDER

#ifdef USE_MAVLINK
bool App::IsSendingPositions() {
  // Guard for initial boot (last_sample_timestamp_ms_ starts at 0)
  if (app.last_sample_timestamp_ms_ == 0) {
    return false;
  }
  // Use 2s window to match heartbeat interval (avoids flapping)
  return (millis() - app.last_sample_timestamp_ms_) < 2000;
}

bool App::IsOriginSent() {
  return app.is_origin_position_sent_;
}
#endif // USE_MAVLINK

#ifdef USE_RATE_STATISTICS
// --- Update rate tracking ---
void App::RecordSampleTimestamp() {
  if (rate_stats_mutex_ == nullptr) return;

  if (xSemaphoreTake(rate_stats_mutex_, pdMS_TO_TICKS(1)) == pdTRUE) {
    uint64_t now = millis();
    sample_timestamps_[sample_timestamps_index_] = now;
    sample_timestamps_index_ = (sample_timestamps_index_ + 1) % kRateWindowSize;
    if (sample_timestamps_count_ < kRateWindowSize) {
      sample_timestamps_count_++;
    }
    xSemaphoreGive(rate_stats_mutex_);
  }
}

void App::CalculateRateStatistics() {
  if (rate_stats_mutex_ == nullptr) return;

  uint64_t now = millis();

  // Only recalculate every 500ms to reduce overhead
  if (now - last_rate_calc_ms_ < 500) {
    return;
  }

  // Acquire mutex for thread-safe access to sample_timestamps_
  if (xSemaphoreTake(rate_stats_mutex_, pdMS_TO_TICKS(10)) != pdTRUE) {
    return;  // Could not acquire mutex, skip this calculation (don't update throttle)
  }

  // Update throttle timestamp only after successful mutex acquisition
  last_rate_calc_ms_ = now;

  // Need at least 2 samples to calculate rate
  if (sample_timestamps_count_ < 2) {
    cached_avg_rate_cHz_ = 0;
    cached_min_rate_cHz_ = 0;
    cached_max_rate_cHz_ = 0;
    xSemaphoreGive(rate_stats_mutex_);
    return;
  }

  // Collect timestamps within the 5-second window
  uint64_t window_start = (now > kRateWindowDurationMs) ? (now - kRateWindowDurationMs) : 0;

  // Count samples in window and find time range
  uint32_t samples_in_window = 0;
  uint64_t oldest_in_window = now;
  uint64_t newest_in_window = 0;

  // Iterate through all stored timestamps
  for (size_t i = 0; i < sample_timestamps_count_; i++) {
    uint64_t ts = sample_timestamps_[i];
    if (ts >= window_start && ts <= now) {
      samples_in_window++;
      if (ts < oldest_in_window) oldest_in_window = ts;
      if (ts > newest_in_window) newest_in_window = ts;
    }
  }

  // Use local variables to calculate all rates, then update cached values atomically
  uint16_t new_avg_rate = 0;
  uint16_t new_min_rate = 0;
  uint16_t new_max_rate = 0;

  // Calculate average rate with overflow protection
  if (samples_in_window >= 2 && newest_in_window > oldest_in_window) {
    uint64_t duration_ms = newest_in_window - oldest_in_window;
    // rate = (samples - 1) / duration_seconds
    // rate_cHz = (samples - 1) * 100000 / duration_ms
    uint64_t rate = ((samples_in_window - 1) * 100000ULL) / duration_ms;
    new_avg_rate = (rate > kMaxRateCHz) ? kMaxRateCHz : static_cast<uint16_t>(rate);
  }

  // For min/max, collect sorted timestamps and compute intervals
  uint64_t sorted_ts[kRateWindowSize];
  size_t sorted_count = 0;
  for (size_t i = 0; i < sample_timestamps_count_; i++) {
    uint64_t ts = sample_timestamps_[i];
    if (ts >= window_start && ts <= now) {
      sorted_ts[sorted_count++] = ts;
    }
  }

  // Release mutex after copying data - sorting and rate calculation can proceed without it
  xSemaphoreGive(rate_stats_mutex_);

  // Simple insertion sort (small array)
  for (size_t i = 1; i < sorted_count; i++) {
    uint64_t key = sorted_ts[i];
    int j = i - 1;
    while (j >= 0 && sorted_ts[j] > key) {
      sorted_ts[j + 1] = sorted_ts[j];
      j--;
    }
    sorted_ts[j + 1] = key;
  }

  // Compute min/max intervals
  uint64_t min_interval_ms = UINT64_MAX;
  uint64_t max_interval_ms = 0;
  if (sorted_count >= 2) {
    for (size_t i = 1; i < sorted_count; i++) {
      uint64_t interval = sorted_ts[i] - sorted_ts[i - 1];
      if (interval > 0) {
        if (interval < min_interval_ms) min_interval_ms = interval;
        if (interval > max_interval_ms) max_interval_ms = interval;
      }
    }

    // Convert intervals to rates (rate = 1/interval) with overflow protection
    // min_interval -> max_rate, max_interval -> min_rate
    if (min_interval_ms > 0 && min_interval_ms < UINT64_MAX) {
      uint64_t rate = 100000ULL / min_interval_ms;
      new_max_rate = (rate > kMaxRateCHz) ? kMaxRateCHz : static_cast<uint16_t>(rate);
    }
    if (max_interval_ms > 0) {
      uint64_t rate = 100000ULL / max_interval_ms;
      new_min_rate = (rate > kMaxRateCHz) ? kMaxRateCHz : static_cast<uint16_t>(rate);
    }
  }

  // Update all cached values together to ensure consistency for readers
  cached_avg_rate_cHz_ = new_avg_rate;
  cached_min_rate_cHz_ = new_min_rate;
  cached_max_rate_cHz_ = new_max_rate;
}

uint16_t App::GetAvgUpdateRateCHz() {
  app.CalculateRateStatistics();
  return app.cached_avg_rate_cHz_;
}

uint16_t App::GetMinRateCHz() {
  app.CalculateRateStatistics();
  return app.cached_min_rate_cHz_;
}

uint16_t App::GetMaxRateCHz() {
  app.CalculateRateStatistics();
  return app.cached_max_rate_cHz_;
}
#endif // USE_RATE_STATISTICS

#ifdef USE_MAVLINK
/**
 * @brief Rotates a 3x3 position covariance matrix by yaw angle
 *
 * Applies the transformation P' = R * P * R^T where R is a 2D rotation matrix.
 * The Z variance is unchanged, but cross-correlations with Z are rotated.
 *
 * @param cov Input covariance [var_x, cov_xy, cov_xz, var_y, cov_yz, var_z]
 * @param yaw_rad Yaw rotation angle in radians
 * @return Rotated covariance array
 */
static std::array<float, 6> rotateCovarianceByYaw(
    const std::array<float, 6>& cov, float yaw_rad)
{
  if (yaw_rad == 0.0f) {
    return cov;
  }

  float c = cosf(yaw_rad);
  float s = sinf(yaw_rad);

  // Input: [var_x, cov_xy, cov_xz, var_y, cov_yz, var_z]
  float var_x = cov[0];
  float cov_xy = cov[1];
  float cov_xz = cov[2];
  float var_y = cov[3];
  float cov_yz = cov[4];
  float var_z = cov[5];

  // 2D rotation for XY plane, Z unchanged
  // R = [c -s 0; s c 0; 0 0 1]
  // P_rot = R * P * R^T

  std::array<float, 6> rotated;
  rotated[0] = c*c*var_x + 2*c*s*cov_xy + s*s*var_y;           // var_x'
  rotated[1] = c*c*cov_xy - s*s*cov_xy + c*s*(var_y - var_x);  // cov_xy'
  rotated[2] = c*cov_xz + s*cov_yz;                             // cov_xz'
  rotated[3] = s*s*var_x - 2*c*s*cov_xy + c*c*var_y;           // var_y'
  rotated[4] = -s*cov_xz + c*cov_yz;                            // cov_yz'
  rotated[5] = var_z;                                           // var_z' (unchanged)

  return rotated;
}

/**
 * @brief Maps 6-element position covariance to MAVLink 21-element array
 *
 * MAVLink uses row-major upper triangular packing for 6x6 covariance.
 * We only have position (3x3), so orientation terms are set to NaN.
 */
static std::array<float, VISION_POSITION_COVARIANCE_SIZE> mapToMAVLinkCovariance(
    const std::array<float, 6>& posCovariance)
{
  std::array<float, VISION_POSITION_COVARIANCE_SIZE> mavCov;

  // Initialize all to NaN (indicates unknown/not-provided)
  for (size_t i = 0; i < mavCov.size(); ++i) {
    mavCov[i] = NAN;
  }

  // Position variances and covariances (3x3 block)
  // Row 0: [var_x, cov_xy, cov_xz, cov_x_roll, cov_x_pitch, cov_x_yaw]
  mavCov[0] = posCovariance[0];   // var(x)
  mavCov[1] = posCovariance[1];   // cov(x,y)
  mavCov[2] = posCovariance[2];   // cov(x,z)
  // indices 3,4,5 = position-orientation cross-covariances -> 0 (no correlation)
  mavCov[3] = 0.0f;
  mavCov[4] = 0.0f;
  mavCov[5] = 0.0f;

  // Row 1: [var_y, cov_yz, cov_y_roll, cov_y_pitch, cov_y_yaw]
  mavCov[6] = posCovariance[3];   // var(y)
  mavCov[7] = posCovariance[4];   // cov(y,z)
  // indices 8,9,10 = position-orientation cross-covariances -> 0
  mavCov[8] = 0.0f;
  mavCov[9] = 0.0f;
  mavCov[10] = 0.0f;

  // Row 2: [var_z, cov_z_roll, cov_z_pitch, cov_z_yaw]
  mavCov[11] = posCovariance[5];  // var(z)
  // indices 12,13,14 = position-orientation cross-covariances -> 0
  mavCov[12] = 0.0f;
  mavCov[13] = 0.0f;
  mavCov[14] = 0.0f;

  // Orientation variances: indices 15,18,20 -> NaN (UWB provides no orientation)
  // These are already set to NaN from initialization

  return mavCov;
}
#endif // USE_MAVLINK (covariance helper functions)

#ifdef HAS_POSITION_OUTPUT
void App::SendSample(float x_m, float y_m, float z_m,
                     std::optional<std::array<float, 6>> positionCovariance)
{
#ifdef USE_BEACON_PROTOCOL
  // Beacon protocol doesn't use covariance
  bcn_konex::BeaconProtocol::SendSample(x_m, y_m, z_m, 0);
#endif

#ifdef USE_MAVLINK
  // Determine Z coordinate based on zCalcMode parameter
  ZCalcMode mode = Front::uwbLittleFSFront.GetParams().zCalcMode;
  float final_z;
#ifdef HAS_RANGEFINDER
  if (mode == ZCalcMode::RANGEFINDER) {
    // Rangefinder mode: use rangefinder Z directly (NAN if unavailable)
    final_z = GetRangefinderZ();
  } else
#endif
  {
    // NONE or UWB mode: use TDoA Z
    final_z = z_m;
  }

  // Apply coordinate system rotation to correct for beacon system orientation
  Vector3f rotated_vector = correct_for_orient_yaw(x_m, y_m, final_z);

  // Check if we have received a heartbeat recently
  if (millis() - app.last_heartbeat_received_timestamp_ms_ < kHeartbeatRcvTimeoutMs) {
    // Defense in depth: also check enableCovMatrix parameter here
    bool sendCovMatrix = positionCovariance.has_value() &&
                         Front::uwbLittleFSFront.GetParams().enableCovMatrix != 0;

    if (sendCovMatrix) {
      // Rotate covariance to match rotated position
      float rotationDegrees = Front::uwbLittleFSFront.GetParams().rotationDegrees;
      float yaw_rad = rotationDegrees * M_PI / 180.0f;

      std::array<float, 6> rotatedCov = rotateCovarianceByYaw(*positionCovariance, yaw_rad);
      std::array<float, VISION_POSITION_COVARIANCE_SIZE> mavCov = mapToMAVLinkCovariance(rotatedCov);

      app.local_position_sensor_.send_vision_position_estimate(
          rotated_vector.x, rotated_vector.y, rotated_vector.z,
          0, 0, 0,  // No orientation from UWB
          &mavCov);
    } else {
      // No covariance - send with nullptr (will use NaN)
      app.local_position_sensor_.send_vision_position_estimate(
          rotated_vector.x, rotated_vector.y, rotated_vector.z,
          0, 0, 0);
    }
    app.last_sample_timestamp_ms_ = millis();
#ifdef USE_RATE_STATISTICS
    app.RecordSampleTimestamp();  // Track for rate statistics
#endif
  }
#endif // USE_MAVLINK
}
#endif // HAS_POSITION_OUTPUT

#ifdef USE_BEACON_PROTOCOL
void App::SendRangeSample(uint8_t id, float range)
{
  bcn_konex::BeaconProtocol::SendRangeSample(id, range);
}
#endif

void App::Start()
{
  // For now the start will do nothing
}

#ifdef USE_MAVLINK
HardwareSerial& App::GetArdupilotSerial()
{
    return Serial1;
}
#endif