#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>
#include <WiFi.h>
#include <etl/delegate.h>

#include "wifi_backend.hpp"
#include "wifi_params.hpp"

/**
 * @brief Telemetry data for device status reporting.
 */
struct DeviceTelemetry {
    bool sending_pos = false;   // True if sent position to ArduPilot in last 2s
    uint8_t anchors_seen = 0;   // Unique anchor IDs in measurement set
    bool origin_sent = false;   // True if GPS origin sent to ArduPilot
    bool rf_enabled = false;    // True if zCalcMode == RANGEFINDER
    bool rf_healthy = false;    // True if receiving non-stale rangefinder data
    // Update rate statistics (centi-Hz for 0.01 Hz precision without floats)
    uint16_t avg_rate_cHz = 0;  // Average update rate in centi-Hz (e.g., 1000 = 10.0 Hz)
    uint16_t min_rate_cHz = 0;  // Min rate in last 5s window
    uint16_t max_rate_cHz = 0;  // Max rate in last 5s window
};

using TelemetryCallback = etl::delegate<DeviceTelemetry()>;

/**
 * @brief UDP Heartbeat Backend for RTLS-Link devices.
 *
 * Sends periodic heartbeat packets to the configured GCS IP address.
 * The heartbeat contains device information in JSON format.
 *
 * Protocol:
 * - Send heartbeat every 2 seconds to GCS IP on configured port
 * - Heartbeat payload (JSON):
 *   {"device":"rtls-link","id":"XX","role":"anchor|tag|...","ip":"X.X.X.X","mac":"XX:XX:XX:XX:XX:XX","uwb_short":"XX","mav_sysid":1,"fw":"X.X.X",
 *    "sending_pos":true,"anchors_seen":4,"origin_sent":true}
 */
class WifiDiscovery : public WifiBackend {
public:
    WifiDiscovery(uint16_t port, const WifiParams& wifiParams);

    void Update() override;

    /**
     * @brief Set the callback to retrieve telemetry data for heartbeat.
     * @param callback Delegate that returns DeviceTelemetry struct
     */
    void SetTelemetryCallback(TelemetryCallback callback);

private:
    void SendHeartbeat();
    const char* ModeToRoleString(uint8_t mode);

private:
    static constexpr uint32_t kHeartbeatIntervalMs = 2000; // 2 seconds

    WiFiUDP m_Udp;
    uint16_t m_Port;
    const WifiParams& m_WifiParams;
    uint32_t m_LastHeartbeat = 0;
    TelemetryCallback m_TelemetryCallback;
};
