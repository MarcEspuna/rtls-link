#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>
#include <WiFi.h>

#include "wifi_backend.hpp"
#include "wifi_params.hpp"

/**
 * @brief UDP Heartbeat Backend for RTLS-Link devices.
 *
 * Sends periodic heartbeat packets to the configured GCS IP address.
 * The heartbeat contains device information in JSON format.
 *
 * Protocol:
 * - Send heartbeat every 2 seconds to GCS IP on configured port
 * - Heartbeat payload (JSON):
 *   {"device":"rtls-link","id":"XX","role":"anchor|tag|...","ip":"X.X.X.X","mac":"XX:XX:XX:XX:XX:XX","uwb_short":"XX","mav_sysid":1,"fw":"X.X.X"}
 */
class WifiDiscovery : public WifiBackend {
public:
    WifiDiscovery(uint16_t port, const WifiParams& wifiParams);

    void Update() override;

private:
    void SendHeartbeat();
    const char* ModeToRoleString(uint8_t mode);

private:
    static constexpr uint32_t kHeartbeatIntervalMs = 2000; // 2 seconds

    WiFiUDP m_Udp;
    uint16_t m_Port;
    const WifiParams& m_WifiParams;
    uint32_t m_LastHeartbeat = 0;
};
