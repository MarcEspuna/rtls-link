#pragma once

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>
#include <WiFi.h>

#include "wifi_backend.hpp"

/**
 * @brief UDP Discovery Backend for RTLS-Link devices.
 *
 * Listens on a configurable UDP port for "RTLS_DISCOVER" broadcast messages
 * and responds with a JSON payload containing device information.
 *
 * Protocol:
 * - Listen on UDP port (default 3333)
 * - On receiving "RTLS_DISCOVER" string, respond with JSON:
 *   {"device":"rtls-link","id":"XX","role":"anchor|tag|...","ip":"X.X.X.X","mac":"XX:XX:XX:XX:XX:XX","uwb_short":"XX","fw":"X.X.X"}
 *
 * Rate limiting is per-sender to allow multiple clients to discover simultaneously.
 */
class WifiDiscovery : public WifiBackend {
public:
    WifiDiscovery(uint16_t port);

    void Update() override;

private:
    void SendDiscoveryResponse(IPAddress& remoteIp, uint16_t remotePort);
    const char* ModeToRoleString(uint8_t mode);

private:
    static constexpr uint32_t kMaxPacketSize = 64;
    static constexpr uint32_t kRateLimitMs = 200;
    static constexpr char kDiscoverCommand[] = "RTLS_DISCOVER";

    WiFiUDP m_Udp;
    uint16_t m_Port;
    uint8_t m_IncomingPacket[kMaxPacketSize];

    // Per-sender rate limiting
    IPAddress m_LastSenderIp;
    uint32_t m_LastResponseTime = 0;
};
