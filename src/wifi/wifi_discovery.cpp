#include "config/features.hpp"

#ifdef USE_WIFI_DISCOVERY

#include "wifi_discovery.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"
#include "version.hpp"
#include "logging/logging.hpp"

#include <esp_mac.h>

WifiDiscovery::WifiDiscovery(uint16_t port, const WifiParams& wifiParams)
    : m_Port(port)
    , m_WifiParams(wifiParams)
{
    LOG_INFO("WifiDiscovery: Heartbeat mode on port %d", m_Port);
}

void WifiDiscovery::SetTelemetryCallback(TelemetryCallback callback) {
    m_TelemetryCallback = callback;
}

void WifiDiscovery::Update() {
    // Send heartbeat at interval
    uint32_t now = millis();
    if (now - m_LastHeartbeat < kHeartbeatIntervalMs) {
        return;
    }
    m_LastHeartbeat = now;

    SendHeartbeat();
}

void WifiDiscovery::SendHeartbeat() {
    // Get GCS IP from wifi params
    IPAddress gcsIp;
    if (!gcsIp.fromString(m_WifiParams.gcsIp.data())) {
        return; // No valid GCS IP configured
    }

    char response[512];  // Increased buffer for telemetry + logging fields

    // Get IP and MAC based on WiFi mode
    bool isAP = (WiFi.getMode() == WIFI_AP);
    IPAddress deviceIp = isAP ? WiFi.softAPIP() : WiFi.localIP();

    // Get MAC without heap allocation (use correct interface type)
    uint8_t mac[6];
    esp_read_mac(mac, isAP ? ESP_MAC_WIFI_SOFTAP : ESP_MAC_WIFI_STA);

    // Get UWB params
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();

    // devShortAddr is a 2-byte array; send printable digits when available
    char shortAddrStr[3] = {};
    size_t shortAddrLen = 0;
    if (uwbParams.devShortAddr[0] != '\0') {
        shortAddrStr[shortAddrLen++] = uwbParams.devShortAddr[0];
    }
    if (uwbParams.devShortAddr[1] != '\0') {
        shortAddrStr[shortAddrLen++] = uwbParams.devShortAddr[1];
    }
    if (shortAddrLen == 0) {
        shortAddrStr[shortAddrLen++] = '0';
    }
    shortAddrStr[shortAddrLen] = '\0';

    // Get telemetry data via callback (or use defaults)
    DeviceTelemetry telemetry = {};
    if (m_TelemetryCallback.is_valid()) {
        telemetry = m_TelemetryCallback();
    }

    // Get log level from compiled setting
#ifdef USE_LOGGING
    uint8_t logLevel = rtls::log::Logger::getCompiledLogLevel();
#else
    uint8_t logLevel = 0; // Disabled
#endif

    int written = snprintf(response, sizeof(response),
        "{\"device\":\"%s\",\"id\":\"%s\",\"role\":\"%s\","
        "\"ip\":\"%d.%d.%d.%d\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
        "\"uwb_short\":\"%s\",\"mav_sysid\":%u,\"fw\":\"%s\","
        "\"sending_pos\":%s,\"anchors_seen\":%u,\"origin_sent\":%s,"
        "\"rf_enabled\":%s,\"rf_healthy\":%s,"
        "\"avg_rate_cHz\":%u,\"min_rate_cHz\":%u,\"max_rate_cHz\":%u,"
        "\"log_level\":%u,\"log_udp_port\":%u,"
        "\"log_serial_enabled\":%s,\"log_udp_enabled\":%s}",
        DEVICE_TYPE,
        shortAddrStr,
        ModeToRoleString(static_cast<uint8_t>(uwbParams.mode)),
        deviceIp[0], deviceIp[1], deviceIp[2], deviceIp[3],
        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
        shortAddrStr,
        uwbParams.mavlinkTargetSystemId,
        FIRMWARE_VERSION,
        telemetry.sending_pos ? "true" : "false",
        static_cast<unsigned int>(telemetry.anchors_seen),
        telemetry.origin_sent ? "true" : "false",
        telemetry.rf_enabled ? "true" : "false",
        telemetry.rf_healthy ? "true" : "false",
        static_cast<unsigned int>(telemetry.avg_rate_cHz),
        static_cast<unsigned int>(telemetry.min_rate_cHz),
        static_cast<unsigned int>(telemetry.max_rate_cHz),
        logLevel,
        m_WifiParams.logUdpPort,
        m_WifiParams.logSerialEnabled ? "true" : "false",
        m_WifiParams.logUdpEnabled ? "true" : "false");

    // Log truncation warning once
    static bool truncation_warned = false;
    if (written >= static_cast<int>(sizeof(response)) && !truncation_warned) {
        LOG_WARN("WifiDiscovery: heartbeat JSON truncated!");
        truncation_warned = true;
    }

    m_Udp.beginPacket(gcsIp, m_Port);
    m_Udp.write((uint8_t*)response, strlen(response));
    m_Udp.endPacket();
}

const char* WifiDiscovery::ModeToRoleString(uint8_t mode) {
    // Maps UWBMode enum values to human-readable strings
    // UWBMode: ANCHOR_MODE_TWR=0, TAG_MODE_TWR=1, CALIBRATION_MODE=2, ANCHOR_TDOA=3, TAG_TDOA=4
    switch (mode) {
        case 0: return "anchor";
        case 1: return "tag";
        case 2: return "calibration";
        case 3: return "anchor_tdoa";
        case 4: return "tag_tdoa";
        default: return "unknown";
    }
}

#endif // USE_WIFI_DISCOVERY
