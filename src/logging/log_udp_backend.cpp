/**
 * @file log_udp_backend.cpp
 * @brief UDP backend implementation for the RTLS-Link logging subsystem
 */

#include "config/features.hpp"

#ifdef USE_LOGGING_UDP

#include "log_udp_backend.hpp"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstdio>
#include <cstring>

namespace rtls {
namespace log {

// Static member initialization
bool LogUdpBackend::s_initialized = false;
char LogUdpBackend::s_targetIp[16] = "";
uint16_t LogUdpBackend::s_targetPort = LogUdpBackend::DEFAULT_LOG_PORT;

// UDP socket (static, module-level)
static WiFiUDP s_udp;

// JSON output buffer
static constexpr size_t UDP_BUFFER_SIZE = 512;
static char s_udpBuffer[UDP_BUFFER_SIZE];

void LogUdpBackend::init() {
    if (s_initialized) {
        return;
    }

    s_initialized = true;
}

void LogUdpBackend::setTarget(const char* ip, uint16_t port) {
    if (ip != nullptr) {
        strncpy(s_targetIp, ip, sizeof(s_targetIp) - 1);
        s_targetIp[sizeof(s_targetIp) - 1] = '\0';
    }
    s_targetPort = port;
}

bool LogUdpBackend::isReady() {
    if (!s_initialized) {
        return false;
    }
    if (s_targetIp[0] == '\0') {
        return false;
    }
    // Check WiFi is connected
    if (WiFi.status() != WL_CONNECTED && WiFi.getMode() != WIFI_AP) {
        return false;
    }
    return true;
}

void LogUdpBackend::send(uint32_t timestamp_ms, LogLevel level,
                          const char* tag, const char* message) {
    if (!isReady()) {
        return;
    }

    // Parse target IP
    IPAddress targetIp;
    if (!targetIp.fromString(s_targetIp)) {
        return;
    }

    // Escape special JSON characters in message
    // For simplicity, we just replace quotes and backslashes
    char escapedMsg[256];
    size_t j = 0;
    for (size_t i = 0; message[i] && j < sizeof(escapedMsg) - 2; i++) {
        if (message[i] == '"' || message[i] == '\\') {
            escapedMsg[j++] = '\\';
        }
        if (message[i] == '\n') {
            escapedMsg[j++] = '\\';
            escapedMsg[j++] = 'n';
        } else if (message[i] == '\r') {
            escapedMsg[j++] = '\\';
            escapedMsg[j++] = 'r';
        } else if (message[i] == '\t') {
            escapedMsg[j++] = '\\';
            escapedMsg[j++] = 't';
        } else {
            escapedMsg[j++] = message[i];
        }
    }
    escapedMsg[j] = '\0';

    // Format as JSON
    int len = snprintf(s_udpBuffer, UDP_BUFFER_SIZE,
                       "{\"ts\":%lu,\"lvl\":\"%s\",\"tag\":\"%s\",\"msg\":\"%s\"}",
                       static_cast<unsigned long>(timestamp_ms),
                       logLevelToString(level),
                       tag,
                       escapedMsg);

    if (len <= 0 || len >= static_cast<int>(UDP_BUFFER_SIZE)) {
        return; // Buffer overflow or error
    }

    // Send UDP packet
    s_udp.beginPacket(targetIp, s_targetPort);
    s_udp.write(reinterpret_cast<const uint8_t*>(s_udpBuffer), len);
    s_udp.endPacket();
}

} // namespace log
} // namespace rtls

#endif // USE_LOGGING_UDP
