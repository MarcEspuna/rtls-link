/**
 * @file log_udp_backend.cpp
 * @brief UDP backend implementation for the RTLS-Link logging subsystem
 */

#include "config/features.hpp"

#ifdef USE_LOGGING_UDP

#include "log_udp_backend.hpp"
#include "protocol/rtls_binary_protocol.hpp"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>

namespace rtls {
namespace log {

// Static member initialization
bool LogUdpBackend::s_initialized = false;
char LogUdpBackend::s_targetIp[16] = "";
uint16_t LogUdpBackend::s_targetPort = LogUdpBackend::DEFAULT_LOG_PORT;

// UDP socket (static, module-level)
static WiFiUDP s_udp;

static rtls::protocol::BinaryFrameBuilder<512> s_logFrame;

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

    s_logFrame.Begin(rtls::protocol::FrameType::LogMessage);
    s_logFrame.AppendU32(timestamp_ms);
    s_logFrame.AppendU8(static_cast<uint8_t>(level));
    s_logFrame.AppendString(tag);
    s_logFrame.AppendString(message);
    s_logFrame.Finish();

    s_udp.beginPacket(targetIp, s_targetPort);
    s_udp.write(s_logFrame.Data(), s_logFrame.Size());
    s_udp.endPacket();
}

} // namespace log
} // namespace rtls

#endif // USE_LOGGING_UDP
