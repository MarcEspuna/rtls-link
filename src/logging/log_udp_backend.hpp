/**
 * @file log_udp_backend.hpp
 * @brief UDP backend for the RTLS-Link logging subsystem
 *
 * Sends log messages over UDP for remote debugging via rtls-link-manager.
 * Messages are formatted as JSON for easy parsing.
 */

#pragma once

#include "config/features.hpp"

#ifdef USE_LOGGING_UDP

#include "log_levels.hpp"
#include <cstdint>

namespace rtls {
namespace log {

/**
 * @brief UDP backend for log message transmission
 *
 * Sends log messages to a configured target IP/port as JSON packets.
 * Thread-safe through use of the main Logger mutex.
 */
class LogUdpBackend {
public:
    /// Default UDP port for log streaming
    static constexpr uint16_t DEFAULT_LOG_PORT = 3334;

    /**
     * @brief Initialize the UDP backend
     *
     * Sets up the UDP socket. Safe to call multiple times.
     */
    static void init();

    /**
     * @brief Set the target IP and port for log messages
     *
     * @param ip Target IP address (e.g., "192.168.1.100")
     * @param port Target UDP port
     */
    static void setTarget(const char* ip, uint16_t port);

    /**
     * @brief Send a log message over UDP
     *
     * Formats the message as JSON and sends to the configured target.
     * Format: {"ts":123456,"lvl":"INFO","tag":"module","msg":"message text"}
     *
     * @param timestamp_ms Timestamp in milliseconds
     * @param level Log level
     * @param tag Module/file tag
     * @param message Pre-formatted message text
     */
    static void send(uint32_t timestamp_ms, LogLevel level,
                     const char* tag, const char* message);

    /**
     * @brief Check if UDP backend is ready to send
     * @return true if initialized and has valid target
     */
    static bool isReady();

private:
    static bool s_initialized;
    static char s_targetIp[16];
    static uint16_t s_targetPort;
};

} // namespace log
} // namespace rtls

#endif // USE_LOGGING_UDP
