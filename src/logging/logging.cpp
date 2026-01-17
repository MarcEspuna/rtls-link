/**
 * @file logging.cpp
 * @brief Implementation of the RTLS-Link logging subsystem
 */

#include "config/features.hpp"

#ifdef USE_LOGGING

#include "logging.hpp"
#include "log_udp_backend.hpp"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <cstdio>
#include <cstring>

namespace rtls {
namespace log {

// Static member initialization
bool Logger::s_initialized = false;
bool Logger::s_serialEnabled = true;   // Serial on by default
bool Logger::s_udpEnabled = false;     // UDP off by default

// Mutex for thread-safe logging
static SemaphoreHandle_t s_logMutex = nullptr;

// Log output buffer
static constexpr size_t LOG_BUFFER_SIZE = 256;
static char s_logBuffer[LOG_BUFFER_SIZE];

void Logger::init() {
    if (s_initialized) {
        return;
    }

    // Create mutex for thread-safe logging
    if (s_logMutex == nullptr) {
        s_logMutex = xSemaphoreCreateMutex();
    }

#ifdef USE_LOGGING_UDP
    // Initialize UDP backend
    LogUdpBackend::init();
#endif

    s_initialized = true;
}

void Logger::log(LogLevel level, const char* tag, const char* format, ...) {
    va_list args;
    va_start(args, format);
    logv(level, tag, format, args);
    va_end(args);
}

void Logger::logv(LogLevel level, const char* tag, const char* format, va_list args) {
    // Ensure initialized
    if (!s_initialized) {
        init();
    }

    // Skip if nothing to output
    if (!s_serialEnabled && !s_udpEnabled) {
        return;
    }

    formatAndOutput(level, tag, format, args);
}

void Logger::formatAndOutput(LogLevel level, const char* tag,
                              const char* format, va_list args) {
    // Take mutex with timeout
    if (s_logMutex != nullptr) {
        if (xSemaphoreTake(s_logMutex, pdMS_TO_TICKS(10)) != pdTRUE) {
            return; // Could not acquire mutex, skip this log
        }
    }

    // Get timestamp
    uint32_t timestamp_ms = millis();

    // Format: [timestamp][LEVEL][tag] message
    // First, format the user message
    char messageBuffer[LOG_BUFFER_SIZE - 64];
    vsnprintf(messageBuffer, sizeof(messageBuffer), format, args);

    // Build the complete log line for Serial
    int len = snprintf(s_logBuffer, LOG_BUFFER_SIZE,
                       "[%lu][%s][%s] %s\n",
                       static_cast<unsigned long>(timestamp_ms),
                       logLevelToChar(level),
                       tag,
                       messageBuffer);

    // Clamp to buffer size
    if (len >= static_cast<int>(LOG_BUFFER_SIZE)) {
        len = LOG_BUFFER_SIZE - 1;
        s_logBuffer[len] = '\0';
    }

#ifdef USE_LOGGING_SERIAL
    // Serial output
    if (s_serialEnabled) {
        Serial.print(s_logBuffer);
    }
#endif

#ifdef USE_LOGGING_UDP
    // UDP output
    if (s_udpEnabled) {
        // Send as JSON for easier parsing by rtls-link-manager
        LogUdpBackend::send(timestamp_ms, level, tag, messageBuffer);
    }
#endif

    // Release mutex
    if (s_logMutex != nullptr) {
        xSemaphoreGive(s_logMutex);
    }
}

void Logger::setSerialEnabled(bool enabled) {
    s_serialEnabled = enabled;
}

void Logger::setUdpEnabled(bool enabled) {
    s_udpEnabled = enabled;
}

bool Logger::isSerialEnabled() {
    return s_serialEnabled;
}

bool Logger::isUdpEnabled() {
    return s_udpEnabled;
}

void Logger::setUdpTarget(const char* ip, uint16_t port) {
#ifdef USE_LOGGING_UDP
    LogUdpBackend::setTarget(ip, port);
#else
    (void)ip;
    (void)port;
#endif
}

} // namespace log
} // namespace rtls

#endif // USE_LOGGING
