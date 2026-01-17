/**
 * @file log_levels.hpp
 * @brief Log level definitions for the RTLS-Link logging subsystem
 *
 * This file defines the available log levels used throughout the firmware.
 * Levels are ordered from most severe (ERROR) to most verbose (VERBOSE).
 */

#pragma once

#include <cstdint>

namespace rtls {
namespace log {

/**
 * @brief Log level enumeration
 *
 * Log levels in order of increasing verbosity:
 * - NONE:    Logging disabled
 * - ERROR:   Error conditions that may cause malfunction
 * - WARN:    Warning conditions that should be investigated
 * - INFO:    Informational messages about normal operation
 * - DEBUG:   Detailed debugging information
 * - VERBOSE: Very detailed trace information
 */
enum class LogLevel : uint8_t {
    NONE    = 0,  ///< Logging disabled
    ERROR   = 1,  ///< Error conditions
    WARN    = 2,  ///< Warning conditions
    INFO    = 3,  ///< Informational messages
    DEBUG   = 4,  ///< Debug information
    VERBOSE = 5   ///< Verbose trace information
};

/**
 * @brief Convert log level to string representation
 * @param level The log level to convert
 * @return Short string representation (E/W/I/D/V)
 */
constexpr const char* logLevelToChar(LogLevel level) {
    switch (level) {
        case LogLevel::ERROR:   return "E";
        case LogLevel::WARN:    return "W";
        case LogLevel::INFO:    return "I";
        case LogLevel::DEBUG:   return "D";
        case LogLevel::VERBOSE: return "V";
        default:                return "?";
    }
}

/**
 * @brief Convert log level to full string representation
 * @param level The log level to convert
 * @return Full string representation (ERROR/WARN/INFO/DEBUG/VERBOSE)
 */
constexpr const char* logLevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::ERROR:   return "ERROR";
        case LogLevel::WARN:    return "WARN";
        case LogLevel::INFO:    return "INFO";
        case LogLevel::DEBUG:   return "DEBUG";
        case LogLevel::VERBOSE: return "VERBOSE";
        default:                return "UNKNOWN";
    }
}

} // namespace log
} // namespace rtls
