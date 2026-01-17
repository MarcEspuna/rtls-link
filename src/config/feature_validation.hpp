/**
 * @file feature_validation.hpp
 * @brief Compile-time feature dependency and conflict validation
 *
 * This file contains preprocessor checks that validate feature flag combinations
 * at compile time. Invalid configurations will produce clear error messages.
 *
 * @note This file is automatically included by features.hpp and should not
 *       be included directly.
 */

#pragma once

// =============================================================================
// MUTUAL EXCLUSION RULES
// =============================================================================

#if defined(USE_MAVLINK) && defined(USE_BEACON_PROTOCOL)
    #error "USE_MAVLINK and USE_BEACON_PROTOCOL are mutually exclusive. Choose one output protocol."
#endif

// =============================================================================
// WIFI SUBSYSTEM DEPENDENCIES
// =============================================================================

#if defined(USE_WIFI_WEBSERVER) && !defined(USE_WIFI)
    #error "USE_WIFI_WEBSERVER requires USE_WIFI to be defined"
#endif

#if defined(USE_WIFI_TCP_LOGGING) && !defined(USE_WIFI)
    #error "USE_WIFI_TCP_LOGGING requires USE_WIFI to be defined"
#endif

#if defined(USE_WIFI_UART_BRIDGE) && !defined(USE_WIFI)
    #error "USE_WIFI_UART_BRIDGE requires USE_WIFI to be defined"
#endif

#if defined(USE_WIFI_DISCOVERY) && !defined(USE_WIFI)
    #error "USE_WIFI_DISCOVERY requires USE_WIFI to be defined"
#endif

#if defined(USE_WIFI_MDNS) && !defined(USE_WIFI)
    #error "USE_WIFI_MDNS requires USE_WIFI to be defined"
#endif

// =============================================================================
// MAVLINK SUBSYSTEM DEPENDENCIES
// =============================================================================

#if defined(USE_MAVLINK_POSITION) && !defined(USE_MAVLINK)
    #error "USE_MAVLINK_POSITION requires USE_MAVLINK to be defined"
#endif

#if defined(USE_MAVLINK_HEARTBEAT) && !defined(USE_MAVLINK)
    #error "USE_MAVLINK_HEARTBEAT requires USE_MAVLINK to be defined"
#endif

#if defined(USE_MAVLINK_ORIGIN) && !defined(USE_MAVLINK)
    #error "USE_MAVLINK_ORIGIN requires USE_MAVLINK to be defined"
#endif

#if defined(USE_MAVLINK_COVARIANCE) && !defined(USE_MAVLINK)
    #error "USE_MAVLINK_COVARIANCE requires USE_MAVLINK to be defined"
#endif

#if defined(USE_MAVLINK_RANGEFINDER) && !defined(USE_MAVLINK)
    #error "USE_MAVLINK_RANGEFINDER requires USE_MAVLINK to be defined"
#endif

// =============================================================================
// BOARD-SPECIFIC FEATURE REQUIREMENTS
// =============================================================================

#if defined(USE_MAVLINK_RANGEFINDER) && !defined(ESP32S3_UWB_BOARD)
    #error "USE_MAVLINK_RANGEFINDER is only supported on ESP32S3_UWB_BOARD (Konex UWB board)"
#endif

// =============================================================================
// OTA SUBSYSTEM DEPENDENCIES
// =============================================================================

#if defined(USE_OTA_WEB) && !defined(USE_WIFI_WEBSERVER)
    #error "USE_OTA_WEB requires USE_WIFI_WEBSERVER to be defined"
#endif

#if defined(USE_OTA_WEB) && !defined(USE_OTA)
    #error "USE_OTA_WEB requires USE_OTA to be defined"
#endif

// =============================================================================
// CONSOLE SUBSYSTEM DEPENDENCIES
// =============================================================================

#if defined(USE_CONSOLE_PARAM_RW) && !defined(USE_CONSOLE)
    #error "USE_CONSOLE_PARAM_RW requires USE_CONSOLE to be defined"
#endif

#if defined(USE_CONSOLE_CONFIG_MGMT) && !defined(USE_CONSOLE)
    #error "USE_CONSOLE_CONFIG_MGMT requires USE_CONSOLE to be defined"
#endif

#if defined(USE_CONSOLE_UWB_CONTROL) && !defined(USE_CONSOLE)
    #error "USE_CONSOLE_UWB_CONTROL requires USE_CONSOLE to be defined"
#endif

#if defined(USE_CONSOLE_LED_CONTROL) && !defined(USE_CONSOLE)
    #error "USE_CONSOLE_LED_CONTROL requires USE_CONSOLE to be defined"
#endif

// =============================================================================
// TAG MODE OUTPUT REQUIREMENTS
// =============================================================================
// Tag modes need an output protocol to send position data

#if defined(USE_UWB_MODE_TWR_TAG) && !defined(USE_MAVLINK) && !defined(USE_BEACON_PROTOCOL)
    #error "USE_UWB_MODE_TWR_TAG requires either USE_MAVLINK or USE_BEACON_PROTOCOL for position output"
#endif

#if defined(USE_UWB_MODE_TDOA_TAG) && !defined(USE_MAVLINK) && !defined(USE_BEACON_PROTOCOL)
    #error "USE_UWB_MODE_TDOA_TAG requires either USE_MAVLINK or USE_BEACON_PROTOCOL for position output"
#endif

// =============================================================================
// MINIMUM VIABLE CONFIGURATION
// =============================================================================
// At least one UWB mode must be enabled for the firmware to be useful

#if !defined(USE_UWB_MODE_TWR_ANCHOR) && \
    !defined(USE_UWB_MODE_TWR_TAG) && \
    !defined(USE_UWB_MODE_TDOA_ANCHOR) && \
    !defined(USE_UWB_MODE_TDOA_TAG) && \
    !defined(USE_UWB_CALIBRATION)
    #error "At least one UWB mode must be enabled (USE_UWB_MODE_* or USE_UWB_CALIBRATION)"
#endif

// =============================================================================
// WARNINGS (non-fatal but potentially unintended configurations)
// =============================================================================

// Note: #warning is not standard C++ but is supported by GCC/Clang
// These are informational only and won't block compilation

#if defined(USE_STATUS_LED_TASK) && defined(MAKERFABS_ESP32_BOARD)
    // Makerfabs board has led_pin=-1, LED task won't function
    // This is just informational - no warning issued
#endif

// MAVLink position feature is highly recommended when using tag mode with MAVLink
#if defined(USE_MAVLINK) && (defined(USE_UWB_MODE_TWR_TAG) || defined(USE_UWB_MODE_TDOA_TAG))
    #if !defined(USE_MAVLINK_POSITION)
        // Note: Uncomment below if you want a compile warning for this case
        // #warning "Tag mode with USE_MAVLINK should typically enable USE_MAVLINK_POSITION"
    #endif
#endif
