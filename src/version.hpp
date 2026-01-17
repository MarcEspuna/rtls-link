#pragma once

#define FIRMWARE_VERSION "1.0.0"
#define DEVICE_TYPE "rtls-link"

// Build date and time (set at compile time)
#define BUILD_DATE __DATE__
#define BUILD_TIME __TIME__

// Board type string for firmware info
#if defined(MAKERFABS_ESP32_BOARD)
    #define BOARD_TYPE "MAKERFABS_ESP32"
#elif defined(ESP32S3_UWB_BOARD)
    #define BOARD_TYPE "ESP32S3_UWB"
#else
    #define BOARD_TYPE "UNKNOWN"
#endif
