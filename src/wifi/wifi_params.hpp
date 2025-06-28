#pragma once

#include <Arduino.h>
#include <EEPROM.h>

#include <etl/array.h>
#include <etl/string.h>

#include "utils/utils.hpp"

static constexpr uint32_t MAX_SSID_LENGTH = 32;
static constexpr uint32_t MAX_PSWD_LENGTH = 64;
static constexpr uint32_t MAX_IP_LENGTH = 15;

enum class WifiMode : uint8_t {
    AP,
    STATION,
    UNDEFINED
};

/**
 * The parameter struct can't have indirect memory structures. We will be coping pointers to memory in that case. 
 * Instead of String, etc use const char[].
*/
struct WifiParams {
    WifiMode mode;                  // True if access point mode, false if station mode
    etl::array<char,MAX_SSID_LENGTH> ssidAP;   // Used when in AP mode
    etl::array<char,MAX_PSWD_LENGTH> pswdAP;   // Used when in AP mode
    etl::array<char,MAX_SSID_LENGTH> ssidST;   // Used when in Station mode
    etl::array<char,MAX_PSWD_LENGTH> pswdST;   // Used when in Station mode
    etl::array<char,MAX_IP_LENGTH>   gcsIp;         // IP address of the device
    uint16_t dbgPort;               // Port of the device debug server
    uint16_t udpPort;               // Port of the device UDP server (Used for the UartBridge)
    uint8_t enableWebServer;        // Enable the web server (Web socket terminal and http page)
    uint8_t enableUartBridge;       // Enable the UART bridge (Bridge between serial port and UDP port)
    uint8_t enableDebugSocket;      // Enable the debug socket (Used for 3D position data visualization)
}ULS_PACKED;


