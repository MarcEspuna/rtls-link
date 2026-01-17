#pragma once

#include "config/features.hpp"

#ifdef USE_WIFI_WEBSERVER

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "wifi_backend.hpp"

/**
 * @brief WebSocket server for command handling.
 * Used by rtls-link-manager tool for device configuration.
 */
class WifiWebSocket : public WifiBackend {
public:
    WifiWebSocket(const char* wsPath, uint16_t port);

    void Update() override;

private:
    AsyncWebServer m_Server;
    AsyncWebSocket m_Ws;
    bool initialized = false;
};

#endif // USE_WIFI_WEBSERVER