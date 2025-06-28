#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include "wifi_backend.hpp"

/**
 * @brief Here we should give the HTML web page, ip and port to the websocket server.
 * 
 */

class WifiWebSocket : public WifiBackend {
public:
    WifiWebSocket(const char* html_page, const char* domain, uint16_t port);

    void Update() override;

private: 
    AsyncWebServer m_Server;
    AsyncWebSocket m_Ws;
    bool initialized = false;

};