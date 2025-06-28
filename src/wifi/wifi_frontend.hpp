#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <EEPROM.h>

#include <etl/span.h>
#include <etl/array.h>
   
#include "wifi/wifi_params.hpp"
#include "front.hpp"
#include "scheduler.hpp"

#include "wifi/wifi_backend.hpp"
#include "wifi/wifi_tcp_server.hpp"

/**
 * @brief Here the idea is that the front end will initialize the wifi connection. It will either initialize the AP or the Station
 * based on parameter configuration. Then it will also construct the different backends, servers, socket connections, serial communications,
 * etc.
 */
 
 /**
  * @brief For now we will have 3 possible backends for the wifi frontend. They are non-exclusives, so we can have all of them running at the same time.
  * - WebSocketServer: Command line socket interface together with a web interface.
  * - UartBridge: A bridge between a serial port and a UDP port. Useful for forwarding mavlink data.
  * - Debug socket server(Maybe we should change it to UDP): Used to visualize 3D position data in realtime. 
  *     -   Expand to get more data from the UWB sensors like signal strenght, health, update rate, error rate, etc. 
  */

 /**
  * @brief NONO. Here we will have UDP, TCP and websocket servers. It will basically be wrappers around ip, port and other parameters uppon initialization 
  * together with freeRTOS data queues.  The application layer will be the one in charge of performing the uart_udp bridge, the websocket server(It's compleatly
  * async so no code there), and TCP socket connections(Primarly used for data debugging).
  */

 /**
  * @brief New parameters will be needed in order to configure and initialize this functionality on startup. 
  * Idea is that the front end will initialize, for example, a TCP server for socket connection if the parameter to log 3D coordinates is set to true.
  * Then it will assign such server to be the backend used for outputting the data.
  * This concept will also be ported to the UDP server. 
  * For the webpage we simply need the HTML page and the rest should be handled by the async web server.
  */

class WifiFront: public Frontend<WifiParams> {
public:
    WifiFront();

    virtual void Init() override;
    virtual void Update() override;

    virtual etl::span<const ParamDef> GetParamLayout() const override;

    virtual const etl::string_view GetParamGroup() const override;

    void UpdateLastTWRSample(float x, float y, float z, uint32_t update_rate);

    // Expose station connection thread for scheduler
    void StationConnectionThread();

private:
    bool SetupAP();
    void SetupStation();
    void SetupWebServer();

    static constexpr uint32_t maxClients = 10;

    etl::vector<WifiBackend*, maxClients> m_Backends;
    WifiTcpServer* m_TcpLoggingServer;
    WifiMode m_currentMode = WifiMode::UNDEFINED; // Store the configured mode
    bool m_stationConnected = false; // Track station connection state
};

namespace Front {
    extern WifiFront wifiFront;
}