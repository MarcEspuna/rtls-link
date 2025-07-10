#include "wifi_frontend_littlefs.hpp"
#include "wifi_websocket.hpp"
#include "wifi_uart_bridge.hpp"
#include "wifi_tcp_server.hpp"

#include "command_handler/command_handler.hpp"
#include <utils/utils.hpp>
#include <etl/vector.h>

// I will define a static FreeRTOS task holder for station connection checks
static StaticTaskHolder<etl::delegate<void()>, 2048> wifi_connection_task = {
    "WifiConnTask", // Task name
    1,              // Frequency: 1Hz
    1,              // Priority
    etl::delegate<void()>(),  // Will be set in Init()
    {},
    {}
};

void WifiLittleFSFrontend::Init() {
    LittleFSFrontend<WifiParams>::Init();
    
    printf("WifiLittleFSFrontend::Init()\n");
    
    m_TcpLoggingServer = new WifiTcpServer(m_Params.dbgPort);
    
    // Set the delegate after initialization
    wifi_connection_task.taskFunction = etl::delegate<void()>::create<WifiLittleFSFrontend, Front::wifiLittleFSFront, &WifiLittleFSFrontend::StationConnectionThread>();
    
    // Schedule the connection task
    Scheduler::scheduler.CreateStaticTask(wifi_connection_task);
    
    UpdateMode(m_Params.mode);
}

void WifiLittleFSFrontend::Update() {
    for (WifiBackend* backend : m_Backends) {
        backend->Update();
    }
    
    if (m_TcpLoggingServer) {
        m_TcpLoggingServer->Update();
    }
}

bool WifiLittleFSFrontend::SetupAP() {
    printf("Setting up WiFi Access Point mode\n");
    printf("SSID: %s, Password: %s\n", m_Params.ssidAP.data(), m_Params.pswdAP.data());

    WiFi.mode(WIFI_AP);
    bool success = WiFi.softAP(m_Params.ssidAP.data(), m_Params.pswdAP.data());
    
    if (success) {
        printf("AP setup successful\n");
        printf("AP IP address: %s\n", WiFi.softAPIP().toString().c_str());
        SetupWebServer();
        return true;
    } else {
        printf("AP setup failed\n");
        return false;
    }
}

void WifiLittleFSFrontend::SetupStation() {
    printf("Setting up WiFi Station mode\n");
    printf("SSID: %s, Password: %s\n", m_Params.ssidST.data(), m_Params.pswdST.data());

    WiFi.mode(WIFI_STA);
    WiFi.begin(m_Params.ssidST.data(), m_Params.pswdST.data());
}

void WifiLittleFSFrontend::SetupWebServer() {
    if (m_Params.enableWebServer) {
        printf("Setting up web server\n");
        WifiWebSocket* webSocketServer = new WifiWebSocket("web.html", "/ws", 80);
        m_Backends.push_back(webSocketServer);
    }
    
    if (m_Params.enableUartBridge) {
        printf("Setting up UART bridge on port %d\n", m_Params.udpPort);
        IPAddress ip;
        if(ip.fromString(m_Params.gcsIp.data())) {
            WifiUartBridge* uartBridge = new WifiUartBridge(Serial2, ip, m_Params.udpPort);
            m_Backends.push_back(uartBridge);
            printf("--- Uart bridge enabled ---\n");
        } else {
            printf("Invalid GSC IP address\n");
        }
    }
}

void WifiLittleFSFrontend::UpdateMode(WifiMode mode) {
    m_currentMode = mode;
    
    switch (mode) {
        case WifiMode::AP:
            SetupAP();
            break;
        case WifiMode::STATION:
            SetupStation();
            break;
        default:
            printf("Undefined WiFi mode\n");
            break;
    }
}

void WifiLittleFSFrontend::StationConnectionThread() {
    if (m_currentMode == WifiMode::STATION) {
        if (WiFi.status() == WL_CONNECTED && !m_stationConnected) {
            printf("WiFi station connected\n");
            printf("Station IP address: %s\n", WiFi.localIP().toString().c_str());
            SetupWebServer();
            m_stationConnected = true;
        } else if (WiFi.status() != WL_CONNECTED && m_stationConnected) {
            printf("WiFi station disconnected\n");
            m_stationConnected = false;
        }
    }
}

void WifiLittleFSFrontend::UpdateLastTWRSample(float x, float y, float z, uint32_t update_rate) {
    // Send latest tag position through dbg socket
    char buffer[100] = {};
    float lastTwrSample[3] = {x, y, z};

    // Do not send nan values
    for (int i = 0; i < 3; i++) {
        if (std::isnan(lastTwrSample[i])) {
            lastTwrSample[i] = 0.0f;
        }
    }

    snprintf(buffer, 100, "%.2f %.2f %.2f %u hz", lastTwrSample[0], lastTwrSample[1], lastTwrSample[2], update_rate);
    if (m_TcpLoggingServer) {
        m_TcpLoggingServer->AddForSending(buffer);
    }
}

namespace Front {
    WifiLittleFSFrontend wifiLittleFSFront;
}