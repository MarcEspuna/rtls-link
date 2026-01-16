#include "config/features.hpp"  // MUST be first project include

#include "wifi_frontend_littlefs.hpp"
#include "wifi_websocket.hpp"
#include "wifi_uart_bridge.hpp"
#include "wifi_tcp_server.hpp"
#include "wifi_discovery.hpp"

#include "command_handler/command_handler.hpp"
#include <utils/utils.hpp>
#include <etl/vector.h>

#include "app.hpp"
#include "uwb/uwb_tdoa_tag.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

// Free function for telemetry callback (ETL delegates require free function or static method)
static DeviceTelemetry GetDeviceTelemetry() {
    DeviceTelemetry t;

#ifdef USE_MAVLINK
    t.sending_pos = App::IsSendingPositions();
    t.origin_sent = App::IsOriginSent();
#else
    t.sending_pos = false;
    t.origin_sent = false;
#endif

    // Only report anchors_seen for TDoA tag mode
#ifdef USE_UWB_MODE_TDOA_TAG
    if (Front::uwbLittleFSFront.GetParams().mode == UWBMode::TAG_TDOA) {
        t.anchors_seen = UWBTagTDoA::GetAnchorsSeenCount();
    } else {
        t.anchors_seen = 0;  // Not applicable for non-TDoA-tag modes
    }
#else
    t.anchors_seen = 0;
#endif

    // Rangefinder status
#ifdef HAS_RANGEFINDER
    t.rf_enabled = App::IsRangefinderEnabled();
    t.rf_healthy = App::IsRangefinderHealthy();
#else
    t.rf_enabled = false;
    t.rf_healthy = false;
#endif

    // Update rate statistics (for tags)
#ifdef USE_RATE_STATISTICS
    t.avg_rate_cHz = App::GetAvgUpdateRateCHz();
    t.min_rate_cHz = App::GetMinRateCHz();
    t.max_rate_cHz = App::GetMaxRateCHz();
#else
    t.avg_rate_cHz = 0;
    t.min_rate_cHz = 0;
    t.max_rate_cHz = 0;
#endif

    return t;
}

// I will define a static FreeRTOS task holder for station connection checks
static StaticTaskHolder<etl::delegate<void()>, 4096> wifi_connection_task = {
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
    
    UpdateMode(m_Params.mode);

    m_TcpLoggingServer = new WifiTcpServer(m_Params.dbgPort);
    
    // Set the delegate after initialization
    wifi_connection_task.taskFunction = etl::delegate<void()>::create<WifiLittleFSFrontend, Front::wifiLittleFSFront, &WifiLittleFSFrontend::StationConnectionThread>();
    
    // Schedule the connection task
    Scheduler::scheduler.CreateStaticTask(wifi_connection_task);
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
    ClearBackends();

    if (m_Params.enableWebServer) {
        printf("Setting up WebSocket server\n");
        WifiWebSocket* webSocketServer = new WifiWebSocket("/ws", 80);
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

    if (m_Params.enableDiscovery) {
        WifiDiscovery* discoveryBackend = new WifiDiscovery(m_Params.discoveryPort, m_Params);
        // Set telemetry callback for heartbeat enrichment
        discoveryBackend->SetTelemetryCallback(
            TelemetryCallback::create<&GetDeviceTelemetry>()
        );
        m_Backends.push_back(discoveryBackend);
        printf("Discovery service enabled on port %d\n", m_Params.discoveryPort);
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



void WifiLittleFSFrontend::ClearBackends() {
    for (auto* backend : m_Backends) {
        delete backend;
    }
    m_Backends.clear();
}

namespace Front {
    WifiLittleFSFrontend wifiLittleFSFront;
}