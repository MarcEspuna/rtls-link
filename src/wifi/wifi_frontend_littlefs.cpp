#include "config/features.hpp"  // MUST be first project include

#include "wifi_frontend_littlefs.hpp"

// Conditional includes based on feature flags
#ifdef USE_WIFI_WEBSERVER
#include "wifi_websocket.hpp"
#endif

#ifdef USE_WIFI_UART_BRIDGE
#include "wifi_uart_bridge.hpp"
#endif

#ifdef USE_WIFI_TCP_LOGGING
#include "wifi_tcp_server.hpp"
#endif

#ifdef USE_WIFI_DISCOVERY
#include "wifi_discovery.hpp"
#endif

#include "logging/logging.hpp"

#include "command_handler/command_handler.hpp"
#include <utils/utils.hpp>
#include <etl/vector.h>

#include "app.hpp"
#include "uwb/uwb_tdoa_tag.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"

// Free function for telemetry callback (ETL delegates require free function or static method)
#ifdef USE_WIFI_DISCOVERY
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
#endif // USE_WIFI_DISCOVERY

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

    LOG_INFO("WifiLittleFSFrontend initialized");

    UpdateMode(m_Params.mode);

#ifdef USE_WIFI_TCP_LOGGING
    m_TcpLoggingServer = new WifiTcpServer(m_Params.dbgPort);
#else
    m_TcpLoggingServer = nullptr;
#endif

    // Apply logging settings from stored params
    ApplyLoggingSettings();

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
    LOG_INFO("WiFi AP mode - SSID: %s", m_Params.ssidAP.data());

    WiFi.mode(WIFI_AP);
    bool success = WiFi.softAP(m_Params.ssidAP.data(), m_Params.pswdAP.data());

    if (success) {
        LOG_INFO("AP IP: %s", WiFi.softAPIP().toString().c_str());
        SetupWebServer();
        return true;
    } else {
        LOG_WARN("AP setup failed");
        return false;
    }
}

void WifiLittleFSFrontend::SetupStation() {
    LOG_INFO("WiFi Station mode - SSID: %s", m_Params.ssidST.data());

    WiFi.mode(WIFI_STA);
    WiFi.begin(m_Params.ssidST.data(), m_Params.pswdST.data());
}

void WifiLittleFSFrontend::SetupWebServer() {
    ClearBackends();

    if (m_Params.enableWebServer) {
#ifdef USE_WIFI_WEBSERVER
        LOG_INFO("WebSocket server enabled");
        WifiWebSocket* webSocketServer = new WifiWebSocket("/ws", 80);
        m_Backends.push_back(webSocketServer);
#else
        LOG_WARN("WebServer requested but USE_WIFI_WEBSERVER not compiled");
#endif
    }

    if (m_Params.enableUartBridge) {
#ifdef USE_WIFI_UART_BRIDGE
        LOG_INFO("UART bridge setup on port %d", m_Params.udpPort);
        IPAddress ip;
        if(ip.fromString(m_Params.gcsIp.data())) {
            WifiUartBridge* uartBridge = new WifiUartBridge(Serial2, ip, m_Params.udpPort);
            m_Backends.push_back(uartBridge);
            LOG_INFO("UART bridge enabled");
        } else {
            LOG_WARN("Invalid GCS IP address");
        }
#else
        LOG_WARN("UART bridge requested but USE_WIFI_UART_BRIDGE not compiled");
#endif
    }

    if (m_Params.enableDiscovery) {
#ifdef USE_WIFI_DISCOVERY
        WifiDiscovery* discoveryBackend = new WifiDiscovery(m_Params.discoveryPort, m_Params);
        // Set telemetry callback for heartbeat enrichment
        discoveryBackend->SetTelemetryCallback(
            TelemetryCallback::create<&GetDeviceTelemetry>()
        );
        m_Backends.push_back(discoveryBackend);
        LOG_INFO("Discovery service enabled on port %d", m_Params.discoveryPort);
#else
        LOG_WARN("Discovery requested but USE_WIFI_DISCOVERY not compiled");
#endif
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
            LOG_ERROR("Undefined WiFi mode");
            break;
    }
}

void WifiLittleFSFrontend::StationConnectionThread() {
    if (m_currentMode == WifiMode::STATION) {
        if (WiFi.status() == WL_CONNECTED && !m_stationConnected) {
            LOG_INFO("WiFi connected - IP: %s", WiFi.localIP().toString().c_str());
            SetupWebServer();
            m_stationConnected = true;

            // Re-apply logging settings now that WiFi is connected
            // (UDP backend requires WiFi to be ready)
            ApplyLoggingSettings();
        } else if (WiFi.status() != WL_CONNECTED && m_stationConnected) {
            LOG_WARN("WiFi disconnected");
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

void WifiLittleFSFrontend::ApplyLoggingSettings() {
#ifdef USE_LOGGING
    // Ensure Logger is initialized before configuring
    rtls::log::Logger::init();

    // Apply serial and UDP enabled settings
    rtls::log::Logger::setSerialEnabled(m_Params.logSerialEnabled != 0);
    rtls::log::Logger::setUdpEnabled(m_Params.logUdpEnabled != 0);

    // Set UDP target IP (use GCS IP) and port for log streaming
    if (m_Params.gcsIp[0] != '\0') {
        rtls::log::Logger::setUdpTarget(m_Params.gcsIp.data(), m_Params.logUdpPort);
    }
#endif
}

ErrorParam WifiLittleFSFrontend::SetParam(const char* name, const void* data, uint32_t len) {
    // Call base class implementation first
    ErrorParam result = LittleFSFrontend<WifiParams>::SetParam(name, data, len);

    if (result == ErrorParam::OK) {
        // Check if this was a logging-related parameter and apply changes
        if (strcmp(name, "logSerialEnabled") == 0 ||
            strcmp(name, "logUdpEnabled") == 0 ||
            strcmp(name, "logUdpPort") == 0 ||
            strcmp(name, "gcsIp") == 0) {
            ApplyLoggingSettings();
        }
    }

    return result;
}

namespace Front {
    WifiLittleFSFrontend wifiLittleFSFront;
}