#include "config/features.hpp"

#ifdef USE_WIFI_WEBSERVER

#include "command_handler/command_handler.hpp"
#include "wifi_websocket.hpp"
#include "logging/logging.hpp"

#ifdef USE_ARDUPILOT_UPDATE
#include "ardupilot_update/session.hpp"
#endif

#ifdef USE_OTA_WEB
#include "ota/ota_handler.hpp"
#endif

static void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
            void *arg, uint8_t *data, size_t len);
#ifdef USE_ARDUPILOT_UPDATE
static void onArduPilotUpdateEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
            AwsEventType type, void *arg, uint8_t *data, size_t len);
#endif

WifiWebSocket::WifiWebSocket(const char* wsPath, uint16_t port)
    : m_Server(port), m_Ws(wsPath)
#ifdef USE_ARDUPILOT_UPDATE
    , m_ArduPilotUpdateWs("/ardupilot-update")
#endif
{
    // Setup WebSocket handler
    m_Ws.onEvent(onEvent);
    m_Server.addHandler(&m_Ws);

#ifdef USE_ARDUPILOT_UPDATE
    m_ArduPilotUpdateWs.onEvent(onArduPilotUpdateEvent);
    m_Server.addHandler(&m_ArduPilotUpdateWs);
#endif

#ifdef USE_OTA_WEB
    // Register OTA update routes
    ota::initOtaRoutes(m_Server);
#endif

    // Start server
    m_Server.begin();
    initialized = true;
    LOG_INFO("WebSocket server started on port %d", port);
}

void WifiWebSocket::Update()
{
    if (initialized) {
        m_Ws.cleanupClients();
#ifdef USE_ARDUPILOT_UPDATE
        m_ArduPilotUpdateWs.cleanupClients();
        ArduPilotUpdateSession::Instance().Update();
#endif
    }
}

static void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            LOG_DEBUG("WebSocket client #%u connected", client->id());
            break;
        case WS_EVT_DISCONNECT:
            LOG_DEBUG("WebSocket client #%u disconnected", client->id());
            break;
        case WS_EVT_DATA:
        {
            AwsFrameInfo *info = (AwsFrameInfo*)arg;
            if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
                data[len] = 0;
                String result = CommandHandler::ExecuteCommand((char*)data);
                // Send back result to the client
                if(client != nullptr) {
                    client->text(result);
                }
            }
            break;
        }
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}

#ifdef USE_ARDUPILOT_UPDATE
static void onArduPilotUpdateEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)server;
    auto& session = ArduPilotUpdateSession::Instance();

    switch (type) {
        case WS_EVT_CONNECT:
            session.OnClientConnect(client);
            break;
        case WS_EVT_DISCONNECT:
            session.OnClientDisconnect(client);
            break;
        case WS_EVT_DATA:
            session.OnClientData(client, static_cast<AwsFrameInfo*>(arg), data, len);
            break;
        case WS_EVT_PONG:
        case WS_EVT_ERROR:
            break;
    }
}
#endif

#endif // USE_WIFI_WEBSERVER
