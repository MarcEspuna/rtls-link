#include "config/features.hpp"

#ifdef USE_WIFI_WEBSERVER

#include "command_handler/command_handler.hpp"
#include "wifi_websocket.hpp"

#ifdef USE_OTA_WEB
#include "ota/ota_handler.hpp"
#endif

static void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
            void *arg, uint8_t *data, size_t len);

WifiWebSocket::WifiWebSocket(const char* wsPath, uint16_t port)
    : m_Server(port), m_Ws(wsPath)
{
    // Setup WebSocket handler
    m_Ws.onEvent(onEvent);
    m_Server.addHandler(&m_Ws);

#ifdef USE_OTA_WEB
    // Register OTA update routes
    ota::initOtaRoutes(m_Server);
#endif

    // Start server
    m_Server.begin();
    initialized = true;
    printf("WebSocket server started on port %d at path %s\n", port, wsPath);
}

void WifiWebSocket::Update()
{
    if (initialized) {
        m_Ws.cleanupClients();
    }
}

static void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
    switch (type) {
        case WS_EVT_CONNECT:
            printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
            break;
        case WS_EVT_DISCONNECT:
            printf("WebSocket client #%u disconnected\n", client->id());
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

#endif // USE_WIFI_WEBSERVER
