#include "command_handler/command_handler.hpp"

#include "wifi_websocket.hpp"

#include <LittleFS.h>

static void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
            void *arg, uint8_t *data, size_t len);

WifiWebSocket::WifiWebSocket(const char* html_page, const char* domain, uint16_t port)
    : m_Server(port), m_Ws(domain)
{
    // Start the file system for the web page
    if (LittleFS.begin()) {
        printf("File system started\n");
    } else {
        printf("Error starting file system\n");
        return;
    }

    // Read the web.html file
    File file = LittleFS.open("/web.html", "r");
    if (!file) {
        printf("Failed to open file for reading\n");
        return;
    } else {
        printf("File opened\n");
    }

    // ********** Setup websocket **********
    m_Ws.onEvent(onEvent);
    m_Server.addHandler(&m_Ws);
    
    // Reserve memory for the html page
    size_t size = file.size();
    char* web_page = new char[size+1];
    if (web_page == nullptr) {
        printf("Failed to allocate memory for html page\n");
        return;
    }
    // Add the null char at the end of the buffer
    web_page[size] = '\0';

    // Read the file into the html_page buffer
    file.readBytes(web_page, size);

    // Use the web.html file as the main page
    m_Server.on("/", HTTP_GET, [web_page](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", web_page);
    });

    // Start server
    m_Server.begin();
    initialized = true;
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