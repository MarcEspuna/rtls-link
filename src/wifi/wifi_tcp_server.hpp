#pragma once

#include "wifi_backend.hpp" 

#include <Arduino.h>
#include <WiFi.h>   
#include <AsyncTCP.h>


class WifiTcpServer : public WifiBackend {
public:
    WifiTcpServer(uint16_t port);

    void Update() override;

    void SendToAllClients(const char* data);
    void AddForSending(const char* data);
private:
    void OnClientConnected(AsyncClient* client);

private:
    AsyncServer m_Server;

    SemaphoreHandle_t mutex;
    std::vector<AsyncClient*> m_Clients;

    char m_Data[100] = {};
    bool m_DataReady = false;
};