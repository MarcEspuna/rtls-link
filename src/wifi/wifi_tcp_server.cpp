#include "wifi_tcp_server.hpp"


WifiTcpServer::WifiTcpServer(uint16_t port)
    : m_Server(port)
{
    // Create a mutex to protect the clients vector
    mutex = xSemaphoreCreateMutex();
    
    m_Server.onClient( [this](void* unused, AsyncClient* client){
        // Call the function to handle the new client
        OnClientConnected(client);
    }, NULL);

  m_Server.begin();
}

void WifiTcpServer::Update()
{
    // Here we could listen and process the data but for now we will ignore it
    if (m_DataReady)
    {
        // Send the data to all clients
        SendToAllClients(m_Data);
        m_DataReady = false;
    }
}




void WifiTcpServer::SendToAllClients(const char *data)
{
    // Protect the clients vector with a mutex
    xSemaphoreTake(mutex, portMAX_DELAY);
    for (auto client : m_Clients)
    {
        if (client->canSend() && client->space() >= strlen(data)) {
            client->write(data);
        }
    }
    xSemaphoreGive(mutex);
}

void WifiTcpServer::AddForSending(const char *data)
{
    xSemaphoreTake(mutex, portMAX_DELAY);
    strncpy(m_Data, data, sizeof(m_Data));
    m_DataReady = true;   
    xSemaphoreGive(mutex);
}

void WifiTcpServer::OnClientConnected(AsyncClient *client)
{
    // Print the client's IP address
    Serial.printf("New Client: %s\n", client->remoteIP().toString().c_str());

    if (m_Clients.size() >= 5) {
        Serial.println("Too many clients connected");
        client->write("Too many clients connected\n");
        delete client;
        return;
    }
    
    // Protect the clients vector with a mutex
    xSemaphoreTake(mutex, portMAX_DELAY);
    m_Clients.push_back(client);
    xSemaphoreGive(mutex);

    // Handle client disconnection
    client->onDisconnect([this](void* unused, AsyncClient* c){
        Serial.printf("Client Disconnected: %s\n", c->remoteIP().toString().c_str());
        xSemaphoreTake(mutex, portMAX_DELAY);
        m_Clients.erase(std::remove(m_Clients.begin(), m_Clients.end(), c), m_Clients.end());
        xSemaphoreGive(mutex);
        delete c;
    }, NULL);
}
