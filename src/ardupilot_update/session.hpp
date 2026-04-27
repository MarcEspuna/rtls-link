#pragma once

#include "config/features.hpp"

#ifdef USE_ARDUPILOT_UPDATE

#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class ArduPilotUpdateSession {
public:
    static ArduPilotUpdateSession& Instance();

    bool Begin(uint8_t targetSystem, uint32_t flightBaud, uint32_t bootloaderBaud, String& error);
    void End(const char* reason = nullptr);
    void Update();

    bool IsActive() const;
    bool IsTunnelReady() const;

    void OnClientConnect(AsyncWebSocketClient* client);
    void OnClientDisconnect(AsyncWebSocketClient* client);
    void OnClientData(AsyncWebSocketClient* client, AwsFrameInfo* info, uint8_t* data, size_t len);

private:
    ArduPilotUpdateSession();

    bool Lock(TickType_t timeout = portMAX_DELAY) const;
    void Unlock() const;
    void ConfigureFlightAndSendReboot(uint8_t targetSystem, uint32_t flightBaud);
    void SendRebootToBootloader(uint8_t targetSystem);
    void SwitchToBootloaderBaud();
    void FinishEnd(const char* reason);
    void RestoreSerial(uint32_t flightBaud);
    void DrainSerialToTunnel();
    void ClearSerialInput();

    static constexpr uint32_t kDefaultFlightBaud = 921600;
    static constexpr uint32_t kDefaultBootloaderBaud = 115200;
    static constexpr uint32_t kBootloaderSwitchDelayMs = 1200;
    static constexpr uint32_t kSessionTimeoutMs = 15UL * 60UL * 1000UL;
    static constexpr size_t kSerialBufferSize = 512;

    mutable SemaphoreHandle_t m_mutex = nullptr;
    bool m_active = false;
    bool m_beginPending = false;
    bool m_endPending = false;
    bool m_rebootSent = false;
    bool m_tunnelReady = false;
    uint8_t m_targetSystem = 0;
    const char* m_endReason = nullptr;
    uint32_t m_flightBaud = kDefaultFlightBaud;
    uint32_t m_bootloaderBaud = kDefaultBootloaderBaud;
    uint32_t m_startedAtMs = 0;
    uint32_t m_switchAtMs = 0;
    AsyncWebSocketClient* m_client = nullptr;
    uint32_t m_clientId = 0;
};

#endif // USE_ARDUPILOT_UPDATE
