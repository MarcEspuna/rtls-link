#include "config/features.hpp"

#ifdef USE_ARDUPILOT_UPDATE

#include "session.hpp"

#include <common/mavlink.h>

#include "bsp/board.hpp"
#include "logging/logging.hpp"

ArduPilotUpdateSession& ArduPilotUpdateSession::Instance()
{
    static ArduPilotUpdateSession session;
    return session;
}

ArduPilotUpdateSession::ArduPilotUpdateSession()
    : m_mutex(xSemaphoreCreateMutex())
{
}

bool ArduPilotUpdateSession::Begin(uint8_t targetSystem, uint32_t flightBaud, uint32_t bootloaderBaud, String& error)
{
    if (!Lock()) {
        error = "ArduPilot update session lock unavailable";
        return false;
    }

    if (m_active) {
        error = "ArduPilot update session already active";
        Unlock();
        return false;
    }

    m_flightBaud = flightBaud == 0 ? kDefaultFlightBaud : flightBaud;
    m_bootloaderBaud = bootloaderBaud == 0 ? kDefaultBootloaderBaud : bootloaderBaud;
    m_startedAtMs = millis();
    m_switchAtMs = 0;
    m_targetSystem = targetSystem;
    m_beginPending = true;
    m_endPending = false;
    m_rebootSent = false;
    m_tunnelReady = false;
    m_endReason = nullptr;
    m_client = nullptr;
    m_clientId = 0;
    m_active = true;
    Unlock();

    LOG_INFO("ArduPilot update session requested (flight=%lu bootloader=%lu target=%u)",
             static_cast<unsigned long>(m_flightBaud),
             static_cast<unsigned long>(m_bootloaderBaud),
             static_cast<unsigned int>(targetSystem));
    return true;
}

void ArduPilotUpdateSession::End(const char* reason)
{
    if (!Lock()) {
        return;
    }

    if (!m_active) {
        Unlock();
        return;
    }

    m_endPending = true;
    m_tunnelReady = false;
    m_endReason = reason;
    Unlock();
}

void ArduPilotUpdateSession::Update()
{
    bool doBegin = false;
    bool doEnd = false;
    bool doSwitch = false;
    bool doDrain = false;
    uint8_t targetSystem = 0;
    uint32_t flightBaud = 0;
    const char* endReason = nullptr;

    if (!Lock()) {
        return;
    }

    if (!m_active) {
        Unlock();
        return;
    }

    const uint32_t now = millis();
    if (m_beginPending) {
        m_beginPending = false;
        doBegin = true;
        targetSystem = m_targetSystem;
        flightBaud = m_flightBaud;
    } else if (m_endPending) {
        doEnd = true;
        endReason = m_endReason;
    } else if (now - m_startedAtMs > kSessionTimeoutMs) {
        m_endPending = true;
        m_tunnelReady = false;
        m_endReason = "timeout";
        doEnd = true;
        endReason = m_endReason;
    } else if (m_rebootSent && !m_tunnelReady && static_cast<int32_t>(now - m_switchAtMs) >= 0) {
        doSwitch = true;
    } else if (m_tunnelReady) {
        doDrain = true;
    }

    Unlock();

    if (doBegin) {
        ConfigureFlightAndSendReboot(targetSystem, flightBaud);
        return;
    }

    if (doEnd) {
        FinishEnd(endReason);
        return;
    }

    if (doSwitch) {
        SwitchToBootloaderBaud();
        return;
    }

    if (doDrain) {
        DrainSerialToTunnel();
    }
}

bool ArduPilotUpdateSession::IsActive() const
{
    if (!Lock()) {
        return false;
    }
    const bool active = m_active;
    Unlock();
    return active;
}

bool ArduPilotUpdateSession::IsTunnelReady() const
{
    if (!Lock()) {
        return false;
    }
    const bool ready = m_active && m_tunnelReady;
    Unlock();
    return ready;
}

void ArduPilotUpdateSession::OnClientConnect(AsyncWebSocketClient* client)
{
    bool accept = false;

    if (client != nullptr && Lock()) {
        if (m_active && (m_client == nullptr || m_client->status() != WS_CONNECTED)) {
            m_client = client;
            m_clientId = client->id();
            accept = true;
        }
        Unlock();
    }

    if (!accept) {
        if (client != nullptr) {
            client->close();
        }
        return;
    }

    LOG_INFO("ArduPilot update tunnel client #%lu connected",
             static_cast<unsigned long>(client->id()));
}

void ArduPilotUpdateSession::OnClientDisconnect(AsyncWebSocketClient* client)
{
    uint32_t clientId = 0;

    if (client != nullptr && Lock()) {
        if (client->id() == m_clientId) {
            clientId = m_clientId;
            m_client = nullptr;
            m_clientId = 0;
            m_endPending = true;
            m_tunnelReady = false;
            m_endReason = "tunnel disconnect";
        }
        Unlock();
    }

    if (clientId != 0) {
        LOG_INFO("ArduPilot update tunnel client #%lu disconnected",
                 static_cast<unsigned long>(clientId));
    }
}

void ArduPilotUpdateSession::OnClientData(AsyncWebSocketClient* client, AwsFrameInfo* info, uint8_t* data, size_t len)
{
    if (client == nullptr || info == nullptr || data == nullptr || len == 0) {
        return;
    }

    if (!Lock()) {
        return;
    }

    const bool shouldWrite = m_active &&
                             m_tunnelReady &&
                             client->id() == m_clientId &&
                             (info->opcode == WS_BINARY || info->opcode == WS_CONTINUATION);
    if (shouldWrite) {
        Serial2.write(data, len);
    }
    Unlock();
}

bool ArduPilotUpdateSession::Lock(TickType_t timeout) const
{
    return m_mutex != nullptr && xSemaphoreTake(m_mutex, timeout) == pdTRUE;
}

void ArduPilotUpdateSession::Unlock() const
{
    xSemaphoreGive(m_mutex);
}

void ArduPilotUpdateSession::ConfigureFlightAndSendReboot(uint8_t targetSystem, uint32_t flightBaud)
{
    const auto& uartPins = bsp::kBoardConfig.mavlink_uart;
    Serial2.end();
    delay(20);
    Serial2.begin(flightBaud, SERIAL_8N1, uartPins.rx_pin, uartPins.tx_pin);
    ClearSerialInput();
    SendRebootToBootloader(targetSystem);

    if (Lock()) {
        if (m_active && !m_endPending) {
            m_rebootSent = true;
            m_switchAtMs = millis() + kBootloaderSwitchDelayMs;
        }
        Unlock();
    }

    LOG_INFO("ArduPilot update reboot command sent");
}

void ArduPilotUpdateSession::SendRebootToBootloader(uint8_t targetSystem)
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN] = {};

    mavlink_msg_command_long_pack(
        255,
        MAV_COMP_ID_MISSIONPLANNER,
        &msg,
        targetSystem,
        MAV_COMP_ID_AUTOPILOT1,
        MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        3.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f);

    const uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial2.write(buffer, len);
    Serial2.flush();
}

void ArduPilotUpdateSession::SwitchToBootloaderBaud()
{
    uint32_t bootloaderBaud = kDefaultBootloaderBaud;

    if (Lock()) {
        if (!m_active || m_endPending || m_tunnelReady) {
            Unlock();
            return;
        }
        bootloaderBaud = m_bootloaderBaud;
        m_tunnelReady = false;
        Unlock();
    }

    const auto& uartPins = bsp::kBoardConfig.mavlink_uart;
    Serial2.flush();
    Serial2.end();
    delay(20);
    Serial2.begin(bootloaderBaud, SERIAL_8N1, uartPins.rx_pin, uartPins.tx_pin);
    ClearSerialInput();

    if (Lock()) {
        if (m_active && !m_endPending) {
            m_tunnelReady = true;
        }
        Unlock();
    }

    LOG_INFO("ArduPilot update tunnel ready at %lu baud",
             static_cast<unsigned long>(bootloaderBaud));
}

void ArduPilotUpdateSession::FinishEnd(const char* reason)
{
    uint32_t flightBaud = kDefaultFlightBaud;

    if (Lock()) {
        flightBaud = m_flightBaud == 0 ? kDefaultFlightBaud : m_flightBaud;
        m_tunnelReady = false;
        m_client = nullptr;
        m_clientId = 0;
        Unlock();
    }

    RestoreSerial(flightBaud);

    if (Lock()) {
        m_active = false;
        m_beginPending = false;
        m_endPending = false;
        m_rebootSent = false;
        m_tunnelReady = false;
        m_endReason = nullptr;
        Unlock();
    }

    LOG_INFO("ArduPilot update session ended%s%s",
             reason ? ": " : "",
             reason ? reason : "");
}

void ArduPilotUpdateSession::RestoreSerial(uint32_t flightBaud)
{
    const auto& uartPins = bsp::kBoardConfig.mavlink_uart;
    Serial2.flush();
    Serial2.end();
    delay(20);
    Serial2.begin(flightBaud,
                  SERIAL_8N1,
                  uartPins.rx_pin,
                  uartPins.tx_pin);
    ClearSerialInput();
}

void ArduPilotUpdateSession::DrainSerialToTunnel()
{
    if (!Lock()) {
        return;
    }

    if (m_client == nullptr || m_client->status() != WS_CONNECTED) {
        Unlock();
        return;
    }

    uint8_t serialBuffer[kSerialBufferSize];
    while (Serial2.available() > 0 && m_client != nullptr && m_client->status() == WS_CONNECTED) {
        const int available = Serial2.available();
        const size_t toRead = min(static_cast<size_t>(available), kSerialBufferSize);
        const size_t bytesRead = Serial2.readBytes(serialBuffer, toRead);
        if (bytesRead == 0) {
            break;
        }
        m_client->binary(serialBuffer, bytesRead);
    }
    Unlock();
}

void ArduPilotUpdateSession::ClearSerialInput()
{
    while (Serial2.available() > 0) {
        Serial2.read();
    }
}

#endif // USE_ARDUPILOT_UPDATE
