#include "config/features.hpp"

#ifdef USE_DRONE_SLEEP_MODE

#include "drone_sleep_controller.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <cstring>

#include "app.hpp"
#include "bsp/board.hpp"
#include "logging/logging.hpp"
#include "uwb/uwb_frontend_littlefs.hpp"
#include "wifi/wifi_frontend_littlefs.hpp"

namespace {

static constexpr uint32_t kSafetyHeartbeatWaitMs = 2500;
static constexpr uint32_t kWakeRestartDelayMs = 500;

bool s_initialized = false;
bool s_sleeping = false;
bool s_restart_pending = false;
uint32_t s_restart_at_ms = 0;

bool isValidPin(int16_t pin)
{
    return pin >= 0 && pin < 49;
}

void writePowerDisablePins(bool disabled)
{
    const auto& pins = bsp::kBoardConfig.power_disable_pins;
    const uint8_t level = disabled ? HIGH : LOW;

    digitalWrite(pins.motors_ardupilot_pin, level);
    digitalWrite(pins.elrs_rx_pin, level);
    digitalWrite(pins.uwb_pin, level);
}

void setWifiPowerSave(bool enabled)
{
    WiFi.setSleep(enabled);
    esp_wifi_set_ps(enabled ? WIFI_PS_MAX_MODEM : WIFI_PS_NONE);
}

} // namespace

void DroneSleepController::Init()
{
    const auto& pins = bsp::kBoardConfig.power_disable_pins;
    if (!IsSupported()) {
        return;
    }

    pinMode(pins.motors_ardupilot_pin, OUTPUT);
    pinMode(pins.elrs_rx_pin, OUTPUT);
    pinMode(pins.uwb_pin, OUTPUT);
    writePowerDisablePins(false);
    s_initialized = true;
    LOG_INFO("Drone sleep power pins initialized");
}

void DroneSleepController::Update()
{
    if (s_restart_pending && static_cast<int32_t>(millis() - s_restart_at_ms) >= 0) {
        ESP.restart();
    }

    if (s_sleeping) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

bool DroneSleepController::IsSleeping()
{
    return s_sleeping;
}

bool DroneSleepController::IsSupported()
{
    const auto& pins = bsp::kBoardConfig.power_disable_pins;
    return isValidPin(pins.motors_ardupilot_pin)
        && isValidPin(pins.elrs_rx_pin)
        && isValidPin(pins.uwb_pin);
}

bool DroneSleepController::EnterSleep(char* message, size_t message_len)
{
    if (!IsSupported()) {
        SetMessage(message, message_len, "Sleep mode unsupported on this board");
        return false;
    }

    if (!s_initialized) {
        Init();
    }

    if (s_sleeping) {
        SetMessage(message, message_len, "Already sleeping");
        return true;
    }

    char safety_reason[96] = {};
    if (!App::WaitForArdupilotDisarmed(kSafetyHeartbeatWaitMs,
                                       safety_reason,
                                       sizeof(safety_reason))) {
        SetMessage(message, message_len, safety_reason);
        return false;
    }

    if (!Front::uwbLittleFSFront.SetRuntimeEnabled(false)) {
        SetMessage(message, message_len, "Failed to disable UWB runtime");
        return false;
    }

    delay(20);
    writePowerDisablePins(true);
    s_sleeping = true;
    setWifiPowerSave(true);
    Front::wifiLittleFSFront.SetLowPowerMode(true);

    LOG_WARN("Drone sleep mode entered");
    SetMessage(message, message_len, "Sleeping");
    return true;
}

bool DroneSleepController::Wake(char* message, size_t message_len)
{
    if (!IsSupported()) {
        SetMessage(message, message_len, "Sleep mode unsupported on this board");
        return false;
    }

    if (!s_initialized) {
        Init();
    }

    writePowerDisablePins(false);
    setWifiPowerSave(false);
    Front::wifiLittleFSFront.SetLowPowerMode(false);

    if (!s_sleeping) {
        SetMessage(message, message_len, "Already awake");
        return true;
    }

    s_sleeping = false;
    s_restart_pending = true;
    s_restart_at_ms = millis() + kWakeRestartDelayMs;

    LOG_WARN("Drone wake requested; ESP restart scheduled");
    SetMessage(message, message_len, "Waking");
    return true;
}

void DroneSleepController::SetMessage(char* message, size_t message_len, const char* text)
{
    if (message == nullptr || message_len == 0) {
        return;
    }

    std::strncpy(message, text == nullptr ? "" : text, message_len - 1);
    message[message_len - 1] = '\0';
}

#endif // USE_DRONE_SLEEP_MODE
