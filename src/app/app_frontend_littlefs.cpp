#include "app_frontend_littlefs.hpp"
#include "bsp/board.hpp"
#include "logging/logging.hpp"

void AppLittleFSFrontend::Init() {
    // Set defaults from BSP before loading from file
    m_Params.led2Pin = bsp::kBoardConfig.led2_pin;
    m_Params.led2State = 0;

    // Load parameters from LittleFS (will override defaults if file exists)
    LittleFSFrontend<AppParams>::Init();

    LOG_DEBUG("App frontend init - LED2: %d, state: %d", m_Params.led2Pin, m_Params.led2State);

    // Initialize LED 2 pin if configured
    if (m_Params.led2Pin >= 0) {
        pinMode(m_Params.led2Pin, OUTPUT);
        digitalWrite(m_Params.led2Pin, m_Params.led2State ? HIGH : LOW);
    }
}

void AppLittleFSFrontend::SetLed2State(bool state) {
    m_Params.led2State = state ? 1 : 0;
    if (m_Params.led2Pin >= 0) {
        digitalWrite(m_Params.led2Pin, state ? HIGH : LOW);
    }
}

bool AppLittleFSFrontend::GetLed2State() const {
    return m_Params.led2State != 0;
}

void AppLittleFSFrontend::ToggleLed2() {
    SetLed2State(!GetLed2State());
}

bool AppLittleFSFrontend::IsLed2Configured() const {
    return m_Params.led2Pin >= 0;
}

namespace Front {
    AppLittleFSFrontend appLittleFSFront;
}
