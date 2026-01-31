#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_ANCHOR

#include <Arduino.h>
#include "logging/logging.hpp"

#include "uwb_tdoa_anchor.hpp"
#include "uwb_frontend_littlefs.hpp"

#include "FunctionalInterrupt.h"

#include "SPI.h"

#include <freertos/task.h>

#define DEFAULT_RX_TIMEOUT 10000

static void txCallback(dwDevice_t *dev);
static void rxCallback(dwDevice_t *dev);
static void rxTimeoutCallback(dwDevice_t *dev);
static void rxFailedCallback(dwDevice_t *dev);

// Helper function to get DW1000 mode array by index
static const uint8_t* getDwModeByIndex(uint8_t idx) {
    switch(idx) {
        case 0: return MODE_SHORTDATA_FAST_ACCURACY;   // 6.8Mb/s, 64MHz PRF, 128 preamble (DEFAULT)
        case 1: return MODE_LONGDATA_FAST_ACCURACY;    // 6.8Mb/s, 64MHz PRF, 1024 preamble
        case 2: return MODE_SHORTDATA_FAST_LOWPOWER;   // 6.8Mb/s, 16MHz PRF, 128 preamble
        case 3: return MODE_LONGDATA_FAST_LOWPOWER;    // 6.8Mb/s, 16MHz PRF, 1024 preamble
        case 4: return MODE_SHORTDATA_MID_ACCURACY;    // 850kb/s, 64MHz PRF, 128 preamble
        case 5: return MODE_LONGDATA_MID_ACCURACY;     // 850kb/s, 64MHz PRF, 1024 preamble
        case 6: return MODE_LONGDATA_RANGE_ACCURACY;   // 110kb/s, 64MHz PRF, 2048 preamble
        case 7: return MODE_LONGDATA_RANGE_LOWPOWER;   // 110kb/s, 16MHz PRF, 2048 preamble
        default: return MODE_SHORTDATA_FAST_ACCURACY;
    }
}

// Helper function to check if mode uses 64MHz PRF (for preamble code selection)
static bool isDwMode64MHzPRF(uint8_t idx) {
    // Modes 2, 3, 7 use 16MHz PRF; others use 64MHz PRF
    return (idx != 2 && idx != 3 && idx != 7);
}

// Helper function to apply TX power settings
static void applyTxPower(dwDevice_t* dev, uint8_t powerLevel, uint8_t smartPowerEnable) {
    if (smartPowerEnable) {
        dwUseSmartPower(dev, true);
    } else {
        dwUseSmartPower(dev, false);
        switch(powerLevel) {
            case 0: // Low power
                dwSetTxPower(dev, 0x07070707ul);
                break;
            case 1: // Medium-low
                dwSetTxPower(dev, 0x0F0F0F0Ful);
                break;
            case 2: // Medium-high
                dwSetTxPower(dev, 0x17171717ul);
                break;
            case 3: // High (large power) - default
            default:
                dwEnableLargePower(dev);
                break;
        }
    }
}

static volatile bool isr_flag = false;

UWBAnchorTDoA::UWBAnchorTDoA(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay)
    : UWBBackend(front, uwb_config)
{
    // NOTE: Look into short data fast accuracy...
    // Using a lambda to attach the class method as an interrupt handler

    LOG_INFO("--- UWB Anchor TDOA Mode ---");

    m_UwbConfig.address[1] = shortAddr[1] - '0';
    m_UwbConfig.address[0] = shortAddr[0] - '0';

    // Spi pins already setup on uwb_backend
    dwInit(&m_Device, &m_Ops);          // Initialize the driver. Init resets user data!
    m_Device.userdata = &m_DwData;

    int result = dwConfigure(&m_Device);      // Configure the DW1000
    if (result != 0) {
        LOG_WARN("DW1000 configuration failed, devid: %u", static_cast<uint32_t>(result));
    }
    LOG_INFO("DW1000 Configured (Anchor TDoA)");

    dwEnableAllLeds(&m_Device);

    // vTaskDelay(pdMS_TO_TICKS(500));

    dwTime_t delay = {.full = 0};
    dwSetAntenaDelay(&m_Device, delay);
    dwAttachSentHandler(&m_Device, txCallback);
    dwAttachReceivedHandler(&m_Device, rxCallback);
    dwAttachReceiveTimeoutHandler(&m_Device, rxTimeoutCallback);
    dwAttachReceiveFailedHandler(&m_Device, rxFailedCallback);
    dwNewConfiguration(&m_Device);
    dwSetDefaults(&m_Device);

    // Get UWB radio settings from parameters
    const auto& uwbParams = Front::uwbLittleFSFront.GetParams();
    const uint8_t* dwMode = getDwModeByIndex(uwbParams.dwMode);
    dwEnableMode(&m_Device, dwMode);
    dwSetChannel(&m_Device, uwbParams.channel);
    // Select preamble code based on PRF (16MHz vs 64MHz)
    if (isDwMode64MHzPRF(uwbParams.dwMode)) {
        dwSetPreambleCode(&m_Device, PREAMBLE_CODE_64MHZ_9);
    } else {
        dwSetPreambleCode(&m_Device, PREAMBLE_CODE_16MHZ_4);
    }

    // Apply TX power settings
    applyTxPower(&m_Device, uwbParams.txPowerLevel, uwbParams.smartPowerEnable);

    dwSetReceiveWaitTimeout(&m_Device, DEFAULT_RX_TIMEOUT);

    dwCommitConfiguration(&m_Device);

    uint32_t dev_id = dwGetDeviceId(&m_Device);
    LOG_INFO("Initialized TDoA Anchor - DevID: 0x%08X, Addr: %c%c", dev_id, shortAddr[0], shortAddr[1]);
    LOG_INFO("  Radio: mode=%u, ch=%u, txPower=%u, smartPwr=%s",
             uwbParams.dwMode, uwbParams.channel, uwbParams.txPowerLevel,
             uwbParams.smartPowerEnable ? "on" : "off");
    LOG_INFO("  Antenna delay: %u", antennaDelay);

    // Pass antenna delay to algorithm so it's broadcast in TX packets
    m_UwbConfig.antennaDelay = antennaDelay;

    // Init the tdoa anchor algorithm
    uwbTdoa2Algorithm.init(&m_UwbConfig, &m_Device);

    attachInterrupt(digitalPinToInterrupt(uwb_config.pins.int_pin), 
    [this]() { 
        this->InterruptHandler(); 
        }, 
    RISING);

    vTaskDelay(pdMS_TO_TICKS(300));

    uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventTimeout);
}


void UWBAnchorTDoA::Update() 
{
    if (isr_flag) {
        dwHandleInterrupt(&m_Device);
        isr_flag = false;
    }

    // static uint64_t last_time = millis();
    // static uint32_t sent = 0;
    // Call libdw1000 loop
    AnchorTDoADispatcher dispatcher(this);
    dispatcher.Dispatch(static_cast<libDw1000::IsrFlags>(m_DwData.interrupt_flags));
}

template<libDw1000::IsrFlags TFlags>
void UWBAnchorTDoA::OnEvent()
{
    if constexpr (TFlags == libDw1000::RX_DONE) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventPacketReceived);
    } else if constexpr (TFlags == libDw1000::TX_DONE) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventPacketSent);
    } else if constexpr (TFlags == libDw1000::RX_TIMEOUT) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveTimeout);
    } else if constexpr (TFlags == libDw1000::RX_FAILED) {
        uwbTdoa2Algorithm.onEvent(&m_Device, uwbEvent_t::eventReceiveFailed);
    }
    m_DwData.interrupt_flags &= ~TFlags;  // Clear the specific flag at the end
}

/* **** libdw1000 **** */
void UWBAnchorTDoA::InterruptHandler()
{
    // Set flag for interrupt handling in main loop (ISR-safe)
    isr_flag = true;
}

/* TODO: Move to FreeRTOS notifications */
static void txCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::TX_DONE;
}

static void rxCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_DONE;
}

static void rxTimeoutCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_TIMEOUT;
}

static void rxFailedCallback(dwDevice_t *dev)
{
    libDw1000::DwData* dw_data = libDw1000::GetUserData(dev);
    dw_data->interrupt_flags |= libDw1000::RX_FAILED;
}

#endif // USE_UWB_MODE_TDOA_ANCHOR