#include <Arduino.h>

#include "uwb_tdoa_anchor.hpp"

#include "FunctionalInterrupt.h"

#include "SPI.h"

#include <freertos/task.h>

#define DEFAULT_RX_TIMEOUT 10000

static void txCallback(dwDevice_t *dev);
static void rxCallback(dwDevice_t *dev);
static void rxTimeoutCallback(dwDevice_t *dev);
static void rxFailedCallback(dwDevice_t *dev);

static volatile bool isr_flag = false;

UWBAnchorTDoA::UWBAnchorTDoA(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay)
    : UWBBackend(front, uwb_config)
{
    // NOTE: Look into short data fast accuracy...
    // Using a lambda to attach the class method as an interrupt handler

    m_UwbConfig.address[1] = shortAddr[1] - '0';
    m_UwbConfig.address[0] = shortAddr[0] - '0';

    // Spi pins already setup on uwb_backend
    dwInit(&m_Device, &m_Ops);          // Initialize the driver. Init resets user data!
    m_Device.userdata = &m_DwData;

    int result = dwConfigure(&m_Device);      // Configure the DW1000    
    if (result != 0) {
        Serial.print("DW1000 configuration failed, devid: ");
        Serial.println(static_cast<uint32_t>(result));
    }
    Serial.print("DW1000 Configured\n");

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

    const uint8_t* mode = MODE_SHORTDATA_FAST_ACCURACY;
    dwEnableMode(&m_Device, mode);
    dwSetChannel(&m_Device, CHANNEL_2);
    dwSetPreambleCode(&m_Device, PREAMBLE_CODE_64MHZ_9);

    dwEnableLargePower(&m_Device);
    dwUseSmartPower(&m_Device, false);
    // dwSetTxPower(&m_Device, 0x1F1F1F1Ful);

    dwSetReceiveWaitTimeout(&m_Device, DEFAULT_RX_TIMEOUT);

    dwCommitConfiguration(&m_Device);

    Serial.print("Initialized TDoA Anchor: ");
    uint32_t dev_id = dwGetDeviceId(&m_Device);
    Serial.println(dev_id, HEX);

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