#include "config/features.hpp"

#ifdef USE_UWB_MODE_TWR_ANCHOR

#include "logging/logging.hpp"

#include <DW1000Ranging.h>
#include <DW1000.h>
#include <DW1000Time.h>

#include "uwb_params.hpp"
#include "uwb_anchor.hpp"

static void newRange();
static void newDevice(DW1000Device *device);
static void inactiveDevice(DW1000Device *device);

UWBAnchor::UWBAnchor(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, uint16_t antennaDelay)
    : UWBBackend(front, uwb_config)
{
    // @note: Take special attention to task refresh rate and priority to not trigger the wdt!
    const bsp::UWBPinout& pinout = uwb_config.pins; 
    DW1000Ranging.initCommunication(pinout.reset_pin, pinout.spi_cs_pin, pinout.int_pin); //Reset, CS, IRQ pin
    
    DW1000.setAntennaDelay(antennaDelay);
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    static auto longAddr = [shortAddr]() -> char* {
      static constexpr uint32_t kLongAddrSize = bsp::kBoardConfig.uwb.long_base_address.length();
      static char addr[kLongAddrSize];
      memcpy(addr, bsp::kBoardConfig.uwb.long_base_address.data(), sizeof(bsp::kBoardConfig.uwb.long_base_address));
      addr[0] = shortAddr[0];
      addr[1] = shortAddr[1];
      return addr;
    }();

    //start the module as an anchor, do not assign random short address
    DW1000Ranging.startAsAnchor(longAddr, DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);

    LOG_INFO("UWB Anchor initialized");
}

void UWBAnchor::Update()
{
    DW1000Ranging.loop();
}

static void newRange()
{
#define NUMBER_OF_DISTANCES 1
  float dist = 0.0;
  for (int i = 0; i < NUMBER_OF_DISTANCES; i++) {
    dist += DW1000Ranging.getDistantDevice()->getRange();
  }
  dist = dist/NUMBER_OF_DISTANCES;
  LOG_DEBUG("Range from 0x%X: %.2f m", DW1000Ranging.getDistantDevice()->getShortAddress(), dist);
}

static void newDevice(DW1000Device *device)
{
  LOG_DEBUG("Device added: 0x%X", device->getShortAddress());
}

static void inactiveDevice(DW1000Device *device)
{
  LOG_DEBUG("Device removed: 0x%X", device->getShortAddress());
}

#endif // USE_UWB_MODE_TWR_ANCHOR
