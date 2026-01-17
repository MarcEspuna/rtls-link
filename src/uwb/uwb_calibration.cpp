#include "config/features.hpp"

#ifdef USE_UWB_CALIBRATION

#include "logging/logging.hpp"

#include <Arduino.h>
#include <DW1000Ranging.h>
#include <DW1000.h>

// Undefine DEBUG macro from DW1000 to avoid conflict with logging system
#undef DEBUG

#include "uwb_config.hpp"
#include "uwb_calibration.hpp"

static float this_anchor_target_distance = 0; //measured distance to anchor in m

static uint16_t this_anchor_Adelay = 16600; //starting value
static uint16_t Adelay_delta = 100; //initial binary search step size
static bool calibrationDone = false;

static void newRange();
static void newDevice(DW1000Device *device);
static void inactiveDevice(DW1000Device *device);

static UWBBackend* s_Backend;       // Needed since DW1000Ranging library is completely static and has C-style callbacks

UWBCalibration::UWBCalibration(IUWBFrontend& front, const bsp::UWBConfig& uwb_config, UWBShortAddr shortAddr, float calibrationDistance)
    : UWBBackend(front, uwb_config)
{
    s_Backend = this;

    this_anchor_target_distance = calibrationDistance;

    const bsp::UWBPinout& pinout = uwb_config.pins; 
    DW1000Ranging.initCommunication(pinout.reset_pin, pinout.spi_cs_pin, pinout.int_pin); //Reset, CS, IRQ pin
    
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachBlinkDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);

    auto longAddr = [shortAddr]() -> char* {
      static constexpr uint32_t kLongAddrSize = sizeof(bsp::kBoardConfig.uwb.long_base_address);
      static char addr[kLongAddrSize];
      memcpy(addr, bsp::kBoardConfig.uwb.long_base_address.data(), sizeof(bsp::kBoardConfig.uwb.long_base_address));
      addr[0] = shortAddr[0];
      addr[1] = shortAddr[1];
      return addr;
    }();
    
    //start the module as an anchor, do not assign random short address
    DW1000Ranging.startAsAnchor(longAddr, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, false);    
    
    DW1000.setAntennaDelay(UWBConstParam::calibr_starting_adelay);

    LOG_INFO("UWB Calibration initialized");
}

void UWBCalibration::Update()
{
  if (calibrationDone) return;
  DW1000Ranging.loop();
}

/**
 * @todo use mean values in order to validate that the sign did change
 * 
 */
static void newRange()
{
  if (calibrationDone) return;

  static float last_delta = 0.0;
  // Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), DEC);

  static float dist = 0;
  static uint32_t counter = 0;
  dist += DW1000Ranging.getDistantDevice()->getRange();
  counter++; 

  // We do a mean of 10 samples before we run one iteration of the antenna delay calibration
  if (counter > 9) {

    dist = dist/10;
    counter = 0;

    if (Adelay_delta < 3) {
      LOG_DEBUG("Calibration complete: dist=%.2f, Adelay=%d", dist, this_anchor_Adelay);
      calibrationDone = true;
      s_Backend->UpdateAntennaDelay(this_anchor_Adelay);
      s_Backend->UpdateMode(UWBMode::ANCHOR_MODE_TWR);        // This signifies that the calibration mode is complete
      return;
    }

    float this_delta = dist - this_anchor_target_distance;  //error in measured distance

    if ( this_delta * last_delta < 0.0) Adelay_delta = Adelay_delta / 2; //sign changed, reduce step size
      last_delta = this_delta;

    if (this_delta > 0.0 ) this_anchor_Adelay += Adelay_delta; //new trial Adelay
    else this_anchor_Adelay -= Adelay_delta;

    LOG_DEBUG("Calibration: dist=%.2f, Adelay=%d", dist, this_anchor_Adelay);

    DW1000.setAntennaDelay(this_anchor_Adelay);
    dist = 0;
  }
}

static void newDevice(DW1000Device *device)
{
  LOG_DEBUG("Device added: 0x%X", device->getShortAddress());
}

static void inactiveDevice(DW1000Device *device)
{
  LOG_DEBUG("Device removed: 0x%X", device->getShortAddress());
}

#endif // USE_UWB_CALIBRATION
