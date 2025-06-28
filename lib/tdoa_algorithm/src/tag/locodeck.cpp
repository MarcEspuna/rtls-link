#include "locodeck.h"

static lpsAlgoOptions_t algoOptions = {
  .rangingState = 0,
  // .userRequestedMode is the wanted algorithm, available as a parameter
  .userRequestedMode = lpsMode_TDoA2,

  // .currentRangingMode is the currently running algorithm, available as a log
  // lpsMode_auto is an impossible mode which forces initialization of the requested mode
  // at startup
  .currentRangingMode = lpsMode_auto,
  .modeAutoSearchDoInitialize = true,
  .modeAutoSearchActive = true,
  .nextSwitchTick = 0
};

uint16_t locoDeckGetRangingState() {
  return algoOptions.rangingState;
}

void locoDeckSetRangingState(const uint16_t newState) {
  algoOptions.rangingState = newState;
}