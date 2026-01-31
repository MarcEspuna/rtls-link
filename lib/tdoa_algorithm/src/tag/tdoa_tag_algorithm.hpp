#pragma once

#include <Arduino.h>

extern "C" {
  #include "libdw1000.h"
}

#include "locodeck.h"

#include "tdoaStats.h"
#include "tdoaStorage.h"
#include "tdoaEngine.h"
#include "tdoaEngineInstance.h"

#include "mac.h"

#define CONFIG_DECK_LOCO_NR_OF_ANCHORS 8
#define LOCODECK_NR_OF_TDOA2_ANCHORS CONFIG_DECK_LOCO_NR_OF_ANCHORS

typedef struct {
  const locoAddress_t anchorAddress[LOCODECK_NR_OF_TDOA2_ANCHORS];

  point_t anchorPosition[LOCODECK_NR_OF_TDOA2_ANCHORS];
  bool combinedAnchorPositionOk;
} lpsTdoa2AlgoOptions_t;


typedef struct {
  uint8_t type;
  uint8_t sequenceNrs[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint32_t timestamps[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint16_t distances[LOCODECK_NR_OF_TDOA2_ANCHORS];
  uint16_t antennaDelay;  // Sending anchor's configured antenna delay (DW1000 ticks)
} __attribute__((packed)) rangePacket2_t;

// Protocol version
#define PACKET_TYPE_TDOA2 0x22

// Positions in payload for received LPP packets
#define LPS_TDOA2_LPP_HEADER (sizeof(rangePacket2_t))
#define LPS_TDOA2_LPP_TYPE (sizeof(rangePacket2_t) + 1)
#define LPS_TDOA2_LPP_PAYLOAD (sizeof(rangePacket2_t) + 2)

// Positions for sent LPP packets
#define LPS_TDOA2_TYPE_INDEX 0
#define LPS_TDOA2_SEND_LPP_PAYLOAD_INDEX 1

#define TDOA2_LPP_PACKET_SEND_TIMEOUT (LOCODECK_NR_OF_TDOA2_ANCHORS * 5)

#define TDOA2_RECEIVE_TIMEOUT 10000

void lpsTdoa2TagSetOptions(lpsTdoa2AlgoOptions_t* newOptions);

// Callback type for inter-anchor distance updates (for dynamic anchor positioning)
// Parameters: fromAnchorId, toAnchorId, distance (in DW1000 timestamp units), fromAnchor's antenna delay (DW1000 ticks)
typedef void (*InterAnchorDistanceCallback)(uint8_t fromAnchor, uint8_t toAnchor, uint16_t distanceTimestampUnits, uint16_t fromAntennaDelay);

// Register a callback to receive inter-anchor distance updates
void uwbTdoa2TagSetDistanceCallback(InterAnchorDistanceCallback callback);

// Get the last reported antenna delay for a specific anchor (DW1000 ticks)
uint16_t uwbTdoa2TagGetAnchorAntennaDelay(uint8_t anchorId);

extern uwbAlgorithm_t uwbTdoa2TagAlgorithm;