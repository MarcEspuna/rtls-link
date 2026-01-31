#pragma once

#include <Arduino.h>

#include <etl/array.h>

#include "utils/utils.hpp"

enum class UWBMode : uint8_t {      // Added mode to names since the UWBLibrary leaks ANCHOR and TAG defines
    ANCHOR_MODE_TWR,
    TAG_MODE_TWR,
    CALIBRATION_MODE,
    ANCHOR_TDOA,
    TAG_TDOA,
    UNKNOWN
};

enum class ZCalcMode : uint8_t {
    NONE = 0,        // Use Z from UWB TDoA estimator (default)
    RANGEFINDER = 1, // Use Z from MAVLink DISTANCE_SENSOR
    UWB = 2          // Reserved for future UWB-based Z calculation
};

// Anchor layout configurations for dynamic position calculation
enum class AnchorLayout : uint8_t {
    RECTANGULAR_0_ORIGIN = 0,  // A0 at origin (default)
    RECTANGULAR_1_ORIGIN = 1,  // A1 at origin
    RECTANGULAR_2_ORIGIN = 2,  // A2 at origin
    RECTANGULAR_3_ORIGIN = 3,  // A3 at origin
    CUSTOM = 255               // Reserved for future custom layouts
};

using UWBShortAddr = etl::array<char, 2>;

/**
 * @brief Used as a mirror for anchor data on the main UWBParams structure.
 * 
 */
struct UWBAnchorParam {
    UWBShortAddr shortAddr;
    float x;
    float y;
    float z;
}ULS_PACKED;

struct UWBParams {
    UWBMode mode;                                       // Device UWB mode
    UWBShortAddr devShortAddr;                          // Device short address
    uint8_t anchorCount;                                // Number of anchors that might be available to the device
    // Anchor devices and it's relative locations
    UWBShortAddr devId1; // Anchor devices short addresses
    float x1;
    float y1;
    float z1;
    UWBShortAddr devId2;
    float x2;
    float y2;
    float z2;
    UWBShortAddr devId3;
    float x3;
    float y3;
    float z3;
    UWBShortAddr devId4;
    float x4;
    float y4;
    float z4;
    UWBShortAddr devId5;
    float x5;
    float y5;
    float z5;
    UWBShortAddr devId6;
    float x6;
    float y6;
    float z6;
    uint16_t ADelay;            // Antenna delay    
    float calDistance;          // Calibration distance settings
    double originLat;           // Origin Latitude
    double originLon;           // Origin Longitude
    float originAlt;            // Origin Altitude
    uint8_t mavlinkTargetSystemId; // MAVLink Target System ID
    float rotationDegrees;      // Rotation degrees to NED frame
    ZCalcMode zCalcMode;        // Z calculation mode (0=None/TDoA, 1=Rangefinder, 2=UWB-reserved)
    // Rangefinder forwarding parameters
    uint8_t rfForwardEnable = 0;        // 0=disabled, 1=enabled (forward DISTANCE_SENSOR to ArduPilot)
    uint8_t rfForwardSensorId = 0xFF;   // Sensor ID override (0xFF = preserve source value)
    uint8_t rfForwardOrientation = 0xFF; // Orientation override (0xFF = preserve source)
    uint8_t rfForwardPreserveSrcIds = 0; // 0=use UWB device IDs (default), 1=preserve source IDs
    // Position estimation parameters
    uint8_t enableCovMatrix = 0;    // 0=disabled, 1=enabled (send covariance to ArduPilot)
    float rmseThreshold = 0.8f;     // RMSE threshold for position validity (meters)
    // UWB Radio settings (TDoA mode only)
    uint8_t channel = 2;            // UWB channel (1-7), default 2
    uint8_t dwMode = 0;             // DW1000 mode index (0=SHORTDATA_FAST_ACCURACY, see getModeByIndex)
    uint8_t txPowerLevel = 3;       // TX power level (0=low, 1=med-low, 2=med-high, 3=high/large)
    uint8_t smartPowerEnable = 0;   // Smart power (0=disabled, 1=enabled)

    // TDoA TDMA schedule (anchors)
    // NOTE: 0 values keep legacy behavior (8 slots, ~2ms slot length).
    uint8_t tdoaSlotCount = 0;          // Active TDMA slots per frame (2-8), 0=legacy (8)
    uint16_t tdoaSlotDurationUs = 0;    // Slot duration in microseconds, 0=legacy (~2ms)

    // Dynamic anchor positioning (TDoA tags)
    uint8_t dynamicAnchorPosEnabled = 0;  // 0=static (use configured positions), 1=dynamic (calculate from inter-anchor distances)
    uint8_t anchorLayout = 0;             // AnchorLayout enum value (0=RECTANGULAR_0_ORIGIN)
    float anchorHeight = 0.0f;            // Height for Z calculation (NED: Z = -height)
    uint8_t anchorPosLocked = 0;          // Bitmask: bit N = anchor N position locked
    uint16_t distanceAvgSamples = 50;     // Number of samples to average before calculating (default: 50)

    // static constant values that will be useful for parameter reading & writing
    static constexpr uint8_t maxAnchorCount = 6;
}ULS_PACKED;

// Parameters that we do not expect to need to change in the future. 
namespace UWBConstParam {
    // Starts the calibration with the specified antenna delay
    static constexpr uint32_t calibr_starting_adelay = 16580;
}

/**
 * @todo We need to add array and custom structs to parameter system in the future to avoid this mess.
 * 
 */
