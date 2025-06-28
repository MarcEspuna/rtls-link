#pragma once

#include <Arduino.h>
#include <etl/array.h>
#include <utils/utils.hpp>

// Define ULS_PACKED if not already defined
#ifndef ULS_PACKED
#define ULS_PACKED __attribute__((packed))
#endif

namespace bcn_konex {

// Protocol constants
static constexpr uint8_t PREAMBLE = 0x55;
static constexpr uint8_t HDR_ADD_ANCHOR = 0x00;
static constexpr uint8_t HDR_REMOVE_ANCHOR = 0x01;
static constexpr uint8_t HDR_SAMPLE = 0x02;
static constexpr uint8_t HDR_RANGE_SAMPLE = 0x03;

// Data structures
struct RelativeLocation
{
    float x__m, y__m, z__m;
}ULS_PACKED;

struct AddAnchor    // Response with ACK with header value needed
{
    uint8_t preamble;
    uint8_t header;            
    uint8_t anchor_id;          
    RelativeLocation location;
    uint16_t crc;
}ULS_PACKED;

struct RemoveAnchor  // Response with ACK with header value needed
{
    uint8_t preamble;
    uint8_t header;   
    uint8_t anchor_id;          
    uint16_t crc;
}ULS_PACKED;

struct Sample
{
    uint8_t preamble;
    uint8_t header;
    RelativeLocation tag_loc;
    uint16_t error__mm;         // Estimated error in mm
    uint16_t crc;
}ULS_PACKED; 

struct RangeSample
{
    uint8_t preamble;
    uint8_t header;
    uint8_t anchor_id;
    float range__m;
    uint16_t crc;
}ULS_PACKED;

template<typename T>
struct FlaggedData
{
    T data;
    bool flag;
};

class BeaconProtocol {
public:
    static void Init();
    
    // Public API for sending data
    static void SetAnchorsToEcho(const etl::array<double, 12>& anchor_locations);
    static void SendAddAnchorEcho();
    static void SendSample(float x_m, float y_m, float z_m, uint16_t error);
    static void SendRangeSample(uint8_t id, float range);
    
    // ACK handling
    static void ACKListener();

private:
    static etl::array<double, 12> s_locations_to_echo;
    static SemaphoreHandle_t s_anchor_mutex;
};

} // namespace bcn_konex
