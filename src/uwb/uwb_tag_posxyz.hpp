#pragma once

#include <etl/span.h>

#include <SimpleKalmanFilter.h>

#include "uwb_backend.hpp"


////////////////// Pozyx //////////////////////////////

#define MSG_HEADER          0x01
#define MSGID_BEACON_CONFIG 0x02
#define MSGID_BEACON_DIST   0x03
#define MSGID_POSITION      0x04

// structure for messages uploaded to ardupilot
union beacon_config_msg {
    struct {
        uint8_t beacon_id;
        uint8_t beacon_count;
        int32_t x;
        int32_t y;
        int32_t z;
    } info;
    uint8_t buf[14];
};
union beacon_distance_msg {
    struct {
        uint8_t beacon_id;
        uint32_t distance;
    } info;
    uint8_t buf[5];
};
union vehicle_position_msg {
    struct {
        int32_t x;
        int32_t y;
        int32_t z;
        int16_t position_error;
    } info;
    uint8_t buf[14];
};
////////////////////////////////////////////////

class UWBTagPosXYZ : public UWBBackend {
public:
    UWBTagPosXYZ(UWBFront& front, const bsp::UWBConfig& uwb_config, etl::span<const UWBAnchorParam> anchors);

    void Update() override;
 
    bool Start() override;
private:
 


};