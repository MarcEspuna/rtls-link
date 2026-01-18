#pragma once

#include "uwb_params.hpp"
#include "../littlefs_frontend.hpp"
#include "uwb_backend.hpp"

#include "uwb_frontend_interface.hpp"

class UWBLittleFSFrontend : public LittleFSFrontend<UWBParams>, public IUWBFrontend {
public:
    UWBLittleFSFrontend() : LittleFSFrontend<UWBParams>("uwb") {}

    virtual void Init() override;
    virtual void Update() override;

    void UpdateAntennaDelay(uint16_t delay) override {
        // Write parameter to eeprom
        // TODO: Implement parameter writing for LittleFS if needed, or just update in memory
        m_Params.ADelay = delay;
        SaveParams();
    }

    void UpdateMode(UWBMode mode) override {
        m_Params.mode = mode;
        SaveParams();
    }

    virtual etl::span<const ParamDef> GetParamLayout() const override {
        return etl::span<const ParamDef>(s_ParamDefs, sizeof(s_ParamDefs)/sizeof(ParamDef));
    }

    virtual const etl::string_view GetParamGroup() const override {
        return etl::string_view("uwb");
    }

    UWBParams& GetParams() {
        return m_Params;
    }

    uint32_t GetConnectedDevices();

    bool StartTag();
    void PerformAnchorCalibration();

protected:
    etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> GetAnchors();

    UWBBackend* m_Backend = nullptr;

public:
    static constexpr ParamDef s_ParamDefs[] = {
        PARAM_DEF(UWBParams, mode),
        PARAM_DEF(UWBParams, devShortAddr),
        PARAM_DEF(UWBParams, anchorCount),
        PARAM_DEF(UWBParams, devId1),
        PARAM_DEF(UWBParams, x1),
        PARAM_DEF(UWBParams, y1),
        PARAM_DEF(UWBParams, z1),
        PARAM_DEF(UWBParams, devId2),
        PARAM_DEF(UWBParams, x2),
        PARAM_DEF(UWBParams, y2),
        PARAM_DEF(UWBParams, z2),
        PARAM_DEF(UWBParams, devId3),
        PARAM_DEF(UWBParams, x3),
        PARAM_DEF(UWBParams, y3),
        PARAM_DEF(UWBParams, z3),
        PARAM_DEF(UWBParams, devId4),
        PARAM_DEF(UWBParams, x4),
        PARAM_DEF(UWBParams, y4),
        PARAM_DEF(UWBParams, z4),
        PARAM_DEF(UWBParams, devId5),
        PARAM_DEF(UWBParams, x5),
        PARAM_DEF(UWBParams, y5),
        PARAM_DEF(UWBParams, z5),
        PARAM_DEF(UWBParams, devId6),
        PARAM_DEF(UWBParams, x6),
        PARAM_DEF(UWBParams, y6),
        PARAM_DEF(UWBParams, z6),
        PARAM_DEF(UWBParams, ADelay),
        PARAM_DEF(UWBParams, calDistance),
        PARAM_DEF(UWBParams, originLat),
        PARAM_DEF(UWBParams, originLon),
        PARAM_DEF(UWBParams, originAlt),
        PARAM_DEF(UWBParams, mavlinkTargetSystemId),
        PARAM_DEF(UWBParams, rotationDegrees),
        PARAM_DEF(UWBParams, zCalcMode),
        PARAM_DEF(UWBParams, rfForwardEnable),
        PARAM_DEF(UWBParams, rfForwardSensorId),
        PARAM_DEF(UWBParams, rfForwardOrientation),
        PARAM_DEF(UWBParams, rfForwardPreserveSrcIds),
        PARAM_DEF(UWBParams, enableCovMatrix),
        PARAM_DEF(UWBParams, rmseThreshold),
        PARAM_DEF(UWBParams, channel),
        PARAM_DEF(UWBParams, dwMode),
        PARAM_DEF(UWBParams, txPowerLevel),
        PARAM_DEF(UWBParams, smartPowerEnable),
        PARAM_DEF(UWBParams, dynamicAnchorPosEnabled),
        PARAM_DEF(UWBParams, anchorLayout),
        PARAM_DEF(UWBParams, anchorHeight),
        PARAM_DEF(UWBParams, anchorPosLocked),
        PARAM_DEF(UWBParams, distanceAvgSamples)
    };
};

namespace Front {
    extern UWBLittleFSFrontend uwbLittleFSFront;
}