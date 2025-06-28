#include <Eigen.h>    // Eigen needs to be included before Arduino.h
#include <Arduino.h>

#include <etl/array.h>
#include <etl/span.h>

#include "param.hpp"

#include "uwb_frontend.hpp"

#include "uwb/uwb_anchor.hpp"
#include "uwb/uwb_tag.hpp"
#include "uwb/uwb_tag_posxyz.hpp"
#include "uwb/uwb_calibration.hpp"
#include "uwb/uwb_tdoa_tag.hpp"
#include "uwb/uwb_tdoa_anchor.hpp"


namespace Front {
    UWBFront uwbFront;
}

UWBFront::UWBFront()
    : Frontend(__FUNCTION__), m_Backend(nullptr)
{
}

void UWBFront::Init()
{
    Frontend::InitEeprom();
    
    UWBMode mode = UWBMode::UNKNOWN;
    Frontend::ReadParam(&UWBParams::mode, &mode);

    UWBShortAddr devShortAddr;
    Frontend::ReadParam(&UWBParams::devShortAddr, &devShortAddr);

    uint16_t antennaDelay = 0;
    Frontend::ReadParam(&UWBParams::ADelay, &antennaDelay);
    
    auto anchors = GetAnchors();
    switch (mode)
    {
    case UWBMode::ANCHOR_MODE_TWR: {
        m_Backend = new UWBAnchor(*this, bsp::kBoardConfig.uwb, devShortAddr, antennaDelay);
        break;
        }
    case UWBMode::TAG_MODE_TWR:{


        // TODO: Read all anchor ids and positions to pass to UWBTag
        m_Backend = new UWBTag(*this, bsp::kBoardConfig.uwb, anchors);       // Should specify all anchor ids and positions
        break;
        }
    case UWBMode::CALIBRATION_MODE: {
        // TODO: Read target distance (in the future we will also need to specify tag ids to calibrate with)
        float calibrationDistance = 0.0f;
        Frontend::ReadParam(&UWBParams::calDistance, &calibrationDistance);
        m_Backend = new UWBCalibration(*this, bsp::kBoardConfig.uwb, devShortAddr, calibrationDistance);
        break;
    }
    case UWBMode::ANCHOR_TDOA: {
        m_Backend = new UWBAnchorTDoA(*this, bsp::kBoardConfig.uwb, devShortAddr, antennaDelay);
        
        break;
    }
    case UWBMode::TAG_TDOA: {
        m_Backend = new UWBTagTDoA(*this, bsp::kBoardConfig.uwb, anchors);

        break;
    }
    default:
        printf("Unknown UWB mode\n");
        break;
    }

    printf("------ UWB Frontend Initialized ------\n");
}

void UWBFront::Update()
{
    if (m_Backend)
    {
        m_Backend->Update();
    }
}


uint32_t UWBFront::GetConnectedDevices()
{
    if (m_Backend)
    {
        return m_Backend->GetNumberOfConnectedDevices();
    }
    return 0;
}

const etl::string_view UWBFront::GetParamGroup() const
{
    static constexpr etl::string_view group = "uwb";
    return group;
}

bool UWBFront::StartTag()
{
    if (m_Backend)
    {
        return m_Backend->Start();
    } 
    return false;
}

void UWBFront::PerformAnchorCalibration()
{
    // Set uwb mode to calibration and reboot.
    UWBMode mode = UWBMode::CALIBRATION_MODE;
    Frontend::WriteParam(&UWBParams::mode, mode);
    // Reboot
    ESP.restart();
}

void UWBFront::UpdateAntennaDelay(uint16_t delay)
{
    // Write parameter to eeprom
    Frontend::WriteParam(&UWBParams::ADelay, delay);
}

void UWBFront::UpdateMode(UWBMode mode)
{
    // Write parameter to eeprom 
    Frontend::WriteParam(&UWBParams::mode, mode);
}

etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> UWBFront::GetAnchors()
{
    etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> anchors;
    anchors.reserve(UWBParams::maxAnchorCount);

    uint8_t anchorCount = 0;
    Frontend::ReadParam(&UWBParams::anchorCount, &anchorCount);
    anchorCount = min(anchorCount, UWBParams::maxAnchorCount);

    for(uint32_t i = 0; i < anchorCount; i++)
    {
        UWBAnchorParam anchorParam = {};
        Frontend::ReadParam(&UWBParams::devId1, &anchorParam.shortAddr, sizeof(UWBAnchorParam)*i); 
        Frontend::ReadParam(&UWBParams::x1, &anchorParam.x, sizeof(UWBAnchorParam)*i);
        Frontend::ReadParam(&UWBParams::y1, &anchorParam.y, sizeof(UWBAnchorParam)*i);
        Frontend::ReadParam(&UWBParams::z1, &anchorParam.z, sizeof(UWBAnchorParam)*i);
        anchors.push_back(anchorParam);
    }
    
    return anchors;
}


etl::span<const ParamDef> UWBFront::GetParamLayout() const 
{
    static etl::array<ParamDef, 29> layout = {{
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
        PARAM_DEF(UWBParams, calDistance)
    }};
    return layout;
}
