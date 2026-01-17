#include "config/features.hpp"

#include "uwb_frontend_littlefs.hpp"

// Conditional includes based on feature flags
#ifdef USE_UWB_MODE_TWR_ANCHOR
#include "uwb_anchor.hpp"
#endif

#ifdef USE_UWB_MODE_TWR_TAG
#include "uwb_tag.hpp"
#endif

#ifdef USE_UWB_CALIBRATION
#include "uwb_calibration.hpp"
#endif

#ifdef USE_UWB_MODE_TDOA_TAG
#include "uwb_tdoa_tag.hpp"
#endif

#ifdef USE_UWB_MODE_TDOA_ANCHOR
#include "uwb_tdoa_anchor.hpp"
#endif

void UWBLittleFSFrontend::Init() {
    LittleFSFrontend<UWBParams>::Init();
    printf("UWBLittleFSFrontend::Init()\n");

    auto anchors = GetAnchors();
    switch (m_Params.mode)
    {
    case UWBMode::ANCHOR_MODE_TWR:
#ifdef USE_UWB_MODE_TWR_ANCHOR
        m_Backend = new UWBAnchor(*this, bsp::kBoardConfig.uwb, m_Params.devShortAddr, m_Params.ADelay);
#else
        printf("WARNING: TWR Anchor mode requested but USE_UWB_MODE_TWR_ANCHOR not compiled\n");
#endif
        break;
    case UWBMode::TAG_MODE_TWR:
#ifdef USE_UWB_MODE_TWR_TAG
        m_Backend = new UWBTag(*this, bsp::kBoardConfig.uwb, anchors);
#else
        printf("WARNING: TWR Tag mode requested but USE_UWB_MODE_TWR_TAG not compiled\n");
#endif
        break;
    case UWBMode::CALIBRATION_MODE:
#ifdef USE_UWB_CALIBRATION
        m_Backend = new UWBCalibration(*this, bsp::kBoardConfig.uwb, m_Params.devShortAddr, m_Params.calDistance);
#else
        printf("WARNING: Calibration mode requested but USE_UWB_CALIBRATION not compiled\n");
#endif
        break;
    case UWBMode::ANCHOR_TDOA:
#ifdef USE_UWB_MODE_TDOA_ANCHOR
        m_Backend = new UWBAnchorTDoA(*this, bsp::kBoardConfig.uwb, m_Params.devShortAddr, m_Params.ADelay);
#else
        printf("WARNING: TDoA Anchor mode requested but USE_UWB_MODE_TDOA_ANCHOR not compiled\n");
#endif
        break;
    case UWBMode::TAG_TDOA:
#ifdef USE_UWB_MODE_TDOA_TAG
        m_Backend = new UWBTagTDoA(*this, bsp::kBoardConfig.uwb, anchors);
#else
        printf("WARNING: TDoA Tag mode requested but USE_UWB_MODE_TDOA_TAG not compiled\n");
#endif
        break;
    default:
        printf("Unknown UWB mode\n");
        break;
    }

    printf("------ UWB Frontend Initialized ------\n");
}

void UWBLittleFSFrontend::Update() {
    if (m_Backend) {
        m_Backend->Update();
    }
}

uint32_t UWBLittleFSFrontend::GetConnectedDevices() {
    if (m_Backend) {
        return m_Backend->GetNumberOfConnectedDevices();
    }
    return 0;
}

bool UWBLittleFSFrontend::StartTag() {
    if (m_Backend) {
        return m_Backend->Start();
    }
    return false;
}

void UWBLittleFSFrontend::PerformAnchorCalibration() {
    // Set uwb mode to calibration and reboot.
    m_Params.mode = UWBMode::CALIBRATION_MODE;
    SaveParams();
    // Reboot
    ESP.restart();
}

etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> UWBLittleFSFrontend::GetAnchors() {
    etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> anchors;
    anchors.reserve(UWBParams::maxAnchorCount);

    uint8_t anchorCount = min(m_Params.anchorCount, UWBParams::maxAnchorCount);

    if (anchorCount >= 1) anchors.push_back({m_Params.devId1, m_Params.x1, m_Params.y1, m_Params.z1});
    if (anchorCount >= 2) anchors.push_back({m_Params.devId2, m_Params.x2, m_Params.y2, m_Params.z2});
    if (anchorCount >= 3) anchors.push_back({m_Params.devId3, m_Params.x3, m_Params.y3, m_Params.z3});
    if (anchorCount >= 4) anchors.push_back({m_Params.devId4, m_Params.x4, m_Params.y4, m_Params.z4});
    if (anchorCount >= 5) anchors.push_back({m_Params.devId5, m_Params.x5, m_Params.y5, m_Params.z5});
    if (anchorCount >= 6) anchors.push_back({m_Params.devId6, m_Params.x6, m_Params.y6, m_Params.z6});
    
    return anchors;
}

namespace Front {
    UWBLittleFSFrontend uwbLittleFSFront;
}