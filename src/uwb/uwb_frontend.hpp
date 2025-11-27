#pragma once

#include <Arduino.h>

#include "uwb/uwb_params.hpp"
#include "front.hpp"

#include "uwb_backend.hpp"


#include "uwb_frontend_interface.hpp"

class UWBFront : public Frontend<UWBParams>, public IUWBFrontend {
public:
    friend class UWBBackend;

    UWBFront();

    virtual void Init() override;
    virtual void Update() override;

    virtual etl::span<const ParamDef> GetParamLayout() const override;

    virtual const etl::string_view GetParamGroup() const override;

    uint32_t GetConnectedDevices();

    bool StartTag();
    void PerformAnchorCalibration();

private:
    void UpdateAntennaDelay(uint16_t delay) override;
    void UpdateMode(UWBMode mode) override;

    etl::vector<UWBAnchorParam, UWBParams::maxAnchorCount> GetAnchors();
    
private:
    UWBBackend* m_Backend;
};

namespace Front {
    extern UWBFront uwbFront;
}