#pragma once

#include <Arduino.h>
#include <etl/array.h>

#include "../littlefs_frontend.hpp"
#include "../utils/utils.hpp"

struct AppParams {
    int16_t led2Pin;    // LED 2 GPIO pin (-1 = disabled)
    uint8_t led2State;  // Current LED 2 state (0 or 1)
}ULS_PACKED;

class AppLittleFSFrontend : public LittleFSFrontend<AppParams> {
public:
    AppLittleFSFrontend() : LittleFSFrontend<AppParams>("app") {}

    virtual void Init() override;
    virtual void Update() override {}

    virtual etl::span<const ParamDef> GetParamLayout() const override {
        return etl::span<const ParamDef>(s_ParamDefs, sizeof(s_ParamDefs)/sizeof(ParamDef));
    }

    virtual const etl::string_view GetParamGroup() const override {
        return etl::string_view("app");
    }

    AppParams& GetParams() { return m_Params; }

    // LED 2 control methods
    void SetLed2State(bool state);
    bool GetLed2State() const;
    void ToggleLed2();
    bool IsLed2Configured() const;

public:
    static constexpr ParamDef s_ParamDefs[] = {
        PARAM_DEF(AppParams, led2Pin),
        PARAM_DEF(AppParams, led2State)
    };
};

namespace Front {
    extern AppLittleFSFrontend appLittleFSFront;
}
