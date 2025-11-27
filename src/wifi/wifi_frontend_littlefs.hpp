#pragma once

#include "wifi_params.hpp"
#include "../littlefs_frontend.hpp"
#include "../scheduler.hpp"
#include "wifi_backend.hpp"

class WifiLittleFSFrontend : public LittleFSFrontend<WifiParams> {
public:
    WifiLittleFSFrontend() : LittleFSFrontend<WifiParams>("wifi") {}

    virtual void Init() override;
    virtual void Update() override;

    virtual etl::span<const ParamDef> GetParamLayout() const override {
        return etl::span<const ParamDef>(s_ParamDefs, sizeof(s_ParamDefs)/sizeof(ParamDef));
    }

    virtual const etl::string_view GetParamGroup() const override {
        return etl::string_view("wifi");
    }

    WifiParams& GetParams() {
        return m_Params;
    }

    void UpdateLastTWRSample(float x, float y, float z, uint32_t update_rate);
    void StationConnectionThread();

private:
    bool SetupAP();
    void SetupStation();
    void SetupWebServer();
    void UpdateMode(WifiMode mode);
    void ClearBackends();

    static constexpr uint32_t maxClients = 10;

    etl::vector<WifiBackend*, maxClients> m_Backends;
    class WifiTcpServer* m_TcpLoggingServer;
    WifiMode m_currentMode = WifiMode::UNDEFINED;
    bool m_stationConnected = false;

public:
    static constexpr ParamDef s_ParamDefs[] = {
        PARAM_DEF(WifiParams, mode),
        PARAM_DEF(WifiParams, ssidAP),
        PARAM_DEF(WifiParams, pswdAP),
        PARAM_DEF(WifiParams, ssidST),
        PARAM_DEF(WifiParams, pswdST),
        PARAM_DEF(WifiParams, gcsIp),
        PARAM_DEF(WifiParams, dbgPort),
        PARAM_DEF(WifiParams, udpPort),
        PARAM_DEF(WifiParams, enableWebServer),
        PARAM_DEF(WifiParams, enableUartBridge),
        PARAM_DEF(WifiParams, enableDebugSocket)
    };
};

namespace Front {
    extern WifiLittleFSFrontend wifiLittleFSFront;
}