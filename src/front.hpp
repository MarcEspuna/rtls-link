#pragma once

#include <Arduino.h>

#include <etl/span.h>
#include <etl/string_view.h>
#include <etl/vector.h>

#include "param.hpp"

// Base class for polymorphism
class IFrontend {
public:
    virtual ~IFrontend() = default;
    virtual void Init() = 0;
    virtual void Update() = 0;
    virtual etl::span<const ParamDef> GetParamLayout() const = 0;
    virtual const etl::string_view GetParamGroup() const = 0;
    virtual ErrorParam SetParam(const char* name, const void* data, uint32_t len) = 0;
    virtual ErrorParam GetParam(const char* name, char* value, uint32_t& len, ParamType& type) = 0;
    virtual ErrorParam LoadParams() = 0;
    virtual ErrorParam SaveParams() = 0;
};


namespace Front {
    constexpr uint32_t MAX_FRONTENDS = 10;
    
    etl::vector<IFrontend*, MAX_FRONTENDS>& Get(); 

    void AddFrontend(IFrontend* frontend);
    void InitFrontends();

    ErrorParam WriteGlobalParam(const char* group, const char* name, const void* data, uint32_t len);
    ErrorParam ReadGlobalParam(const char* group, const char* name, char* value, uint32_t& len, ParamType& type);
    
    ErrorParam LoadAllParams();
    ErrorParam SaveAllParams();
}
