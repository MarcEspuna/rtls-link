#pragma once

#include <Arduino.h>
#include <EEPROM.h>

#include <etl/span.h>
#include <etl/string_view.h>
#include <etl/vector.h>

#include "utils/utils.hpp"
#include "logging/logging.hpp"

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

/**
 * @brief Frontend class
 * 
 */
template<typename TParams>
class Frontend : public IFrontend {
public:
    Frontend(const char* eepromUniqueName) : m_EEPROM(eepromUniqueName) {
        Front::AddFrontend(this);
        LOG_DEBUG("Frontend '%s' created", eepromUniqueName);
    }

    // TODO: Fix this. Should be direct call to EEPROM member instead of Read or Write function calls
    // Important @note! data should have always space for the nul character! Do not 
    virtual ErrorParam SetParam(const char* name, const void* data, uint32_t len) override {
        for (const ParamDef& param : GetParamLayout()) {
            if (strcmp(param.name, name) == 0) {
                // Check param.type and transform data accordingly. Incomming data is always string in ASCII format.
                // For now, we assume that the data is always in the correct format.
                if (param.type != ParamType::STRING)
                {
                    uint16_t neededLen = max(param.len, static_cast<uint16_t>(4)); // Always have a buffer of at least 4 bytes for the transformation (even if param is less than 4 bytes)
                    char dataTransformed[neededLen];        
                    if (Utils::TransformStrToData(param.type, (const char*)data, dataTransformed) != Utils::ErrorTransform::OK) {
                        return ErrorParam::INVALID_DATA;
                    }
                    m_EEPROM.writeBytes(param.address, dataTransformed, param.len); // Always write the lenght of the parameter
                    m_EEPROM.commit();
                } else {
                    // Never write more data than the EEPROM parameter lenght
                    if (len == param.len) {
                        m_EEPROM.writeBytes(param.address, data, len);
                        m_EEPROM.commit();
                    } else if (len < param.len) {   // We need to give space for the null char character
                        m_EEPROM.writeBytes(param.address, data, len+1);        // So we copy in the null character
                        m_EEPROM.commit();
                    } else {
                        return ErrorParam::PARAM_TOO_LONG;
                    }
                }
                return ErrorParam::OK;
            }
        }
        return ErrorParam::NAME_NOT_FOUND;
    }

    // TODO: Fix this. Should be direct call to EEPROM member instead of Read or Write function calls
    // Important! 
    virtual ErrorParam GetParam(const char* name, char* value, uint32_t& len, ParamType& type) override {
        for (const ParamDef& param : GetParamLayout()) {
            if (strcmp(param.name, name) == 0) {
                // Check param.type and transform data accordingly. Outgoing data should always be string in ASCII format.
                if (param.type != ParamType::STRING)
                {
                    char dataTransformed[32] = {};
                    m_EEPROM.readBytes(param.address, dataTransformed, param.len);
                    Utils::TransformDataToStr(param.type, dataTransformed, value);
                    len = min(strlen(value), static_cast<uint32_t>(32));
                } else {
                    len = m_EEPROM.readBytes(param.address, value, param.len);
                    len = min(len, strlen((const char*)value));
                }
                value[len] = '\0'; // Null terminate the string 
                type = param.type;
                if (!Utils::IsAscii(value, len)) {
                    // Fill buffer with zeros
                    memset(value, 0, len);
                }
                return ErrorParam::OK;
            }
        }
        return ErrorParam::NAME_NOT_FOUND;
    }

    virtual ErrorParam LoadParams() override {
        return ErrorParam::OK;
    }

    virtual ErrorParam SaveParams() override {
        return ErrorParam::OK;
    }

protected:
    void InitEeprom() {
        if (!m_EEPROM.begin(sizeof(TParams))) {
            LOG_ERROR("EEPROM init failed: %d bytes", sizeof(TParams));
        }
    }

protected:
    template<typename M, size_t size>
    bool WriteParam(etl::array<M,size> TParams::*member, const etl::array<M,size>& param) {
        size_t bytesWritten = m_EEPROM.writeBytes(Utils::offset_of(member), param.data(), size * sizeof(M));
        bool ret = m_EEPROM.commit();
        return bytesWritten == size * sizeof(M) && ret;
    }

    template <typename M>
    bool WriteParam(M TParams::*member, M param) {
        size_t bytesWritten = m_EEPROM.writeBytes(Utils::offset_of(member), &param, sizeof(M));
        bool ret = m_EEPROM.commit();
        return bytesWritten == sizeof(M) && ret;
    }

    template <typename M>
    bool ReadParam(M TParams::*member, M* param, uint32_t offset = 0) {
        size_t bytesRead = m_EEPROM.readBytes(Utils::offset_of(member) + offset, param, sizeof(M));
        return bytesRead == sizeof(M);
    }

    template <typename M, size_t size>
    bool ReadParam(etl::array<M,size> TParams::*member, etl::array<M,size>& param, uint32_t offset = 0) {
        size_t bytesRead = m_EEPROM.readBytes(Utils::offset_of(member), param.data(), size * sizeof(M));
        return bytesRead == size * sizeof(M);
    }

protected:
    EEPROMClass m_EEPROM;
};


// ************ Frontend instances ************
#include "uwb/uwb_frontend.hpp"
// Note: app_frontend_littlefs.hpp is not included here to avoid circular dependency
// Include it directly where needed (e.g., command_handler.cpp)