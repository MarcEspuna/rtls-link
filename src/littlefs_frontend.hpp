#pragma once

#include <Arduino.h>
#include <LittleFS.h>

#include <etl/span.h>
#include <etl/string_view.h>
#include <etl/vector.h>
#include <etl/string.h>

#include "utils/utils.hpp"
#include "param.hpp"
#include "front.hpp"
#include "logging/logging.hpp"

template<typename TParams>
class LittleFSFrontend : public IFrontend {
public:
    LittleFSFrontend(const char* groupName) : m_GroupName(groupName) {
        Front::AddFrontend(this);
        LOG_INFO("LittleFS frontend '%s' created", groupName);
    }

    void Init() override {
        LoadParams();
    }

    void Update() override {}

    virtual ErrorParam SetParam(const char* name, const void* data, uint32_t len) override {
        for (const ParamDef& param : GetParamLayout()) {
            if (strcmp(param.name, name) == 0) {
                if (param.type != ParamType::STRING) {
                    uint16_t neededLen = max(param.len, static_cast<uint16_t>(4));
                    char dataTransformed[neededLen];        
                    if (Utils::TransformStrToData(param.type, (const char*)data, dataTransformed) != Utils::ErrorTransform::OK) {
                        return ErrorParam::INVALID_DATA;
                    }
                    memcpy(reinterpret_cast<char*>(&m_Params) + param.address, dataTransformed, param.len);
                } else {
                    if (len == param.len) {
                        memcpy(reinterpret_cast<char*>(&m_Params) + param.address, data, len);
                    } else if (len < param.len) {
                        memcpy(reinterpret_cast<char*>(&m_Params) + param.address, data, len);
                        reinterpret_cast<char*>(&m_Params)[param.address + len] = '\0';
                    } else {
                        return ErrorParam::PARAM_TOO_LONG;
                    }
                }
                return SaveParams();
            }
        }
        return ErrorParam::NAME_NOT_FOUND;
    }

    virtual ErrorParam GetParam(const char* name, char* value, uint32_t& len, ParamType& type) override {
        for (const ParamDef& param : GetParamLayout()) {
            if (strcmp(param.name, name) == 0) {
                if (param.type != ParamType::STRING) {
                    char dataTransformed[32] = {};
                    memcpy(dataTransformed, reinterpret_cast<char*>(&m_Params) + param.address, param.len);
                    Utils::TransformDataToStr(param.type, dataTransformed, value);
                    len = min(strlen(value), static_cast<uint32_t>(32));
                } else {
                    const char* str_param = reinterpret_cast<const char*>(&m_Params) + param.address;
                    len = min(strlen(str_param), static_cast<uint32_t>(param.len));
                    strncpy(value, str_param, len);
                }
                value[len] = '\0';
                type = param.type;
                if (!Utils::IsAscii(value, len)) {
                    memset(value, 0, len);
                }
                return ErrorParam::OK;
            }
        }
        return ErrorParam::NAME_NOT_FOUND;
    }

    virtual ErrorParam LoadParams() override {
        static bool initialized = false;
        if (!initialized) {
            if (!LittleFS.begin(true)) {
                LOG_ERROR("Failed to initialize LittleFS");
                return ErrorParam::FILE_SYSTEM_ERROR;
            }
            initialized = true;
        }

        File file = LittleFS.open("/params.txt", "r");
        if (!file) {
            LOG_INFO("No params.txt - using defaults for %s", m_GroupName.data());
            return ErrorParam::FILE_NOT_FOUND;
        }

        etl::string<64> groupPrefix;
        groupPrefix.assign(m_GroupName.data());
        groupPrefix += ".";
        
        while (file.available()) {
            String line = file.readStringUntil('\n');
            line.trim();
            
            // Skip empty lines and comments
            if (line.length() == 0 || line.startsWith("#")) {
                continue;
            }
            
            // Find the colon separator
            int colonIndex = line.indexOf(':');
            if (colonIndex == -1) {
                continue;
            }
            
            String key = line.substring(0, colonIndex);
            String value = line.substring(colonIndex + 1);
            key.trim();
            value.trim();
            
            // Check if this parameter belongs to our group
            if (!key.startsWith(groupPrefix.c_str())) {
                continue;
            }
            
            // Extract parameter name (remove group prefix)
            String paramName = key.substring(groupPrefix.size());
            
            // Find matching parameter definition
            for (const ParamDef& param : GetParamLayout()) {
                if (strcmp(param.name, paramName.c_str()) == 0) {
                    if (param.type == ParamType::STRING) {
                        strncpy(reinterpret_cast<char*>(&m_Params) + param.address, value.c_str(), param.len - 1);
                        reinterpret_cast<char*>(&m_Params)[param.address + param.len - 1] = '\0';
                    } else {
                        uint16_t neededLen = max(param.len, static_cast<uint16_t>(4));
                        char dataTransformed[neededLen];
                        if (Utils::TransformStrToData(param.type, value.c_str(), dataTransformed) == Utils::ErrorTransform::OK) {
                            memcpy(reinterpret_cast<char*>(&m_Params) + param.address, dataTransformed, param.len);
                        }
                    }
                    break;
                }
            }
        }
        
        file.close();
        LOG_DEBUG("Loaded params for %s", m_GroupName.data());
        return ErrorParam::OK;
    }

    virtual ErrorParam SaveParams() override {
        static bool initialized = false;
        if (!initialized) {
            if (!LittleFS.begin()) {
                LOG_ERROR("Failed to initialize LittleFS");
                return ErrorParam::FILE_SYSTEM_ERROR;
            }
            initialized = true;
        }

        // Open original file for reading and temp file for writing
        File file = LittleFS.open("/params.txt", "r");
        File tempFile = LittleFS.open("/params.tmp", "w");

        if (!tempFile) {
            LOG_ERROR("Failed to open temp file");
            if (file) file.close();
            return ErrorParam::FILE_SYSTEM_ERROR;
        }

        etl::string<64> groupPrefix;
        groupPrefix.assign(m_GroupName.data());
        groupPrefix += ".";

        // Copy lines from other groups to temp file
        if (file) {
            while (file.available()) {
                String line = file.readStringUntil('\n');
                line.trim();
                
                if (line.length() == 0) continue;

                if (!line.startsWith(groupPrefix.c_str())) {
                    tempFile.println(line);
                }
            }
            file.close();
        }

        // Write our group's parameters
        for (const ParamDef& param : GetParamLayout()) {
            etl::string<128> paramLine;
            paramLine.assign(m_GroupName.data());
            paramLine += ".";
            paramLine += param.name;
            paramLine += ": ";
            
            if (param.type == ParamType::STRING) {
                const char* str_param = reinterpret_cast<const char*>(&m_Params) + param.address;
                // Ensure we don't read past the buffer if it's not null-terminated within len
                size_t actualLen = 0;
                while (actualLen < param.len && str_param[actualLen] != '\0') {
                    actualLen++;
                }
                paramLine.append(str_param, actualLen);
            } else {
                char dataTransformed[32] = {};
                memcpy(dataTransformed, reinterpret_cast<char*>(&m_Params) + param.address, param.len);
                
                char str_val[32];
                Utils::TransformDataToStr(param.type, dataTransformed, str_val);
                paramLine += str_val;
            }
            
            tempFile.println(paramLine.c_str());
        }

        tempFile.close();

        // Replace original file with temp file
        LittleFS.remove("/params.txt");
        if (!LittleFS.rename("/params.tmp", "/params.txt")) {
            LOG_ERROR("Failed to rename temp file");
            return ErrorParam::FILE_SYSTEM_ERROR;
        }

        LOG_DEBUG("Saved params for %s", m_GroupName.data());
        return ErrorParam::OK;
    }

protected:
    template<typename M, size_t size>
    bool WriteParam(etl::array<M,size> TParams::*member, const etl::array<M,size>& param) {
        memcpy(reinterpret_cast<char*>(&m_Params) + Utils::offset_of(member), param.data(), size * sizeof(M));
        return true;
    }

    template <typename M>
    bool WriteParam(M TParams::*member, M param) {
        memcpy(reinterpret_cast<char*>(&m_Params) + Utils::offset_of(member), &param, sizeof(M));
        return true;
    }

    template <typename M>
    bool ReadParam(M TParams::*member, M* param, uint32_t offset = 0) {
        memcpy(param, reinterpret_cast<char*>(&m_Params) + Utils::offset_of(member) + offset, sizeof(M));
        return true;
    }

    template <typename M, size_t size>
    bool ReadParam(etl::array<M,size> TParams::*member, etl::array<M,size>& param, uint32_t offset = 0) {
        memcpy(param.data(), reinterpret_cast<char*>(&m_Params) + Utils::offset_of(member), size * sizeof(M));
        return true;
    }

protected:
    TParams m_Params;
    etl::string_view m_GroupName;
};