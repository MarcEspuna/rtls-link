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

template<typename TParams>
class LittleFSFrontend : public IFrontend {
public:
    LittleFSFrontend(const char* groupName) : m_GroupName(groupName) {
        Front::AddFrontend(this);
        printf("----- LittleFSFrontend %s created -----\n", groupName);
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
                return ErrorParam::OK;
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
        if (!LittleFS.begin()) {
            printf("Failed to initialize LittleFS\n");
            return ErrorParam::FILE_SYSTEM_ERROR;
        }

        File file = LittleFS.open("/params.txt", "r");
        if (!file) {
            printf("No params.txt file found, using defaults for group %s\n", m_GroupName.data());
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
                        char dataTransformed[param.len];
                        if (Utils::TransformStrToData(param.type, value.c_str(), dataTransformed) == Utils::ErrorTransform::OK) {
                            memcpy(reinterpret_cast<char*>(&m_Params) + param.address, dataTransformed, param.len);
                        }
                    }
                    break;
                }
            }
        }
        
        file.close();
        printf("Loaded parameters for group %s\n", m_GroupName.data());
        return ErrorParam::OK;
    }

    virtual ErrorParam SaveParams() override {
        if (!LittleFS.begin()) {
            printf("Failed to initialize LittleFS\n");
            return ErrorParam::FILE_SYSTEM_ERROR;
        }

        // Read existing file content
        etl::string<2048> fileContent;
        File file = LittleFS.open("/params.txt", "r");
        if (file) {
            while (file.available()) {
                String line = file.readStringUntil('\n');
                line += "\n";
                
                // Check if this line belongs to our group
                etl::string<64> groupPrefix;
                groupPrefix.assign(m_GroupName.data());
                groupPrefix += ".";
                if (!line.startsWith(groupPrefix.c_str())) {
                    // Keep lines from other groups
                    if (fileContent.size() + line.length() < fileContent.capacity()) {
                        fileContent += line.c_str();
                    }
                }
            }
            file.close();
        }

        // Add our group's parameters
        for (const ParamDef& param : GetParamLayout()) {
            etl::string<128> paramLine;
            paramLine.assign(m_GroupName.data());
            paramLine += ".";
            paramLine += param.name;
            paramLine += ": ";
            
            if (param.type == ParamType::STRING) {
                const char* str_param = reinterpret_cast<const char*>(&m_Params) + param.address;
                paramLine += str_param;
            } else {
                char dataTransformed[32] = {};
                memcpy(dataTransformed, reinterpret_cast<char*>(&m_Params) + param.address, param.len);
                
                char str_val[32];
                Utils::TransformDataToStr(param.type, dataTransformed, str_val);
                paramLine += str_val;
            }
            paramLine += "\n";
            
            if (fileContent.size() + paramLine.size() < fileContent.capacity()) {
                fileContent += paramLine.c_str();
            }
        }

        // Write the updated content back
        file = LittleFS.open("/params.txt", "w");
        if (!file) {
            printf("Failed to open params.txt for writing\n");
            return ErrorParam::FILE_SYSTEM_ERROR;
        }

        file.print(fileContent.c_str());
        file.close();

        printf("Saved parameters for group %s\n", m_GroupName.data());
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