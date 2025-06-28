#include "utils.hpp"

namespace Utils {

// Utility function to transform data based on the specified ParamType
ErrorTransform TransformStrToData(const ParamType type, const char* inputData, void* outputData) {
    char* endPtr = nullptr; // Pointer to the next character after the numerical value

    switch (type) {
        case ParamType::ENUM:   // Enums are treated as uint types
        case ParamType::UINT32:
        case ParamType::UINT16:
        case ParamType::UINT8: {
            unsigned long val = std::strtoul(inputData, &endPtr, 10);
            if (*endPtr != '\0' || errno == ERANGE) {
                return ErrorTransform::CONVERSION_ERROR;
            }
            *static_cast<uint32_t*>(outputData) = static_cast<uint32_t>(val);
            break;
        }
        case ParamType::INT32:
        case ParamType::INT16:
        case ParamType::INT8: {
            long val = std::strtol(inputData, &endPtr, 10);
            if (*endPtr != '\0' || errno == ERANGE) {
                return ErrorTransform::CONVERSION_ERROR;
            }
            *static_cast<int32_t*>(outputData) = static_cast<int32_t>(val);
            break;
        }
        case ParamType::FLOAT: {
            float val = std::strtof(inputData, &endPtr);
            if (*endPtr != '\0' || errno == ERANGE) {
                return ErrorTransform::CONVERSION_ERROR;
            }
            *static_cast<float*>(outputData) = val;
            break;
        }
        case ParamType::DOUBLE: {
            double val = std::strtod(inputData, &endPtr);
            if (*endPtr != '\0' || errno == ERANGE) {
                return ErrorTransform::CONVERSION_ERROR;
            }
            *static_cast<double*>(outputData) = val;
            break;
        }
        case ParamType::BOOL: {
            if (strcmp(inputData, "true") == 0) {
                *static_cast<bool*>(outputData) = true;
            } else if (strcmp(inputData, "false") == 0) {
                *static_cast<bool*>(outputData) = false;
            } else {
                return ErrorTransform::INVALID_DATA;
            }
            break;
        }
        default:
            return ErrorTransform::INVALID_TYPE;
    }
    return ErrorTransform::OK;
}

ErrorTransform TransformDataToStr(const ParamType type, const void *inputData, char* outputData)
{
    switch (type) {
        case ParamType::ENUM:   // Enums are treated as uint types
        case ParamType::UINT32:
        case ParamType::UINT16:
        case ParamType::UINT8: {
            snprintf(outputData, 10, "%u", *static_cast<const uint32_t*>(inputData));
            break;
        }
        case ParamType::INT32:
        case ParamType::INT16:
        case ParamType::INT8: {
            snprintf(outputData, 10, "%d", *static_cast<const int32_t*>(inputData));
            break;
        }
        case ParamType::FLOAT: {
            snprintf(outputData, 10, "%f", *static_cast<const float*>(inputData));
            break;
        }
        case ParamType::DOUBLE: {
            snprintf(outputData, 10, "%f", *static_cast<const double*>(inputData));
            break;
        }
        case ParamType::BOOL: {
            snprintf(outputData, 10, "%s", *static_cast<const bool*>(inputData) ? "true" : "false");
            break;
        }
        default:
            return ErrorTransform::INVALID_TYPE;
    }
    return ErrorTransform::OK;      // @todo: Perform proper error handling
}

/**
 * @brief Get the Refresh rate in hz
 * 
 * @param last_update_ms 
 * @return uint32_t 
 */
uint32_t GetRefreshRate(uint32_t& last_update_ms)
{
    uint32_t current_time = millis();
    uint32_t refresh_rate_hz = 1000 / (current_time - last_update_ms);
    last_update_ms = current_time;  
    return refresh_rate_hz;
}

/**
 * @brief Check if a string is ASCII
 * 
 * @param str 
 * @return true 
 * @return false 
 */
bool IsAscii(const char* str, size_t len) {
    for (size_t i = 0; i < len; i++) {
        if (static_cast<unsigned char>(str[i]) > 127) {
            return false; // Non-ASCII character found
        }
    }
    return true; // All characters are ASCII
}


}