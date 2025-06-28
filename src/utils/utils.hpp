#pragma once

#define ULS_PACKED __attribute__((__packed__))

#include <cstdint>

#include <Arduino.h>

#include "application/param.hpp"

#include <etl/queue_lockable.h>

namespace Utils {

    template <typename T, size_t VSize, size_t VMemory_Model = etl::memory_model::MEMORY_MODEL_MEDIUM>
    class FreeRTOSLockQueue : public etl::queue_lockable<T, VSize, VMemory_Model> {
    public:
        FreeRTOSLockQueue() : etl::queue_lockable<T, VSize, VMemory_Model>() {
            this->lock_ = xSemaphoreCreateMutex();
        }

        virtual ~FreeRTOSLockQueue() {
            vSemaphoreDelete(this->lock_);
        }

        virtual void lock() const override {
            xSemaphoreTake(this->lock_, portMAX_DELAY) == pdTRUE;
        }

        virtual void unlock() const override {
            xSemaphoreGive(this->lock_) == pdTRUE;
        }

    private:
        SemaphoreHandle_t lock_;
    
    };


    template <typename T, typename M>
    constexpr size_t offset_of(M T::*member) {
        return reinterpret_cast<size_t>(&(((T*)0)->*member));
    }

    enum class ErrorTransform {
        OK = 0,
        INVALID_TYPE,
        INVALID_DATA,
        CONVERSION_ERROR,
    };

    // Utility function to transform data based on the specified ParamType
    ErrorTransform TransformStrToData(const ParamType type, const char* inputData, void* outputData);

    ErrorTransform TransformDataToStr(const ParamType type, const void* inputData, char* outputData);

    uint32_t GetRefreshRate(uint32_t& last_update_ms);

    bool IsAscii(const char* str, size_t len);


    

};