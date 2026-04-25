#pragma once

#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <stdint.h>

#include "uwb_params.hpp"

class TDoAAnchorModel {
public:
    enum Mode : uint8_t {
        MODE_OFF = 0,
        MODE_MONITOR = 1,
        MODE_LOCKED_ANCHOR_MODEL = 2
    };

    enum Domain : uint8_t {
        DOMAIN_RAW_EFFECTIVE = 0,
        DOMAIN_PROPAGATION = 1
    };

    TDoAAnchorModel();

    void Configure(const UWBParams& params);
    void Reset();
    bool StartCollection(const UWBParams& params);
    bool Lock(const UWBParams& params);

    bool ProcessInterAnchorTof(uint8_t fromAnchor,
                               uint8_t toAnchor,
                               uint16_t rawDistanceTimestampUnits,
                               uint16_t fromAntennaDelay,
                               uint16_t toAntennaDelay,
                               uint16_t* outDistanceTimestampUnits,
                               const UWBParams& params);

    String StatusJson() const;
    String ExportJson() const;
    String CollectStatusJson() const;

private:
    static constexpr uint8_t kAnchorCount = 4;
    static constexpr uint8_t kPairCount = 6;
    static constexpr uint16_t kMaxSamplesPerPair = 160;

    struct PairState {
        uint8_t a = 0;
        uint8_t b = 0;
        uint16_t sampleCount = 0;
        uint32_t totalSamples = 0;
        uint16_t lockedTof = 0;
        uint16_t mad = 0;
        bool locked = false;
        bool healthy = true;
        uint16_t residualCount = 0;
        uint16_t residualBad = 0;
        uint32_t residualAbsMax = 0;
    };

    enum CollectState : uint8_t {
        COLLECT_IDLE = 0,
        COLLECT_ACTIVE = 1,
        COLLECT_DONE = 2,
        COLLECT_FAILED = 3
    };

    static bool FindPair(uint8_t a, uint8_t b, uint8_t& index, bool& reversed);
    static uint16_t DomainValue(Domain domain, uint16_t rawDistanceTimestampUnits, uint16_t fromAntennaDelay, uint16_t toAntennaDelay);
    static uint16_t RobustEstimate(const uint16_t* samples, uint16_t count, uint16_t& outMad);
    static const char* ModeName(Mode mode);
    static const char* DomainName(Domain domain);

    void ClearSamplesLocked();
    void ResetHealthLocked();
    bool LockLocked(const UWBParams& params);
    bool LoadPersistedModelLocked();
    bool PersistModelLocked() const;
    void ClearPersistedModelLocked();
    bool HasLockedModelLocked() const;
    uint8_t HealthyLockedPairCountLocked() const;
    bool EnsureSampleStorageLocked();
    uint16_t* SamplesForPairLocked(uint8_t pairIndex) const;
    void AppendPairJson(String& out, const PairState& pair, bool includeSamples) const;

    mutable SemaphoreHandle_t m_mutex = nullptr;
    PairState m_pairs[kPairCount];
    CollectState m_collectState = COLLECT_IDLE;
    uint32_t m_collectWindowMs = 10000;
    uint32_t m_collectStartMs = 0;
    uint32_t m_collectFirstSampleMs = 0;
    uint16_t m_minSamplesPerPair = 20;
    Domain m_domain = DOMAIN_RAW_EFFECTIVE;
    bool m_modelLocked = false;
    bool m_fallbackActive = false;
    char m_lastError[48] = {};
    uint32_t m_modelVersion = 0;
    Mode m_mode = MODE_OFF;
    uint16_t* m_samples = nullptr;
    bool m_modelPersisted = false;
};

#endif // USE_UWB_MODE_TDOA_TAG
