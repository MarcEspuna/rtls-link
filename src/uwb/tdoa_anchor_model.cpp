#include "config/features.hpp"

#ifdef USE_UWB_MODE_TDOA_TAG

#include "tdoa_anchor_model.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstring>

#include <LittleFS.h>

#include "logging/logging.hpp"
#include "tdoa_common.hpp"
#include "tdoa_pairs.hpp"

namespace {
constexpr const char* kPersistedModelPath = "/tdoa_anchor_model.bin";
constexpr uint32_t kPersistedModelMagic = 0x414F4454; // TDOA, little-endian.
constexpr uint16_t kPersistedModelSchema = 1;
constexpr uint8_t kPersistedModelPairCount = 6;

struct PersistedAnchorModel {
    uint32_t magic = kPersistedModelMagic;
    uint16_t schema = kPersistedModelSchema;
    uint8_t domain = 0;
    uint8_t pairCount = 0;
    uint32_t modelVersion = 0;
    uint16_t lockedTof[kPersistedModelPairCount] = {};
    uint16_t mad[kPersistedModelPairCount] = {};
    uint32_t checksum = 0;
};

uint32_t ModelChecksum(const PersistedAnchorModel& model)
{
    const auto* bytes = reinterpret_cast<const uint8_t*>(&model);
    const size_t checksumOffset = offsetof(PersistedAnchorModel, checksum);
    uint32_t hash = 2166136261u;
    for (size_t i = 0; i < checksumOffset; i++) {
        hash ^= bytes[i];
        hash *= 16777619u;
    }
    return hash;
}
}

TDoAAnchorModel::TDoAAnchorModel()
{
    m_mutex = xSemaphoreCreateMutex();

    for (uint8_t i = 0; i < kPairCount; i++) {
        const tdoa::AnchorPair pair = tdoa::PairByIndex<kAnchorCount>(i);
        m_pairs[i].a = pair.a;
        m_pairs[i].b = pair.b;
    }
}

void TDoAAnchorModel::Configure(const UWBParams& params)
{
    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    m_domain = (params.tdoaAnchorModelDomain == DOMAIN_PROPAGATION)
        ? DOMAIN_PROPAGATION
        : DOMAIN_RAW_EFFECTIVE;
    m_mode = (params.tdoaAnchorModelMode <= MODE_LOCKED_ANCHOR_MODEL)
        ? static_cast<Mode>(params.tdoaAnchorModelMode)
        : MODE_OFF;
    m_collectWindowMs = params.tdoaAnchorModelCollectWindowMs == 0 ? 10000 : params.tdoaAnchorModelCollectWindowMs;
    m_minSamplesPerPair = params.tdoaAnchorModelMinSamplesPerPair == 0 ? 20 : params.tdoaAnchorModelMinSamplesPerPair;

    if (params.tdoaAnchorModelStartupCollect != 0 && params.tdoaAnchorModelMode != MODE_OFF) {
        if (EnsureSampleStorageLocked()) {
            ClearSamplesLocked();
            ResetHealthLocked();
            m_collectState = COLLECT_ACTIVE;
            m_collectStartMs = 0;
            m_collectFirstSampleMs = 0;
            std::strncpy(m_lastError, "startup collection armed", sizeof(m_lastError) - 1);
        } else {
            m_collectState = COLLECT_FAILED;
            std::strncpy(m_lastError, "sample allocation failed", sizeof(m_lastError) - 1);
        }
    } else if (LoadPersistedModelLocked()) {
        std::strncpy(m_lastError, "persisted model loaded", sizeof(m_lastError) - 1);
    }

    xSemaphoreGive(m_mutex);
}

void TDoAAnchorModel::Reset()
{
    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    ClearSamplesLocked();
    ResetHealthLocked();
    m_collectState = COLLECT_IDLE;
    m_collectStartMs = 0;
    m_collectFirstSampleMs = 0;
    m_modelLocked = false;
    m_fallbackActive = false;
    m_modelPersisted = false;
    m_modelVersion++;
    m_lastError[0] = '\0';
    for (auto& pair : m_pairs) {
        pair.locked = false;
        pair.lockedTof = 0;
        pair.mad = 0;
    }
    ClearPersistedModelLocked();

    xSemaphoreGive(m_mutex);
}

bool TDoAAnchorModel::StartCollection(const UWBParams& params)
{
    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    m_domain = (params.tdoaAnchorModelDomain == DOMAIN_PROPAGATION)
        ? DOMAIN_PROPAGATION
        : DOMAIN_RAW_EFFECTIVE;
    m_mode = (params.tdoaAnchorModelMode <= MODE_LOCKED_ANCHOR_MODEL)
        ? static_cast<Mode>(params.tdoaAnchorModelMode)
        : MODE_OFF;
    m_collectWindowMs = params.tdoaAnchorModelCollectWindowMs == 0 ? 10000 : params.tdoaAnchorModelCollectWindowMs;
    m_minSamplesPerPair = params.tdoaAnchorModelMinSamplesPerPair == 0 ? 20 : params.tdoaAnchorModelMinSamplesPerPair;
    if (!EnsureSampleStorageLocked()) {
        m_collectState = COLLECT_FAILED;
        std::strncpy(m_lastError, "sample allocation failed", sizeof(m_lastError) - 1);
        xSemaphoreGive(m_mutex);
        return false;
    }
    ClearSamplesLocked();
    ResetHealthLocked();
    m_modelLocked = false;
    m_modelPersisted = false;
    m_collectState = COLLECT_ACTIVE;
    m_collectStartMs = 0;
    m_collectFirstSampleMs = 0;
    m_fallbackActive = false;
    for (auto& pair : m_pairs) {
        pair.locked = false;
        pair.lockedTof = 0;
        pair.mad = 0;
    }
    std::strncpy(m_lastError, "collection active", sizeof(m_lastError) - 1);

    xSemaphoreGive(m_mutex);
    return true;
}

bool TDoAAnchorModel::Lock(const UWBParams& params)
{
    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    bool ok = LockLocked(params);
    xSemaphoreGive(m_mutex);
    return ok;
}

bool TDoAAnchorModel::ProcessInterAnchorTof(uint8_t fromAnchor,
                                            uint8_t toAnchor,
                                            uint16_t rawDistanceTimestampUnits,
                                            uint16_t fromAntennaDelay,
                                            uint16_t toAntennaDelay,
                                            uint16_t* outDistanceTimestampUnits,
                                            const UWBParams& params)
{
    if (outDistanceTimestampUnits == nullptr) {
        return false;
    }

    uint8_t pairIndex = 0;
    bool reversed = false;
    if (!FindPair(fromAnchor, toAnchor, pairIndex, reversed)) {
        return false;
    }

    const Mode mode = (params.tdoaAnchorModelMode <= MODE_LOCKED_ANCHOR_MODEL)
        ? static_cast<Mode>(params.tdoaAnchorModelMode)
        : MODE_OFF;
    const Domain domain = (params.tdoaAnchorModelDomain == DOMAIN_PROPAGATION)
        ? DOMAIN_PROPAGATION
        : DOMAIN_RAW_EFFECTIVE;
    const uint16_t domainValue = DomainValue(domain, rawDistanceTimestampUnits, fromAntennaDelay, toAntennaDelay);

    bool overrideTof = false;

    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    PairState& pair = m_pairs[pairIndex];
    m_domain = domain;
    m_mode = mode;

    if (m_collectState == COLLECT_ACTIVE) {
        const uint32_t now = millis();
        if (m_collectFirstSampleMs == 0) {
            m_collectFirstSampleMs = now;
            m_collectStartMs = now;
        }

        uint16_t* pairSamples = SamplesForPairLocked(pairIndex);
        if (pairSamples != nullptr && pair.sampleCount < kMaxSamplesPerPair) {
            pairSamples[pair.sampleCount++] = domainValue;
        }
        pair.totalSamples++;

        const uint32_t elapsed = now - m_collectStartMs;
        if (elapsed >= (params.tdoaAnchorModelCollectWindowMs == 0 ? m_collectWindowMs : params.tdoaAnchorModelCollectWindowMs)) {
            if (LockLocked(params)) {
                m_collectState = COLLECT_DONE;
            } else {
                m_collectState = COLLECT_FAILED;
            }
        }
    }

    if (m_modelLocked && pair.locked && mode != MODE_OFF) {
        const int32_t residual = static_cast<int32_t>(domainValue) - static_cast<int32_t>(pair.lockedTof);
        const uint32_t absResidual = static_cast<uint32_t>(residual < 0 ? -residual : residual);
        pair.residualCount++;
        pair.residualAbsMax = std::max(pair.residualAbsMax, absResidual);
        const uint16_t healthThreshold = params.tdoaAnchorModelHealthThresholdTicks == 0
            ? 250
            : params.tdoaAnchorModelHealthThresholdTicks;
        if (absResidual > healthThreshold) {
            pair.residualBad++;
        }

        const uint16_t healthWindow = params.tdoaAnchorModelHealthWindow == 0 ? 50 : params.tdoaAnchorModelHealthWindow;
        if (pair.residualCount >= healthWindow) {
            pair.healthy = pair.residualBad == 0;
            pair.residualCount = 0;
            pair.residualBad = 0;
            pair.residualAbsMax = 0;
        }

        if (mode == MODE_LOCKED_ANCHOR_MODEL && pair.healthy) {
            uint32_t outputTof = pair.lockedTof;
            if (domain == DOMAIN_PROPAGATION) {
                outputTof += static_cast<uint32_t>(fromAntennaDelay) + static_cast<uint32_t>(toAntennaDelay);
            }
            if (outputTof > UINT16_MAX) {
                outputTof = UINT16_MAX;
            }
            *outDistanceTimestampUnits = static_cast<uint16_t>(outputTof);
            overrideTof = true;
        }
    }

    const uint8_t requestedHealthQuorum = params.tdoaAnchorModelHealthQuorum == 0 ? 5 : params.tdoaAnchorModelHealthQuorum;
    const uint8_t healthQuorum = std::min<uint8_t>(requestedHealthQuorum, kPairCount);
    m_fallbackActive = mode == MODE_LOCKED_ANCHOR_MODEL && (!m_modelLocked || HealthyLockedPairCountLocked() < healthQuorum);
    xSemaphoreGive(m_mutex);

    (void)reversed;
    return overrideTof;
}

String TDoAAnchorModel::StatusJson() const
{
    String out;
    out.reserve(900);

    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return "{\"error\":\"anchor model mutex unavailable\"}";
    }

    out = "{\"mode\":\"";
    out += ModeName(m_mode);
    out += "\",\"locked\":";
    out += (m_modelLocked ? "true" : "false");
    out += ",\"domain\":\"";
    out += DomainName(m_domain);
    out += "\",\"version\":";
    out += String(m_modelVersion);
    out += ",\"persisted\":";
    out += (m_modelPersisted ? "true" : "false");
    out += ",\"collectState\":";
    out += String(static_cast<unsigned int>(m_collectState));
    out += ",\"fallbackActive\":";
    out += (m_fallbackActive ? "true" : "false");
    out += ",\"healthyPairs\":";
    out += String(static_cast<unsigned int>(HealthyLockedPairCountLocked()));
    out += ",\"lastError\":\"";
    out += m_lastError;
    out += "\",\"pairs\":[";
    for (uint8_t i = 0; i < kPairCount; i++) {
        if (i > 0) out += ",";
        AppendPairJson(out, m_pairs[i], false);
    }
    out += "]}";

    xSemaphoreGive(m_mutex);
    return out;
}

String TDoAAnchorModel::ExportJson() const
{
    String out;
    out.reserve(700);

    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return "{\"error\":\"anchor model mutex unavailable\"}";
    }

    out = "{\"version\":";
    out += String(m_modelVersion);
    out += ",\"persisted\":";
    out += (m_modelPersisted ? "true" : "false");
    out += ",\"domain\":\"";
    out += DomainName(m_domain);
    out += "\",\"locked\":";
    out += (m_modelLocked ? "true" : "false");
    out += ",\"pairs\":[";
    for (uint8_t i = 0; i < kPairCount; i++) {
        if (i > 0) out += ",";
        AppendPairJson(out, m_pairs[i], false);
    }
    out += "]}";

    xSemaphoreGive(m_mutex);
    return out;
}

String TDoAAnchorModel::CollectStatusJson() const
{
    String out;
    out.reserve(700);

    if (xSemaphoreTake(m_mutex, portMAX_DELAY) != pdTRUE) {
        return "{\"error\":\"anchor model mutex unavailable\"}";
    }

    uint32_t elapsed = 0;
    if (m_collectState == COLLECT_ACTIVE && m_collectStartMs != 0) {
        elapsed = millis() - m_collectStartMs;
    }

    out = "{\"state\":";
    out += String(static_cast<unsigned int>(m_collectState));
    out += ",\"elapsedMs\":";
    out += String(elapsed);
    out += ",\"windowMs\":";
    out += String(m_collectWindowMs);
    out += ",\"minSamplesPerPair\":";
    out += String(static_cast<unsigned int>(m_minSamplesPerPair));
    out += ",\"domain\":\"";
    out += DomainName(m_domain);
    out += "\",\"pairs\":[";
    for (uint8_t i = 0; i < kPairCount; i++) {
        if (i > 0) out += ",";
        AppendPairJson(out, m_pairs[i], false);
    }
    out += "]}";

    xSemaphoreGive(m_mutex);
    return out;
}

bool TDoAAnchorModel::FindPair(uint8_t a, uint8_t b, uint8_t& index, bool& reversed)
{
    tdoa::AnchorPair pair;
    if (!tdoa::CanonicalizePair(a, b, kAnchorCount, pair, reversed)) {
        return false;
    }

    const uint8_t pairIndex = tdoa::PairIndexCanonical(pair, kAnchorCount);
    if (pairIndex >= kPairCount) {
        return false;
    }
    index = pairIndex;
    return true;
}

uint16_t TDoAAnchorModel::DomainValue(Domain domain, uint16_t rawDistanceTimestampUnits, uint16_t fromAntennaDelay, uint16_t toAntennaDelay)
{
    if (domain == DOMAIN_RAW_EFFECTIVE) {
        return rawDistanceTimestampUnits;
    }

    const int32_t corrected = static_cast<int32_t>(rawDistanceTimestampUnits)
        - static_cast<int32_t>(fromAntennaDelay)
        - static_cast<int32_t>(toAntennaDelay);
    return corrected <= 0 ? 0 : static_cast<uint16_t>(corrected);
}

uint16_t TDoAAnchorModel::RobustEstimate(const uint16_t* samples, uint16_t count, uint16_t& outMad)
{
    uint16_t sorted[kMaxSamplesPerPair] = {};
    uint16_t deviations[kMaxSamplesPerPair] = {};
    for (uint16_t i = 0; i < count; i++) {
        sorted[i] = samples[i];
    }
    std::sort(sorted, sorted + count);

    const uint16_t median = sorted[count / 2];
    for (uint16_t i = 0; i < count; i++) {
        const int32_t deviation = static_cast<int32_t>(samples[i]) - static_cast<int32_t>(median);
        deviations[i] = static_cast<uint16_t>(deviation < 0 ? -deviation : deviation);
    }
    std::sort(deviations, deviations + count);
    outMad = deviations[count / 2];

    const float robustSigma = std::max(1.0f, 1.4826f * static_cast<float>(outMad));
    const float huberK = 1.5f;
    float weightedSum = 0.0f;
    float weightSum = 0.0f;
    for (uint16_t i = 0; i < count; i++) {
        const float residual = std::fabs(static_cast<float>(samples[i]) - static_cast<float>(median));
        const float weight = residual <= huberK * robustSigma
            ? 1.0f
            : (huberK * robustSigma) / residual;
        weightedSum += weight * static_cast<float>(samples[i]);
        weightSum += weight;
    }

    if (weightSum <= 0.0f) {
        return median;
    }
    return static_cast<uint16_t>((weightedSum / weightSum) + 0.5f);
}

const char* TDoAAnchorModel::ModeName(Mode mode)
{
    switch (mode) {
        case MODE_MONITOR: return "MONITOR";
        case MODE_LOCKED_ANCHOR_MODEL: return "LOCKED_ANCHOR_MODEL";
        case MODE_OFF:
        default: return "OFF";
    }
}

const char* TDoAAnchorModel::DomainName(Domain domain)
{
    switch (domain) {
        case DOMAIN_PROPAGATION: return "PROPAGATION";
        case DOMAIN_RAW_EFFECTIVE:
        default: return "RAW_EFFECTIVE";
    }
}

void TDoAAnchorModel::ClearSamplesLocked()
{
    for (auto& pair : m_pairs) {
        pair.sampleCount = 0;
        pair.totalSamples = 0;
    }
}

void TDoAAnchorModel::ResetHealthLocked()
{
    for (auto& pair : m_pairs) {
        pair.healthy = true;
        pair.residualCount = 0;
        pair.residualBad = 0;
        pair.residualAbsMax = 0;
    }
}

bool TDoAAnchorModel::LockLocked(const UWBParams& params)
{
    m_minSamplesPerPair = params.tdoaAnchorModelMinSamplesPerPair == 0 ? 20 : params.tdoaAnchorModelMinSamplesPerPair;

    for (const auto& pair : m_pairs) {
        if (pair.sampleCount < m_minSamplesPerPair) {
            std::snprintf(m_lastError, sizeof(m_lastError), "pair %u%u has %u samples",
                          pair.a, pair.b, static_cast<unsigned int>(pair.sampleCount));
            m_modelLocked = false;
            m_modelPersisted = false;
            return false;
        }
    }

    for (auto& pair : m_pairs) {
        uint8_t pairIndex = 0;
        bool reversed = false;
        if (!FindPair(pair.a, pair.b, pairIndex, reversed)) {
            m_modelLocked = false;
            m_modelPersisted = false;
            std::strncpy(m_lastError, "invalid pair table", sizeof(m_lastError) - 1);
            return false;
        }

        const uint16_t* pairSamples = SamplesForPairLocked(pairIndex);
        if (pairSamples == nullptr) {
            m_modelLocked = false;
            m_modelPersisted = false;
            std::strncpy(m_lastError, "sample allocation missing", sizeof(m_lastError) - 1);
            return false;
        }

        pair.lockedTof = RobustEstimate(pairSamples, pair.sampleCount, pair.mad);
        pair.locked = true;
    }

    ResetHealthLocked();
    m_modelLocked = true;
    m_fallbackActive = false;
    m_collectState = COLLECT_DONE;
    m_modelVersion++;
    m_modelPersisted = PersistModelLocked();
    std::strncpy(m_lastError,
                 m_modelPersisted ? "model locked" : "model locked, persist failed",
                 sizeof(m_lastError) - 1);
    LOG_INFO("TDoA anchor model locked v%u domain=%s", m_modelVersion, DomainName(m_domain));
    return true;
}

bool TDoAAnchorModel::LoadPersistedModelLocked()
{
    File file = LittleFS.open(kPersistedModelPath, "r");
    if (!file) {
        m_modelPersisted = false;
        return false;
    }

    PersistedAnchorModel stored = {};
    const size_t bytesRead = file.read(reinterpret_cast<uint8_t*>(&stored), sizeof(stored));
    file.close();

    if (bytesRead != sizeof(stored)
        || stored.magic != kPersistedModelMagic
        || stored.schema != kPersistedModelSchema
        || stored.pairCount != kPairCount
        || stored.domain > DOMAIN_PROPAGATION
        || stored.checksum != ModelChecksum(stored)) {
        m_modelPersisted = false;
        std::strncpy(m_lastError, "persisted model invalid", sizeof(m_lastError) - 1);
        return false;
    }

    const Domain storedDomain = static_cast<Domain>(stored.domain);
    if (storedDomain != m_domain) {
        m_modelPersisted = false;
        std::strncpy(m_lastError, "persisted model domain mismatch", sizeof(m_lastError) - 1);
        return false;
    }

    for (uint8_t i = 0; i < kPairCount; i++) {
        m_pairs[i].lockedTof = stored.lockedTof[i];
        m_pairs[i].mad = stored.mad[i];
        m_pairs[i].locked = true;
    }

    ResetHealthLocked();
    m_modelLocked = true;
    m_fallbackActive = false;
    m_collectState = COLLECT_DONE;
    m_modelVersion = stored.modelVersion;
    m_modelPersisted = true;
    LOG_INFO("TDoA anchor model loaded v%u domain=%s", m_modelVersion, DomainName(m_domain));
    return true;
}

bool TDoAAnchorModel::PersistModelLocked() const
{
    if (!HasLockedModelLocked()) {
        return false;
    }

    PersistedAnchorModel stored = {};
    stored.domain = static_cast<uint8_t>(m_domain);
    stored.pairCount = kPairCount;
    stored.modelVersion = m_modelVersion;
    for (uint8_t i = 0; i < kPairCount; i++) {
        stored.lockedTof[i] = m_pairs[i].lockedTof;
        stored.mad[i] = m_pairs[i].mad;
    }
    stored.checksum = ModelChecksum(stored);

    File file = LittleFS.open(kPersistedModelPath, "w");
    if (!file) {
        LOG_WARN("TDoA anchor model persist open failed");
        return false;
    }

    const size_t bytesWritten = file.write(reinterpret_cast<const uint8_t*>(&stored), sizeof(stored));
    file.close();

    if (bytesWritten != sizeof(stored)) {
        LOG_WARN("TDoA anchor model persist write failed");
        LittleFS.remove(kPersistedModelPath);
        return false;
    }
    return true;
}

void TDoAAnchorModel::ClearPersistedModelLocked()
{
    LittleFS.remove(kPersistedModelPath);
}

bool TDoAAnchorModel::HasLockedModelLocked() const
{
    if (!m_modelLocked) {
        return false;
    }
    for (const auto& pair : m_pairs) {
        if (!pair.locked) {
            return false;
        }
    }
    return true;
}

uint8_t TDoAAnchorModel::HealthyLockedPairCountLocked() const
{
    uint8_t count = 0;
    for (const auto& pair : m_pairs) {
        if (pair.locked && pair.healthy) {
            count++;
        }
    }
    return count;
}

bool TDoAAnchorModel::EnsureSampleStorageLocked()
{
    if (m_samples != nullptr) {
        return true;
    }

    m_samples = new uint16_t[static_cast<size_t>(kPairCount) * kMaxSamplesPerPair]();
    return m_samples != nullptr;
}

uint16_t* TDoAAnchorModel::SamplesForPairLocked(uint8_t pairIndex) const
{
    if (m_samples == nullptr || pairIndex >= kPairCount) {
        return nullptr;
    }

    return &m_samples[static_cast<size_t>(pairIndex) * kMaxSamplesPerPair];
}

void TDoAAnchorModel::AppendPairJson(String& out, const PairState& pair, bool includeSamples) const
{
    out += "{\"pair\":\"";
    out += String(static_cast<unsigned int>(pair.a));
    out += String(static_cast<unsigned int>(pair.b));
    out += "\",\"samples\":";
    out += String(static_cast<unsigned int>(pair.sampleCount));
    out += ",\"total\":";
    out += String(pair.totalSamples);
    out += ",\"locked\":";
    out += (pair.locked ? String(static_cast<unsigned int>(pair.lockedTof)) : String("null"));
    out += ",\"mad\":";
    out += String(static_cast<unsigned int>(pair.mad));
    out += ",\"healthy\":";
    out += (pair.healthy ? "true" : "false");
    out += ",\"residualMax\":";
    out += String(pair.residualAbsMax);

    if (includeSamples) {
        uint8_t pairIndex = 0;
        bool reversed = false;
        uint16_t* pairSamples = nullptr;
        if (FindPair(pair.a, pair.b, pairIndex, reversed)) {
            pairSamples = SamplesForPairLocked(pairIndex);
        }

        out += ",\"values\":[";
        for (uint16_t i = 0; pairSamples != nullptr && i < pair.sampleCount; i++) {
            if (i > 0) out += ",";
            out += String(static_cast<unsigned int>(pairSamples[i]));
        }
        out += "]";
    }
    out += "}";
}

#endif // USE_UWB_MODE_TDOA_TAG
