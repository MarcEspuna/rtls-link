/**
 * @file dynamicAnchorPositions.cpp
 * @brief Implementation of dynamic anchor position calculation
 */

#include "dynamicAnchorPositions.hpp"
#include <cstring>

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char* TAG = "DynAnchorPos";
#endif

void DynamicAnchorPositionCalculator::init(const DynamicAnchorConfig& config) {
    m_config = config;

    // Validate and clamp configuration parameters
    if (m_config.avgSampleCount < 1) {
        m_config.avgSampleCount = 1;
    }
    if (m_config.avgSampleCount > 1000) {
        m_config.avgSampleCount = 1000;
    }
    if (m_config.anchorHeight < 0) {
        m_config.anchorHeight = 0;
    }

    reset();
}

void DynamicAnchorPositionCalculator::reset() {
    // Reset all accumulators
    for (uint8_t i = 0; i < MAX_DYNAMIC_ANCHORS; i++) {
        for (uint8_t j = 0; j < MAX_DYNAMIC_ANCHORS; j++) {
            m_accumulators[i][j].reset();
            m_averagedDistances[i][j] = 0.0f;
        }
        m_validDistanceMask[i] = 0;
        m_lockedPositions[i] = {0, 0, 0, 0};
        m_lastCalculatedPositions[i] = {0, 0, 0, 0};
    }
}

void DynamicAnchorPositionCalculator::updateDistance(uint8_t fromAnchor, uint8_t toAnchor, float distanceMeters) {
    // Validate inputs
    if (fromAnchor >= MAX_DYNAMIC_ANCHORS || toAnchor >= MAX_DYNAMIC_ANCHORS || fromAnchor == toAnchor) {
        return;
    }

    // Filter out invalid distances
    if (distanceMeters <= 0.0f || !std::isfinite(distanceMeters)) {
        return;
    }

    // Get current timestamp (in ticks from FreeRTOS)
#ifdef ESP_PLATFORM
    uint32_t now = xTaskGetTickCount();
#else
    uint32_t now = 0;  // Fallback for native testing
#endif

    // Add to accumulators (symmetric - distance A->B == B->A)
    m_accumulators[fromAnchor][toAnchor].add(distanceMeters, now);
    m_accumulators[toAnchor][fromAnchor].add(distanceMeters, now);

#ifdef ESP_PLATFORM
    // Log progress every 10 samples to help diagnose accumulation issues
    uint16_t count = m_accumulators[fromAnchor][toAnchor].count;
    if (count % 10 == 0 && count > 0) {
        ESP_LOGD(TAG, "Distance %d-%d: %d/%d samples (avg: %.3fm)",
                 fromAnchor, toAnchor, count, m_config.avgSampleCount,
                 m_accumulators[fromAnchor][toAnchor].average());
    }
#endif

    // Check if we have enough samples to finalize this pair
    if (m_accumulators[fromAnchor][toAnchor].isReady(m_config.avgSampleCount)) {
        // Transfer accumulated average to final value
        float avgDistance = m_accumulators[fromAnchor][toAnchor].average();
        m_averagedDistances[fromAnchor][toAnchor] = avgDistance;
        m_averagedDistances[toAnchor][fromAnchor] = avgDistance;

        // Mark as valid
        m_validDistanceMask[fromAnchor] |= (1 << toAnchor);
        m_validDistanceMask[toAnchor] |= (1 << fromAnchor);

        // Reset accumulators for next averaging cycle
        m_accumulators[fromAnchor][toAnchor].reset();
        m_accumulators[toAnchor][fromAnchor].reset();
    }
}

bool DynamicAnchorPositionCalculator::canCalculate() const {
    // For rectangular layout with 4 anchors, we need:
    // - d01 (A0-A1)
    // - d03 (A0-A3)
    // - d02 or d13 (diagonal - for validation)
    // Minimum: need at least the two perpendicular sides

    if (m_config.anchorCount < 4) {
        return false;
    }

    // Check if we have the essential distances for rectangular layout
    // Need at least d01 and d03 (the two sides from origin)
    bool hasD01 = (m_validDistanceMask[0] & (1 << 1)) != 0;
    bool hasD03 = (m_validDistanceMask[0] & (1 << 3)) != 0;

    if (!hasD01 || !hasD03) {
        return false;
    }

#ifdef ESP_PLATFORM
    // Check for stale data and warn (but don't reset - per user preference)
    uint32_t now = xTaskGetTickCount();
    static uint32_t lastStaleWarning = 0;

    // Check essential pairs for staleness
    bool d01Stale = m_accumulators[0][1].isStale(now, STALENESS_TIMEOUT_TICKS);
    bool d03Stale = m_accumulators[0][3].isStale(now, STALENESS_TIMEOUT_TICKS);

    if ((d01Stale || d03Stale) && (now - lastStaleWarning) > 5000) {
        ESP_LOGW(TAG, "Dynamic anchor data stale: d01=%s d03=%s (no updates in 5s)",
                 d01Stale ? "STALE" : "ok", d03Stale ? "STALE" : "ok");
        lastStaleWarning = now;
    }
#endif

    return true;
}

bool DynamicAnchorPositionCalculator::calculatePositions(point_t* positions, uint8_t maxCount) {
    if (positions == nullptr || maxCount == 0) {
        return false;
    }

    uint8_t count = (maxCount < m_config.anchorCount) ? maxCount : m_config.anchorCount;

    // Calculate new positions based on layout
    point_t newPositions[MAX_DYNAMIC_ANCHORS];
    bool success = false;

    switch (m_config.layout) {
        case 0:  // RECTANGULAR_0_ORIGIN
        case 1:  // RECTANGULAR_1_ORIGIN
        case 2:  // RECTANGULAR_2_ORIGIN
        case 3:  // RECTANGULAR_3_ORIGIN
            success = calculateRectangular(newPositions, count);
            break;
        default:
            // CUSTOM or unknown - not supported yet
            return false;
    }

    if (!success) {
        return false;
    }

    // Apply locking: use locked positions for locked anchors, calculated for others
    for (uint8_t i = 0; i < count; i++) {
        if (isAnchorLocked(i)) {
            positions[i] = m_lockedPositions[i];
        } else {
            positions[i] = newPositions[i];
            m_lastCalculatedPositions[i] = newPositions[i];
        }
    }

    return true;
}

bool DynamicAnchorPositionCalculator::calculateRectangular(point_t* positions, uint8_t count) {
    if (count < 4) {
        return false;
    }

    // Get the essential distances
    float d01 = m_averagedDistances[0][1];  // A0 to A1 (North side in RECTANGULAR_0_ORIGIN)
    float d03 = m_averagedDistances[0][3];  // A0 to A3 (East side in RECTANGULAR_0_ORIGIN)

    // Validate we have positive distances
    if (d01 <= 0.0f || d03 <= 0.0f) {
        return false;
    }

    // Optional: validate with diagonal if available
    float d02 = m_averagedDistances[0][2];  // Diagonal A0-A2
    float d13 = m_averagedDistances[1][3];  // Diagonal A1-A3
    if (d02 > 0.0f && d13 > 0.0f) {
        if (!validateRectangular(d01, d03, d02, d13)) {
            // Geometry doesn't match rectangular layout - continue anyway but could warn
        }
    }

    // Calculate Z coordinate (NED convention: Z = -height)
    float z = -m_config.anchorHeight;

    // Calculate positions based on layout origin
    // All layouts are rectangular with 4 anchors at corners
    // The origin anchor and axis orientation changes based on layout

    switch (m_config.layout) {
        case 0:  // RECTANGULAR_0_ORIGIN: A0 at origin
            // A0 = (0, 0, Z)       - Origin
            // A1 = (d01, 0, Z)     - North (+X)
            // A3 = (0, d03, Z)     - East (+Y)
            // A2 = (d01, d03, Z)   - Northeast corner
            positions[0] = {0, 0.0f, 0.0f, z};
            positions[1] = {0, d01, 0.0f, z};
            positions[2] = {0, d01, d03, z};
            positions[3] = {0, 0.0f, d03, z};
            break;

        case 1:  // RECTANGULAR_1_ORIGIN: A1 at origin
            // A1 = (0, 0, Z)       - Origin
            // A0 = (-d01, 0, Z)    - South (-X)
            // A2 = (0, d03, Z)     - East (+Y)
            // A3 = (-d01, d03, Z)  - Southeast corner
            positions[0] = {0, -d01, 0.0f, z};
            positions[1] = {0, 0.0f, 0.0f, z};
            positions[2] = {0, 0.0f, d03, z};
            positions[3] = {0, -d01, d03, z};
            break;

        case 2:  // RECTANGULAR_2_ORIGIN: A2 at origin
            // A2 = (0, 0, Z)       - Origin
            // A1 = (-d01, 0, Z)    - South (-X)
            // A3 = (0, -d03, Z)    - West (-Y)
            // A0 = (-d01, -d03, Z) - Southwest corner
            positions[0] = {0, -d01, -d03, z};
            positions[1] = {0, 0.0f, -d03, z};
            positions[2] = {0, 0.0f, 0.0f, z};
            positions[3] = {0, -d01, 0.0f, z};
            break;

        case 3:  // RECTANGULAR_3_ORIGIN: A3 at origin
            // A3 = (0, 0, Z)       - Origin
            // A0 = (d01, 0, Z)     - North (+X)
            // A2 = (0, d03, Z)     - East (+Y)
            // A1 = (d01, d03, Z)   - Northeast corner
            positions[0] = {0, d01, 0.0f, z};
            positions[1] = {0, d01, d03, z};
            positions[2] = {0, 0.0f, d03, z};
            positions[3] = {0, 0.0f, 0.0f, z};
            break;

        default:
            return false;
    }

    return true;
}

bool DynamicAnchorPositionCalculator::validateRectangular(float d01, float d03, float d02, float d13) {
    // For a rectangle:
    // - Diagonals should be equal (d02 == d13)
    // - Diagonal should equal sqrt(d01^2 + d03^2) (Pythagorean theorem)

    // Calculate expected diagonal
    float expectedDiagonal = std::sqrt(d01 * d01 + d03 * d03);

    // Allow 10% tolerance for measurement noise
    float tolerance = 0.1f;

    // Check if diagonals are roughly equal to each other
    float diagDiff = std::abs(d02 - d13);
    float diagAvg = (d02 + d13) / 2.0f;
    if (diagAvg > 0.0f && (diagDiff / diagAvg) > tolerance) {
        return false;
    }

    // Check if diagonal matches Pythagorean expectation
    float diagError = std::abs(diagAvg - expectedDiagonal);
    if (expectedDiagonal > 0.0f && (diagError / expectedDiagonal) > tolerance) {
        return false;
    }

    return true;
}

void DynamicAnchorPositionCalculator::lockAnchor(uint8_t anchorId) {
    if (anchorId >= MAX_DYNAMIC_ANCHORS) {
        return;
    }

    // Store current calculated position before locking
    m_lockedPositions[anchorId] = m_lastCalculatedPositions[anchorId];
    m_config.lockedMask |= (1 << anchorId);
}

void DynamicAnchorPositionCalculator::unlockAnchor(uint8_t anchorId) {
    if (anchorId >= MAX_DYNAMIC_ANCHORS) {
        return;
    }

    m_config.lockedMask &= ~(1 << anchorId);
}

bool DynamicAnchorPositionCalculator::isAnchorLocked(uint8_t anchorId) const {
    if (anchorId >= MAX_DYNAMIC_ANCHORS) {
        return false;
    }

    return (m_config.lockedMask & (1 << anchorId)) != 0;
}

float DynamicAnchorPositionCalculator::getAveragedDistance(uint8_t from, uint8_t to) const {
    if (from >= MAX_DYNAMIC_ANCHORS || to >= MAX_DYNAMIC_ANCHORS) {
        return 0.0f;
    }

    return m_averagedDistances[from][to];
}

uint16_t DynamicAnchorPositionCalculator::getSampleCount(uint8_t from, uint8_t to) const {
    if (from >= MAX_DYNAMIC_ANCHORS || to >= MAX_DYNAMIC_ANCHORS) {
        return 0;
    }

    return m_accumulators[from][to].count;
}

bool DynamicAnchorPositionCalculator::isDistanceReady(uint8_t from, uint8_t to) const {
    if (from >= MAX_DYNAMIC_ANCHORS || to >= MAX_DYNAMIC_ANCHORS) {
        return false;
    }

    return (m_validDistanceMask[from] & (1 << to)) != 0;
}

void DynamicAnchorPositionCalculator::finalizeAverages() {
    // This is called internally when accumulators are ready
    // Already handled in updateDistance() for immediate finalization
}
