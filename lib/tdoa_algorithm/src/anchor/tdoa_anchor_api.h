/**
 * @file tdoa_anchor_api.h
 * @brief Public API helpers for the TDoA anchor algorithm.
 *
 * The upstream Bitcraze TDoA anchor implementation keeps all algorithm state in a
 * single internal static context. For calibration and diagnostics we expose a
 * small, C-linkage API to:
 *  - read the latest inter-anchor raw ToF measurements (DW1000 timestamp units)
 *  - get/set the antenna delay value broadcast in anchor packets
 *
 * NOTE: The inter-anchor distances are raw (uncorrected) and include antenna
 * delays from both endpoints. A consumer must subtract both anchors' antenna
 * delays to obtain a corrected distance.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Copy the latest inter-anchor distances for this anchor.
 *
 * @param[out] out_distances Pointer to a buffer to receive distances.
 * @param[in]  max_len Maximum number of entries to copy.
 * @return true if copied, false if invalid args or not initialized.
 */
bool uwbTdoa2AnchorGetDistances(uint16_t* out_distances, uint8_t max_len);

/**
 * @brief Get the current anchor ID (0..7) used by the TDMA schedule.
 */
uint8_t uwbTdoa2AnchorGetAnchorId(void);

/**
 * @brief Get the antenna delay (DW1000 ticks) currently broadcast by this anchor.
 */
uint16_t uwbTdoa2AnchorGetAntennaDelay(void);

/**
 * @brief Set the antenna delay (DW1000 ticks) to be broadcast by this anchor.
 *
 * This does not affect the raw inter-anchor ToF computation (which is kept
 * uncorrected in the anchors), but it enables immediate correction on the tag
 * side without requiring an anchor reboot.
 */
void uwbTdoa2AnchorSetAntennaDelay(uint16_t delay);

#ifdef __cplusplus
} // extern "C"
#endif

