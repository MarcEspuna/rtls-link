/*
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie firmware.
 *
 * Copyright 2018, Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * tdoaEngine.c is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with tdoaEngine.c. If not, see <http://www.gnu.org/licenses/>.
 */


/*
Implementation of LPS TDoA Tag functionality

The tag is assumed to move around in a large system of anchors. Any anchor ids
can be used, and the same anchor id can even be used by multiple anchors as long
as they are not visible in the same area. It is assumed that the anchor density
is evenly distributed in the covered volume and that 5-20 anchors are visible
in every point. The tag is attached to a physical object and the expected
velocity is a few m/s, this means that anchors are within range for a time
period of seconds.

The implementation must handle
1. An infinite number of anchors, where around 20 are visible at one time
2. Any anchor ids
3. Dynamically changing visibility of anchors over time
4. Random TX times from anchors with possible packet collisions and packet loss

*/

#include <string.h>
#include <stdio.h>
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
#include <esp_timer.h>
#endif

#define DEBUG_MODULE "TDOA_ENGINE"

static uint64_t getTdoaSolvedTimestampUs()
{
#if defined(ESP_PLATFORM) || defined(ARDUINO_ARCH_ESP32)
  return static_cast<uint64_t>(esp_timer_get_time());
#else
  return 0;
#endif
}

#include "tdoaEngine.h"
#include "tdoaStats.h"
#include "clockCorrectionEngine.h"
#include "physicalConstants.h"
#include "tdoa_geometric_matcher.hpp"

#include <atomic>
#include <string.h>

// Decay applied to the geometric matcher's Fisher-info accumulator on each
// formed measurement, so it reflects a recent window (~1/(1-decay) samples).
static constexpr float kTdoaGeometricInfoDecay = 0.95f;

// Seqlock for the cross-task geometric-matcher state (geo.{hasPrior,priorPos,
// anchorValid,anchorPos}): written by the estimator task, read by the UWB
// ranging task. Readers take a consistent snapshot with a bounded retry and
// never block or disable interrupts (the ranging loop is timing-sensitive).
// geo.info is NOT covered: it is owned by the ranging task (accumulate path) and
// only read while hasPrior is true.
struct GeoSnapshot {
  uint8_t hasPrior;
  float priorPos[3];
  uint8_t anchorValid[TDOA_ENGINE_MAX_ANCHORS];
  float anchorPos[TDOA_ENGINE_MAX_ANCHORS][3];
};

static inline void geoWriteBegin(tdoaEngineState_t* s) {
  s->geo.seq++;                                        // -> odd: write in progress
  std::atomic_thread_fence(std::memory_order_release);
}
static inline void geoWriteEnd(tdoaEngineState_t* s) {
  std::atomic_thread_fence(std::memory_order_release);
  s->geo.seq++;                                        // -> even: stable
}

// Returns false if no stable snapshot could be taken (writer too active).
static bool geoSnapshot(const tdoaEngineState_t* s, GeoSnapshot* out) {
  for (int tries = 0; tries < 8; ++tries) {
    const uint32_t s1 = s->geo.seq;
    if (s1 & 1u) {                                     // mid-write
      continue;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    out->hasPrior = s->geo.hasPrior;
    memcpy(out->priorPos, s->geo.priorPos, sizeof(out->priorPos));
    memcpy(out->anchorValid, s->geo.anchorValid, sizeof(out->anchorValid));
    memcpy(out->anchorPos, s->geo.anchorPos, sizeof(out->anchorPos));
    std::atomic_thread_fence(std::memory_order_acquire);
    if (s1 == s->geo.seq) {
      return true;
    }
  }
  return false;
}

void tdoaEngineInit(tdoaEngineState_t* engineState, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm) {
  tdoaStorageInitialize(engineState->anchorInfoArray);
  // tdoaStatsInit(&engineState->stats, now_ms);
  engineState->sendTdoaToEstimator = sendTdoaToEstimator;
  engineState->locodeckTsFreq = locodeckTsFreq;
  engineState->matchingAlgorithm = matchingAlgorithm;

  engineState->matching.offset = 0;

  memset(&engineState->geo, 0, sizeof(engineState->geo));
}

static void enqueueTDOA(const tdoaAnchorContext_t* anchorACtx, const tdoaAnchorContext_t* anchorBCtx, double distanceDiff, tdoaEngineState_t* engineState) {
  tdoaStats_t* stats = &engineState->stats;

  tdoaMeasurement_t tdoa = {
    .distanceDiff = static_cast<float>(distanceDiff),
    .stdDev = TDOA_ENGINE_MEASUREMENT_NOISE_STD,
    .solvedTimestampUs = getTdoaSolvedTimestampUs()
  };

  // *** Added by me ***
  uint8_t idA = tdoaStorageGetId(anchorACtx);
  uint8_t idB = tdoaStorageGetId(anchorBCtx);
  if (idA == stats->anchorId && idB == stats->remoteAnchorId) {
    stats->tdoa = distanceDiff;
  }
  if (idB == stats->anchorId && idA == stats->remoteAnchorId) {
    stats->tdoa = -distanceDiff;
  }
  tdoa.anchorIds[0] = idA;
  tdoa.anchorIds[1] = idB;
 
  engineState->sendTdoaToEstimator(&tdoa);

  // For now here will never enter
  // if (tdoaStorageGetAnchorPosition(anchorACtx, &tdoa.anchorPositions[0]) && tdoaStorageGetAnchorPosition(anchorBCtx, &tdoa.anchorPositions[1])) {
  //   // STATS_CNT_RATE_EVENT(&stats->packetsToEstimator);
 
  //   uint8_t idA = tdoaStorageGetId(anchorACtx);
  //   uint8_t idB = tdoaStorageGetId(anchorBCtx);
  //   if (idA == stats->anchorId && idB == stats->remoteAnchorId) {
  //     stats->tdoa = distanceDiff;
  //   }
  //   if (idB == stats->anchorId && idA == stats->remoteAnchorId) {
  //     stats->tdoa = -distanceDiff;
  //   }
  //   tdoa.anchorIds[0] = idA;
  //   tdoa.anchorIds[1] = idB;
 
  //   engineState->sendTdoaToEstimator(&tdoa);
  // }
}

static bool updateClockCorrection(tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, tdoaStats_t* stats) {
  bool sampleIsReliable = false;

  const int64_t latest_rxAn_by_T_in_cl_T = tdoaStorageGetRxTime(anchorCtx);
  const int64_t latest_txAn_in_cl_An = tdoaStorageGetTxTime(anchorCtx);

  if (latest_rxAn_by_T_in_cl_T != 0 && latest_txAn_in_cl_An != 0) {
    double clockCorrectionCandidate = clockCorrectionEngineCalculate(rxAn_by_T_in_cl_T, latest_rxAn_by_T_in_cl_T, txAn_in_cl_An, latest_txAn_in_cl_An, TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP);
    sampleIsReliable = clockCorrectionEngineUpdate(tdoaStorageGetClockCorrectionStorage(anchorCtx), clockCorrectionCandidate);

    if (sampleIsReliable){
      if (tdoaStorageGetId(anchorCtx) == stats->anchorId) {
        stats->clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);
        // STATS_CNT_RATE_EVENT(&stats->clockCorrectionCount);
      }
    } else {
      // printf("Sample not reliable\n");
    }
  }

  return sampleIsReliable;
}


/**
 * @brief Ingredients: 
 * - ToF of anchor R to anchor N in anchor N clock
 * - Reception timestamp of anchor N, of packet sent by anchor R. Clock in N anchor timebase.
 * - Clock correction
 * - Reception timestamp of us(tag), packet sent by anchor R our clock(tag)
 * 
 * - Now we get the delta of TX by anchor R to Tx by anchor N in clock of anchor N.
 *    -   
 */
static int64_t calcTDoA(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  const uint8_t otherAnchorId = tdoaStorageGetId(otherAnchorCtx);

  const int64_t tof_Ar_to_An_in_cl_An = tdoaStorageGetRemoteTimeOfFlight(anchorCtx, otherAnchorId);
  const int64_t rxAr_by_An_in_cl_An = tdoaStorageGetRemoteRxTime(anchorCtx, otherAnchorId);
  const double clockCorrection = tdoaStorageGetClockCorrection(anchorCtx);

  const int64_t rxAr_by_T_in_cl_T = tdoaStorageGetRxTime(otherAnchorCtx);

  const int64_t delta_txAr_to_txAn_in_cl_An = (tof_Ar_to_An_in_cl_An + tdoaEngineTruncateToAnchorTimeStamp(txAn_in_cl_An - rxAr_by_An_in_cl_An));
  const int64_t timeDiffOfArrival_in_cl_T =  tdoaEngineTruncateToAnchorTimeStamp(rxAn_by_T_in_cl_T - rxAr_by_T_in_cl_T) - delta_txAr_to_txAn_in_cl_An  * clockCorrection;

  return timeDiffOfArrival_in_cl_T;
}

static double calcDistanceDiff(const tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const double locodeckTsFreq) {
  const int64_t tdoa = calcTDoA(otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T);
  return SPEED_OF_LIGHT * tdoa / locodeckTsFreq;
}

static bool matchRandomAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
  engineState->matching.offset++;
  int remoteCount = 0;
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

  uint32_t now_ms = anchorCtx->currentTime_ms;

  // Loop over the candidates and pick the first one that is useful
  // An offset (updated for each call) is added to make sure we start at
  // different positions in the list and vary which candidate to choose
  for (int i = engineState->matching.offset; i < (remoteCount + engineState->matching.offset); i++) {
    uint8_t index = i % remoteCount;
    const uint8_t candidateAnchorId = engineState->matching.id[index];
    if (!doExcludeId || (excludedId != candidateAnchorId)) {
      if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
        if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx) && tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          return true;
        }
      }
    }
  }

  otherAnchorCtx->anchorInfo = 0;
  return false;
}

static bool matchYoungestAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
    int remoteCount = 0;
    tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

    uint32_t now_ms = anchorCtx->currentTime_ms;
    uint32_t youmgestUpdateTime = 0;
    int bestId = -1;

    for (int index = 0; index < remoteCount; index++) {
      const uint8_t candidateAnchorId = engineState->matching.id[index];
      if (!doExcludeId || (excludedId != candidateAnchorId)) {
        if (tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
          if (tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
            uint32_t updateTime = tdoaStorageGetLastUpdateTime(otherAnchorCtx);
            if (updateTime > youmgestUpdateTime) {
              if (engineState->matching.seqNr[index] == tdoaStorageGetSeqNr(otherAnchorCtx)) {
                youmgestUpdateTime = updateTime;
                bestId = candidateAnchorId;
              }
            }
          }
        }
      }
    }

    if (bestId >= 0) {
      tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, bestId, now_ms, otherAnchorCtx);
      return true;
    }

    otherAnchorCtx->anchorInfo = 0;
    return false;
}

// Change 3: E-optimal partner selection. Among valid candidates (same validity
// conditions as matchYoungestAnchor), pick the partner B that maximizes the
// minimum eigenvalue of the window Fisher information after adding the row
// gradient for pair (A,B), evaluated at the last tag position. Falls back to
// matchYoungestAnchor on cold start (no prior) or if no candidate qualifies.
static bool matchGeometricAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
  const uint8_t aId = tdoaStorageGetId(anchorCtx);
  GeoSnapshot geo;
  if (!geoSnapshot(engineState, &geo)
      || !geo.hasPrior
      || aId >= TDOA_ENGINE_MAX_ANCHORS
      || !geo.anchorValid[aId]) {
    return matchYoungestAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
  }

  int remoteCount = 0;
  tdoaStorageGetRemoteSeqNrList(anchorCtx, &remoteCount, engineState->matching.seqNr, engineState->matching.id);

  const uint32_t now_ms = anchorCtx->currentTime_ms;

  const tdoa_geometric::Vec3 p{geo.priorPos[0],
                               geo.priorPos[1],
                               geo.priorPos[2]};
  const tdoa_geometric::Vec3 pa{geo.anchorPos[aId][0],
                                geo.anchorPos[aId][1],
                                geo.anchorPos[aId][2]};

  tdoa_geometric::SymInfo3 info;
  info.xx = engineState->geo.info[0];
  info.xy = engineState->geo.info[1];
  info.xz = engineState->geo.info[2];
  info.yy = engineState->geo.info[3];
  info.yz = engineState->geo.info[4];
  info.zz = engineState->geo.info[5];

  float bestScore = -1.0f;
  int bestId = -1;
  for (int index = 0; index < remoteCount; index++) {
    const uint8_t candidateAnchorId = engineState->matching.id[index];
    if (doExcludeId && excludedId == candidateAnchorId) {
      continue;
    }
    if (candidateAnchorId >= TDOA_ENGINE_MAX_ANCHORS || !geo.anchorValid[candidateAnchorId]) {
      continue;
    }
    if (!tdoaStorageGetRemoteTimeOfFlight(anchorCtx, candidateAnchorId)) {
      continue;
    }
    if (!tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, candidateAnchorId, now_ms, otherAnchorCtx)) {
      continue;
    }
    if (engineState->matching.seqNr[index] != tdoaStorageGetSeqNr(otherAnchorCtx)) {
      continue;
    }

    const tdoa_geometric::Vec3 pb{geo.anchorPos[candidateAnchorId][0],
                                  geo.anchorPos[candidateAnchorId][1],
                                  geo.anchorPos[candidateAnchorId][2]};
    const tdoa_geometric::Vec3 g = tdoa_geometric::rowGradient(p, pa, pb);
    const float score = tdoa_geometric::eOptimalScore(info, g);
    if (score > bestScore) {
      bestScore = score;
      bestId = candidateAnchorId;
    }
  }

  if (bestId >= 0) {
    tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, static_cast<uint8_t>(bestId), now_ms, otherAnchorCtx);
    return true;
  }

  // No geometric candidate qualified — fall back to youngest.
  return matchYoungestAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
}

// Update the decaying Fisher-info accumulator with the formed pair's gradient.
static void geometricAccumulate(tdoaEngineState_t* engineState, const tdoaAnchorContext_t* anchorCtx, const tdoaAnchorContext_t* otherAnchorCtx) {
  if (engineState->matchingAlgorithm != TdoaEngineMatchingAlgorithmGeometric) {
    return;
  }
  const uint8_t aId = tdoaStorageGetId(anchorCtx);
  const uint8_t bId = tdoaStorageGetId(otherAnchorCtx);
  if (aId >= TDOA_ENGINE_MAX_ANCHORS || bId >= TDOA_ENGINE_MAX_ANCHORS) {
    return;
  }
  GeoSnapshot geo;
  if (!geoSnapshot(engineState, &geo) || !geo.hasPrior) {
    return;
  }
  if (!geo.anchorValid[aId] || !geo.anchorValid[bId]) {
    return;
  }
  const tdoa_geometric::Vec3 p{geo.priorPos[0],
                               geo.priorPos[1],
                               geo.priorPos[2]};
  const tdoa_geometric::Vec3 pa{geo.anchorPos[aId][0],
                                geo.anchorPos[aId][1],
                                geo.anchorPos[aId][2]};
  const tdoa_geometric::Vec3 pb{geo.anchorPos[bId][0],
                                geo.anchorPos[bId][1],
                                geo.anchorPos[bId][2]};
  const tdoa_geometric::Vec3 g = tdoa_geometric::rowGradient(p, pa, pb);

  tdoa_geometric::SymInfo3 info;
  info.xx = engineState->geo.info[0];
  info.xy = engineState->geo.info[1];
  info.xz = engineState->geo.info[2];
  info.yy = engineState->geo.info[3];
  info.yz = engineState->geo.info[4];
  info.zz = engineState->geo.info[5];
  info.accumulate(g, kTdoaGeometricInfoDecay);
  engineState->geo.info[0] = info.xx;
  engineState->geo.info[1] = info.xy;
  engineState->geo.info[2] = info.xz;
  engineState->geo.info[3] = info.yy;
  engineState->geo.info[4] = info.yz;
  engineState->geo.info[5] = info.zz;
}

static bool findSuitableAnchor(tdoaEngineState_t* engineState, tdoaAnchorContext_t* otherAnchorCtx, const tdoaAnchorContext_t* anchorCtx, const bool doExcludeId, const uint8_t excludedId) {
  bool result = false;

  if (tdoaStorageGetClockCorrection(anchorCtx) > 0.0) {
    switch(engineState->matchingAlgorithm) {
      case TdoaEngineMatchingAlgorithmRandom:
        result = matchRandomAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
        break;

      case TdoaEngineMatchingAlgorithmYoungest:
        result = matchYoungestAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
        break;

      case TdoaEngineMatchingAlgorithmGeometric:
        result = matchGeometricAnchor(engineState, otherAnchorCtx, anchorCtx, doExcludeId, excludedId);
        break;

      default:
        // Do nothing
        break;
    }
  }
  if (!result) {
    // printf("Removed mesurement!\n");
    // TODO: Think of a way to not clutter the log
  }
  return result;
}

void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx) {
  tdoaStorageGetCreateAnchorCtx(engineState->anchorInfoArray, anchorId, currentTime_ms, anchorCtx);
}

void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T) {
  tdoaEngineProcessPacketFiltered(engineState, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, false, 0);
}

bool tdoaEngineProcessPacketFiltered(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const bool doExcludeId, const uint8_t excludedId) {
  bool timeIsGood = updateClockCorrection(anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, &engineState->stats);
  if (timeIsGood) {
    tdoaAnchorContext_t otherAnchorCtx;
    if (findSuitableAnchor(engineState, &otherAnchorCtx, anchorCtx, doExcludeId, excludedId)) {
      double tdoaDistDiff = calcDistanceDiff(&otherAnchorCtx, anchorCtx, txAn_in_cl_An, rxAn_by_T_in_cl_T, engineState->locodeckTsFreq);
      enqueueTDOA(&otherAnchorCtx, anchorCtx, tdoaDistDiff, engineState);
      geometricAccumulate(engineState, anchorCtx, &otherAnchorCtx);
    }
  }
  return timeIsGood;
}

void tdoaEngineSetAnchorPosition(tdoaEngineState_t* engineState, uint8_t anchorId, float x, float y, float z) {
  if (anchorId >= TDOA_ENGINE_MAX_ANCHORS) {
    return;
  }
  geoWriteBegin(engineState);
  engineState->geo.anchorPos[anchorId][0] = x;
  engineState->geo.anchorPos[anchorId][1] = y;
  engineState->geo.anchorPos[anchorId][2] = z;
  engineState->geo.anchorValid[anchorId] = 1;
  geoWriteEnd(engineState);
}

void tdoaEngineSetPriorPosition(tdoaEngineState_t* engineState, float x, float y, float z) {
  geoWriteBegin(engineState);
  engineState->geo.priorPos[0] = x;
  engineState->geo.priorPos[1] = y;
  engineState->geo.priorPos[2] = z;
  engineState->geo.hasPrior = 1;
  geoWriteEnd(engineState);
}

void tdoaEngineClearPrior(tdoaEngineState_t* engineState) {
  // Only flip hasPrior (seqlock-protected). The Fisher accumulator `info` is
  // owned by the ranging task; it is never used while hasPrior == 0 and decays
  // (kTdoaGeometricInfoDecay) once a new prior is set, so we do not write it
  // from this (estimator) task — that would be an unsynchronized cross-task store.
  geoWriteBegin(engineState);
  engineState->geo.hasPrior = 0;
  geoWriteEnd(engineState);
}
