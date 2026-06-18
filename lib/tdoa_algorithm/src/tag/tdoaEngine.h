#ifndef __TDOA_ENGINE_H__
#define __TDOA_ENGINE_H__

#include <atomic>  // seqlock sequence counter for the cross-task matcher state

#include "tdoaStorage.h"
#include "tdoaStats.h"

#if CONFIG_DECK_LOCO_LONGER_RANGE
#define TDOA_ENGINE_MEASUREMENT_NOISE_STD 0.30f
#else
#define TDOA_ENGINE_MEASUREMENT_NOISE_STD 0.15f
#endif

typedef void (*tdoaEngineSendTdoaToEstimator)(tdoaMeasurement_t* tdoaMeasurement);

typedef enum {
  TdoaEngineMatchingAlgorithmNone = 0,
  TdoaEngineMatchingAlgorithmRandom,
  TdoaEngineMatchingAlgorithmYoungest,
  TdoaEngineMatchingAlgorithmGeometric,  // E-optimal partner selection (Change 3)
} tdoaEngineMatchingAlgorithm_t;

// Number of anchors the geometric matcher tracks positions for.
#define TDOA_ENGINE_MAX_ANCHORS 8

typedef struct {
  // State
  tdaoAnchorInfoArray_t anchorInfoArray;
  tdoaStats_t stats;

  // Configuration
  tdoaEngineSendTdoaToEstimator sendTdoaToEstimator;
  double locodeckTsFreq;
  tdoaEngineMatchingAlgorithm_t matchingAlgorithm;

  // Matching algorithm data
  struct {
    uint8_t seqNr[REMOTE_ANCHOR_DATA_COUNT];
    uint8_t id[REMOTE_ANCHOR_DATA_COUNT];
    uint8_t offset;
  } matching;

  // Geometric (E-optimal) matcher data (Change 3). Anchor positions and the
  // last tag position are pushed from the integration layer. `info` is a
  // decaying 3x3 Fisher-information accumulator (packed [xx,xy,xz,yy,yz,zz]).
  struct {
    // Seqlock: anchor positions/prior are written by the estimator task and read
    // by the (timing-sensitive) UWB ranging task. `seq` is even when stable, odd
    // mid-write; readers take a consistent snapshot with a bounded retry instead
    // of a critical section (no interrupt-disable in the ranging hot loop).
    // std::atomic (not volatile) so the load/store carry the acquire/release
    // ordering the C++ memory model requires to make the snapshot race-free.
    std::atomic<uint32_t> seq;
    uint8_t hasPrior;                              // 1 if priorPos is valid
    float priorPos[3];
    uint8_t anchorValid[TDOA_ENGINE_MAX_ANCHORS];
    float anchorPos[TDOA_ENGINE_MAX_ANCHORS][3];
    float info[6];                                 // ranging-task-local accumulator
  } geo;
} tdoaEngineState_t;

void tdoaEngineInit(tdoaEngineState_t* state, const uint32_t now_ms, tdoaEngineSendTdoaToEstimator sendTdoaToEstimator, const double locodeckTsFreq, const tdoaEngineMatchingAlgorithm_t matchingAlgorithm);
// 
void tdoaEngineGetAnchorCtxForPacketProcessing(tdoaEngineState_t* engineState, const uint8_t anchorId, const uint32_t currentTime_ms, tdoaAnchorContext_t* anchorCtx);
void tdoaEngineProcessPacket(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T);

// Geometric matcher configuration (Change 3).
void tdoaEngineSetAnchorPosition(tdoaEngineState_t* engineState, uint8_t anchorId, float x, float y, float z);
void tdoaEngineSetPriorPosition(tdoaEngineState_t* engineState, float x, float y, float z);
void tdoaEngineClearPrior(tdoaEngineState_t* engineState);
bool tdoaEngineProcessPacketFiltered(tdoaEngineState_t* engineState, tdoaAnchorContext_t* anchorCtx, const int64_t txAn_in_cl_An, const int64_t rxAn_by_T_in_cl_T, const bool doExcludeId, const uint8_t excludedId);
 
#define TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP 0x00FFFFFFFF
static inline uint64_t tdoaEngineTruncateToAnchorTimeStamp(uint64_t fullTimeStamp) {
  return fullTimeStamp & TDOA_ENGINE_TRUNCATE_TO_ANCHOR_TS_BITMAP;
}

#endif // __TDOA_ENGINE_H__
