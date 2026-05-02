#include <gtest/gtest.h>

#include <vector>

#include "../../lib/tdoa_algorithm/src/tag/clockCorrectionEngine.h"
#include "../../lib/tdoa_algorithm/src/tag/tdoaEngine.h"
#include "../../lib/tdoa_algorithm/src/tag/tdoaStorage.h"

#include "../../lib/tdoa_algorithm/src/tag/clockCorrectionEngine.cpp"
#include "../../lib/tdoa_algorithm/src/tag/tdoaStorage.cpp"
#undef DEBUG_MODULE
#include "../../lib/tdoa_algorithm/src/tag/tdoaEngine.cpp"

namespace {

std::vector<tdoaMeasurement_t> g_measurements;
constexpr double kLocodeckTsFreq = 499.2e6 * 128;

void recordMeasurement(tdoaMeasurement_t* measurement)
{
    g_measurements.push_back(*measurement);
}

tdoaAnchorContext_t createAnchor(tdoaEngineState_t& state, uint8_t id, uint32_t nowMs)
{
    tdoaAnchorContext_t ctx;
    tdoaEngineGetAnchorCtxForPacketProcessing(&state, id, nowMs, &ctx);
    return ctx;
}

void makeCandidate(tdoaEngineState_t& state,
                   tdoaAnchorContext_t& incomingAnchor,
                   uint8_t candidateId,
                   uint8_t candidateSeqNr,
                   int64_t candidateRxByTag,
                   int64_t candidateRxByIncomingAnchor,
                   int64_t tofToIncomingAnchor,
                   uint32_t nowMs)
{
    tdoaAnchorContext_t candidate = createAnchor(state, candidateId, nowMs);
    tdoaStorageSetRxTxData(&candidate, candidateRxByTag, 500000 + candidateId, candidateSeqNr);
    tdoaStorageSetRemoteRxTime(&incomingAnchor, candidateId, candidateRxByIncomingAnchor, candidateSeqNr);
    tdoaStorageSetRemoteTimeOfFlight(&incomingAnchor, candidateId, tofToIncomingAnchor);
}

} // namespace

TEST(TDoAEngine, AllEligiblePolicyEmitsEveryMatchingCandidateForOnePacket)
{
    g_measurements.clear();

    tdoaEngineState_t state = {};
    tdoaEngineInit(&state,
                   0,
                   recordMeasurement,
                   kLocodeckTsFreq,
                   TdoaEngineMatchingAlgorithmAllEligible);

    constexpr uint32_t nowMs = 100;
    constexpr uint8_t incomingAnchorId = 3;
    tdoaAnchorContext_t incomingAnchor = createAnchor(state, incomingAnchorId, nowMs);

    tdoaStorageSetRxTxData(&incomingAnchor, 1000000, 2000000, 42);
    tdoaStorageGetClockCorrectionStorage(&incomingAnchor)->clockCorrection = 1.0;

    makeCandidate(state, incomingAnchor, 0, 10, 900000, 1900000, 1000, nowMs);
    makeCandidate(state, incomingAnchor, 1, 11, 910000, 1910000, 1100, nowMs);
    makeCandidate(state, incomingAnchor, 2, 12, 920000, 1920000, 1200, nowMs);

    const bool processed = tdoaEngineProcessPacketFiltered(&state, &incomingAnchor, 2010000, 1010000, false, 0);

    EXPECT_TRUE(processed);
    ASSERT_EQ(g_measurements.size(), 3);
    EXPECT_EQ(g_measurements[0].anchorIdA, 0);
    EXPECT_EQ(g_measurements[0].anchorIdB, incomingAnchorId);
    EXPECT_EQ(g_measurements[1].anchorIdA, 1);
    EXPECT_EQ(g_measurements[1].anchorIdB, incomingAnchorId);
    EXPECT_EQ(g_measurements[2].anchorIdA, 2);
    EXPECT_EQ(g_measurements[2].anchorIdB, incomingAnchorId);
}

TEST(TDoAEngine, YoungestPolicyStillEmitsOneMatchingCandidateForOnePacket)
{
    g_measurements.clear();

    tdoaEngineState_t state = {};
    tdoaEngineInit(&state,
                   0,
                   recordMeasurement,
                   kLocodeckTsFreq,
                   TdoaEngineMatchingAlgorithmYoungest);

    constexpr uint32_t nowMs = 100;
    constexpr uint8_t incomingAnchorId = 3;
    tdoaAnchorContext_t incomingAnchor = createAnchor(state, incomingAnchorId, nowMs);

    tdoaStorageSetRxTxData(&incomingAnchor, 1000000, 2000000, 42);
    tdoaStorageGetClockCorrectionStorage(&incomingAnchor)->clockCorrection = 1.0;

    makeCandidate(state, incomingAnchor, 0, 10, 900000, 1900000, 1000, nowMs);
    makeCandidate(state, incomingAnchor, 1, 11, 910000, 1910000, 1100, nowMs);
    makeCandidate(state, incomingAnchor, 2, 12, 920000, 1920000, 1200, nowMs);

    const bool processed = tdoaEngineProcessPacketFiltered(&state, &incomingAnchor, 2010000, 1010000, false, 0);

    EXPECT_TRUE(processed);
    EXPECT_EQ(g_measurements.size(), 1);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
