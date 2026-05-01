#pragma once

#include <algorithm>
#include <stdint.h>

namespace tdoa {

struct AnchorPair {
    uint8_t a = 0;
    uint8_t b = 0;
};

inline bool CanonicalizePair(uint8_t anchorA,
                             uint8_t anchorB,
                             uint8_t anchorCount,
                             AnchorPair& outPair,
                             bool& outReversed)
{
    if (anchorA >= anchorCount || anchorB >= anchorCount || anchorA == anchorB) {
        return false;
    }

    outPair.a = std::min(anchorA, anchorB);
    outPair.b = std::max(anchorA, anchorB);
    outReversed = anchorA != outPair.a;
    return true;
}

constexpr uint8_t PairCount(uint8_t anchorCount)
{
    return static_cast<uint8_t>((anchorCount * (anchorCount - 1)) / 2);
}

inline uint8_t PairIndexCanonical(const AnchorPair& pair, uint8_t anchorCount)
{
    return static_cast<uint8_t>(
        (pair.a * (2 * anchorCount - pair.a - 1)) / 2 + (pair.b - pair.a - 1));
}

inline int8_t PairIndex(uint8_t anchorA, uint8_t anchorB, uint8_t anchorCount)
{
    AnchorPair pair;
    bool reversed = false;
    if (!CanonicalizePair(anchorA, anchorB, anchorCount, pair, reversed)) {
        return -1;
    }

    (void)reversed;
    return static_cast<int8_t>(PairIndexCanonical(pair, anchorCount));
}

template <uint8_t AnchorCount>
constexpr AnchorPair PairByIndex(uint8_t pairIndex)
{
    uint8_t index = 0;
    for (uint8_t a = 0; a < AnchorCount; ++a) {
        for (uint8_t b = a + 1; b < AnchorCount; ++b) {
            if (index == pairIndex) {
                return AnchorPair{a, b};
            }
            ++index;
        }
    }
    return AnchorPair{};
}

} // namespace tdoa
