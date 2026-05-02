#pragma once

#include <stdint.h>

namespace tdoa {

static constexpr uint8_t kMaxParsedAnchorCount = 8;

template <typename ShortAddr>
inline bool ParseAnchorId(const ShortAddr& shortAddr,
                          uint8_t& outAnchorId,
                          uint8_t anchorCount = kMaxParsedAnchorCount)
{
    if (shortAddr[0] < '0' || shortAddr[0] > '9') {
        return false;
    }

    uint8_t value = static_cast<uint8_t>(shortAddr[0] - '0');
    if (shortAddr[1] != '\0') {
        if (shortAddr[1] < '0' || shortAddr[1] > '9') {
            return false;
        }
        value = static_cast<uint8_t>(value * 10 + static_cast<uint8_t>(shortAddr[1] - '0'));
    }

    if (value >= anchorCount) {
        return false;
    }

    outAnchorId = value;
    return true;
}


} // namespace tdoa
