#pragma once

#include <stdint.h>

#include "tdoa_pairs.hpp"
#include "uwb_params.hpp"

namespace tdoa {

inline bool ParseAnchorId(const UWBShortAddr& shortAddr, uint8_t& outAnchorId)
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

    if (value > 7) {
        return false;
    }

    outAnchorId = value;
    return true;
}


} // namespace tdoa
