#pragma once

#include <Arduino.h>

extern "C" {
    #include "libdw1000.h"
}

#include "uwb.h"

extern uwbAlgorithm_t uwbTdoa2Algorithm;