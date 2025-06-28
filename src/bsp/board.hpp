#pragma once

#if defined(MAKERFABS_ESP32_BOARD)
#include "makerfabs_uwb/board.hpp"
#include "makerfabs_uwb/config.hpp"
#elif defined(ESP32S3_UWB_BOARD)
#include "konex_uwb_v1/board.hpp"
#include "konex_uwb_v1/config.hpp"
#else
#error "Board not supported"
#endif