#pragma once

// FAST_CODE macro: places functions in IRAM for deterministic execution.
// Enabled by USE_FAST_CODE feature flag. No-op on native test builds.
//
// Use on: ISR handlers, SPI transfers, UWB packet callbacks.
// Budget: ~5-13KB of ~128KB available IRAM.

#if defined(USE_FAST_CODE) && (defined(ESP32) || defined(ESP32S3))
  #include <esp_attr.h>
  #define FAST_CODE IRAM_ATTR
#else
  #define FAST_CODE
#endif
