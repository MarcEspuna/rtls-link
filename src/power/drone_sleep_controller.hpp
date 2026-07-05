#pragma once

#include "config/features.hpp"

#ifdef USE_DRONE_SLEEP_MODE

#include <cstddef>
#include <cstdint>

class DroneSleepController {
public:
    static void Init();
    static void Update();

    static bool IsSleeping();
    static bool IsSupported();

    static bool EnterSleep(char* message, size_t message_len);
    static bool Wake(char* message, size_t message_len);

private:
    static void SetMessage(char* message, size_t message_len, const char* text);
};

#endif // USE_DRONE_SLEEP_MODE
