#include "uwb_frontend_littlefs.hpp"

void UWBLittleFSFrontend::Init() {
    LittleFSFrontend<UWBParams>::Init();
    printf("UWBLittleFSFrontend::Init()\n");
}

void UWBLittleFSFrontend::Update() {
    // Basic update functionality
}

namespace Front {
    UWBLittleFSFrontend uwbLittleFSFront;
}