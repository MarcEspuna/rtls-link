# CLAUDE.md

## Project Overview

This is an ESP32/ESP32S3 firmware project for real-time localization systems (RTLS) using Ultra-Wideband (UWB) technology. The firmware implements Time Difference of Arrival (TDoA) algorithms for 2D/3D positioning and provides MAVLink integration for drone applications. In addition to that, it has a desktop application ( `tools/rtls-link-manager` , which is a git submodule with it's corresponding repository ) that it's used for configuring, monitoring and debugging this devices. 

## Build System

This project uses PlatformIO for build management with multiple environments:

### Build Commands
- `pio run` - Build all environments
- `pio run -e esp32_application` - Build for ESP32 (Makerfabs board)
- `pio run -e esp32s3_application` - Build for ESP32S3 (custom UWB board)
- `pio run -e native` - Build native tests with GoogleTest

### Testing
- `pio test` - Run all tests
- `pio test -e native` - Run native tests only
- `pio test -e esp32_application` - Run hardware tests on ESP32

### Upload
- `pio run -e esp32_application -t upload` - Upload to ESP32
- `pio run -e esp32s3_application -t upload` - Upload to ESP32S3

### Compilation Testing Requirements
**CRITICAL**: Before pushing any changes to remote branches, ALWAYS verify compilation:
- `pio run -e esp32_application` - Must compile successfully
- `pio run -e esp32s3_application` - Must compile successfully
- Both targets must build without errors before any git push operation
- This prevents broken builds from being introduced to the codebase

### Board Support
- **ESP32 (Makerfabs)**: `MAKERFABS_ESP32_BOARD` flag
- **ESP32S3 (Konex UWB)**: `ESP32S3_UWB_BOARD` flag

### Main internal FreeRTOS tasks
Main tasks run at different frequencies:
- Application task: 10Hz
- WiFi task: 50Hz
- Console task: 50Hz
- UWB ranging: Main loop (continuous)

## Library Dependencies

ESP32 environments use:
- ETL (Embedded Template Library) for containers
- Eigen for matrix operations
- AsyncTCP/ESPAsyncWebServer for web interface
- SimpleCLI for command parsing
- Many others ...

## Important Files

- `src/main.cpp`: Application entry point and task setup
- `src/app.cpp`: Main application logic
- `src/bsp/`: Board support package definitions
- `src/uwb/`: UWB-related implementations
- `src/wifi/`: WiFi and web server implementations
- `scripts/`: Python utilities for testing and parameter management
- `tools/` : Tools like the `rtls-link-manager` desktop application for configuring and interacting with the system 
- `platformio.ini`: Build configuration

# Important notes

- When adding new features to the code base, make sure to wrap it into a macro preprocessor feature switch (see features.hpp and user_defines.txt ). If there are any dependencies that need to be made ( like maybe feature X needs of feature Y, make sure to update the feature_validation.hpp ).
- When adding a new functionality from scratch. Always create a new branch called `feature/<title>` before starting to write the code.
- Always when adding a new feature to the firmware ( basically src folder ) you need to maintain in "sync" compatibility with the new feature of the desktop app that you can find inside `tools/rtls-link-manager`. The rtls-link-manager is a git submodule with it's own repository. You should create there the same corresponding branch. When opening the PRs, on the rtls-link repository, you should add a link with the matching feature PR of the `rtls-link-manager`. 