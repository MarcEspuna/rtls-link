# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an ESP32/ESP32S3 firmware project for real-time localization systems (RTLS) using Ultra-Wideband (UWB) technology. The firmware implements Time Difference of Arrival (TDoA) algorithms for 2D/3D positioning and provides MAVLink integration for drone applications.

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

## Development Environment

### Docker Setup
The project includes a containerized development environment:
```bash
cd docker
docker-compose up -d
docker exec -it platformio-development bash
```

### Board Support
- **ESP32 (Makerfabs)**: `MAKERFABS_ESP32_BOARD` flag
- **ESP32S3 (Konex UWB)**: `ESP32S3_UWB_BOARD` flag

## Architecture

### Core Components

1. **Task Scheduler** (`src/scheduler.hpp`):
   - FreeRTOS-based task management
   - Static and dynamic task creation with ETL delegates
   - Periodic and continuous task types

2. **Frontend/Backend Pattern**:
   - **UWB Frontend** (`src/uwb/uwb_frontend.hpp`): High-level UWB interface
   - **UWB Backend** (`src/uwb/uwb_backend.hpp`): Hardware abstraction layer
   - **WiFi Frontend/Backend**: Network communication management

3. **Application Core** (`src/app.hpp`):
   - Main application logic
   - MAVLink position sensor integration
   - Coordinate transformation with NED frame orientation

4. **Parameter System** (`src/front.hpp`):
   - EEPROM-based parameter storage
   - Type-safe parameter access
   - ASCII string-based parameter interface

### UWB Implementation

The UWB subsystem supports multiple modes:
- **TWR (Two-Way Ranging)**: Anchor/Tag modes
- **TDoA (Time Difference of Arrival)**: Anchor/Tag modes
- **Calibration**: Antenna delay calibration

Key libraries:
- `DW1000_mf`: DW1000 chip driver
- `Trilat`: Trilateration algorithms
- `tdoa_algorithm`: TDoA positioning
- `tdoa_estimator`: Position estimation

### MAVLink Integration

The system integrates with ArduPilot via MAVLink:
- Local position sensor messages
- UART communication interface
- Configurable system/component IDs
- Automatic origin coordinate transmission

## Key Configuration

### Coordinate System
- Uses NED (North-East-Down) coordinate frame
- Configurable rotation offset: 40 degrees (hardcoded)
- Origin coordinates: Barcelona area (hardcoded)

### Task Configuration
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
- SimpleKalmanFilter for filtering
- ArduinoLog for logging

## Board-Specific Notes

### ESP32S3 Configuration
- Uses `huge_app.csv` partition table
- Eigen stack allocation limited due to memory constraints
- USB serial communication specifics

### ESP32 Configuration
- Standard ESP-WROVER-KIT board
- Makerfabs-specific board definitions

## Testing Strategy

The project includes unit tests for:
- 2D/3D trilateration algorithms
- TDoA estimation
- Eigen matrix operations
- Direct 3D positioning

Test data and plotting utilities are available in the `test/` directory.

## Important Files

- `src/main.cpp`: Application entry point and task setup
- `src/app.cpp`: Main application logic
- `src/bsp/`: Board support package definitions
- `src/uwb/`: UWB-related implementations
- `src/wifi/`: WiFi and web server implementations
- `scripts/`: Python utilities for testing and parameter management
- `platformio.ini`: Build configuration