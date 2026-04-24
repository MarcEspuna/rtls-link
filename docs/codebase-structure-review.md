# Codebase Structure Review

## Purpose

This report summarizes the current structure of the `rtls-link` repository, the apparent product goals, and the main areas where refactor and cleanup would likely provide the most value.

## High-Level Product Shape

This repository contains two tightly related systems:

- Firmware for ESP32 and ESP32S3 UWB RTLS devices.
- A desktop management application in `tools/rtls-link-manager` for discovery, configuration, monitoring, logging, and OTA updates.

The firmware is responsible for UWB ranging and positioning, runtime configuration, network services, and output integration such as MAVLink. The desktop app acts as the operator-facing control plane around those devices.

## Firmware Architecture Summary

### Build and Feature Model

The build is organized through PlatformIO in [platformio.ini](../platformio.ini), with separate environments for:

- `esp32_application`
- `esp32s3_application`
- `native`

Compile-time behavior is driven by:

- [src/config/features.hpp](../src/config/features.hpp)
- [src/config/feature_validation.hpp](../src/config/feature_validation.hpp)
- board-specific define files under `boards/`

This is one of the stronger parts of the codebase. The feature model is explicit, centralized, and guarded with compile-time validation.

### Startup and Runtime Flow

At startup, [src/main.cpp](../src/main.cpp):

- initializes `App`
- initializes the scheduler
- initializes registered frontends
- initializes console/command handling when enabled
- creates periodic tasks for app, WiFi, console, and status LED

The notable exception is UWB runtime execution: UWB still runs directly in `loop()` through `Front::uwbLittleFSFront.Update()`. The code comments indicate that moving it into a periodic task caused watchdog or timing problems.

That means runtime ownership is currently split across:

- `loop()`
- scheduler-created tasks
- subsystem-created tasks

This works, but it makes execution behavior harder to reason about.

### Main Firmware Modules

- [src/app.cpp](../src/app.cpp) and [src/app.hpp](../src/app.hpp)
  - cross-cutting application logic
  - MAVLink output
  - origin/heartbeat handling
  - rangefinder forwarding
  - health and update-rate tracking

- [src/front.cpp](../src/front.cpp), [src/front.hpp](../src/front.hpp), and [src/littlefs_frontend.hpp](../src/littlefs_frontend.hpp)
  - frontend registry
  - grouped parameter access
  - shared LittleFS-backed persistence in `/params.txt`

- [src/uwb/uwb_frontend_littlefs.cpp](../src/uwb/uwb_frontend_littlefs.cpp)
  - selects and owns the active UWB backend for the configured mode

- `src/uwb/*`
  - actual ranging and positioning implementations
  - TWR anchor/tag
  - TDoA anchor/tag
  - calibration

- [src/wifi/wifi_frontend_littlefs.cpp](../src/wifi/wifi_frontend_littlefs.cpp)
  - WiFi mode management
  - backend lifecycle
  - discovery, WebSocket, UART bridge, and TCP logging setup

- [src/command_handler/command_handler.cpp](../src/command_handler/command_handler.cpp)
  - shared command surface for console and WebSocket
  - parameter commands
  - config commands
  - diagnostics
  - control commands

- [src/config_manager/config_manager.cpp](../src/config_manager/config_manager.cpp)
  - named config snapshots layered on top of the main parameter store

## Desktop App Architecture Summary

The desktop app in `tools/rtls-link-manager` is a React frontend over a Tauri backend.

### Frontend

The main state owner is [tools/rtls-link-manager/src/App.tsx](../tools/rtls-link-manager/src/App.tsx):

- discovered devices
- selected device
- selected anchors/tags
- active tab
- expert mode

The frontend mostly orchestrates:

- live device discovery state
- configuration workflows
- presets
- firmware updates
- logging UI

### Backend

The Tauri backend in [tools/rtls-link-manager/src-tauri/src/lib.rs](../tools/rtls-link-manager/src-tauri/src/lib.rs):

- starts device discovery
- starts log receiver services
- registers config and preset storage
- exposes Tauri commands for the frontend

The IPC boundary is wrapped in [tools/rtls-link-manager/src/lib/tauri-api.ts](../tools/rtls-link-manager/src/lib/tauri-api.ts), which is a good structure overall.

## Main Structural Strengths

- Compile-time feature switches are centralized and reasonably disciplined.
- Subsystems are grouped by domain rather than mixed arbitrarily.
- The desktop app has a clear frontend/backend split.
- Shared utilities under `tools/rtls-link-manager/shared/` help avoid some frontend duplication.
- Native tests exist around estimator/math code, which protects the algorithm layer better than the orchestration layer.

## Main Structural Weaknesses

### 1. Heavy Global Coupling

The firmware is composed mostly through globals and static registration:

- `App app`
- `Front::uwbLittleFSFront`
- `Front::wifiLittleFSFront`
- global/static command handling state

Subsystems reach into one another directly rather than through narrow interfaces. This is visible in:

- [src/app.cpp](../src/app.cpp)
- [src/wifi/wifi_frontend_littlefs.cpp](../src/wifi/wifi_frontend_littlefs.cpp)
- [src/uwb/uwb_tag.cpp](../src/uwb/uwb_tag.cpp)

### 2. Parameter and Persistence Logic Is Too Diffuse

Parameter definition, parameter access, persistence, config snapshots, and command serialization are spread across:

- `front.*`
- `littlefs_frontend.hpp`
- `config_manager.*`
- `command_handler.*`

The current text-based `/params.txt` approach is workable, but it is duplicated and fragile.

### 3. Runtime Ownership Is Not Centralized

The current execution model mixes:

- `loop()`
- periodic tasks via `Scheduler`
- tasks created inside subsystems

That makes timing and watchdog behavior harder to analyze and harder to refactor safely.

### 4. CommandHandler Is Monolithic

[src/command_handler/command_handler.cpp](../src/command_handler/command_handler.cpp) is handling too many concerns in one place:

- command registration
- response formatting
- parameter quirks
- config snapshot logic
- diagnostics
- LED control
- UWB control

This is functional but hard to evolve cleanly.

### 5. Legacy and Duplicate Paths Still Exist

There is still an older EEPROM-era UWB frontend implementation in:

- [src/uwb/uwb_frontend.cpp](../src/uwb/uwb_frontend.cpp)
- [src/uwb/uwb_frontend.hpp](../src/uwb/uwb_frontend.hpp)

The active firmware path uses the LittleFS-based frontend instead. Keeping both increases maintenance cost and architectural noise.

On the desktop side, there is also duplicated configuration UI logic between:

- [tools/rtls-link-manager/src/components/ConfigModal/ConfigModal.tsx](../tools/rtls-link-manager/src/components/ConfigModal/ConfigModal.tsx)
- [tools/rtls-link-manager/src/components/ConfigPanel/ConfigPanel.tsx](../tools/rtls-link-manager/src/components/ConfigPanel/ConfigPanel.tsx)

## Recommended Refactor Priorities

### Priority 1: Remove Dead and Duplicate Paths

Start with low-risk cleanup that reduces cognitive load:

- remove the unused EEPROM-era UWB frontend path
- reduce duplicated frontend config workflows in the manager app
- consolidate duplicated device-list/panel patterns where practical

### Priority 2: Introduce a Clear Parameter Repository Layer

Define one place that owns:

- param definitions
- serialization/deserialization
- persistence
- config snapshot import/export

This should replace the current spread across `LittleFSFrontend`, `ConfigManager`, and command formatting logic.

### Priority 3: Split Command Handling by Domain

Keep one command executor surface, but split implementation into modules such as:

- param commands
- config commands
- diagnostics commands
- app/device commands

That would make the command system easier to test and maintain.

### Priority 4: Define Explicit Runtime Boundaries

Clarify who owns:

- task creation
- event delivery
- status publication
- sample publication

In particular, a small boundary between UWB, App, and WiFi would help reduce direct reach-through.

Examples:

- `PositionPublisher`
- `RuntimeStatusProvider`
- `TelemetrySnapshot`

### Priority 5: Tighten Desktop IPC Contracts

The Tauri boundary is already reasonably clean, but it would benefit from:

- stronger typed return values
- fewer `any` and loose JSON-shaped responses
- keeping all backend communication behind `tauri-api.ts`

## Suggested Refactor Order

1. Remove dead code and obvious UI duplication.
2. Refactor parameter/config persistence into a dedicated repository/serializer layer.
3. Split `CommandHandler` into domain-specific command modules.
4. Centralize task/runtime ownership.
5. Introduce cleaner telemetry and output interfaces between UWB, App, and WiFi.

## Important Note on Risk

The current automated tests are stronger around estimator/math code than around firmware orchestration and configuration workflows. Because of that, the highest-risk refactors are not in the solver code, but in:

- runtime wiring
- parameter persistence
- command handling
- cross-subsystem integration

That suggests a practical strategy: refactor the control plane first, not the positioning algorithms.
