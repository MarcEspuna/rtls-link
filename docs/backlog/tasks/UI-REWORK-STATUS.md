# UI Rework Implementation Status

This document tracks the implementation status of the UI rework for the rtls-link-manager desktop application.

## Completed Work

### Phase 1: Layout Restructuring
- [x] Created new layout structure with `AppShell`, `Sidebar`, and `Header` components
- [x] Implemented tabbed navigation (Anchors, Tags, Presets)
- [x] Added expert mode toggle in header with localStorage persistence
- [x] Created `useSettings` hook for persisting user preferences

### Phase 2: Device Role Separation
- [x] Added role helper functions (`isAnchorRole`, `isTagRole`) in `shared/types.ts`
- [x] Created `AnchorsPanel` and `AnchorCard` components for anchor devices
- [x] Created `TagsPanel` and `TagCard` components for tag devices
- [x] Implemented separate selection state per device type (`selectedAnchorIps`, `selectedTagIps`)
- [x] IP-based device identification (dropped UWB ID as primary identifier)

### Phase 3: Health Status System
- [x] Created `healthStatus.ts` with pure health calculation logic
- [x] Created `HealthBadge` component with color-coded indicators
- [x] Implemented health levels: healthy (green), warning (yellow), degraded (orange), unknown (gray)
- [x] Added 14 unit tests for health status calculation
- [x] Proper handling of old firmware (undefined telemetry = unknown, not warning)

Health calculation rules for tags:
- `sendingPos === false` → degraded
- `anchorsSeen < 3` → warning
- `originSent === false` → warning
- `rfEnabled && !rfHealthy` → warning
- Undefined fields → unknown (graceful degradation)

### Phase 4: Expert Mode
- [x] Integrated expert mode toggle into header
- [x] Updated `ConfigEditor` with conditional field rendering
- [x] Basic mode shows: UWB mode, short address, anchor list, origin coordinates
- [x] Expert mode adds: WiFi settings, rotation, MAVLink settings, Z calculation, rangefinder settings, covariance/RMSE

### Phase 5: Unified Presets System
- [x] Extended Rust backend with preset types (`Preset`, `PresetInfo`, `PresetType`, `LocationData`)
- [x] Created `PresetStorageService` in Rust (mirrors ConfigStorageService pattern)
- [x] Added Tauri commands: `list_presets`, `get_preset`, `save_preset`, `delete_preset`
- [x] Updated frontend API (`tauri-api.ts`) with preset functions
- [x] Created `PresetsPanel` component with unified preset management
- [x] Supports two preset types: `full` (complete DeviceConfig) and `locations` (anchors + origin only)
- [x] Location presets upload to tags only (anchors are skipped automatically)

### Phase 6: WebSocket Consolidation
- [x] Created `deviceCommands.ts` utility module with reusable WebSocket functions
- [x] Refactored `BulkActions` to use shared utilities
- [x] Refactored `PresetsPanel` to use shared utilities
- [x] Utilities: `sendDeviceCommand`, `sendDeviceCommands`, `executeBulkCommand`, `executeBulkOperation`

---

## Remaining Work

### 1. Debug Socket Integration (Deferred)
**Status**: Skipped for now (per user decision)

The firmware needs to implement a proper debug socket protocol. Once available:
- Add a Debug tab to the sidebar
- Subscribe to debug events from devices
- Display real-time debug information
- This will be essential for developing automatic anchor positioning

**Firmware requirement**: Implement debug socket protocol (WebSocket or dedicated UDP port)

### 2. Automatic Anchor Positioning
**Status**: Not started - requires firmware changes first

The idea is for tags to automatically determine anchor positions by analyzing distance measurements that anchors encode in UWB packets.

**Dependencies**:
- Firmware must encode distance measurements in UWB packets
- Algorithm to calculate anchor positions assuming rectangular shape
- Origin anchor (anchor 0) defines the coordinate system origin

**UI considerations**:
- Add "Auto-position anchors" button in Presets or Tags panel
- Show progress/status during positioning
- Preview calculated positions before saving
- Potentially add a 2D visualization component (`LocationPreview`)

### 3. Firmware Telemetry Updates
**Status**: Desktop app ready, firmware may need updates

The desktop app can now display telemetry-based health status, but the firmware needs to provide these fields in the heartbeat/telemetry:
- `sendingPos`: boolean - whether tag is actively sending position data
- `anchorsSeen`: number - count of anchors the tag can see
- `originSent`: boolean - whether origin coordinates have been sent
- `rfEnabled`: boolean - whether rangefinder is enabled
- `rfHealthy`: boolean - whether rangefinder is working correctly

**Current behavior**: Missing telemetry fields result in "unknown" health status (gray badge)

### 4. OTA Updates Tab
**Status**: Not in current scope

Future feature to add firmware update capability directly from the desktop app.

**Considerations**:
- Add OTA tab to sidebar
- Upload firmware binary
- Select devices for update
- Progress tracking per device
- Rollback capability

### 5. UWB Diagnostics Tab
**Status**: Not in current scope

Future feature for advanced UWB debugging and diagnostics.

**Considerations**:
- Real-time ranging data visualization
- Anchor-to-anchor distances
- Signal quality metrics
- Timing diagnostics

---

## File Structure Created

```
src/
├── components/
│   ├── Layout/
│   │   ├── AppShell.tsx, AppShell.module.css
│   │   ├── Sidebar.tsx, Sidebar.module.css
│   │   ├── Header.tsx, Header.module.css
│   │   └── index.ts
│   ├── Anchors/
│   │   ├── AnchorsPanel.tsx, AnchorsPanel.module.css
│   │   ├── AnchorCard.tsx, AnchorCard.module.css
│   │   └── index.ts
│   ├── Tags/
│   │   ├── TagsPanel.tsx, TagsPanel.module.css
│   │   ├── TagCard.tsx, TagCard.module.css
│   │   └── index.ts
│   ├── Presets/
│   │   ├── PresetsPanel.tsx, PresetsPanel.module.css
│   │   └── index.ts
│   └── common/
│       └── HealthBadge.tsx
├── lib/
│   ├── healthStatus.ts
│   └── deviceCommands.ts
├── hooks/
│   └── useSettings.ts
└── App.tsx (refactored)

src-tauri/src/
├── preset_storage/
│   ├── mod.rs
│   └── service.rs
├── commands/
│   └── presets.rs
├── types.rs (extended with Preset types)
└── lib.rs (registers preset service and commands)

shared/
├── types.ts (extended with role helpers, Preset types)
└── tests/
    └── healthStatus.test.ts (14 tests)
```

---

## Key Design Decisions

1. **Offline devices**: Keep current TTL behavior - devices disappear from UI after 5s of no heartbeat
2. **Presets**: Merged "Local Configs" and "Locations" into unified Presets system with two types
3. **Location uploads**: Target tags only - anchors don't need position data
4. **Expert mode**: Persisted to localStorage, affects ConfigEditor field visibility
5. **Health status**: Unknown telemetry = "unknown" status (not warning) for backwards compatibility

---

## Testing

Run tests:
```bash
cd tools/rtls-link-manager
npm test
```

Current test coverage:
- 14 health status tests (all passing)
- 4 commands tests
- 2 config tests

Build verification:
```bash
npm run build
# or
npm run tauri dev  # for development
```

---

## Migration Notes

### For Users
- Old "Local Configs" are conceptually replaced by "Presets" with `type: 'full'`
- UI has moved to tabbed layout - Anchors, Tags, and Presets are now separate tabs
- Expert mode toggle in top-right header controls advanced field visibility

### For Firmware Development
When updating firmware to support new features:

1. **Telemetry fields to add to heartbeat**:
   ```cpp
   struct TelemetryData {
     bool sendingPos;      // Is tag actively sending position?
     uint8_t anchorsSeen;  // Number of anchors in range
     bool originSent;      // Have origin coordinates been sent?
     bool rfEnabled;       // Is rangefinder enabled?
     bool rfHealthy;       // Is rangefinder healthy?
   };
   ```

2. **Debug socket** (future):
   - Define protocol for debug messages
   - Consider WebSocket on separate port or path (e.g., `/debug`)
   - Message format should include: timestamp, level, component, message

3. **Auto-positioning** (future):
   - Anchors must encode distance measurements in UWB packets
   - Define packet format for inter-anchor ranging data
   - Tags must be able to read and process this data
