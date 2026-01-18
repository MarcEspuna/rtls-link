# RTLS-Link CLI

Command-line interface for RTLS-Link device management. Provides terminal access to all RTLS-Link device management functionality, enabling automation via scripts and headless operation.

## Installation

```bash
cd tools/rtls-link-cli
cargo build --release
```

The binary will be at `target/release/rtls-link-cli`.

## Quick Start

```bash
# Discover devices on the network
rtls-link-cli discover

# Show status of all devices with health analysis
rtls-link-cli status all --health

# Backup device configuration
rtls-link-cli config backup 192.168.1.100 -o config.json

# Send a raw command
rtls-link-cli cmd 192.168.1.100 "version"
```

## Commands

### Global Options

```
--json           Output in JSON format (for scripting)
--timeout <MS>   Command timeout in milliseconds (default: 5000)
-v, --verbose    Verbose output
--strict         Exit non-zero on any partial failure
```

### discover

Discover devices on the network.

```bash
# One-shot discovery (5 seconds)
rtls-link-cli discover

# Custom duration
rtls-link-cli discover -d 10

# Watch mode (continuous)
rtls-link-cli discover -w

# Filter by role
rtls-link-cli discover --filter-role tag-tdoa

# JSON output for scripting
rtls-link-cli discover --json
```

### status

Show device status with optional health analysis.

```bash
# Single device
rtls-link-cli status 192.168.1.100

# All devices
rtls-link-cli status all

# With health analysis
rtls-link-cli status all --health
```

### config

Device configuration management.

```bash
# Backup configuration
rtls-link-cli config backup 192.168.1.100 -o config.json

# Apply configuration from file
rtls-link-cli config apply 192.168.1.100 config.json

# Read a single parameter
rtls-link-cli config read 192.168.1.100 -g wifi -n gcsIp

# Write a parameter (with optional save)
rtls-link-cli config write 192.168.1.100 -g wifi -n gcsIp -d "192.168.1.1" --save

# List saved configurations on device
rtls-link-cli config list 192.168.1.100

# Save current config with name
rtls-link-cli config save-as 192.168.1.100 my-config

# Load named configuration
rtls-link-cli config load 192.168.1.100 my-config

# Delete named configuration
rtls-link-cli config delete 192.168.1.100 my-config
```

### preset

Local preset management (shared storage with rtls-link-manager).

```bash
# List all presets
rtls-link-cli preset list

# Show preset details
rtls-link-cli preset show my-preset

# Save preset from device
rtls-link-cli preset save my-preset --from-device 192.168.1.100

# Save locations-only preset
rtls-link-cli preset save field-setup --from-device 192.168.1.100 --preset-type locations

# Delete preset
rtls-link-cli preset delete my-preset

# Upload preset to device(s)
rtls-link-cli preset upload my-preset 192.168.1.100
rtls-link-cli preset upload my-preset all --filter-role tag-tdoa
```

### ota

Firmware updates via HTTP OTA.

```bash
# Update single device
rtls-link-cli ota update 192.168.1.100 firmware.bin

# Update all devices
rtls-link-cli ota update all firmware.bin

# Update with role filter and concurrency
rtls-link-cli ota update all firmware.bin --filter-role tag --concurrency 5
```

### logs

Stream logs from devices (UDP).

```bash
# Stream logs from all devices
rtls-link-cli logs

# Filter by device
rtls-link-cli logs 192.168.1.100

# Filter by log level
rtls-link-cli logs -l warn

# Filter by tag pattern
rtls-link-cli logs -t "uwb*"

# Output as NDJSON
rtls-link-cli logs --ndjson
```

### cmd

Send raw commands to devices.

```bash
# Simple command
rtls-link-cli cmd 192.168.1.100 "version"

# Command expecting JSON response
rtls-link-cli cmd 192.168.1.100 "backup-config" --expect-json
```

### bulk

Bulk device operations.

```bash
# Toggle LED on all devices
rtls-link-cli bulk toggle-led

# Reboot all tags
rtls-link-cli bulk reboot --filter-role tag

# Start positioning on specific devices
rtls-link-cli bulk start --ips 192.168.1.100,192.168.1.101

# Send custom command to all devices
rtls-link-cli bulk cmd "version"
```

## Exit Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | General error |
| 2 | Network error |
| 3 | Device command failed |
| 4 | Invalid arguments |
| 5 | Partial failure (bulk, with --strict) |

## Scripting Examples

### Backup all devices

```bash
#!/bin/bash
for ip in $(rtls-link-cli discover --json | jq -r '.devices[].ip'); do
    rtls-link-cli config backup "$ip" -o "backup_${ip}.json"
done
```

### Check device health

```bash
#!/bin/bash
rtls-link-cli status all --health --json | jq '.[] | select(.health.level != "healthy")'
```

### Automated firmware update

```bash
#!/bin/bash
rtls-link-cli ota update all firmware.bin --strict
if [ $? -eq 0 ]; then
    echo "All devices updated successfully"
else
    echo "Some devices failed to update"
    exit 1
fi
```

## Interoperability

This CLI shares preset storage with `rtls-link-manager` (the desktop app). Presets saved with either tool are accessible from both.

Storage location:
- Linux: `~/.local/share/rtls-link-manager/presets/`
- macOS: `~/Library/Application Support/rtls-link-manager/presets/`
- Windows: `%APPDATA%\rtls-link\rtls-link-manager\presets\`

## License

MIT
