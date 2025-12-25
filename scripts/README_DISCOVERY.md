# RTLS-Link Device Discovery Tool

A Python utility to discover RTLS-Link devices on your local network using UDP broadcast.

## Requirements

- Python 3.6+
- No external dependencies (uses only standard library)

## Usage

### Basic Discovery

```bash
python discovery.py
```

This will auto-detect your network's broadcast address and scan for devices.

### Specify Broadcast Address

```bash
python discovery.py --broadcast 192.168.0.255
```

Use this when auto-detection fails or you want to scan a specific subnet.

### Options

| Option | Short | Description | Default |
|--------|-------|-------------|---------|
| `--broadcast` | `-b` | Broadcast address to use | Auto-detect |
| `--port` | `-p` | UDP port for discovery | 3333 |
| `--timeout` | `-t` | Response timeout in seconds | 2.0 |
| `--json` | `-j` | Output raw JSON instead of table | False |

### Examples

```bash
# Scan with custom timeout
python discovery.py --timeout 5

# Scan specific subnet with custom port
python discovery.py --broadcast 10.0.0.255 --port 3333

# Get raw JSON output (useful for scripting)
python discovery.py --json
```

## Output

### Table Format (default)

```
Discovering RTLS-Link devices on 192.168.0.255:3333...

Found 3 device(s):

ID           Role         IP               MAC                  UWB    MAV   Firmware
----------------------------------------------------------------------------------
01           anchor       192.168.0.101    AA:BB:CC:DD:EE:01    01     1     1.0.0
02           anchor       192.168.0.102    AA:BB:CC:DD:EE:02    02     1     1.0.0
1            tag_tdoa     192.168.0.100    9C:13:9E:99:7D:E0    1      2     1.0.0
```

### JSON Format (`--json`)

```json
[
  {
    "device": "rtls-link",
    "id": "01",
    "role": "anchor",
    "ip": "192.168.0.101",
    "mac": "AA:BB:CC:DD:EE:01",
    "uwb_short": "01",
    "mav_sysid": 1,
    "fw": "1.0.0"
  }
]
```

## Response Fields

| Field | Description |
|-------|-------------|
| `device` | Device type (always "rtls-link") |
| `id` | Device ID (same as UWB short address) |
| `role` | UWB role: `anchor`, `tag`, `anchor_tdoa`, `tag_tdoa`, `calibration` |
| `ip` | Device IP address |
| `mac` | Device MAC address |
| `uwb_short` | UWB short address (2 characters) |
| `mav_sysid` | MAVLink target system ID |
| `fw` | Firmware version |

## Protocol Details

- **Port:** 3333 (UDP) - configurable via `wifi.discoveryPort` parameter on device
- **Command:** Send `RTLS_DISCOVER` as UDP payload
- **Response:** JSON payload with device information

## Troubleshooting

### No devices found

1. **Check network connectivity:** Can you ping the device?
   ```bash
   ping 192.168.0.100
   ```

2. **Verify broadcast address:** Make sure you're using the correct subnet broadcast
   ```bash
   python discovery.py --broadcast 192.168.0.255
   ```

3. **Check device is running:** The serial output should show:
   ```
   WifiDiscovery: Listening on UDP port 3333
   Discovery service enabled on port 3333
   ```

4. **Firewall:** Ensure UDP port 3333 is not blocked

### WSL2 Limitations

WSL2 uses NAT networking and cannot reliably send/receive UDP to devices on the physical LAN. **Run this script from Windows directly**, not from WSL.

```powershell
# From Windows PowerShell
python C:\path\to\discovery.py --broadcast 192.168.0.255
```

### Socket Error: Unreachable Network

Check that:
- You're connected to the correct network
- The broadcast address is correct (e.g., `192.168.0.255` not `102.168.0.255`)
- Your firewall allows outbound UDP

## Device Configuration

Discovery is enabled by default on RTLS-Link devices. To configure:

```
# Via device console
wifi.enableDiscovery: 1      # Enable (default)
wifi.enableDiscovery: 0      # Disable
wifi.discoveryPort: 3333     # Change port (default: 3333)
```
