#!/usr/bin/env python3
"""
RTLS-Link Device Discovery Tool

Discovers RTLS-Link devices on the local network using UDP broadcast.
Sends "RTLS_DISCOVER" to the broadcast address and collects JSON responses.

Usage:
    python discovery.py                         # Auto-detect broadcast address
    python discovery.py --broadcast 192.168.1.255  # Manual broadcast
    python discovery.py --port 3333             # Custom port (default: 3333)
    python discovery.py --timeout 3             # Custom timeout in seconds
"""

import argparse
import json
import socket
import subprocess
import re
import sys
from typing import List, Dict, Optional


DISCOVER_COMMAND = b"RTLS_DISCOVER"
DEFAULT_PORT = 3333
DEFAULT_TIMEOUT = 2.0


def get_broadcast_address() -> str:
    """
    Compute subnet broadcast address from local interface.
    Prefers common LAN subnets (192.168.x.x, 10.x.x.x non-VPN).
    Falls back to 255.255.255.255 if detection fails.
    """
    candidates = []
    try:
        # Linux: parse `ip addr` output
        result = subprocess.run(['ip', 'addr'], capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            # Look for "inet X.X.X.X/XX brd Y.Y.Y.Y" pattern
            match = re.search(r'inet (\d+\.\d+\.\d+\.\d+)/\d+ brd (\d+\.\d+\.\d+\.\d+)', line)
            if match:
                ip = match.group(1)
                brd = match.group(2)
                # Skip loopback
                if ip.startswith('127.'):
                    continue
                # Prioritize 192.168.x.x (most common home/office LAN)
                if ip.startswith('192.168.'):
                    return brd
                # Collect other candidates
                candidates.append((ip, brd))
    except Exception:
        pass

    # Return first non-192.168 candidate if any (e.g., 10.0.x.x LAN)
    # Skip WSL/Docker-like ranges (10.255.x.x, 172.17.x.x)
    for ip, brd in candidates:
        if ip.startswith('10.') and not ip.startswith('10.255.'):
            return brd

    # Last resort: use 255.255.255.255
    return '255.255.255.255'


def discover_devices(broadcast_addr: str, port: int, timeout: float) -> List[Dict]:
    """
    Send discovery broadcast and collect responses.

    Args:
        broadcast_addr: Broadcast address to use
        port: UDP port to use
        timeout: Timeout in seconds

    Returns:
        List of device info dictionaries
    """
    devices = []
    seen_ips = set()

    # Create UDP socket with broadcast enabled
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.settimeout(timeout)

    try:
        # Bind to any address to receive responses
        sock.bind(('', 0))

        # Send discovery broadcast
        sock.sendto(DISCOVER_COMMAND, (broadcast_addr, port))

        # Collect responses until timeout
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                ip = addr[0]

                # Skip duplicates
                if ip in seen_ips:
                    continue
                seen_ips.add(ip)

                # Parse JSON response
                try:
                    device_info = json.loads(data.decode('utf-8'))
                    device_info['_source_ip'] = ip  # Add source IP for reference
                    devices.append(device_info)
                except json.JSONDecodeError:
                    print(f"Warning: Invalid JSON from {ip}: {data}", file=sys.stderr)

            except socket.timeout:
                break

    finally:
        sock.close()

    return devices


def print_devices_table(devices: List[Dict]) -> None:
    """Print discovered devices in a formatted table."""
    if not devices:
        print("No devices found.")
        return

    print(f"\nFound {len(devices)} device(s):\n")

    # Header
    header = f"{'ID':<12} {'Role':<12} {'IP':<16} {'MAC':<20} {'UWB':<6} {'MAV':<5} {'Firmware':<10}"
    print(header)
    print("-" * len(header))

    # Rows
    for dev in devices:
        mav_sysid = dev.get('mav_sysid', 'N/A')
        if isinstance(mav_sysid, int):
            mav_sysid = str(mav_sysid)
        row = "{:<12} {:<12} {:<16} {:<20} {:<6} {:<5} {:<10}".format(
            dev.get('id', 'N/A')[:12],
            dev.get('role', 'N/A')[:12],
            dev.get('ip', dev.get('_source_ip', 'N/A'))[:16],
            dev.get('mac', 'N/A')[:20],
            dev.get('uwb_short', 'N/A')[:6],
            str(mav_sysid)[:5],
            dev.get('fw', 'N/A')[:10]
        )
        print(row)


def main():
    parser = argparse.ArgumentParser(
        description='Discover RTLS-Link devices on the local network'
    )
    parser.add_argument(
        '--broadcast', '-b',
        type=str,
        default=None,
        help='Broadcast address (auto-detected if not specified)'
    )
    parser.add_argument(
        '--port', '-p',
        type=int,
        default=DEFAULT_PORT,
        help=f'UDP port (default: {DEFAULT_PORT})'
    )
    parser.add_argument(
        '--timeout', '-t',
        type=float,
        default=DEFAULT_TIMEOUT,
        help=f'Timeout in seconds (default: {DEFAULT_TIMEOUT})'
    )
    parser.add_argument(
        '--json', '-j',
        action='store_true',
        help='Output raw JSON instead of table'
    )

    args = parser.parse_args()

    # Determine broadcast address
    broadcast_addr = args.broadcast or get_broadcast_address()

    print(f"Discovering RTLS-Link devices on {broadcast_addr}:{args.port}...")

    devices = discover_devices(broadcast_addr, args.port, args.timeout)

    if args.json:
        print(json.dumps(devices, indent=2))
    else:
        print_devices_table(devices)


if __name__ == '__main__':
    main()
