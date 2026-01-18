//! UDP discovery for RTLS-Link devices.
//!
//! Uses SO_REUSEPORT to allow concurrent operation with rtls-link-manager.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::time::Duration;

use socket2::{Domain, Protocol, Socket, Type};
use tokio::net::UdpSocket;
use tokio::time::{timeout, Instant};

use crate::error::{CliError, NetworkError};
use crate::types::{Device, DeviceRole};

/// Default UDP discovery port
pub const DISCOVERY_PORT: u16 = 3333;

/// Discovery options
#[derive(Debug, Clone)]
pub struct DiscoveryOptions {
    /// Port to listen on
    pub port: u16,
    /// Discovery duration
    pub duration: Duration,
    /// Whether to run continuously (watch mode)
    pub watch: bool,
    /// Callback for new devices (watch mode)
    pub on_device: Option<fn(&Device)>,
}

impl Default for DiscoveryOptions {
    fn default() -> Self {
        Self {
            port: DISCOVERY_PORT,
            duration: Duration::from_secs(5),
            watch: false,
            on_device: None,
        }
    }
}

/// Discover devices on the network.
///
/// This function listens for UDP heartbeats and returns discovered devices.
/// Uses SO_REUSEPORT to allow running alongside rtls-link-manager.
pub async fn discover_devices(options: DiscoveryOptions) -> Result<Vec<Device>, CliError> {
    let socket = create_reusable_socket(options.port)?;
    let socket = UdpSocket::from_std(socket.into())?;

    let mut devices: HashMap<String, Device> = HashMap::new();
    let mut buf = vec![0u8; 2048];
    let start = Instant::now();

    loop {
        // Check if we should stop (non-watch mode)
        if !options.watch && start.elapsed() >= options.duration {
            break;
        }

        // Use timeout for recv to allow periodic checks
        let recv_timeout = Duration::from_millis(500);
        match timeout(recv_timeout, socket.recv_from(&mut buf)).await {
            Ok(Ok((len, addr))) => {
                if let Ok(device) = parse_heartbeat(&buf[..len], addr.ip().to_string()) {
                    let is_new = !devices.contains_key(&device.ip);
                    devices.insert(device.ip.clone(), device.clone());

                    // Call callback for new devices in watch mode
                    if is_new {
                        if let Some(callback) = options.on_device {
                            callback(&device);
                        }
                    }
                }
            }
            Ok(Err(e)) => {
                // Log error but continue
                eprintln!("UDP receive error: {}", e);
            }
            Err(_) => {
                // Timeout - continue loop
            }
        }
    }

    let mut device_list: Vec<Device> = devices.into_values().collect();
    device_list.sort_by(|a, b| a.ip.cmp(&b.ip));

    Ok(device_list)
}

/// Create a UDP socket with SO_REUSEPORT for concurrent operation.
fn create_reusable_socket(port: u16) -> Result<std::net::UdpSocket, NetworkError> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))
        .map_err(NetworkError::UdpBind)?;

    // Enable address reuse
    socket.set_reuse_address(true).map_err(NetworkError::UdpBind)?;

    // Enable port reuse (allows multiple processes to bind)
    #[cfg(unix)]
    socket.set_reuse_port(true).map_err(NetworkError::UdpBind)?;

    // Bind to the discovery port
    let addr: SocketAddr = format!("0.0.0.0:{}", port).parse().unwrap();
    socket.bind(&addr.into()).map_err(NetworkError::UdpBind)?;

    // Set non-blocking for tokio
    socket.set_nonblocking(true).map_err(NetworkError::UdpBind)?;

    Ok(socket.into())
}

/// Parse a heartbeat packet into a Device struct.
fn parse_heartbeat(data: &[u8], ip: String) -> Result<Device, serde_json::Error> {
    let json: serde_json::Value = serde_json::from_slice(data)?;

    let log_level = json["log_level"].as_u64().map(|v| v as u8);

    Ok(Device {
        ip,
        id: json["id"].as_str().unwrap_or("").to_string(),
        role: DeviceRole::from_str(json["role"].as_str().unwrap_or("")),
        mac: json["mac"].as_str().unwrap_or("").to_string(),
        uwb_short: json["uwb_short"].as_str().unwrap_or("0").to_string(),
        mav_sys_id: json["mav_sysid"].as_u64().unwrap_or(0) as u8,
        firmware: json["fw"].as_str().unwrap_or("").to_string(),
        online: Some(true),
        last_seen: Some(chrono::Utc::now()),
        sending_pos: json["sending_pos"].as_bool(),
        anchors_seen: json["anchors_seen"].as_u64().map(|v| v as u8),
        origin_sent: json["origin_sent"].as_bool(),
        rf_enabled: json["rf_enabled"].as_bool(),
        rf_healthy: json["rf_healthy"].as_bool(),
        avg_rate_c_hz: json["avg_rate_cHz"].as_u64().map(|v| v as u16),
        min_rate_c_hz: json["min_rate_cHz"].as_u64().map(|v| v as u16),
        max_rate_c_hz: json["max_rate_cHz"].as_u64().map(|v| v as u16),
        log_level,
        log_udp_port: json["log_udp_port"].as_u64().map(|v| v as u16),
        log_serial_enabled: json["log_serial_enabled"].as_bool(),
        log_udp_enabled: json["log_udp_enabled"].as_bool(),
    })
}

/// Watch for devices continuously, calling callback for each update.
pub async fn watch_devices<F>(options: DiscoveryOptions, mut on_update: F) -> Result<(), CliError>
where
    F: FnMut(&[Device]),
{
    let socket = create_reusable_socket(options.port)?;
    let socket = UdpSocket::from_std(socket.into())?;

    let mut devices: HashMap<String, (Device, Instant)> = HashMap::new();
    let mut buf = vec![0u8; 2048];
    let ttl = Duration::from_secs(5);

    loop {
        // Use timeout for recv to allow periodic pruning
        let recv_timeout = Duration::from_secs(1);
        match timeout(recv_timeout, socket.recv_from(&mut buf)).await {
            Ok(Ok((len, addr))) => {
                if let Ok(device) = parse_heartbeat(&buf[..len], addr.ip().to_string()) {
                    devices.insert(device.ip.clone(), (device, Instant::now()));
                }
            }
            Ok(Err(e)) => {
                eprintln!("UDP receive error: {}", e);
            }
            Err(_) => {
                // Timeout - continue to prune
            }
        }

        // Prune stale devices
        let now = Instant::now();
        devices.retain(|_, (_, last_seen)| now.duration_since(*last_seen) < ttl);

        // Call update callback
        let device_list: Vec<Device> = devices.values().map(|(d, _)| d.clone()).collect();
        on_update(&device_list);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_heartbeat() {
        let json = r#"{
            "id": "device1",
            "role": "tag_tdoa",
            "mac": "AA:BB:CC:DD:EE:FF",
            "uwb_short": "1",
            "mav_sysid": 1,
            "fw": "1.0.0",
            "sending_pos": true,
            "anchors_seen": 3
        }"#;

        let device = parse_heartbeat(json.as_bytes(), "192.168.1.100".to_string()).unwrap();

        assert_eq!(device.ip, "192.168.1.100");
        assert_eq!(device.id, "device1");
        assert_eq!(device.role, DeviceRole::TagTdoa);
        assert_eq!(device.sending_pos, Some(true));
        assert_eq!(device.anchors_seen, Some(3));
    }

    #[test]
    fn test_parse_minimal_heartbeat() {
        let json = r#"{"id": "test", "role": "anchor"}"#;
        let device = parse_heartbeat(json.as_bytes(), "10.0.0.1".to_string()).unwrap();

        assert_eq!(device.ip, "10.0.0.1");
        assert_eq!(device.id, "test");
        assert_eq!(device.role, DeviceRole::Anchor);
    }
}
