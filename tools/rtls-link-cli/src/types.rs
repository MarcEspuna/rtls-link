//! Type definitions for RTLS-Link CLI.
//!
//! These types mirror the definitions in rtls-link-manager to ensure compatibility
//! with device protocols and shared preset storage.

use chrono::{DateTime, Utc};
use serde::{Deserialize, Serialize};

/// Represents a discovered RTLS-Link device.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Device {
    /// Device IP address (primary identifier for runtime)
    pub ip: String,
    /// Device identifier string
    pub id: String,
    /// Operating mode/role
    pub role: DeviceRole,
    /// MAC address
    pub mac: String,
    /// UWB short address (1-2 digits)
    pub uwb_short: String,
    /// MAVLink system ID
    pub mav_sys_id: u8,
    /// Firmware version
    pub firmware: String,
    /// Whether the device is currently online
    #[serde(skip_serializing_if = "Option::is_none")]
    pub online: Option<bool>,
    /// Last seen timestamp
    #[serde(skip_serializing_if = "Option::is_none")]
    pub last_seen: Option<DateTime<Utc>>,
    /// Whether device is sending positions to ArduPilot
    #[serde(skip_serializing_if = "Option::is_none")]
    pub sending_pos: Option<bool>,
    /// Number of unique anchors in measurement set
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anchors_seen: Option<u8>,
    /// Whether GPS origin was sent to ArduPilot
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_sent: Option<bool>,
    /// Whether rangefinder mode is enabled (zCalcMode == RANGEFINDER)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rf_enabled: Option<bool>,
    /// Whether receiving non-stale rangefinder data
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rf_healthy: Option<bool>,
    /// Average update rate in centi-Hz (e.g., 1000 = 10.0 Hz)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub avg_rate_c_hz: Option<u16>,
    /// Min update rate in last 5s window (centi-Hz)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub min_rate_c_hz: Option<u16>,
    /// Max update rate in last 5s window (centi-Hz)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub max_rate_c_hz: Option<u16>,
    /// Compiled log level (0=NONE, 1=ERROR, 2=WARN, 3=INFO, 4=DEBUG, 5=VERBOSE)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_level: Option<u8>,
    /// UDP port for log streaming
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_udp_port: Option<u16>,
    /// Whether Serial logging is enabled at runtime
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_serial_enabled: Option<bool>,
    /// Whether UDP log streaming is enabled at runtime
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_udp_enabled: Option<bool>,
}

/// Device operating role/mode.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DeviceRole {
    /// TWR Anchor mode (0)
    Anchor,
    /// TWR Tag mode (1)
    Tag,
    /// TDoA Anchor mode (3)
    AnchorTdoa,
    /// TDoA Tag mode (4)
    TagTdoa,
    /// Calibration mode (2)
    Calibration,
    /// Unknown/unrecognized mode
    Unknown,
}

impl DeviceRole {
    /// Parse a role string from device heartbeat
    pub fn from_str(s: &str) -> Self {
        match s {
            "anchor" => DeviceRole::Anchor,
            "tag" => DeviceRole::Tag,
            "anchor_tdoa" => DeviceRole::AnchorTdoa,
            "tag_tdoa" => DeviceRole::TagTdoa,
            "calibration" => DeviceRole::Calibration,
            _ => DeviceRole::Unknown,
        }
    }

    /// Check if role is an anchor type
    pub fn is_anchor(&self) -> bool {
        matches!(self, DeviceRole::Anchor | DeviceRole::AnchorTdoa)
    }

    /// Check if role is a tag type
    pub fn is_tag(&self) -> bool {
        matches!(self, DeviceRole::Tag | DeviceRole::TagTdoa)
    }

    /// Get display name
    pub fn display_name(&self) -> &'static str {
        match self {
            DeviceRole::Anchor => "Anchor",
            DeviceRole::Tag => "Tag",
            DeviceRole::AnchorTdoa => "Anchor (TDoA)",
            DeviceRole::TagTdoa => "Tag (TDoA)",
            DeviceRole::Calibration => "Calibration",
            DeviceRole::Unknown => "Unknown",
        }
    }
}

impl std::fmt::Display for DeviceRole {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.display_name())
    }
}

/// Complete device configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeviceConfig {
    /// WiFi configuration
    pub wifi: WifiConfig,
    /// UWB configuration
    pub uwb: UwbConfig,
    /// Application configuration
    pub app: AppConfig,
}

/// WiFi network configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct WifiConfig {
    /// WiFi mode: 0 = AP, 1 = Station
    pub mode: u8,
    /// Access Point SSID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ssid_a_p: Option<String>,
    /// Access Point password
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pswd_a_p: Option<String>,
    /// Station mode SSID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub ssid_s_t: Option<String>,
    /// Station mode password
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pswd_s_t: Option<String>,
    /// Ground Control Station IP
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gcs_ip: Option<String>,
    /// UDP port for MAVLink
    #[serde(skip_serializing_if = "Option::is_none")]
    pub udp_port: Option<u16>,
    /// Enable web server (0 or 1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enable_web_server: Option<u8>,
    /// Enable UDP discovery (0 or 1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub enable_discovery: Option<u8>,
    /// UDP discovery port
    #[serde(skip_serializing_if = "Option::is_none")]
    pub discovery_port: Option<u16>,
    /// UDP port for log streaming
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_udp_port: Option<u16>,
    /// Whether Serial logging is enabled at runtime
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_serial_enabled: Option<u8>,
    /// Whether UDP log streaming is enabled at runtime
    #[serde(skip_serializing_if = "Option::is_none")]
    pub log_udp_enabled: Option<u8>,
}

/// UWB and positioning configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct UwbConfig {
    /// UWB mode: 0=TWR_ANCHOR, 1=TWR_TAG, 2=CALIBRATION, 3=TDOA_ANCHOR, 4=TDOA_TAG
    pub mode: u8,
    /// Device's UWB short address
    pub dev_short_addr: String,
    /// Number of anchors
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anchor_count: Option<u8>,
    /// Anchor configurations
    #[serde(skip_serializing_if = "Option::is_none")]
    pub anchors: Option<Vec<AnchorConfig>>,
    /// GPS origin latitude
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_lat: Option<f64>,
    /// GPS origin longitude
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_lon: Option<f64>,
    /// GPS origin altitude
    #[serde(skip_serializing_if = "Option::is_none")]
    pub origin_alt: Option<f64>,
    /// MAVLink target system ID
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mavlink_target_system_id: Option<u8>,
    /// Coordinate rotation in degrees
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rotation_degrees: Option<f64>,
    /// Z calculation mode: 0=None (TDoA Z), 1=Rangefinder, 2=UWB (reserved)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub z_calc_mode: Option<u8>,
    /// UWB channel (1-7), default 2
    #[serde(skip_serializing_if = "Option::is_none")]
    pub channel: Option<u8>,
    /// DW1000 mode index (0-7), default 0 (SHORTDATA_FAST_ACCURACY)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub dw_mode: Option<u8>,
    /// TX power level (0-3), default 3 (high)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub tx_power_level: Option<u8>,
    /// Smart power enable (0=disabled, 1=enabled)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub smart_power_enable: Option<u8>,
}

/// Single anchor configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnchorConfig {
    /// Anchor UWB short address
    pub id: String,
    /// X position in meters
    pub x: f64,
    /// Y position in meters
    pub y: f64,
    /// Z position in meters
    pub z: f64,
}

/// Application-level configuration.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AppConfig {
    /// LED 2 GPIO pin
    #[serde(skip_serializing_if = "Option::is_none")]
    pub led2_pin: Option<u8>,
    /// LED 2 state (0 or 1)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub led2_state: Option<u8>,
}

// ==================== Preset Types ====================

/// Type of preset: full device configuration or locations only.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum PresetType {
    /// Full device configuration
    Full,
    /// Locations only (anchors + origin + rotation)
    Locations,
}

impl std::fmt::Display for PresetType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PresetType::Full => write!(f, "full"),
            PresetType::Locations => write!(f, "locations"),
        }
    }
}

/// GPS origin coordinates.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsOrigin {
    /// Latitude in degrees
    pub lat: f64,
    /// Longitude in degrees
    pub lon: f64,
    /// Altitude in meters
    pub alt: f64,
}

/// Location data for a preset.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocationData {
    /// GPS origin coordinates
    pub origin: GpsOrigin,
    /// Rotation in degrees
    pub rotation: f64,
    /// Anchor configurations
    pub anchors: Vec<AnchorConfig>,
}

/// Unified preset that can be either full config or locations only.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct Preset {
    /// Preset name
    pub name: String,
    /// Optional description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Preset type
    #[serde(rename = "type")]
    pub preset_type: PresetType,
    /// Full device configuration (for type = Full)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub config: Option<DeviceConfig>,
    /// Location data (for type = Locations)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub locations: Option<LocationData>,
    /// Creation timestamp (ISO 8601)
    pub created_at: String,
    /// Last update timestamp (ISO 8601)
    pub updated_at: String,
}

/// Metadata for a preset (without the full config data).
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct PresetInfo {
    /// Preset name
    pub name: String,
    /// Preset type
    #[serde(rename = "type")]
    pub preset_type: PresetType,
    /// Optional description
    #[serde(skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// Creation timestamp (ISO 8601)
    pub created_at: String,
    /// Last update timestamp (ISO 8601)
    pub updated_at: String,
}

// ==================== Log Types ====================

/// Log level for filtering
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LogLevel {
    Verbose = 5,
    Debug = 4,
    Info = 3,
    Warn = 2,
    Error = 1,
    None = 0,
}

impl LogLevel {
    pub fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "verbose" | "v" => Some(LogLevel::Verbose),
            "debug" | "d" => Some(LogLevel::Debug),
            "info" | "i" => Some(LogLevel::Info),
            "warn" | "warning" | "w" => Some(LogLevel::Warn),
            "error" | "e" => Some(LogLevel::Error),
            "none" | "n" => Some(LogLevel::None),
            _ => None,
        }
    }

    pub fn from_u8(level: u8) -> Self {
        match level {
            5 => LogLevel::Verbose,
            4 => LogLevel::Debug,
            3 => LogLevel::Info,
            2 => LogLevel::Warn,
            1 => LogLevel::Error,
            _ => LogLevel::None,
        }
    }

    pub fn as_str(&self) -> &'static str {
        match self {
            LogLevel::Verbose => "VERBOSE",
            LogLevel::Debug => "DEBUG",
            LogLevel::Info => "INFO",
            LogLevel::Warn => "WARN",
            LogLevel::Error => "ERROR",
            LogLevel::None => "NONE",
        }
    }
}

impl std::fmt::Display for LogLevel {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.as_str())
    }
}

/// A single log message from a device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogMessage {
    /// Source device IP
    pub ip: String,
    /// Log level
    pub level: LogLevel,
    /// Log tag/component
    pub tag: String,
    /// Log message content
    pub message: String,
    /// Timestamp (if provided by device)
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timestamp: Option<u64>,
}
