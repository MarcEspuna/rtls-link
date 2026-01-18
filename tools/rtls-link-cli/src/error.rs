//! Error types for RTLS-Link CLI.

use thiserror::Error;

/// Exit codes for the CLI
pub mod exit_codes {
    pub const SUCCESS: i32 = 0;
    pub const GENERAL_ERROR: i32 = 1;
    pub const NETWORK_ERROR: i32 = 2;
    pub const DEVICE_ERROR: i32 = 3;
    pub const INVALID_ARGS: i32 = 4;
    pub const PARTIAL_FAILURE: i32 = 5;
}

/// Main error type for the CLI
#[derive(Error, Debug)]
pub enum CliError {
    #[error("Network error: {0}")]
    Network(#[from] NetworkError),

    #[error("Device error: {0}")]
    Device(#[from] DeviceError),

    #[error("Configuration error: {0}")]
    Config(#[from] ConfigError),

    #[error("Storage error: {0}")]
    Storage(#[from] StorageError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Invalid argument: {0}")]
    InvalidArgument(String),

    #[error("Partial failure: {succeeded} succeeded, {failed} failed")]
    PartialFailure { succeeded: usize, failed: usize },

    #[error("No devices found")]
    NoDevicesFound,

    #[error("Timeout: {0}")]
    Timeout(String),

    #[error("{0}")]
    Other(String),
}

impl CliError {
    /// Get the exit code for this error
    pub fn exit_code(&self) -> i32 {
        match self {
            CliError::Network(_) => exit_codes::NETWORK_ERROR,
            CliError::Device(_) => exit_codes::DEVICE_ERROR,
            CliError::Config(_) => exit_codes::GENERAL_ERROR,
            CliError::Storage(_) => exit_codes::GENERAL_ERROR,
            CliError::Io(_) => exit_codes::GENERAL_ERROR,
            CliError::InvalidArgument(_) => exit_codes::INVALID_ARGS,
            CliError::PartialFailure { .. } => exit_codes::PARTIAL_FAILURE,
            CliError::NoDevicesFound => exit_codes::GENERAL_ERROR,
            CliError::Timeout(_) => exit_codes::NETWORK_ERROR,
            CliError::Other(_) => exit_codes::GENERAL_ERROR,
        }
    }
}

/// Network-related errors
#[derive(Error, Debug)]
pub enum NetworkError {
    #[error("Failed to bind UDP socket: {0}")]
    UdpBind(#[source] std::io::Error),

    #[error("Failed to connect WebSocket to {addr}: {source}")]
    WebSocketConnect {
        addr: String,
        #[source]
        source: tokio_tungstenite::tungstenite::Error,
    },

    #[error("WebSocket error: {0}")]
    WebSocket(#[from] tokio_tungstenite::tungstenite::Error),

    #[error("HTTP request failed: {0}")]
    Http(#[from] reqwest::Error),

    #[error("Connection timeout to {0}")]
    ConnectionTimeout(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

/// Device command errors
#[derive(Error, Debug)]
pub enum DeviceError {
    #[error("Device not found: {0}")]
    NotFound(String),

    #[error("Command failed on {ip}: {message}")]
    CommandFailed { ip: String, message: String },

    #[error("Invalid response from {ip}: {message}")]
    InvalidResponse { ip: String, message: String },

    #[error("Device {ip} is offline")]
    Offline { ip: String },

    #[error("OTA update failed on {ip}: {message}")]
    OtaFailed { ip: String, message: String },
}

/// Configuration errors
#[derive(Error, Debug)]
pub enum ConfigError {
    #[error("Failed to parse config: {0}")]
    ParseError(#[from] serde_json::Error),

    #[error("Invalid parameter: group={group}, name={name}")]
    InvalidParameter { group: String, name: String },

    #[error("Config not found: {0}")]
    NotFound(String),

    #[error("Invalid config file: {0}")]
    InvalidFile(String),
}

/// Storage errors
#[derive(Error, Debug)]
pub enum StorageError {
    #[error("Failed to access storage directory: {0}")]
    DirectoryAccess(String),

    #[error("Preset not found: {0}")]
    PresetNotFound(String),

    #[error("Invalid preset name: {0}")]
    InvalidPresetName(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Serialization error: {0}")]
    Serialization(#[from] serde_json::Error),
}

/// Result type for CLI operations
pub type Result<T> = std::result::Result<T, CliError>;
