//! CLI argument definitions using clap.

use clap::{Args, Parser, Subcommand, ValueEnum};

/// RTLS-Link CLI - Command-line interface for RTLS-Link device management
#[derive(Parser, Debug)]
#[command(name = "rtls-link-cli")]
#[command(author, version, about, long_about = None)]
#[command(propagate_version = true)]
pub struct Cli {
    /// Output in JSON format
    #[arg(long, global = true)]
    pub json: bool,

    /// Command timeout in milliseconds
    #[arg(long, global = true, default_value = "5000", env = "RTLS_CLI_TIMEOUT")]
    pub timeout: u64,

    /// Verbose output
    #[arg(short, long, global = true)]
    pub verbose: bool,

    /// Exit non-zero on any partial failure (for bulk operations)
    #[arg(long, global = true)]
    pub strict: bool,

    #[command(subcommand)]
    pub command: Commands,
}

#[derive(Subcommand, Debug)]
pub enum Commands {
    /// Discover devices on the network
    Discover(DiscoverArgs),

    /// Show device status with health analysis
    Status(StatusArgs),

    /// Device configuration management
    Config(ConfigArgs),

    /// Local preset management
    Preset(PresetArgs),

    /// Firmware updates (OTA)
    Ota(OtaArgs),

    /// Log streaming from devices
    Logs(LogsArgs),

    /// Send raw commands to devices
    Cmd(CmdArgs),

    /// Bulk device operations
    Bulk(BulkArgs),
}

// ==================== Discover ====================

#[derive(Args, Debug)]
pub struct DiscoverArgs {
    /// Watch mode - continuously discover devices
    #[arg(short, long)]
    pub watch: bool,

    /// Discovery duration in seconds (ignored in watch mode)
    #[arg(short, long, default_value = "5")]
    pub duration: u64,

    /// Filter by role
    #[arg(long, value_enum)]
    pub filter_role: Option<RoleFilter>,
}

#[derive(ValueEnum, Clone, Debug)]
pub enum RoleFilter {
    Anchor,
    Tag,
    AnchorTdoa,
    TagTdoa,
    Calibration,
}

// ==================== Status ====================

#[derive(Args, Debug)]
pub struct StatusArgs {
    /// Device IP address or "all" for all discovered devices
    pub target: String,

    /// Show detailed health analysis
    #[arg(long)]
    pub health: bool,

    /// Discovery duration when using "all" (seconds)
    #[arg(long, default_value = "3")]
    pub discovery_duration: u64,
}

// ==================== Config ====================

#[derive(Args, Debug)]
pub struct ConfigArgs {
    #[command(subcommand)]
    pub command: ConfigCommands,
}

#[derive(Subcommand, Debug)]
pub enum ConfigCommands {
    /// Backup device configuration to a file
    Backup(ConfigBackupArgs),

    /// Apply configuration from a file to a device
    Apply(ConfigApplyArgs),

    /// Read a single parameter from a device
    Read(ConfigReadArgs),

    /// Write a single parameter to a device
    Write(ConfigWriteArgs),

    /// List saved configurations on a device
    List(ConfigListArgs),

    /// Save current device config to a named slot on device
    SaveAs(ConfigSaveAsArgs),

    /// Load a named configuration from device storage
    Load(ConfigLoadArgs),

    /// Delete a named configuration from device storage
    Delete(ConfigDeleteArgs),
}

#[derive(Args, Debug)]
pub struct ConfigBackupArgs {
    /// Device IP address
    pub ip: String,

    /// Output file (default: stdout or <ip>_config.json)
    #[arg(short, long)]
    pub output: Option<String>,
}

#[derive(Args, Debug)]
pub struct ConfigApplyArgs {
    /// Device IP address or "all" for all discovered devices
    pub target: String,

    /// Configuration file to apply
    pub file: String,

    /// Skip devShortAddr (preserve device identity)
    #[arg(long, default_value = "true")]
    pub skip_short_addr: bool,

    /// Filter by role when target is "all"
    #[arg(long, value_enum)]
    pub filter_role: Option<RoleFilter>,

    /// Concurrency limit for bulk operations
    #[arg(long, default_value = "3")]
    pub concurrency: usize,
}

#[derive(Args, Debug)]
pub struct ConfigReadArgs {
    /// Device IP address
    pub ip: String,

    /// Parameter group (wifi, uwb, app)
    #[arg(short, long)]
    pub group: String,

    /// Parameter name
    #[arg(short, long)]
    pub name: String,
}

#[derive(Args, Debug)]
pub struct ConfigWriteArgs {
    /// Device IP address
    pub ip: String,

    /// Parameter group (wifi, uwb, app)
    #[arg(short, long)]
    pub group: String,

    /// Parameter name
    #[arg(short, long)]
    pub name: String,

    /// Parameter value
    #[arg(short = 'd', long = "data")]
    pub value: String,

    /// Save to flash after writing
    #[arg(long)]
    pub save: bool,
}

#[derive(Args, Debug)]
pub struct ConfigListArgs {
    /// Device IP address
    pub ip: String,
}

#[derive(Args, Debug)]
pub struct ConfigSaveAsArgs {
    /// Device IP address
    pub ip: String,

    /// Configuration name
    pub name: String,
}

#[derive(Args, Debug)]
pub struct ConfigLoadArgs {
    /// Device IP address
    pub ip: String,

    /// Configuration name
    pub name: String,
}

#[derive(Args, Debug)]
pub struct ConfigDeleteArgs {
    /// Device IP address
    pub ip: String,

    /// Configuration name
    pub name: String,
}

// ==================== Preset ====================

#[derive(Args, Debug)]
pub struct PresetArgs {
    #[command(subcommand)]
    pub command: PresetCommands,
}

#[derive(Subcommand, Debug)]
pub enum PresetCommands {
    /// List all local presets
    List,

    /// Show details of a preset
    Show(PresetShowArgs),

    /// Save a preset from a device or file
    Save(PresetSaveArgs),

    /// Delete a local preset
    Delete(PresetDeleteArgs),

    /// Upload a preset to device(s)
    Upload(PresetUploadArgs),
}

#[derive(Args, Debug)]
pub struct PresetShowArgs {
    /// Preset name
    pub name: String,
}

#[derive(Args, Debug)]
pub struct PresetSaveArgs {
    /// Preset name
    pub name: String,

    /// Source device IP to backup from
    #[arg(long)]
    pub from_device: Option<String>,

    /// Source config file
    #[arg(long)]
    pub from_file: Option<String>,

    /// Preset type
    #[arg(long, value_enum, default_value = "full")]
    pub preset_type: PresetTypeArg,

    /// Optional description
    #[arg(short, long)]
    pub description: Option<String>,
}

#[derive(ValueEnum, Clone, Debug)]
pub enum PresetTypeArg {
    Full,
    Locations,
}

#[derive(Args, Debug)]
pub struct PresetDeleteArgs {
    /// Preset name
    pub name: String,

    /// Skip confirmation
    #[arg(short, long)]
    pub force: bool,
}

#[derive(Args, Debug)]
pub struct PresetUploadArgs {
    /// Preset name
    pub name: String,

    /// Target: device IP, "all", or comma-separated IPs
    pub target: String,

    /// Filter by role when target is "all"
    #[arg(long, value_enum)]
    pub filter_role: Option<RoleFilter>,

    /// Concurrency limit for bulk operations
    #[arg(long, default_value = "3")]
    pub concurrency: usize,
}

// ==================== OTA ====================

#[derive(Args, Debug)]
pub struct OtaArgs {
    #[command(subcommand)]
    pub command: OtaCommands,
}

#[derive(Subcommand, Debug)]
pub enum OtaCommands {
    /// Update firmware on device(s)
    Update(OtaUpdateArgs),
}

#[derive(Args, Debug)]
pub struct OtaUpdateArgs {
    /// Target: device IP, "all", or comma-separated IPs
    pub target: String,

    /// Firmware binary file
    pub firmware: String,

    /// Filter by role when target is "all"
    #[arg(long, value_enum)]
    pub filter_role: Option<RoleFilter>,

    /// Concurrency limit for bulk operations
    #[arg(long, default_value = "3")]
    pub concurrency: usize,
}

// ==================== Logs ====================

#[derive(Args, Debug)]
pub struct LogsArgs {
    /// Device IP address (optional, default: all devices)
    pub ip: Option<String>,

    /// Minimum log level to display
    #[arg(short, long, default_value = "info")]
    pub level: String,

    /// Filter by tag pattern (glob-style, e.g., "uwb*")
    #[arg(short, long)]
    pub tag: Option<String>,

    /// UDP port to listen on
    #[arg(long, default_value = "3334")]
    pub port: u16,

    /// Output as newline-delimited JSON (NDJSON)
    #[arg(long)]
    pub ndjson: bool,
}

// ==================== Cmd ====================

#[derive(Args, Debug)]
pub struct CmdArgs {
    /// Device IP address
    pub ip: String,

    /// Command to send
    pub command: String,

    /// Expect JSON response
    #[arg(long)]
    pub expect_json: bool,
}

// ==================== Bulk ====================

#[derive(Args, Debug)]
pub struct BulkArgs {
    #[command(subcommand)]
    pub command: BulkCommands,
}

#[derive(Subcommand, Debug)]
pub enum BulkCommands {
    /// Toggle LED on all devices
    ToggleLed(BulkTargetArgs),

    /// Reboot all devices
    Reboot(BulkTargetArgs),

    /// Start positioning on all devices
    Start(BulkTargetArgs),

    /// Send a raw command to all devices
    Cmd(BulkCmdArgs),
}

#[derive(Args, Debug)]
pub struct BulkTargetArgs {
    /// Filter by role
    #[arg(long, value_enum)]
    pub filter_role: Option<RoleFilter>,

    /// Specific IPs (comma-separated)
    #[arg(long)]
    pub ips: Option<String>,

    /// Concurrency limit
    #[arg(long, default_value = "5")]
    pub concurrency: usize,

    /// Discovery duration (seconds)
    #[arg(long, default_value = "3")]
    pub discovery_duration: u64,
}

#[derive(Args, Debug)]
pub struct BulkCmdArgs {
    /// Command to send
    pub command: String,

    /// Filter by role
    #[arg(long, value_enum)]
    pub filter_role: Option<RoleFilter>,

    /// Specific IPs (comma-separated)
    #[arg(long)]
    pub ips: Option<String>,

    /// Concurrency limit
    #[arg(long, default_value = "5")]
    pub concurrency: usize,

    /// Discovery duration (seconds)
    #[arg(long, default_value = "3")]
    pub discovery_duration: u64,
}
