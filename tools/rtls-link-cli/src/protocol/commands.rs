//! Command string builders for RTLS-Link device protocol.
//!
//! These commands are sent over WebSocket to devices at ws://<ip>/ws

/// Commands that return JSON responses
pub const JSON_COMMANDS: &[&str] = &[
    "backup-config",
    "list-configs",
    "save-config-as",
    "load-config-named",
    "read-config-named",
    "delete-config",
    "toggle-led2",
    "get-led2-state",
    "firmware-info",
];

/// Check if a command is expected to return JSON
pub fn is_json_command(cmd: &str) -> bool {
    JSON_COMMANDS.iter().any(|c| cmd.starts_with(c))
}

/// Command builders for device protocol
pub struct Commands;

impl Commands {
    // ==================== Parameter commands ====================

    /// Read all parameters, optionally filtered by group
    pub fn read_all(group: Option<&str>) -> String {
        match group {
            Some(g) => format!("readall {}", g),
            None => "readall all".to_string(),
        }
    }

    /// Read a single parameter
    pub fn read_param(group: &str, name: &str) -> String {
        format!("read -group {} -name {}", group, name)
    }

    /// Write a parameter value
    ///
    /// Values are properly escaped for the protocol
    pub fn write_param(group: &str, name: &str, value: &str) -> String {
        let safe_value = value.replace('\\', "\\\\").replace('"', "\\\"");
        format!("write -group {} -name {} -data \"{}\"", group, name, safe_value)
    }

    // ==================== Config commands ====================

    /// Backup current configuration (returns JSON)
    pub fn backup_config() -> &'static str {
        "backup-config"
    }

    /// Save current config to flash
    pub fn save_config() -> &'static str {
        "save-config"
    }

    /// Load config from flash
    pub fn load_config() -> &'static str {
        "load-config"
    }

    /// List saved configurations on device (returns JSON)
    pub fn list_configs() -> &'static str {
        "list-configs"
    }

    /// Save current config with a name (returns JSON)
    pub fn save_config_as(name: &str) -> String {
        format!("save-config-as -name {}", name)
    }

    /// Load a named configuration (returns JSON)
    pub fn load_config_named(name: &str) -> String {
        format!("load-config-named -name {}", name)
    }

    /// Read a named configuration without loading (returns JSON)
    pub fn read_config_named(name: &str) -> String {
        format!("read-config-named -name {}", name)
    }

    /// Delete a named configuration (returns JSON)
    pub fn delete_config(name: &str) -> String {
        format!("delete-config -name {}", name)
    }

    // ==================== Control commands ====================

    /// Toggle LED2 state (returns JSON)
    pub fn toggle_led() -> &'static str {
        "toggle-led2"
    }

    /// Get LED2 state (returns JSON)
    pub fn get_led_state() -> &'static str {
        "get-led2-state"
    }

    /// Reboot the device
    pub fn reboot() -> &'static str {
        "reboot"
    }

    /// Start positioning
    pub fn start() -> &'static str {
        "start"
    }

    // ==================== System info commands ====================

    /// Get firmware version
    pub fn get_version() -> &'static str {
        "version"
    }

    /// Get firmware info (returns JSON)
    pub fn get_firmware_info() -> &'static str {
        "firmware-info"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_read_all() {
        assert_eq!(Commands::read_all(None), "readall all");
        assert_eq!(Commands::read_all(Some("wifi")), "readall wifi");
    }

    #[test]
    fn test_read_param() {
        assert_eq!(
            Commands::read_param("wifi", "ssidST"),
            "read -group wifi -name ssidST"
        );
    }

    #[test]
    fn test_write_param() {
        assert_eq!(
            Commands::write_param("wifi", "ssidST", "MyNetwork"),
            "write -group wifi -name ssidST -data \"MyNetwork\""
        );
    }

    #[test]
    fn test_write_param_escaping() {
        // Test escaping of quotes and backslashes
        assert_eq!(
            Commands::write_param("wifi", "ssidST", "Test\"Network"),
            "write -group wifi -name ssidST -data \"Test\\\"Network\""
        );
        assert_eq!(
            Commands::write_param("wifi", "pswdST", "pass\\word"),
            "write -group wifi -name pswdST -data \"pass\\\\word\""
        );
    }

    #[test]
    fn test_is_json_command() {
        assert!(is_json_command("backup-config"));
        assert!(is_json_command("list-configs"));
        assert!(is_json_command("save-config-as -name test"));
        assert!(!is_json_command("version"));
        assert!(!is_json_command("reboot"));
        assert!(!is_json_command("save-config"));
    }
}
