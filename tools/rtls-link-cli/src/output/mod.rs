//! Output formatting for CLI results.

pub mod json;
pub mod table;

pub use json::JsonOutput;
pub use table::TableOutput;

use crate::types::Device;
use crate::health::DeviceHealth;

/// Output formatter trait
pub trait OutputFormatter {
    /// Format device list
    fn format_devices(&self, devices: &[Device]) -> String;

    /// Format device status with optional health
    fn format_device_status(&self, device: &Device, health: Option<&DeviceHealth>) -> String;

    /// Format a generic message
    fn format_message(&self, message: &str) -> String;

    /// Format an error
    fn format_error(&self, error: &str) -> String;

    /// Format command result
    fn format_command_result(&self, ip: &str, command: &str, result: &str, success: bool) -> String;

    /// Format bulk operation results
    fn format_bulk_results(&self, results: &[(String, bool, String)]) -> String;
}

/// Get the appropriate formatter based on JSON flag
pub fn get_formatter(json: bool) -> Box<dyn OutputFormatter> {
    if json {
        Box::new(JsonOutput::new())
    } else {
        Box::new(TableOutput::new())
    }
}
