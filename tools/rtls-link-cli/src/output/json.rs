//! JSON-formatted output for CLI.

use serde::Serialize;
use serde_json::{json, Value};

use super::OutputFormatter;
use crate::health::DeviceHealth;
use crate::types::Device;

pub struct JsonOutput;

impl JsonOutput {
    pub fn new() -> Self {
        Self
    }

    fn to_json<T: Serialize>(value: &T) -> String {
        serde_json::to_string_pretty(value).unwrap_or_else(|_| "{}".to_string())
    }
}

impl Default for JsonOutput {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputFormatter for JsonOutput {
    fn format_devices(&self, devices: &[Device]) -> String {
        let output = json!({
            "devices": devices,
            "count": devices.len()
        });
        Self::to_json(&output)
    }

    fn format_device_status(&self, device: &Device, health: Option<&DeviceHealth>) -> String {
        let mut output = serde_json::to_value(device).unwrap_or(json!({}));

        if let Some(health) = health {
            if let Value::Object(ref mut map) = output {
                map.insert("health".to_string(), json!({
                    "level": health.level.as_str(),
                    "issues": health.issues
                }));
            }
        }

        Self::to_json(&output)
    }

    fn format_message(&self, message: &str) -> String {
        Self::to_json(&json!({
            "message": message
        }))
    }

    fn format_error(&self, error: &str) -> String {
        Self::to_json(&json!({
            "error": error
        }))
    }

    fn format_command_result(&self, ip: &str, command: &str, result: &str, success: bool) -> String {
        // Try to parse result as JSON
        let result_value: Value = serde_json::from_str(result)
            .unwrap_or_else(|_| json!(result));

        Self::to_json(&json!({
            "ip": ip,
            "command": command,
            "success": success,
            "result": result_value
        }))
    }

    fn format_bulk_results(&self, results: &[(String, bool, String)]) -> String {
        let items: Vec<Value> = results
            .iter()
            .map(|(ip, success, message)| {
                // Try to parse message as JSON
                let message_value: Value = serde_json::from_str(message)
                    .unwrap_or_else(|_| json!(message));

                json!({
                    "ip": ip,
                    "success": success,
                    "result": message_value
                })
            })
            .collect();

        let success_count = results.iter().filter(|(_, s, _)| *s).count();
        let fail_count = results.len() - success_count;

        Self::to_json(&json!({
            "results": items,
            "summary": {
                "total": results.len(),
                "succeeded": success_count,
                "failed": fail_count
            }
        }))
    }
}
