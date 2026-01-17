//! Table-formatted output for CLI.

use colored::*;
use comfy_table::{Cell, Color, ContentArrangement, Table};

use super::OutputFormatter;
use crate::health::{DeviceHealth, HealthLevel};
use crate::types::Device;

pub struct TableOutput;

impl TableOutput {
    pub fn new() -> Self {
        Self
    }

    fn health_color(level: &HealthLevel) -> Color {
        match level {
            HealthLevel::Healthy => Color::Green,
            HealthLevel::Warning => Color::Yellow,
            HealthLevel::Degraded => Color::Red,
            HealthLevel::Unknown => Color::Grey,
        }
    }

    fn health_icon(level: &HealthLevel) -> &'static str {
        match level {
            HealthLevel::Healthy => "[OK]",
            HealthLevel::Warning => "[!]",
            HealthLevel::Degraded => "[X]",
            HealthLevel::Unknown => "[?]",
        }
    }
}

impl Default for TableOutput {
    fn default() -> Self {
        Self::new()
    }
}

impl OutputFormatter for TableOutput {
    fn format_devices(&self, devices: &[Device]) -> String {
        if devices.is_empty() {
            return "No devices found.".to_string();
        }

        let mut table = Table::new();
        table.set_content_arrangement(ContentArrangement::Dynamic);
        table.set_header(vec!["IP", "ID", "Role", "UWB Addr", "Firmware", "MAV ID"]);

        for device in devices {
            table.add_row(vec![
                Cell::new(&device.ip),
                Cell::new(&device.id),
                Cell::new(device.role.display_name()),
                Cell::new(&device.uwb_short),
                Cell::new(&device.firmware),
                Cell::new(device.mav_sys_id.to_string()),
            ]);
        }

        format!("{}\n\nFound {} device(s)", table, devices.len())
    }

    fn format_device_status(&self, device: &Device, health: Option<&DeviceHealth>) -> String {
        let mut lines = Vec::new();

        lines.push(format!("Device: {} ({})", device.ip, device.id));
        lines.push(format!("  Role:       {}", device.role.display_name()));
        lines.push(format!("  UWB Addr:   {}", device.uwb_short));
        lines.push(format!("  Firmware:   {}", device.firmware));
        lines.push(format!("  MAV SysID:  {}", device.mav_sys_id));
        lines.push(format!("  MAC:        {}", device.mac));

        if let Some(health) = health {
            let icon = Self::health_icon(&health.level);
            let level_str = health.level.as_str();
            lines.push(format!("  Health:     {} {}", icon, level_str));

            if !health.issues.is_empty() {
                for issue in &health.issues {
                    lines.push(format!("    - {}", issue));
                }
            }
        }

        // Tag-specific telemetry
        if device.role.is_tag() {
            lines.push("  Telemetry:".to_string());

            if let Some(v) = device.sending_pos {
                let status = if v { "Yes".green() } else { "No".red() };
                lines.push(format!("    Sending Pos:  {}", status));
            }

            if let Some(v) = device.anchors_seen {
                let _color = if v >= 3 { "green" } else { "red" };
                lines.push(format!("    Anchors Seen: {} (need 3+)", v));
            }

            if let Some(v) = device.origin_sent {
                let status = if v { "Yes".green() } else { "No".yellow() };
                lines.push(format!("    Origin Sent:  {}", status));
            }

            if let Some(rf_enabled) = device.rf_enabled {
                if rf_enabled {
                    let rf_status = match device.rf_healthy {
                        Some(true) => "Healthy".green(),
                        Some(false) => "Unhealthy".red(),
                        None => "Unknown".yellow(),
                    };
                    lines.push(format!("    Rangefinder:  {}", rf_status));
                }
            }

            if let Some(avg_rate) = device.avg_rate_c_hz {
                let hz = avg_rate as f64 / 100.0;
                lines.push(format!("    Update Rate:  {:.1} Hz", hz));
            }
        }

        // Logging info
        if device.log_level.is_some() || device.log_udp_enabled.is_some() {
            lines.push("  Logging:".to_string());

            if let Some(level) = device.log_level {
                let level_name = match level {
                    0 => "NONE",
                    1 => "ERROR",
                    2 => "WARN",
                    3 => "INFO",
                    4 => "DEBUG",
                    5 => "VERBOSE",
                    _ => "?",
                };
                lines.push(format!("    Level:   {}", level_name));
            }

            if let Some(udp) = device.log_udp_enabled {
                let status = if udp { "Enabled" } else { "Disabled" };
                if let Some(port) = device.log_udp_port {
                    lines.push(format!("    UDP:     {} (port {})", status, port));
                } else {
                    lines.push(format!("    UDP:     {}", status));
                }
            }

            if let Some(serial) = device.log_serial_enabled {
                let status = if serial { "Enabled" } else { "Disabled" };
                lines.push(format!("    Serial:  {}", status));
            }
        }

        lines.join("\n")
    }

    fn format_message(&self, message: &str) -> String {
        message.to_string()
    }

    fn format_error(&self, error: &str) -> String {
        format!("{} {}", "Error:".red().bold(), error)
    }

    fn format_command_result(&self, ip: &str, command: &str, result: &str, success: bool) -> String {
        let status = if success {
            "[OK]".green()
        } else {
            "[FAIL]".red()
        };

        format!("{} {} '{}'\n{}", status, ip, command, result)
    }

    fn format_bulk_results(&self, results: &[(String, bool, String)]) -> String {
        let mut table = Table::new();
        table.set_content_arrangement(ContentArrangement::Dynamic);
        table.set_header(vec!["IP", "Status", "Result"]);

        let mut success_count = 0;
        let mut fail_count = 0;

        for (ip, success, message) in results {
            let status_cell = if *success {
                success_count += 1;
                Cell::new("OK").fg(Color::Green)
            } else {
                fail_count += 1;
                Cell::new("FAIL").fg(Color::Red)
            };

            table.add_row(vec![
                Cell::new(ip),
                status_cell,
                Cell::new(message),
            ]);
        }

        let summary = format!(
            "\nSummary: {} succeeded, {} failed",
            success_count.to_string().green(),
            fail_count.to_string().red()
        );

        format!("{}{}", table, summary)
    }
}
