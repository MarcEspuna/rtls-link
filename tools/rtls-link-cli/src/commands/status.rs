//! Status command implementation.

use std::time::Duration;

use crate::cli::StatusArgs;
use crate::device::discovery::{discover_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::error::CliError;
use crate::health::calculate_device_health;
use crate::output::{get_formatter, OutputFormatter};
use crate::types::Device;

/// Run the status command
pub async fn run_status(args: StatusArgs, timeout: u64, json: bool) -> Result<(), CliError> {
    let formatter = get_formatter(json);
    let _timeout_duration = Duration::from_millis(timeout);

    if args.target.to_lowercase() == "all" {
        // Discover devices and show status for all
        let options = DiscoveryOptions {
            port: DISCOVERY_PORT,
            duration: Duration::from_secs(args.discovery_duration),
            watch: false,
            on_device: None,
        };

        let devices = discover_devices(options).await?;

        if devices.is_empty() {
            return Err(CliError::NoDevicesFound);
        }

        if json {
            let mut results = Vec::new();
            for device in &devices {
                let health = if args.health {
                    Some(calculate_device_health(device))
                } else {
                    None
                };
                results.push((device.clone(), health));
            }

            // Output as JSON array
            let output: Vec<serde_json::Value> = results
                .iter()
                .map(|(device, health)| {
                    let mut value = serde_json::to_value(device).unwrap();
                    if let Some(h) = health {
                        if let serde_json::Value::Object(ref mut map) = value {
                            map.insert(
                                "health".to_string(),
                                serde_json::json!({
                                    "level": h.level.as_str(),
                                    "issues": h.issues
                                }),
                            );
                        }
                    }
                    value
                })
                .collect();

            println!("{}", serde_json::to_string_pretty(&output).unwrap());
        } else {
            for device in &devices {
                let health = if args.health {
                    Some(calculate_device_health(device))
                } else {
                    None
                };
                println!("{}\n", formatter.format_device_status(device, health.as_ref()));
            }
        }
    } else {
        // Single device status
        let ip = &args.target;

        // Try to get live status via discovery or fall back to basic info
        let device = get_device_status(ip, Duration::from_secs(2)).await?;

        let health = if args.health {
            Some(calculate_device_health(&device))
        } else {
            None
        };

        println!("{}", formatter.format_device_status(&device, health.as_ref()));
    }

    Ok(())
}

/// Get device status by listening for its heartbeat
async fn get_device_status(ip: &str, timeout: Duration) -> Result<Device, CliError> {
    let options = DiscoveryOptions {
        port: DISCOVERY_PORT,
        duration: timeout,
        watch: false,
        on_device: None,
    };

    let devices = discover_devices(options).await?;

    devices
        .into_iter()
        .find(|d| d.ip == ip)
        .ok_or_else(|| CliError::NoDevicesFound)
}
