//! Discover command implementation.

use std::io::{self, Write};
use std::time::Duration;

use colored::*;

use crate::cli::{DiscoverArgs, RoleFilter};
use crate::device::discovery::{discover_devices, watch_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::error::CliError;
use crate::output::{get_formatter, OutputFormatter};
use crate::types::{Device, DeviceRole};

/// Run the discover command
pub async fn run_discover(args: DiscoverArgs, json: bool) -> Result<(), CliError> {
    let formatter = get_formatter(json);

    let options = DiscoveryOptions {
        port: DISCOVERY_PORT,
        duration: Duration::from_secs(args.duration),
        watch: args.watch,
        on_device: None,
    };

    if args.watch {
        run_watch_mode(options, args.filter_role, json).await
    } else {
        run_oneshot_mode(options, args.filter_role, formatter.as_ref()).await
    }
}

async fn run_oneshot_mode(
    options: DiscoveryOptions,
    filter_role: Option<RoleFilter>,
    formatter: &dyn OutputFormatter,
) -> Result<(), CliError> {
    println!("Discovering devices for {} seconds...", options.duration.as_secs());

    let devices = discover_devices(options).await?;

    // Apply role filter
    let devices = filter_devices(devices, filter_role);

    println!("{}", formatter.format_devices(&devices));

    if devices.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    Ok(())
}

async fn run_watch_mode(
    options: DiscoveryOptions,
    filter_role: Option<RoleFilter>,
    json: bool,
) -> Result<(), CliError> {
    println!("Watching for devices (press Ctrl+C to stop)...\n");

    let filter = filter_role.clone();

    watch_devices(options, move |devices| {
        let devices = filter_devices(devices.to_vec(), filter.clone());

        // Clear screen and print header
        print!("\x1B[2J\x1B[1;1H");
        println!("{}", "RTLS-Link Device Watch".bold());
        println!("{}", "Press Ctrl+C to stop".dimmed());
        println!();

        if json {
            let output = serde_json::json!({
                "devices": devices,
                "count": devices.len()
            });
            println!("{}", serde_json::to_string_pretty(&output).unwrap());
        } else {
            // Print simple table
            println!(
                "{:<16} {:<20} {:<12} {:<8} {:<12}",
                "IP".bold(),
                "ID".bold(),
                "Role".bold(),
                "UWB".bold(),
                "Firmware".bold()
            );
            println!("{}", "-".repeat(70));

            for device in &devices {
                let _role_color = match device.role {
                    DeviceRole::Tag | DeviceRole::TagTdoa => "cyan",
                    DeviceRole::Anchor | DeviceRole::AnchorTdoa => "green",
                    _ => "yellow",
                };

                println!(
                    "{:<16} {:<20} {:<12} {:<8} {:<12}",
                    device.ip,
                    truncate(&device.id, 18),
                    device.role.display_name(),
                    device.uwb_short,
                    device.firmware
                );
            }

            println!();
            println!("Found {} device(s)", devices.len());
        }

        io::stdout().flush().ok();
    })
    .await
}

fn filter_devices(devices: Vec<Device>, filter: Option<RoleFilter>) -> Vec<Device> {
    match filter {
        Some(RoleFilter::Anchor) => devices
            .into_iter()
            .filter(|d| d.role == DeviceRole::Anchor)
            .collect(),
        Some(RoleFilter::Tag) => devices
            .into_iter()
            .filter(|d| d.role == DeviceRole::Tag)
            .collect(),
        Some(RoleFilter::AnchorTdoa) => devices
            .into_iter()
            .filter(|d| d.role == DeviceRole::AnchorTdoa)
            .collect(),
        Some(RoleFilter::TagTdoa) => devices
            .into_iter()
            .filter(|d| d.role == DeviceRole::TagTdoa)
            .collect(),
        Some(RoleFilter::Calibration) => devices
            .into_iter()
            .filter(|d| d.role == DeviceRole::Calibration)
            .collect(),
        None => devices,
    }
}

fn truncate(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        format!("{}...", &s[..max_len - 3])
    }
}
