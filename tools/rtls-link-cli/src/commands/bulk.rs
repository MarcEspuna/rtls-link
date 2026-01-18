//! Bulk device operations.

use std::time::Duration;

use crate::cli::{BulkArgs, BulkCmdArgs, BulkCommands, BulkTargetArgs, RoleFilter};
use crate::device::discovery::{discover_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::device::websocket::BatchSender;
use crate::error::CliError;
use crate::output::{get_formatter, OutputFormatter};
use crate::protocol::commands::Commands;
use crate::types::{Device, DeviceRole};

/// Run bulk command
pub async fn run_bulk(args: BulkArgs, timeout: u64, json: bool, strict: bool) -> Result<(), CliError> {
    match args.command {
        BulkCommands::ToggleLed(target) => {
            run_bulk_command(
                Commands::toggle_led(),
                &target,
                timeout,
                json,
                strict,
            )
            .await
        }
        BulkCommands::Reboot(target) => {
            run_bulk_command(Commands::reboot(), &target, timeout, json, strict).await
        }
        BulkCommands::Start(target) => {
            run_bulk_command(Commands::start(), &target, timeout, json, strict).await
        }
        BulkCommands::Cmd(args) => {
            run_bulk_raw_command(&args.command, &args, timeout, json, strict).await
        }
    }
}

async fn run_bulk_command(
    command: &str,
    target: &BulkTargetArgs,
    timeout: u64,
    json: bool,
    strict: bool,
) -> Result<(), CliError> {
    let ips = get_target_ips(target).await?;

    if ips.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    let formatter = get_formatter(json);
    let sender = BatchSender::new(timeout, target.concurrency);

    println!(
        "Running '{}' on {} device(s)...",
        command,
        ips.len()
    );

    let results = sender.send_to_all(&ips, command).await;

    let formatted_results: Vec<(String, bool, String)> = results
        .into_iter()
        .map(|(ip, result)| {
            let success = result.is_ok();
            let message = match result {
                Ok(response) => {
                    // Truncate long responses
                    if response.len() > 100 {
                        format!("{}...", &response[..100])
                    } else {
                        response.trim().to_string()
                    }
                }
                Err(e) => e.to_string(),
            };
            (ip, success, message)
        })
        .collect();

    println!("{}", formatter.format_bulk_results(&formatted_results));

    let failed_count = formatted_results.iter().filter(|(_, s, _)| !s).count();
    if strict && failed_count > 0 {
        return Err(CliError::PartialFailure {
            succeeded: formatted_results.len() - failed_count,
            failed: failed_count,
        });
    }

    Ok(())
}

async fn run_bulk_raw_command(
    command: &str,
    args: &BulkCmdArgs,
    timeout: u64,
    json: bool,
    strict: bool,
) -> Result<(), CliError> {
    let target = BulkTargetArgs {
        filter_role: args.filter_role.clone(),
        ips: args.ips.clone(),
        concurrency: args.concurrency,
        discovery_duration: args.discovery_duration,
    };

    let ips = get_target_ips(&target).await?;

    if ips.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    let formatter = get_formatter(json);
    let sender = BatchSender::new(timeout, args.concurrency);

    println!(
        "Running '{}' on {} device(s)...",
        command,
        ips.len()
    );

    let results = sender.send_to_all(&ips, command).await;

    let formatted_results: Vec<(String, bool, String)> = results
        .into_iter()
        .map(|(ip, result)| {
            let success = result.is_ok();
            let message = match result {
                Ok(response) => {
                    if response.len() > 100 {
                        format!("{}...", &response[..100])
                    } else {
                        response.trim().to_string()
                    }
                }
                Err(e) => e.to_string(),
            };
            (ip, success, message)
        })
        .collect();

    println!("{}", formatter.format_bulk_results(&formatted_results));

    let failed_count = formatted_results.iter().filter(|(_, s, _)| !s).count();
    if strict && failed_count > 0 {
        return Err(CliError::PartialFailure {
            succeeded: formatted_results.len() - failed_count,
            failed: failed_count,
        });
    }

    Ok(())
}

async fn get_target_ips(target: &BulkTargetArgs) -> Result<Vec<String>, CliError> {
    if let Some(ref ips_str) = target.ips {
        // Use explicit IPs
        Ok(ips_str.split(',').map(|s| s.trim().to_string()).collect())
    } else {
        // Discover devices
        let options = DiscoveryOptions {
            port: DISCOVERY_PORT,
            duration: Duration::from_secs(target.discovery_duration),
            watch: false,
            on_device: None,
        };

        let devices = discover_devices(options).await?;
        let devices = filter_devices_by_role(devices, target.filter_role.clone());

        Ok(devices.into_iter().map(|d| d.ip).collect())
    }
}

fn filter_devices_by_role(devices: Vec<Device>, filter: Option<RoleFilter>) -> Vec<Device> {
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
