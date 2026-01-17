//! OTA firmware update commands.

use std::path::Path;
use std::time::Duration;

use crate::cli::{OtaArgs, OtaCommands, RoleFilter};
use crate::device::discovery::{discover_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::device::ota::{upload_firmware_bulk, upload_firmware_with_progress};
use crate::error::CliError;
use crate::output::{get_formatter, OutputFormatter};
use crate::types::{Device, DeviceRole};

/// Run the OTA command
pub async fn run_ota(args: OtaArgs, json: bool, strict: bool) -> Result<(), CliError> {
    match args.command {
        OtaCommands::Update(args) => {
            run_update(
                &args.target,
                &args.firmware,
                args.filter_role,
                args.concurrency,
                json,
                strict,
            )
            .await
        }
    }
}

async fn run_update(
    target: &str,
    firmware: &str,
    filter_role: Option<RoleFilter>,
    concurrency: usize,
    json: bool,
    strict: bool,
) -> Result<(), CliError> {
    let firmware_path = Path::new(firmware);

    if !firmware_path.exists() {
        return Err(CliError::InvalidArgument(format!(
            "Firmware file not found: {}",
            firmware
        )));
    }

    let formatter = get_formatter(json);

    // Get target devices
    let ips: Vec<String> = if target.to_lowercase() == "all" {
        let options = DiscoveryOptions {
            port: DISCOVERY_PORT,
            duration: Duration::from_secs(3),
            watch: false,
            on_device: None,
        };
        let devices = discover_devices(options).await?;
        let devices = filter_devices_by_role(devices, filter_role);
        devices.into_iter().map(|d| d.ip).collect()
    } else if target.contains(',') {
        target.split(',').map(|s| s.trim().to_string()).collect()
    } else {
        vec![target.to_string()]
    };

    if ips.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    if ips.len() == 1 && !json {
        // Single device with progress
        let ip = &ips[0];
        println!("Uploading firmware to {}...", ip);
        upload_firmware_with_progress(ip, firmware_path).await?;
        println!("Firmware upload complete. Device will reboot.");
    } else {
        // Bulk upload
        println!("Uploading firmware to {} device(s)...", ips.len());

        let results = upload_firmware_bulk(&ips, firmware_path, concurrency).await;

        let formatted_results: Vec<(String, bool, String)> = results
            .into_iter()
            .map(|(ip, result)| {
                let success = result.is_ok();
                let message = match result {
                    Ok(_) => "Firmware uploaded".to_string(),
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
    }

    Ok(())
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
