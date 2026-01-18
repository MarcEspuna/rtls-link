//! Preset commands implementation.

use std::time::Duration;

use chrono::Utc;

use crate::cli::{PresetArgs, PresetCommands, PresetTypeArg, RoleFilter};
use crate::device::discovery::{discover_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::device::websocket::send_command;
use crate::error::{CliError, StorageError};
use crate::output::{get_formatter, OutputFormatter};
use crate::protocol::commands::Commands;
use crate::protocol::config_params::{config_to_params, location_to_params};
use crate::protocol::response::parse_json_response;
use crate::storage::PresetStorage;
use crate::types::{
    Device, DeviceConfig, DeviceRole, GpsOrigin, LocationData, Preset, PresetType,
};

/// Run the preset command
pub async fn run_preset(args: PresetArgs, timeout: u64, json: bool, strict: bool) -> Result<(), CliError> {
    let _formatter = get_formatter(json);
    let timeout_duration = Duration::from_millis(timeout);

    match args.command {
        PresetCommands::List => run_list(json).await,
        PresetCommands::Show(args) => run_show(&args.name, json).await,
        PresetCommands::Save(args) => {
            run_save(
                &args.name,
                args.from_device.as_deref(),
                args.from_file.as_deref(),
                args.preset_type,
                args.description.as_deref(),
                timeout_duration,
                json,
            )
            .await
        }
        PresetCommands::Delete(args) => run_delete(&args.name, args.force, json).await,
        PresetCommands::Upload(args) => {
            run_upload(
                &args.name,
                &args.target,
                args.filter_role,
                args.concurrency,
                timeout_duration,
                json,
                strict,
            )
            .await
        }
    }
}

async fn run_list(json: bool) -> Result<(), CliError> {
    let storage = PresetStorage::new().map_err(CliError::Storage)?;
    let presets = storage.list().map_err(CliError::Storage)?;

    if json {
        let output = serde_json::json!({
            "presets": presets,
            "count": presets.len()
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        if presets.is_empty() {
            println!("No presets saved.");
        } else {
            println!("Saved presets:");
            for preset in &presets {
                let desc = preset
                    .description
                    .as_ref()
                    .map(|d| format!(" - {}", d))
                    .unwrap_or_default();
                println!("  {} [{}]{}", preset.name, preset.preset_type, desc);
            }
            println!("\n{} preset(s) total", presets.len());
        }
    }

    Ok(())
}

async fn run_show(name: &str, json: bool) -> Result<(), CliError> {
    let storage = PresetStorage::new().map_err(CliError::Storage)?;
    let preset = storage.get(name).map_err(CliError::Storage)?;

    if json {
        println!("{}", serde_json::to_string_pretty(&preset).unwrap());
    } else {
        println!("Preset: {}", preset.name);
        println!("Type: {}", preset.preset_type);
        if let Some(ref desc) = preset.description {
            println!("Description: {}", desc);
        }
        println!("Created: {}", preset.created_at);
        println!("Updated: {}", preset.updated_at);

        match preset.preset_type {
            PresetType::Full => {
                if let Some(ref config) = preset.config {
                    println!("\nConfiguration:");
                    println!(
                        "{}",
                        serde_json::to_string_pretty(config).unwrap_or_default()
                    );
                }
            }
            PresetType::Locations => {
                if let Some(ref locations) = preset.locations {
                    println!("\nLocations:");
                    println!(
                        "  Origin: {:.6}, {:.6}, {:.1}m",
                        locations.origin.lat, locations.origin.lon, locations.origin.alt
                    );
                    println!("  Rotation: {}°", locations.rotation);
                    println!("  Anchors:");
                    for anchor in &locations.anchors {
                        println!(
                            "    {} @ ({:.2}, {:.2}, {:.2})",
                            anchor.id, anchor.x, anchor.y, anchor.z
                        );
                    }
                }
            }
        }
    }

    Ok(())
}

async fn run_save(
    name: &str,
    from_device: Option<&str>,
    from_file: Option<&str>,
    preset_type: PresetTypeArg,
    description: Option<&str>,
    timeout: Duration,
    json: bool,
) -> Result<(), CliError> {
    let storage = PresetStorage::new().map_err(CliError::Storage)?;

    // Get config from device or file
    let config = if let Some(ip) = from_device {
        let response = send_command(ip, Commands::backup_config(), timeout).await?;
        parse_json_response::<DeviceConfig>(&response, ip)?
    } else if let Some(file) = from_file {
        let content = std::fs::read_to_string(file)
            .map_err(|e| CliError::Other(format!("Failed to read file: {}", e)))?;
        serde_json::from_str(&content).map_err(|e| CliError::Config(e.into()))?
    } else {
        return Err(CliError::InvalidArgument(
            "Must specify --from-device or --from-file".to_string(),
        ));
    };

    let now = Utc::now().to_rfc3339();
    let preset_type = match preset_type {
        PresetTypeArg::Full => PresetType::Full,
        PresetTypeArg::Locations => PresetType::Locations,
    };

    let preset = match preset_type {
        PresetType::Full => Preset {
            name: name.to_string(),
            description: description.map(String::from),
            preset_type: PresetType::Full,
            config: Some(config),
            locations: None,
            created_at: now.clone(),
            updated_at: now,
        },
        PresetType::Locations => {
            // Extract location data from config
            let locations = LocationData {
                origin: GpsOrigin {
                    lat: config.uwb.origin_lat.unwrap_or(0.0),
                    lon: config.uwb.origin_lon.unwrap_or(0.0),
                    alt: config.uwb.origin_alt.unwrap_or(0.0),
                },
                rotation: config.uwb.rotation_degrees.unwrap_or(0.0),
                anchors: config.uwb.anchors.unwrap_or_default(),
            };

            Preset {
                name: name.to_string(),
                description: description.map(String::from),
                preset_type: PresetType::Locations,
                config: None,
                locations: Some(locations),
                created_at: now.clone(),
                updated_at: now,
            }
        }
    };

    storage.save(&preset).map_err(CliError::Storage)?;

    if json {
        let output = serde_json::json!({
            "success": true,
            "name": name,
            "type": preset.preset_type.to_string()
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        println!("Preset '{}' saved ({} type)", name, preset.preset_type);
    }

    Ok(())
}

async fn run_delete(name: &str, force: bool, json: bool) -> Result<(), CliError> {
    let storage = PresetStorage::new().map_err(CliError::Storage)?;

    if !force {
        // Check if preset exists
        if !storage.exists(name) {
            return Err(CliError::Storage(StorageError::PresetNotFound(
                name.to_string(),
            )));
        }

        // In a real CLI, we'd prompt for confirmation here
        // For now, just proceed
    }

    storage.delete(name).map_err(CliError::Storage)?;

    if json {
        let output = serde_json::json!({
            "success": true,
            "name": name
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        println!("Preset '{}' deleted", name);
    }

    Ok(())
}

async fn run_upload(
    name: &str,
    target: &str,
    filter_role: Option<RoleFilter>,
    _concurrency: usize,
    timeout: Duration,
    json: bool,
    strict: bool,
) -> Result<(), CliError> {
    let storage = PresetStorage::new().map_err(CliError::Storage)?;
    let preset = storage.get(name).map_err(CliError::Storage)?;

    // Get target devices
    let ips = if target.to_lowercase() == "all" {
        let options = DiscoveryOptions {
            port: DISCOVERY_PORT,
            duration: Duration::from_secs(3),
            watch: false,
            on_device: None,
        };
        let devices = discover_devices(options).await?;
        let devices = filter_devices_by_role(devices, filter_role);

        // Location presets only go to tags
        let devices = if preset.preset_type == PresetType::Locations {
            devices
                .into_iter()
                .filter(|d| d.role.is_tag())
                .collect()
        } else {
            devices
        };

        devices.into_iter().map(|d| d.ip).collect()
    } else if target.contains(',') {
        target.split(',').map(|s| s.trim().to_string()).collect()
    } else {
        vec![target.to_string()]
    };

    if ips.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    let formatter = get_formatter(json);
    let mut results = Vec::new();

    for ip in &ips {
        let result = upload_preset_to_device(ip, &preset, timeout).await;
        let success = result.is_ok();
        let message = match &result {
            Ok(_) => "Preset uploaded".to_string(),
            Err(e) => e.to_string(),
        };
        results.push((ip.clone(), success, message));
    }

    println!("{}", formatter.format_bulk_results(&results));

    let failed_count = results.iter().filter(|(_, s, _)| !s).count();
    if strict && failed_count > 0 {
        return Err(CliError::PartialFailure {
            succeeded: results.len() - failed_count,
            failed: failed_count,
        });
    }

    Ok(())
}

async fn upload_preset_to_device(ip: &str, preset: &Preset, timeout: Duration) -> Result<(), CliError> {
    let params = match preset.preset_type {
        PresetType::Full => {
            if let Some(ref config) = preset.config {
                config_to_params(config)
            } else {
                return Err(CliError::Other("Preset has no config data".to_string()));
            }
        }
        PresetType::Locations => {
            if let Some(ref locations) = preset.locations {
                location_to_params(locations)
            } else {
                return Err(CliError::Other("Preset has no location data".to_string()));
            }
        }
    };

    // Write each parameter
    for (group, name, value) in &params {
        let cmd = Commands::write_param(group, name, value);
        send_command(ip, &cmd, timeout).await?;
    }

    // Save config
    if preset.preset_type == PresetType::Full {
        // For full presets, save with the preset name
        let cmd = Commands::save_config_as(&preset.name);
        send_command(ip, &cmd, timeout).await?;
    } else {
        // For location presets, just save
        send_command(ip, Commands::save_config(), timeout).await?;
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
