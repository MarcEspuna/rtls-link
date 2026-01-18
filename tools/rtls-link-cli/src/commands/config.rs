//! Configuration commands implementation.

use std::time::Duration;

use crate::cli::{ConfigArgs, ConfigCommands, RoleFilter};
use crate::device::discovery::{discover_devices, DiscoveryOptions, DISCOVERY_PORT};
use crate::device::websocket::send_command;
use crate::error::{CliError, ConfigError};
use crate::output::{get_formatter, OutputFormatter};
use crate::protocol::commands::Commands;
use crate::protocol::config_params::config_to_params;
use crate::protocol::response::parse_json_response;
use crate::types::{Device, DeviceConfig, DeviceRole};

/// Run the config command
pub async fn run_config(args: ConfigArgs, timeout: u64, json: bool, strict: bool) -> Result<(), CliError> {
    let _formatter = get_formatter(json);
    let timeout_duration = Duration::from_millis(timeout);

    match args.command {
        ConfigCommands::Backup(args) => {
            run_backup(&args.ip, args.output.as_deref(), timeout_duration, json).await
        }
        ConfigCommands::Apply(args) => {
            run_apply(
                &args.target,
                &args.file,
                args.skip_short_addr,
                args.filter_role,
                args.concurrency,
                timeout_duration,
                json,
                strict,
            )
            .await
        }
        ConfigCommands::Read(args) => {
            run_read(&args.ip, &args.group, &args.name, timeout_duration, json).await
        }
        ConfigCommands::Write(args) => {
            run_write(
                &args.ip,
                &args.group,
                &args.name,
                &args.value,
                args.save,
                timeout_duration,
                json,
            )
            .await
        }
        ConfigCommands::List(args) => run_list(&args.ip, timeout_duration, json).await,
        ConfigCommands::SaveAs(args) => run_save_as(&args.ip, &args.name, timeout_duration, json).await,
        ConfigCommands::Load(args) => run_load(&args.ip, &args.name, timeout_duration, json).await,
        ConfigCommands::Delete(args) => run_delete(&args.ip, &args.name, timeout_duration, json).await,
    }
}

async fn run_backup(
    ip: &str,
    output: Option<&str>,
    timeout: Duration,
    _json_output: bool,
) -> Result<(), CliError> {
    let response = send_command(ip, Commands::backup_config(), timeout).await?;

    // Parse the config from response
    let config: DeviceConfig = parse_json_response(&response, ip)?;

    let config_json = serde_json::to_string_pretty(&config).map_err(ConfigError::ParseError)?;

    if let Some(output_path) = output {
        std::fs::write(output_path, &config_json)
            .map_err(|e| CliError::Other(format!("Failed to write file: {}", e)))?;
        println!("Configuration saved to {}", output_path);
    } else {
        println!("{}", config_json);
    }

    Ok(())
}

async fn run_apply(
    target: &str,
    file: &str,
    _skip_short_addr: bool,
    filter_role: Option<RoleFilter>,
    _concurrency: usize,
    timeout: Duration,
    json_output: bool,
    strict: bool,
) -> Result<(), CliError> {
    // Read config file
    let config_content = std::fs::read_to_string(file)
        .map_err(|e| CliError::Other(format!("Failed to read config file: {}", e)))?;

    let config: DeviceConfig =
        serde_json::from_str(&config_content).map_err(ConfigError::ParseError)?;

    // Get params to write
    let params = config_to_params(&config);

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
        devices.into_iter().map(|d| d.ip).collect()
    } else if target.contains(',') {
        target.split(',').map(|s| s.trim().to_string()).collect()
    } else {
        vec![target.to_string()]
    };

    if ips.is_empty() {
        return Err(CliError::NoDevicesFound);
    }

    let formatter = get_formatter(json_output);
    let mut results = Vec::new();

    for ip in &ips {
        let result = apply_config_to_device(ip, &params, timeout).await;
        let success = result.is_ok();
        let message = match &result {
            Ok(_) => "Configuration applied".to_string(),
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

async fn apply_config_to_device(
    ip: &str,
    params: &[(String, String, String)],
    timeout: Duration,
) -> Result<(), CliError> {
    // Write each parameter
    for (group, name, value) in params {
        let cmd = Commands::write_param(group, name, value);
        send_command(ip, &cmd, timeout).await?;
    }

    // Save config
    send_command(ip, Commands::save_config(), timeout).await?;

    Ok(())
}

async fn run_read(
    ip: &str,
    group: &str,
    name: &str,
    timeout: Duration,
    json_output: bool,
) -> Result<(), CliError> {
    let cmd = Commands::read_param(group, name);
    let response = send_command(ip, &cmd, timeout).await?;

    if json_output {
        let output = serde_json::json!({
            "group": group,
            "name": name,
            "value": response.trim()
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        println!("{}", response.trim());
    }

    Ok(())
}

async fn run_write(
    ip: &str,
    group: &str,
    name: &str,
    value: &str,
    save: bool,
    timeout: Duration,
    json_output: bool,
) -> Result<(), CliError> {
    let cmd = Commands::write_param(group, name, value);
    let _response = send_command(ip, &cmd, timeout).await?;

    if save {
        send_command(ip, Commands::save_config(), timeout).await?;
    }

    if json_output {
        let output = serde_json::json!({
            "success": true,
            "group": group,
            "name": name,
            "value": value,
            "saved": save
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        println!("Parameter written: {}:{} = {}", group, name, value);
        if save {
            println!("Configuration saved to flash.");
        }
    }

    Ok(())
}

async fn run_list(ip: &str, timeout: Duration, json_output: bool) -> Result<(), CliError> {
    let response = send_command(ip, Commands::list_configs(), timeout).await?;

    // Parse JSON response
    let configs: serde_json::Value = parse_json_response(&response, ip)?;

    if json_output {
        println!("{}", serde_json::to_string_pretty(&configs).unwrap());
    } else {
        if let Some(arr) = configs.as_array() {
            if arr.is_empty() {
                println!("No saved configurations on device.");
            } else {
                println!("Saved configurations:");
                for config in arr {
                    if let Some(name) = config.as_str() {
                        println!("  - {}", name);
                    } else if let Some(name) = config.get("name").and_then(|n| n.as_str()) {
                        println!("  - {}", name);
                    }
                }
            }
        } else {
            println!("{}", response);
        }
    }

    Ok(())
}

async fn run_save_as(ip: &str, name: &str, timeout: Duration, json_output: bool) -> Result<(), CliError> {
    let cmd = Commands::save_config_as(name);
    let response = send_command(ip, &cmd, timeout).await?;

    if json_output {
        // Try to parse as JSON
        if let Ok(json) = parse_json_response::<serde_json::Value>(&response, ip) {
            println!("{}", serde_json::to_string_pretty(&json).unwrap());
        } else {
            println!(
                "{}",
                serde_json::json!({
                    "success": true,
                    "name": name
                })
            );
        }
    } else {
        println!("Configuration saved as '{}'", name);
    }

    Ok(())
}

async fn run_load(ip: &str, name: &str, timeout: Duration, json_output: bool) -> Result<(), CliError> {
    let cmd = Commands::load_config_named(name);
    let response = send_command(ip, &cmd, timeout).await?;

    if json_output {
        if let Ok(json) = parse_json_response::<serde_json::Value>(&response, ip) {
            println!("{}", serde_json::to_string_pretty(&json).unwrap());
        } else {
            println!(
                "{}",
                serde_json::json!({
                    "success": true,
                    "name": name
                })
            );
        }
    } else {
        println!("Configuration '{}' loaded", name);
    }

    Ok(())
}

async fn run_delete(ip: &str, name: &str, timeout: Duration, json_output: bool) -> Result<(), CliError> {
    let cmd = Commands::delete_config(name);
    let response = send_command(ip, &cmd, timeout).await?;

    if json_output {
        if let Ok(json) = parse_json_response::<serde_json::Value>(&response, ip) {
            println!("{}", serde_json::to_string_pretty(&json).unwrap());
        } else {
            println!(
                "{}",
                serde_json::json!({
                    "success": true,
                    "name": name
                })
            );
        }
    } else {
        println!("Configuration '{}' deleted", name);
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
