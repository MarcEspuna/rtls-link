//! Raw command execution.

use std::time::Duration;

use crate::cli::CmdArgs;
use crate::device::websocket::send_command;
use crate::error::CliError;
use crate::output::{get_formatter, OutputFormatter};
use crate::protocol::commands::is_json_command;

/// Run the cmd command
pub async fn run_cmd(args: CmdArgs, timeout: u64, json: bool) -> Result<(), CliError> {
    let formatter = get_formatter(json);
    let timeout_duration = Duration::from_millis(timeout);

    let expect_json = args.expect_json || is_json_command(&args.command);

    let response = send_command(&args.ip, &args.command, timeout_duration).await?;

    if json {
        // Try to parse response as JSON if expected
        if expect_json {
            if let Ok(json_value) = serde_json::from_str::<serde_json::Value>(&response) {
                println!(
                    "{}",
                    formatter.format_command_result(&args.ip, &args.command, &serde_json::to_string_pretty(&json_value).unwrap(), true)
                );
            } else {
                // Find JSON in response
                if let Some(start) = response.find('{') {
                    if let Ok(json_value) = serde_json::from_str::<serde_json::Value>(&response[start..]) {
                        println!(
                            "{}",
                            formatter.format_command_result(&args.ip, &args.command, &serde_json::to_string_pretty(&json_value).unwrap(), true)
                        );
                    } else {
                        println!(
                            "{}",
                            formatter.format_command_result(&args.ip, &args.command, &response, true)
                        );
                    }
                } else {
                    println!(
                        "{}",
                        formatter.format_command_result(&args.ip, &args.command, &response, true)
                    );
                }
            }
        } else {
            println!(
                "{}",
                formatter.format_command_result(&args.ip, &args.command, &response, true)
            );
        }
    } else {
        // For table output, just print the response directly
        if expect_json {
            // Try to pretty-print JSON
            if let Some(start) = response.find('{') {
                if let Ok(json_value) = serde_json::from_str::<serde_json::Value>(&response[start..]) {
                    println!("{}", serde_json::to_string_pretty(&json_value).unwrap());
                    return Ok(());
                }
            }
        }
        println!("{}", response);
    }

    Ok(())
}
