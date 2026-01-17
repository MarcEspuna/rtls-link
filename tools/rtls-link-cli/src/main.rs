//! RTLS-Link CLI - Command-line interface for RTLS-Link device management.
//!
//! This tool provides terminal access to all RTLS-Link device management functionality,
//! enabling automation via scripts and headless operation.

mod cli;
mod commands;
mod device;
mod error;
mod health;
mod output;
mod protocol;
mod storage;
mod types;

use clap::Parser;

use cli::{Cli, Commands};
use error::{exit_codes, CliError};

#[tokio::main]
async fn main() {
    let cli = Cli::parse();

    let result = run(cli).await;

    match result {
        Ok(()) => std::process::exit(exit_codes::SUCCESS),
        Err(e) => {
            eprintln!("Error: {}", e);
            std::process::exit(e.exit_code());
        }
    }
}

async fn run(cli: Cli) -> Result<(), CliError> {
    match cli.command {
        Commands::Discover(args) => {
            commands::run_discover(args, cli.json).await
        }
        Commands::Status(args) => {
            commands::run_status(args, cli.timeout, cli.json).await
        }
        Commands::Config(args) => {
            commands::run_config(args, cli.timeout, cli.json, cli.strict).await
        }
        Commands::Preset(args) => {
            commands::run_preset(args, cli.timeout, cli.json, cli.strict).await
        }
        Commands::Ota(args) => {
            commands::run_ota(args, cli.json, cli.strict).await
        }
        Commands::Logs(args) => {
            commands::run_logs(args, cli.json).await
        }
        Commands::Cmd(args) => {
            commands::run_cmd(args, cli.timeout, cli.json).await
        }
        Commands::Bulk(args) => {
            commands::run_bulk(args, cli.timeout, cli.json, cli.strict).await
        }
    }
}
