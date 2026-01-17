//! Log streaming command.

use std::io::{self, Write};
use std::net::SocketAddr;

use colored::*;
use regex::Regex;
use socket2::{Domain, Protocol, Socket, Type};
use tokio::net::UdpSocket;

use crate::cli::LogsArgs;
use crate::error::{CliError, NetworkError};
use crate::types::{LogLevel, LogMessage};

/// Default log UDP port
const LOG_PORT: u16 = 3334;

/// Run the logs command
pub async fn run_logs(args: LogsArgs, json: bool) -> Result<(), CliError> {
    let min_level = LogLevel::from_str(&args.level)
        .ok_or_else(|| CliError::InvalidArgument(format!("Invalid log level: {}", args.level)))?;

    let tag_pattern = args.tag.as_ref().map(|p| {
        // Convert glob pattern to regex
        let regex_pattern = p.replace('*', ".*").replace('?', ".");
        Regex::new(&format!("^{}$", regex_pattern)).ok()
    }).flatten();

    let socket = create_log_socket(args.port)?;
    let socket = UdpSocket::from_std(socket.into())?;

    println!(
        "Listening for logs on port {} (level >= {}){}",
        args.port,
        min_level,
        if args.ip.is_some() {
            format!(" from {}", args.ip.as_ref().unwrap())
        } else {
            String::new()
        }
    );
    println!("Press Ctrl+C to stop.\n");

    let mut buf = vec![0u8; 4096];

    loop {
        let (len, addr) = socket.recv_from(&mut buf).await.map_err(NetworkError::Io)?;

        let ip = addr.ip().to_string();

        // Filter by IP if specified
        if let Some(ref filter_ip) = args.ip {
            if &ip != filter_ip {
                continue;
            }
        }

        // Try to parse as JSON log message
        if let Ok(log_msg) = parse_log_message(&buf[..len], &ip) {
            // Filter by level
            if (log_msg.level as u8) > (min_level as u8) {
                continue;
            }

            // Filter by tag pattern
            if let Some(ref pattern) = tag_pattern {
                if !pattern.is_match(&log_msg.tag) {
                    continue;
                }
            }

            // Output
            if args.ndjson || json {
                let output = serde_json::json!({
                    "ip": log_msg.ip,
                    "level": log_msg.level.as_str().to_lowercase(),
                    "tag": log_msg.tag,
                    "message": log_msg.message,
                    "timestamp": log_msg.timestamp
                });
                println!("{}", serde_json::to_string(&output).unwrap());
            } else {
                print_colored_log(&log_msg);
            }

            io::stdout().flush().ok();
        }
    }
}

fn create_log_socket(port: u16) -> Result<std::net::UdpSocket, NetworkError> {
    let socket = Socket::new(Domain::IPV4, Type::DGRAM, Some(Protocol::UDP))
        .map_err(NetworkError::UdpBind)?;

    // Enable address reuse
    socket.set_reuse_address(true).map_err(NetworkError::UdpBind)?;

    #[cfg(unix)]
    socket.set_reuse_port(true).map_err(NetworkError::UdpBind)?;

    // Bind to the log port
    let addr: SocketAddr = format!("0.0.0.0:{}", port).parse().unwrap();
    socket.bind(&addr.into()).map_err(NetworkError::UdpBind)?;

    // Set non-blocking for tokio
    socket.set_nonblocking(true).map_err(NetworkError::UdpBind)?;

    Ok(socket.into())
}

fn parse_log_message(data: &[u8], ip: &str) -> Result<LogMessage, serde_json::Error> {
    let json: serde_json::Value = serde_json::from_slice(data)?;

    let level_num = json["level"].as_u64().unwrap_or(3) as u8;
    let level = LogLevel::from_u8(level_num);

    Ok(LogMessage {
        ip: ip.to_string(),
        level,
        tag: json["tag"].as_str().unwrap_or("").to_string(),
        message: json["msg"].as_str()
            .or_else(|| json["message"].as_str())
            .unwrap_or("")
            .to_string(),
        timestamp: json["ts"].as_u64().or_else(|| json["timestamp"].as_u64()),
    })
}

fn print_colored_log(log: &LogMessage) {
    let level_str = format!("{:>7}", log.level.as_str());
    let level_colored = match log.level {
        LogLevel::Error => level_str.red().bold(),
        LogLevel::Warn => level_str.yellow(),
        LogLevel::Info => level_str.green(),
        LogLevel::Debug => level_str.blue(),
        LogLevel::Verbose => level_str.dimmed(),
        LogLevel::None => level_str.normal(),
    };

    let ip_str = format!("{:>15}", log.ip);
    let tag_str = format!("[{}]", log.tag).cyan();

    println!("{} {} {} {}", ip_str.dimmed(), level_colored, tag_str, log.message);
}
