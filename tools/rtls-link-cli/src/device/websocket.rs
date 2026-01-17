//! WebSocket client for device communication.

use std::time::Duration;

use futures_util::{SinkExt, StreamExt};
use tokio::time::timeout;
use tokio_tungstenite::{connect_async, tungstenite::Message};

use crate::error::{CliError, DeviceError, NetworkError};
use crate::protocol::response::is_error_response;

/// WebSocket connection to a device
pub struct DeviceConnection {
    ip: String,
    timeout: Duration,
}

impl DeviceConnection {
    /// Create a new connection to a device
    pub fn new(ip: &str, timeout_ms: u64) -> Self {
        Self {
            ip: ip.to_string(),
            timeout: Duration::from_millis(timeout_ms),
        }
    }

    /// Get the WebSocket URL for this device
    fn ws_url(&self) -> String {
        format!("ws://{}/ws", self.ip)
    }

    /// Send a command and wait for response
    pub async fn send(&self, command: &str) -> Result<String, CliError> {
        send_command(&self.ip, command, self.timeout).await
    }

    /// Send multiple commands sequentially
    pub async fn send_batch(&self, commands: &[String]) -> Result<Vec<String>, CliError> {
        let mut responses = Vec::new();

        for cmd in commands {
            let response = self.send(cmd).await?;
            responses.push(response);
        }

        Ok(responses)
    }
}

/// Send a command to a device and wait for response.
///
/// Opens a fresh WebSocket connection, sends the command, waits for response, and closes.
pub async fn send_command(ip: &str, command: &str, cmd_timeout: Duration) -> Result<String, CliError> {
    let url = format!("ws://{}/ws", ip);

    // Connect with timeout
    let connect_timeout = Duration::from_secs(5);
    let (ws_stream, _) = timeout(connect_timeout, connect_async(&url))
        .await
        .map_err(|_| NetworkError::ConnectionTimeout(ip.to_string()))?
        .map_err(|e| NetworkError::WebSocketConnect {
            addr: url.clone(),
            source: e,
        })?;

    let (mut write, mut read) = ws_stream.split();

    // Send command
    write
        .send(Message::Text(command.to_string()))
        .await
        .map_err(NetworkError::WebSocket)?;

    // Wait for response with timeout
    let response = timeout(cmd_timeout, async {
        let mut response_parts = Vec::new();

        while let Some(msg) = read.next().await {
            match msg {
                Ok(Message::Text(text)) => {
                    response_parts.push(text);
                    // For most commands, we get a single response
                    // For readall, we might get multiple
                    break;
                }
                Ok(Message::Close(_)) => break,
                Ok(_) => continue,
                Err(e) => return Err(NetworkError::WebSocket(e)),
            }
        }

        Ok(response_parts.join("\n"))
    })
    .await
    .map_err(|_| CliError::Timeout(format!("Command to {} timed out", ip)))?
    .map_err(CliError::Network)?;

    // Check for error response
    if let Some(error_msg) = is_error_response(&response) {
        return Err(CliError::Device(DeviceError::CommandFailed {
            ip: ip.to_string(),
            message: error_msg,
        }));
    }

    Ok(response)
}

/// Send a command to a device with retry logic
pub async fn send_command_with_retry(
    ip: &str,
    command: &str,
    cmd_timeout: Duration,
    max_retries: usize,
) -> Result<String, CliError> {
    let mut last_error = None;

    for attempt in 0..=max_retries {
        match send_command(ip, command, cmd_timeout).await {
            Ok(response) => return Ok(response),
            Err(e) => {
                last_error = Some(e);
                if attempt < max_retries {
                    // Wait before retry
                    tokio::time::sleep(Duration::from_millis(500)).await;
                }
            }
        }
    }

    Err(last_error.unwrap())
}

/// Batch command sender with concurrency control
pub struct BatchSender {
    timeout: Duration,
    concurrency: usize,
}

impl BatchSender {
    pub fn new(timeout_ms: u64, concurrency: usize) -> Self {
        Self {
            timeout: Duration::from_millis(timeout_ms),
            concurrency,
        }
    }

    /// Send a command to multiple devices concurrently
    pub async fn send_to_all(
        &self,
        ips: &[String],
        command: &str,
    ) -> Vec<(String, Result<String, CliError>)> {
        use futures::stream::{self, StreamExt};

        let results: Vec<_> = stream::iter(ips.iter().cloned())
            .map(|ip| {
                let cmd = command.to_string();
                let timeout = self.timeout;
                async move {
                    let result = send_command(&ip, &cmd, timeout).await;
                    (ip, result)
                }
            })
            .buffer_unordered(self.concurrency)
            .collect()
            .await;

        results
    }
}
