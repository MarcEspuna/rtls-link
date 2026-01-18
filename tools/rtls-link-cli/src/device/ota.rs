//! OTA firmware upload functionality.

use std::path::Path;
use std::time::Duration;

use indicatif::{ProgressBar, ProgressStyle};
use reqwest::multipart;
use tokio::fs::File;
use tokio::io::AsyncReadExt;

use crate::error::{CliError, DeviceError, NetworkError};

/// Upload firmware to a device via HTTP.
///
/// Uses the OTA update endpoint at http://<ip>/update
pub async fn upload_firmware(ip: &str, firmware_path: &Path) -> Result<(), CliError> {
    // Read firmware file
    let mut file = File::open(firmware_path).await.map_err(|e| {
        CliError::Other(format!(
            "Failed to open firmware file '{}': {}",
            firmware_path.display(),
            e
        ))
    })?;

    let mut firmware_data = Vec::new();
    file.read_to_end(&mut firmware_data).await.map_err(|e| {
        CliError::Other(format!(
            "Failed to read firmware file '{}': {}",
            firmware_path.display(),
            e
        ))
    })?;

    let _file_size = firmware_data.len();

    // Create multipart form
    let file_name = firmware_path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("firmware.bin")
        .to_string();

    let part = multipart::Part::bytes(firmware_data)
        .file_name(file_name)
        .mime_str("application/octet-stream")
        .map_err(|e| CliError::Other(format!("Failed to create multipart: {}", e)))?;

    let form = multipart::Form::new().part("firmware", part);

    // Send request
    let client = reqwest::Client::builder()
        .timeout(Duration::from_secs(120))
        .build()
        .map_err(NetworkError::Http)?;

    let url = format!("http://{}/update", ip);

    let response = client
        .post(&url)
        .multipart(form)
        .send()
        .await
        .map_err(NetworkError::Http)?;

    if !response.status().is_success() {
        let status = response.status();
        let body = response.text().await.unwrap_or_default();
        return Err(CliError::Device(DeviceError::OtaFailed {
            ip: ip.to_string(),
            message: format!("HTTP {}: {}", status, body),
        }));
    }

    Ok(())
}

/// Upload firmware with progress display
pub async fn upload_firmware_with_progress(ip: &str, firmware_path: &Path) -> Result<(), CliError> {
    // Read firmware file
    let mut file = File::open(firmware_path).await.map_err(|e| {
        CliError::Other(format!(
            "Failed to open firmware file '{}': {}",
            firmware_path.display(),
            e
        ))
    })?;

    let mut firmware_data = Vec::new();
    file.read_to_end(&mut firmware_data).await.map_err(|e| {
        CliError::Other(format!(
            "Failed to read firmware file '{}': {}",
            firmware_path.display(),
            e
        ))
    })?;

    let file_size = firmware_data.len() as u64;

    // Create progress bar
    let pb = ProgressBar::new(file_size);
    pb.set_style(
        ProgressStyle::default_bar()
            .template("{spinner:.green} [{elapsed_precise}] [{bar:40.cyan/blue}] {bytes}/{total_bytes} ({eta})")
            .unwrap()
            .progress_chars("#>-"),
    );
    pb.set_message(format!("Uploading to {}", ip));

    // Create multipart form
    let file_name = firmware_path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("firmware.bin")
        .to_string();

    let part = multipart::Part::bytes(firmware_data)
        .file_name(file_name)
        .mime_str("application/octet-stream")
        .map_err(|e| CliError::Other(format!("Failed to create multipart: {}", e)))?;

    let form = multipart::Form::new().part("firmware", part);

    // Send request
    let client = reqwest::Client::builder()
        .timeout(Duration::from_secs(120))
        .build()
        .map_err(NetworkError::Http)?;

    let url = format!("http://{}/update", ip);

    let response = client
        .post(&url)
        .multipart(form)
        .send()
        .await
        .map_err(NetworkError::Http)?;

    pb.finish_with_message(format!("Upload to {} complete", ip));

    if !response.status().is_success() {
        let status = response.status();
        let body = response.text().await.unwrap_or_default();
        return Err(CliError::Device(DeviceError::OtaFailed {
            ip: ip.to_string(),
            message: format!("HTTP {}: {}", status, body),
        }));
    }

    Ok(())
}

/// Upload firmware to multiple devices with progress
pub async fn upload_firmware_bulk(
    ips: &[String],
    firmware_path: &Path,
    concurrency: usize,
) -> Vec<(String, Result<(), CliError>)> {
    use futures::stream::{self, StreamExt};

    // Read firmware file once
    let firmware_data = match tokio::fs::read(firmware_path).await {
        Ok(data) => data,
        Err(e) => {
            let error = CliError::Other(format!(
                "Failed to read firmware file '{}': {}",
                firmware_path.display(),
                e
            ));
            return ips.iter().map(|ip| (ip.clone(), Err(error.clone()))).collect();
        }
    };

    let file_name = firmware_path
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("firmware.bin")
        .to_string();

    // Upload to each device
    let results: Vec<_> = stream::iter(ips.iter().cloned())
        .map(|ip| {
            let data = firmware_data.clone();
            let name = file_name.clone();
            async move {
                let result = upload_firmware_data(&ip, data, &name).await;
                (ip, result)
            }
        })
        .buffer_unordered(concurrency)
        .collect()
        .await;

    results
}

/// Upload firmware data (already loaded) to a device
async fn upload_firmware_data(ip: &str, data: Vec<u8>, file_name: &str) -> Result<(), CliError> {
    let part = multipart::Part::bytes(data)
        .file_name(file_name.to_string())
        .mime_str("application/octet-stream")
        .map_err(|e| CliError::Other(format!("Failed to create multipart: {}", e)))?;

    let form = multipart::Form::new().part("firmware", part);

    let client = reqwest::Client::builder()
        .timeout(Duration::from_secs(120))
        .build()
        .map_err(NetworkError::Http)?;

    let url = format!("http://{}/update", ip);

    let response = client
        .post(&url)
        .multipart(form)
        .send()
        .await
        .map_err(NetworkError::Http)?;

    if !response.status().is_success() {
        let status = response.status();
        let body = response.text().await.unwrap_or_default();
        return Err(CliError::Device(DeviceError::OtaFailed {
            ip: ip.to_string(),
            message: format!("HTTP {}: {}", status, body),
        }));
    }

    Ok(())
}

// Need to implement Clone for CliError for bulk operations
impl Clone for CliError {
    fn clone(&self) -> Self {
        match self {
            CliError::Network(e) => CliError::Other(format!("Network error: {}", e)),
            CliError::Device(e) => CliError::Other(format!("Device error: {}", e)),
            CliError::Config(e) => CliError::Other(format!("Config error: {}", e)),
            CliError::Storage(e) => CliError::Other(format!("Storage error: {}", e)),
            CliError::Io(e) => CliError::Other(format!("IO error: {}", e)),
            CliError::InvalidArgument(s) => CliError::InvalidArgument(s.clone()),
            CliError::PartialFailure { succeeded, failed } => CliError::PartialFailure {
                succeeded: *succeeded,
                failed: *failed,
            },
            CliError::NoDevicesFound => CliError::NoDevicesFound,
            CliError::Timeout(s) => CliError::Timeout(s.clone()),
            CliError::Other(s) => CliError::Other(s.clone()),
        }
    }
}
