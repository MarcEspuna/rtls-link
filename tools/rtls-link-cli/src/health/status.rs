//! Health status calculation for RTLS-Link devices.
//!
//! Ported from rtls-link-manager's healthStatus.ts

use crate::types::Device;
use serde::Serialize;

/// Health level classification
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum HealthLevel {
    /// Device is operating normally
    Healthy,
    /// Minor issues detected
    Warning,
    /// Critical issues affecting operation
    Degraded,
    /// Unable to determine health
    Unknown,
}

impl HealthLevel {
    pub fn as_str(&self) -> &'static str {
        match self {
            HealthLevel::Healthy => "healthy",
            HealthLevel::Warning => "warning",
            HealthLevel::Degraded => "degraded",
            HealthLevel::Unknown => "unknown",
        }
    }
}

/// Device health information
#[derive(Debug, Clone, Serialize)]
pub struct DeviceHealth {
    /// Overall health level
    pub level: HealthLevel,
    /// List of issues detected
    pub issues: Vec<String>,
}

/// Calculate the health status of a device.
///
/// - Anchors: Always healthy if online (offline devices are pruned by TTL)
/// - Tags: Based on telemetry fields (sendingPos, anchorsSeen, originSent, rfHealthy)
/// - Unknown telemetry (undefined) = 'unknown' level, not warning
pub fn calculate_device_health(device: &Device) -> DeviceHealth {
    // Anchors are simple - if we see them, they're healthy
    if device.role.is_anchor() {
        return DeviceHealth {
            level: HealthLevel::Healthy,
            issues: Vec::new(),
        };
    }

    // Tags need telemetry analysis
    if device.role.is_tag() {
        return calculate_tag_health(device);
    }

    // Calibration or unknown roles
    DeviceHealth {
        level: HealthLevel::Unknown,
        issues: Vec::new(),
    }
}

fn calculate_tag_health(device: &Device) -> DeviceHealth {
    let mut issues = Vec::new();
    let mut has_telemetry = false;

    // Check if we have any telemetry data at all
    if device.sending_pos.is_some()
        || device.anchors_seen.is_some()
        || device.origin_sent.is_some()
        || device.rf_enabled.is_some()
    {
        has_telemetry = true;
    }

    // No telemetry = unknown (old firmware or not yet received)
    if !has_telemetry {
        return DeviceHealth {
            level: HealthLevel::Unknown,
            issues: vec!["No telemetry data".to_string()],
        };
    }

    // Check sendingPos - critical for tag operation
    if device.sending_pos == Some(false) {
        issues.push("Not sending positions".to_string());
    }

    // Check anchorsSeen - need at least 3 for 2D positioning
    if let Some(anchors) = device.anchors_seen {
        if anchors < 3 {
            let plural = if anchors == 1 { "" } else { "s" };
            issues.push(format!("Only seeing {} anchor{}", anchors, plural));
        }
    }

    // Check originSent - needed for ArduPilot integration
    if device.origin_sent == Some(false) {
        issues.push("Origin not sent to autopilot".to_string());
    }

    // Check rangefinder health (only if enabled)
    if device.rf_enabled == Some(true) && device.rf_healthy == Some(false) {
        issues.push("Rangefinder unhealthy".to_string());
    }

    // Determine level based on issues
    if issues.is_empty() {
        return DeviceHealth {
            level: HealthLevel::Healthy,
            issues: Vec::new(),
        };
    }

    // Not sending positions is the most critical
    if device.sending_pos == Some(false) {
        return DeviceHealth {
            level: HealthLevel::Degraded,
            issues,
        };
    }

    // Anchor count < 3 is concerning
    if let Some(anchors) = device.anchors_seen {
        if anchors < 3 {
            return DeviceHealth {
                level: HealthLevel::Degraded,
                issues,
            };
        }
    }

    // Other issues are warnings
    DeviceHealth {
        level: HealthLevel::Warning,
        issues,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::DeviceRole;

    fn make_device(role: DeviceRole) -> Device {
        Device {
            ip: "192.168.1.1".to_string(),
            id: "test".to_string(),
            role,
            mac: "AA:BB:CC:DD:EE:FF".to_string(),
            uwb_short: "1".to_string(),
            mav_sys_id: 1,
            firmware: "1.0.0".to_string(),
            online: Some(true),
            last_seen: None,
            sending_pos: None,
            anchors_seen: None,
            origin_sent: None,
            rf_enabled: None,
            rf_healthy: None,
            avg_rate_c_hz: None,
            min_rate_c_hz: None,
            max_rate_c_hz: None,
            log_level: None,
            log_udp_port: None,
            log_serial_enabled: None,
            log_udp_enabled: None,
        }
    }

    #[test]
    fn test_anchor_always_healthy() {
        let device = make_device(DeviceRole::Anchor);
        let health = calculate_device_health(&device);
        assert_eq!(health.level, HealthLevel::Healthy);
        assert!(health.issues.is_empty());
    }

    #[test]
    fn test_tag_no_telemetry_unknown() {
        let device = make_device(DeviceRole::Tag);
        let health = calculate_device_health(&device);
        assert_eq!(health.level, HealthLevel::Unknown);
    }

    #[test]
    fn test_tag_healthy() {
        let mut device = make_device(DeviceRole::TagTdoa);
        device.sending_pos = Some(true);
        device.anchors_seen = Some(4);
        device.origin_sent = Some(true);

        let health = calculate_device_health(&device);
        assert_eq!(health.level, HealthLevel::Healthy);
        assert!(health.issues.is_empty());
    }

    #[test]
    fn test_tag_not_sending_pos_degraded() {
        let mut device = make_device(DeviceRole::TagTdoa);
        device.sending_pos = Some(false);
        device.anchors_seen = Some(4);

        let health = calculate_device_health(&device);
        assert_eq!(health.level, HealthLevel::Degraded);
        assert!(health.issues.iter().any(|i| i.contains("Not sending")));
    }

    #[test]
    fn test_tag_low_anchor_count_degraded() {
        let mut device = make_device(DeviceRole::TagTdoa);
        device.sending_pos = Some(true);
        device.anchors_seen = Some(2);

        let health = calculate_device_health(&device);
        assert_eq!(health.level, HealthLevel::Degraded);
        assert!(health.issues.iter().any(|i| i.contains("2 anchors")));
    }

    #[test]
    fn test_tag_origin_not_sent_warning() {
        let mut device = make_device(DeviceRole::TagTdoa);
        device.sending_pos = Some(true);
        device.anchors_seen = Some(4);
        device.origin_sent = Some(false);

        let health = calculate_device_health(&device);
        assert_eq!(health.level, HealthLevel::Warning);
        assert!(health.issues.iter().any(|i| i.contains("Origin")));
    }
}
