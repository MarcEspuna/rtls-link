//! Configuration to parameter conversion.
//!
//! Converts DeviceConfig to an array of [group, name, value] tuples
//! for uploading to devices via write commands.
//!
//! IMPORTANT: devShortAddr is intentionally skipped to preserve device identity.

use crate::types::{DeviceConfig, LocationData};

/// Convert a DeviceConfig to parameter tuples.
///
/// Each tuple is (group, name, value).
/// Note: devShortAddr is intentionally skipped to preserve device identity.
pub fn config_to_params(config: &DeviceConfig) -> Vec<(String, String, String)> {
    let mut params = Vec::new();

    // WiFi params
    params.push(("wifi".to_string(), "mode".to_string(), config.wifi.mode.to_string()));

    if let Some(ref v) = config.wifi.ssid_a_p {
        params.push(("wifi".to_string(), "ssidAP".to_string(), v.clone()));
    }
    if let Some(ref v) = config.wifi.pswd_a_p {
        params.push(("wifi".to_string(), "pswdAP".to_string(), v.clone()));
    }
    if let Some(ref v) = config.wifi.ssid_s_t {
        params.push(("wifi".to_string(), "ssidST".to_string(), v.clone()));
    }
    if let Some(ref v) = config.wifi.pswd_s_t {
        params.push(("wifi".to_string(), "pswdST".to_string(), v.clone()));
    }
    if let Some(ref v) = config.wifi.gcs_ip {
        params.push(("wifi".to_string(), "gcsIp".to_string(), v.clone()));
    }
    if let Some(v) = config.wifi.udp_port {
        params.push(("wifi".to_string(), "udpPort".to_string(), v.to_string()));
    }
    if let Some(v) = config.wifi.enable_web_server {
        params.push(("wifi".to_string(), "enableWebServer".to_string(), v.to_string()));
    }
    if let Some(v) = config.wifi.enable_discovery {
        params.push(("wifi".to_string(), "enableDiscovery".to_string(), v.to_string()));
    }
    if let Some(v) = config.wifi.discovery_port {
        params.push(("wifi".to_string(), "discoveryPort".to_string(), v.to_string()));
    }
    if let Some(v) = config.wifi.log_udp_port {
        params.push(("wifi".to_string(), "logUdpPort".to_string(), v.to_string()));
    }
    if let Some(v) = config.wifi.log_serial_enabled {
        params.push(("wifi".to_string(), "logSerialEnabled".to_string(), v.to_string()));
    }
    if let Some(v) = config.wifi.log_udp_enabled {
        params.push(("wifi".to_string(), "logUdpEnabled".to_string(), v.to_string()));
    }

    // UWB params
    params.push(("uwb".to_string(), "mode".to_string(), config.uwb.mode.to_string()));
    // NOTE: devShortAddr intentionally skipped - preserved per-device

    // Flatten anchors array to devId1/x1/y1/z1, devId2/x2/y2/z2, etc.
    if let Some(ref anchors) = config.uwb.anchors {
        if !anchors.is_empty() {
            params.push(("uwb".to_string(), "anchorCount".to_string(), anchors.len().to_string()));
            for (i, anchor) in anchors.iter().enumerate() {
                let idx = i + 1; // 1-indexed in firmware
                params.push(("uwb".to_string(), format!("devId{}", idx), anchor.id.clone()));
                params.push(("uwb".to_string(), format!("x{}", idx), anchor.x.to_string()));
                params.push(("uwb".to_string(), format!("y{}", idx), anchor.y.to_string()));
                params.push(("uwb".to_string(), format!("z{}", idx), anchor.z.to_string()));
            }
        }
    } else if let Some(v) = config.uwb.anchor_count {
        params.push(("uwb".to_string(), "anchorCount".to_string(), v.to_string()));
    }

    if let Some(v) = config.uwb.origin_lat {
        params.push(("uwb".to_string(), "originLat".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.origin_lon {
        params.push(("uwb".to_string(), "originLon".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.origin_alt {
        params.push(("uwb".to_string(), "originAlt".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.mavlink_target_system_id {
        params.push(("uwb".to_string(), "mavlinkTargetSystemId".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.rotation_degrees {
        params.push(("uwb".to_string(), "rotationDegrees".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.z_calc_mode {
        params.push(("uwb".to_string(), "zCalcMode".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.channel {
        params.push(("uwb".to_string(), "channel".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.dw_mode {
        params.push(("uwb".to_string(), "dwMode".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.tx_power_level {
        params.push(("uwb".to_string(), "txPowerLevel".to_string(), v.to_string()));
    }
    if let Some(v) = config.uwb.smart_power_enable {
        params.push(("uwb".to_string(), "smartPowerEnable".to_string(), v.to_string()));
    }

    // App params
    if let Some(v) = config.app.led2_pin {
        params.push(("app".to_string(), "led2Pin".to_string(), v.to_string()));
    }
    if let Some(v) = config.app.led2_state {
        params.push(("app".to_string(), "led2State".to_string(), v.to_string()));
    }

    params
}

/// Convert LocationData to parameter tuples.
///
/// This is used for location-only presets and only includes:
/// - Origin (lat, lon, alt)
/// - Rotation
/// - Anchors
pub fn location_to_params(location: &LocationData) -> Vec<(String, String, String)> {
    let mut params = Vec::new();

    // Origin
    params.push(("uwb".to_string(), "originLat".to_string(), location.origin.lat.to_string()));
    params.push(("uwb".to_string(), "originLon".to_string(), location.origin.lon.to_string()));
    params.push(("uwb".to_string(), "originAlt".to_string(), location.origin.alt.to_string()));

    // Rotation
    params.push(("uwb".to_string(), "rotationDegrees".to_string(), location.rotation.to_string()));

    // Anchors
    if !location.anchors.is_empty() {
        params.push(("uwb".to_string(), "anchorCount".to_string(), location.anchors.len().to_string()));
        for (i, anchor) in location.anchors.iter().enumerate() {
            let idx = i + 1;
            params.push(("uwb".to_string(), format!("devId{}", idx), anchor.id.clone()));
            params.push(("uwb".to_string(), format!("x{}", idx), anchor.x.to_string()));
            params.push(("uwb".to_string(), format!("y{}", idx), anchor.y.to_string()));
            params.push(("uwb".to_string(), format!("z{}", idx), anchor.z.to_string()));
        }
    }

    params
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::types::{AnchorConfig, AppConfig, UwbConfig, WifiConfig};

    #[test]
    fn test_config_to_params_basic() {
        let config = DeviceConfig {
            wifi: WifiConfig {
                mode: 1,
                ssid_a_p: None,
                pswd_a_p: None,
                ssid_s_t: Some("TestNetwork".to_string()),
                pswd_s_t: Some("password123".to_string()),
                gcs_ip: Some("192.168.1.1".to_string()),
                udp_port: Some(14550),
                enable_web_server: Some(1),
                enable_discovery: Some(1),
                discovery_port: Some(3333),
                log_udp_port: None,
                log_serial_enabled: None,
                log_udp_enabled: None,
            },
            uwb: UwbConfig {
                mode: 4,
                dev_short_addr: "1".to_string(), // Should be skipped
                anchor_count: None,
                anchors: Some(vec![
                    AnchorConfig { id: "1".to_string(), x: 0.0, y: 0.0, z: 1.5 },
                    AnchorConfig { id: "2".to_string(), x: 3.0, y: 0.0, z: 1.5 },
                ]),
                origin_lat: Some(41.4036),
                origin_lon: Some(2.1744),
                origin_alt: Some(100.0),
                mavlink_target_system_id: Some(1),
                rotation_degrees: Some(0.0),
                z_calc_mode: Some(1),
                channel: None,
                dw_mode: None,
                tx_power_level: None,
                smart_power_enable: None,
            },
            app: AppConfig {
                led2_pin: Some(2),
                led2_state: Some(0),
            },
        };

        let params = config_to_params(&config);

        // Check that devShortAddr is NOT in the params
        assert!(!params.iter().any(|(_, n, _)| n == "devShortAddr"));

        // Check that anchors are flattened
        assert!(params.iter().any(|(g, n, v)| g == "uwb" && n == "anchorCount" && v == "2"));
        assert!(params.iter().any(|(g, n, v)| g == "uwb" && n == "devId1" && v == "1"));
        assert!(params.iter().any(|(g, n, v)| g == "uwb" && n == "x1" && v == "0"));
        assert!(params.iter().any(|(g, n, v)| g == "uwb" && n == "devId2" && v == "2"));

        // Check other params
        assert!(params.iter().any(|(g, n, v)| g == "wifi" && n == "mode" && v == "1"));
        assert!(params.iter().any(|(g, n, v)| g == "wifi" && n == "ssidST" && v == "TestNetwork"));
    }
}
