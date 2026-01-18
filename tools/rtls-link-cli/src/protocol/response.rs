//! Response parsing utilities for device protocol.

use crate::error::DeviceError;

/// Parse a JSON response from device output.
///
/// Device responses may contain prefix text before the JSON, so we
/// find the first `{` and parse from there.
pub fn parse_json_response<T: serde::de::DeserializeOwned>(
    response: &str,
    device_ip: &str,
) -> Result<T, DeviceError> {
    // Find the first '{' to locate JSON start
    let json_start = response.find('{').ok_or_else(|| DeviceError::InvalidResponse {
        ip: device_ip.to_string(),
        message: "No JSON found in response".to_string(),
    })?;

    let json_str = &response[json_start..];

    serde_json::from_str(json_str).map_err(|e| DeviceError::InvalidResponse {
        ip: device_ip.to_string(),
        message: format!("Failed to parse JSON: {}", e),
    })
}

/// Response parser with configurable behavior
pub struct ResponseParser {
    /// Whether to expect JSON response
    pub expect_json: bool,
}

impl ResponseParser {
    pub fn new(expect_json: bool) -> Self {
        Self { expect_json }
    }

    /// Parse response based on configuration
    pub fn parse(&self, response: &str, device_ip: &str) -> Result<ParsedResponse, DeviceError> {
        if self.expect_json {
            let value: serde_json::Value = parse_json_response(response, device_ip)?;
            Ok(ParsedResponse::Json(value))
        } else {
            Ok(ParsedResponse::Text(response.to_string()))
        }
    }
}

/// Parsed response from a device
#[derive(Debug)]
pub enum ParsedResponse {
    /// JSON response
    Json(serde_json::Value),
    /// Plain text response
    Text(String),
}

impl ParsedResponse {
    /// Get as JSON value (if JSON)
    pub fn as_json(&self) -> Option<&serde_json::Value> {
        match self {
            ParsedResponse::Json(v) => Some(v),
            ParsedResponse::Text(_) => None,
        }
    }

    /// Get as text
    pub fn as_text(&self) -> &str {
        match self {
            ParsedResponse::Json(_v) => "",
            ParsedResponse::Text(s) => s,
        }
    }

    /// Convert to pretty string
    pub fn to_pretty_string(&self) -> String {
        match self {
            ParsedResponse::Json(v) => serde_json::to_string_pretty(v).unwrap_or_default(),
            ParsedResponse::Text(s) => s.clone(),
        }
    }
}

/// Check if a command response indicates an error
pub fn is_error_response(response: &str) -> Option<String> {
    let lower = response.to_lowercase();

    // Common error patterns
    if lower.contains("error:") {
        // Extract error message
        if let Some(pos) = lower.find("error:") {
            let msg = response[pos + 6..].trim();
            return Some(msg.to_string());
        }
    }

    if lower.contains("failed") || lower.contains("invalid") {
        return Some(response.trim().to_string());
    }

    // Check for JSON error response
    if let Some(start) = response.find('{') {
        if let Ok(json) = serde_json::from_str::<serde_json::Value>(&response[start..]) {
            if let Some(success) = json.get("success") {
                if success == false {
                    if let Some(msg) = json.get("message").or_else(|| json.get("error")) {
                        return Some(msg.as_str().unwrap_or("Unknown error").to_string());
                    }
                    return Some("Command failed".to_string());
                }
            }
            if let Some(error) = json.get("error") {
                return Some(error.as_str().unwrap_or("Unknown error").to_string());
            }
        }
    }

    None
}

/// Parse a readall response into key-value pairs
pub fn parse_readall_response(response: &str) -> Vec<(String, String, String)> {
    let mut params = Vec::new();
    let mut current_group = String::new();

    for line in response.lines() {
        let line = line.trim();

        // Skip empty lines
        if line.is_empty() {
            continue;
        }

        // Check for group header (e.g., "[wifi]")
        if line.starts_with('[') && line.ends_with(']') {
            current_group = line[1..line.len()-1].to_string();
            continue;
        }

        // Parse key=value pairs
        if let Some(eq_pos) = line.find('=') {
            let name = line[..eq_pos].trim().to_string();
            let value = line[eq_pos+1..].trim().to_string();

            if !current_group.is_empty() {
                params.push((current_group.clone(), name, value));
            }
        }
    }

    params
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_json_with_prefix() {
        let response = r#"OK
{"success": true, "value": 42}"#;

        let result: serde_json::Value = parse_json_response(response, "192.168.1.1").unwrap();
        assert_eq!(result["success"], true);
        assert_eq!(result["value"], 42);
    }

    #[test]
    fn test_parse_json_no_prefix() {
        let response = r#"{"success": true}"#;
        let result: serde_json::Value = parse_json_response(response, "192.168.1.1").unwrap();
        assert_eq!(result["success"], true);
    }

    #[test]
    fn test_parse_json_no_json() {
        let response = "OK - command completed";
        let result: Result<serde_json::Value, _> = parse_json_response(response, "192.168.1.1");
        assert!(result.is_err());
    }

    #[test]
    fn test_is_error_response() {
        assert!(is_error_response("Error: command not found").is_some());
        assert!(is_error_response("Failed to write parameter").is_some());
        assert!(is_error_response(r#"{"success": false, "message": "Invalid param"}"#).is_some());
        assert!(is_error_response("OK - success").is_none());
        assert!(is_error_response(r#"{"success": true}"#).is_none());
    }

    #[test]
    fn test_parse_readall_response() {
        let response = r#"
[wifi]
mode=1
ssidST=TestNetwork
pswdST=password123

[uwb]
mode=4
devShortAddr=1
"#;

        let params = parse_readall_response(response);
        assert_eq!(params.len(), 5);
        assert_eq!(params[0], ("wifi".to_string(), "mode".to_string(), "1".to_string()));
        assert_eq!(params[1], ("wifi".to_string(), "ssidST".to_string(), "TestNetwork".to_string()));
        assert_eq!(params[3], ("uwb".to_string(), "mode".to_string(), "4".to_string()));
    }
}
