//! Local preset storage.
//!
//! Shares the same storage directory as rtls-link-manager for interoperability.

use std::path::PathBuf;

use directories::ProjectDirs;

use crate::error::StorageError;
use crate::types::{Preset, PresetInfo};

/// Preset storage service
pub struct PresetStorage {
    preset_dir: PathBuf,
}

impl PresetStorage {
    /// Create a new preset storage instance.
    ///
    /// Uses the same directory as rtls-link-manager for shared presets.
    pub fn new() -> Result<Self, StorageError> {
        let project_dirs = ProjectDirs::from("", "rtls-link", "rtls-link-manager")
            .ok_or_else(|| StorageError::DirectoryAccess("Could not determine app data directory".to_string()))?;

        let preset_dir = project_dirs.data_dir().join("presets");

        // Create directory if it doesn't exist
        std::fs::create_dir_all(&preset_dir).map_err(StorageError::Io)?;

        Ok(Self { preset_dir })
    }

    /// Get the path for a preset file
    fn preset_path(&self, name: &str) -> PathBuf {
        self.preset_dir.join(format!("{}.json", name))
    }

    /// Validate a preset name
    fn validate_name(name: &str) -> Result<(), StorageError> {
        if name.is_empty() {
            return Err(StorageError::InvalidPresetName("Name cannot be empty".to_string()));
        }

        // Check for invalid characters
        if name.contains('/') || name.contains('\\') || name.contains('\0') {
            return Err(StorageError::InvalidPresetName(
                "Name contains invalid characters".to_string(),
            ));
        }

        // Check for reserved names
        if name == "." || name == ".." {
            return Err(StorageError::InvalidPresetName("Invalid preset name".to_string()));
        }

        Ok(())
    }

    /// List all presets
    pub fn list(&self) -> Result<Vec<PresetInfo>, StorageError> {
        let mut presets = Vec::new();

        for entry in std::fs::read_dir(&self.preset_dir).map_err(StorageError::Io)? {
            let entry = entry.map_err(StorageError::Io)?;
            let path = entry.path();

            if path.extension().map_or(false, |ext| ext == "json") {
                if let Ok(content) = std::fs::read_to_string(&path) {
                    if let Ok(preset) = serde_json::from_str::<Preset>(&content) {
                        presets.push(PresetInfo {
                            name: preset.name,
                            preset_type: preset.preset_type,
                            description: preset.description,
                            created_at: preset.created_at,
                            updated_at: preset.updated_at,
                        });
                    }
                }
            }
        }

        // Sort by name
        presets.sort_by(|a, b| a.name.cmp(&b.name));

        Ok(presets)
    }

    /// Get a preset by name
    pub fn get(&self, name: &str) -> Result<Preset, StorageError> {
        Self::validate_name(name)?;

        let path = self.preset_path(name);
        if !path.exists() {
            return Err(StorageError::PresetNotFound(name.to_string()));
        }

        let content = std::fs::read_to_string(&path).map_err(StorageError::Io)?;
        let preset: Preset = serde_json::from_str(&content).map_err(StorageError::Serialization)?;

        Ok(preset)
    }

    /// Save a preset
    pub fn save(&self, preset: &Preset) -> Result<(), StorageError> {
        Self::validate_name(&preset.name)?;

        let path = self.preset_path(&preset.name);
        let content = serde_json::to_string_pretty(preset).map_err(StorageError::Serialization)?;

        std::fs::write(&path, content).map_err(StorageError::Io)?;

        Ok(())
    }

    /// Delete a preset
    pub fn delete(&self, name: &str) -> Result<(), StorageError> {
        Self::validate_name(name)?;

        let path = self.preset_path(name);
        if !path.exists() {
            return Err(StorageError::PresetNotFound(name.to_string()));
        }

        std::fs::remove_file(&path).map_err(StorageError::Io)?;

        Ok(())
    }

    /// Check if a preset exists
    pub fn exists(&self, name: &str) -> bool {
        Self::validate_name(name).is_ok() && self.preset_path(name).exists()
    }
}

impl Default for PresetStorage {
    fn default() -> Self {
        Self::new().expect("Failed to initialize preset storage")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_validate_name() {
        assert!(PresetStorage::validate_name("valid-name").is_ok());
        assert!(PresetStorage::validate_name("my_preset_1").is_ok());
        assert!(PresetStorage::validate_name("").is_err());
        assert!(PresetStorage::validate_name(".").is_err());
        assert!(PresetStorage::validate_name("..").is_err());
        assert!(PresetStorage::validate_name("path/to/file").is_err());
    }
}
