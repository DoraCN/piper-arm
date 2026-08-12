//! SDK software limits (joint angle limits and gripper range).

use std::collections::HashMap;

use crate::error::{Error, Result};

/// SDK software limits, mirroring `C_PiperParamManager`.
#[derive(Debug, Clone)]
pub struct PiperParamManager {
    /// Joint angle limits in radians: joint name -> (min, max).
    joint_limit: HashMap<String, (f64, f64)>,
    /// Gripper stroke range in meters: (min, max).
    gripper_range: (f64, f64),
}

impl Default for PiperParamManager {
    fn default() -> Self {
        // NOTE: the j2 limit uses 3.14 exactly as documented.
        #[allow(clippy::approx_constant)]
        let j2 = 3.14f64;
        Self {
            joint_limit: HashMap::from([
                ("j1".to_string(), (-2.6179, 2.6179)),
                ("j2".to_string(), (0.0, j2)),
                ("j3".to_string(), (-2.967, 0.0)),
                ("j4".to_string(), (-1.745, 1.745)),
                ("j5".to_string(), (-1.22, 1.22)),
                ("j6".to_string(), (-2.09439, 2.09439)),
            ]),
            gripper_range: (0.0, 0.07),
        }
    }
}

impl PiperParamManager {
    pub fn new() -> Self {
        Self::default()
    }

    /// Get the (min, max) joint angle limit in radians.
    pub fn get_joint_limit(&self, joint_name: &str) -> Result<(f64, f64)> {
        self.joint_limit
            .get(joint_name)
            .copied()
            .ok_or_else(|| {
                Error::ValueError(format!(
                    "joint_name '{joint_name}' not in [j1..j6]"
                ))
            })
    }

    /// Get the (min, max) gripper stroke range in meters.
    pub fn get_gripper_range(&self) -> (f64, f64) {
        self.gripper_range
    }

    /// Set a joint angle limit (radians). `min_val` must be <= `max_val`.
    pub fn set_joint_limit(&mut self, joint_name: &str, min_val: f64, max_val: f64) -> Result<()> {
        if !["j1", "j2", "j3", "j4", "j5", "j6"].contains(&joint_name) {
            return Err(Error::ValueError(format!(
                "joint_name '{joint_name}' not in [j1..j6]"
            )));
        }
        if max_val < min_val {
            return Err(Error::ValueError("max_val should be >= min_val".into()));
        }
        self.joint_limit.insert(joint_name.to_string(), (min_val, max_val));
        Ok(())
    }

    /// Set the gripper stroke range (meters).
    pub fn set_gripper_range(&mut self, min_val: f64, max_val: f64) -> Result<()> {
        if max_val < min_val {
            return Err(Error::ValueError("max_val should be >= min_val".into()));
        }
        self.gripper_range = (min_val, max_val);
        Ok(())
    }

    /// Reset all limits to the factory defaults.
    pub fn reset_default(&mut self) {
        *self = PiperParamManager::default();
    }
}
