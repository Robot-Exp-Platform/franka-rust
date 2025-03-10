use serde::{Deserialize, Serialize};

use super::robot_types::CommandIDConfig;

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct GripperStateInter {
    pub message_id: u32,
    pub width: f64,
    pub max_width: f64,
    pub is_grasped: bool,
    pub temperature: u16,
}

#[derive(Debug, Clone)]
pub struct GripperState {
    /// Current gripper opening width. Unit: \[m\].
    pub width: f64,

    /// Maximum gripper opening width.
    /// This parameter is estimated by homing the gripper.
    /// After changing the gripper fingers, a homing needs to be done. Unit: \[m\].
    pub max_width: f64,

    /// Indicates whether an object is currently grasped.
    pub is_grasped: bool,

    /// Current gripper temperature. Unit: [°C].
    pub temperature: u16,
}

impl Into<GripperState> for GripperStateInter {
    fn into(self) -> GripperState {
        GripperState {
            width: self.width,
            max_width: self.max_width,
            is_grasped: self.is_grasped,
            temperature: self.temperature,
        }
    }
}

impl CommandIDConfig for GripperStateInter {
    fn set_command_id(&mut self, id: u32) {
        self.message_id = id;
    }
    fn command_id(&self) -> u32 {
        self.message_id
    }
}
