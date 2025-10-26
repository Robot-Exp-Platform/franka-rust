use std::fmt::Display;

use serde::{Deserialize, Serialize};

use super::robot_types::CommandIDConfig;

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
#[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct GripperStateInter {
    pub message_id: u64,
    pub width: f64,
    pub max_width: f64,
    pub is_grasped: bool,
    pub temperature: u16,
}

impl From<GripperStateInter> for GripperState {
    fn from(state: GripperStateInter) -> Self {
        GripperState {
            width: state.width,
            max_width: state.max_width,
            is_grasped: state.is_grasped,
            temperature: state.temperature,
        }
    }
}

impl CommandIDConfig<u64> for GripperStateInter {
    fn set_command_id(&mut self, id: u64) {
        self.message_id = id;
    }
    fn command_id(&self) -> u64 {
        self.message_id
    }
}

impl Display for GripperStateInter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let gripper_state: GripperState = (*self).into();
        let message_id = self.message_id;
        write!(
            f,
            r"gripper state:
    | message_id: {message_id},
    | width: {}, max_width: {}, is_grasped: {}, temperature: {}",
            gripper_state.width,
            gripper_state.max_width,
            gripper_state.is_grasped,
            gripper_state.temperature
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn display_gripper_state() {
        let gripper_state = GripperStateInter {
            message_id: 1,
            width: 0.5,
            max_width: 0.6,
            is_grasped: true,
            temperature: 30,
        };
        println!("{gripper_state}");
    }
}
