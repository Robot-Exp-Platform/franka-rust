use serde::{Deserialize, Serialize};
use std::fmt::Display;

use super::{
    gripper_state::GripperStateInter,
    robot_types::{CommandFilter, CommandIDConfig},
};

#[derive(Debug, Default, Serialize, Deserialize, Clone, Copy)]
#[repr(C, packed)]
pub struct GripperCommand {}

impl Display for GripperCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "gripper command {{}}")
    }
}

impl CommandIDConfig<u64> for GripperCommand {
    fn set_command_id(&mut self, _id: u64) {}
    fn command_id(&self) -> u64 {
        0
    }
}

impl CommandFilter<GripperStateInter> for GripperCommand {
    fn filter(self, _: &GripperStateInter) -> Self {
        GripperCommand {}
    }
}
