use crate::types::robot_state::{ControllerMode, MotionGeneratorMode};

#[derive(Default)]
pub enum ControlMode {
    #[default]
    Idle,
    Moving(MotionGeneratorMode, ControllerMode),
    Guiding,
    UserStopped,
}
