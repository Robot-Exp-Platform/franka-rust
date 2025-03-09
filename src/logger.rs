/// Command sent to the robot. Structure used only for logging purposes.
use crate::types::control_types::*;

#[derive(Debug, Clone)]
pub struct RobotCommandLog {
    /// sent joint positions.
    pub joint_positions: JointPositions,

    /// sent joint velocities.
    pub joint_velocities: JointVelocities,

    /// sent cartesian positions.
    pub cartesian_pose: CartesianPose,

    /// sent cartesian velocities.
    pub cartesian_velocities: CartesianVelocities,

    /// sent torques.
    pub torques: Torques,
}
