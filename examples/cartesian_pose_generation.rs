use franka_rust::{FrankaRobot, types::robot_types::SetCollisionBehaviorData};
use nalgebra as na;
use robot_behavior::{ArmBehavior, MotionType, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = FrankaRobot::new("172.16.0.3");
    robot.set_default_behavior()?;

    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [20., 20., 18., 18., 16., 14., 12.],
        upper_torque_thresholds_acceleration: [20., 20., 18., 18., 16., 14., 12.],
        lower_torque_thresholds_nominal: [20., 20., 18., 18., 16., 14., 12.],
        upper_torque_thresholds_nominal: [20., 20., 18., 18., 16., 14., 12.],
        lower_force_thresholds_acceleration: [20., 20., 20., 25., 25., 25.],
        upper_force_thresholds_acceleration: [20., 20., 20., 25., 25., 25.],
        lower_force_thresholds_nominal: [20., 20., 20., 25., 25., 25.],
        upper_force_thresholds_nominal: [20., 20., 20., 25., 25., 25.],
    })?;

    robot.move_to(MotionType::CartesianQuat(na::Isometry3::identity()), 0.5)?;

    Ok(())
}
