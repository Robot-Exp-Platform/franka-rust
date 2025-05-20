use franka_rust::{
    FRANKA_ROBOT_DEFAULT_JOINT, FrankaRobot, types::robot_types::SetCollisionBehaviorData,
};
use robot_behavior::{ArmBehavior, ArmPreplannedMotion, MotionType, RobotResult};

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

    robot
        .with_speed(0.3)
        .move_to(MotionType::Joint(FRANKA_ROBOT_DEFAULT_JOINT))?;

    Ok(())
}
