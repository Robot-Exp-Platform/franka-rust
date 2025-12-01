use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{ArmParam, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
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
        .with_scale(0.3)
        .move_joint(&FrankaEmika::JOINT_DEFAULT)?;

    Ok(())
}
