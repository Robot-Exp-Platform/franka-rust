use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{RobotResult, behavior::*};
use std::time::Duration;

#[tokio::main]
async fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    robot.set_default_behavior()?;
    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [10.0; 7],
        upper_torque_thresholds_acceleration: [100.0; 7],
        lower_torque_thresholds_nominal: [10.0; 7],
        upper_torque_thresholds_nominal: [100.0; 7],
        lower_force_thresholds_acceleration: [10.0; 6],
        upper_force_thresholds_acceleration: [100.0; 6],
        lower_force_thresholds_nominal: [10.0; 6],
        upper_force_thresholds_nominal: [100.0; 6],
    })?;

    let handle = robot.joint_impedance_async(
        &[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        &[50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    )?;

    tokio::time::sleep(Duration::from_secs(10)).await;
    handle.finish();

    Ok(())
}
