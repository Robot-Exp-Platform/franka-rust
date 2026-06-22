use std::time::Duration;

use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{RobotResult, behavior::*, controller::cartesian_impedance_session};

#[tokio::main]
async fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    robot.set_default_behavior()?;
    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [100.0; 7],
        upper_torque_thresholds_acceleration: [100.0; 7],
        lower_torque_thresholds_nominal: [100.0; 7],
        upper_torque_thresholds_nominal: [100.0; 7],
        lower_force_thresholds_acceleration: [100.0; 6],
        upper_force_thresholds_acceleration: [100.0; 6],
        lower_force_thresholds_nominal: [100.0; 6],
        upper_force_thresholds_nominal: [100.0; 6],
    })?;

    let model = robot.model()?;
    let (controller, handle) = cartesian_impedance_session::<_, 7>(
        model,
        None,
        [150.0, 150.0, 150.0, 10.0, 10.0, 10.0],
        [
            2.0 * 150.0_f64.sqrt(),
            2.0 * 150.0_f64.sqrt(),
            2.0 * 150.0_f64.sqrt(),
            2.0 * 10.0_f64.sqrt(),
            2.0 * 10.0_f64.sqrt(),
            2.0 * 10.0_f64.sqrt(),
        ],
    );
    robot.control_with::<ArmTorqueControl<7>, _>(controller)?;

    tokio::time::sleep(Duration::from_secs(10)).await;
    handle.finish();
    robot.waiting_for_finish()?;

    Ok(())
}
