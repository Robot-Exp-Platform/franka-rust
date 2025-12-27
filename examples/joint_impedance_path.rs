use anyhow::Result;
use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{MotionType, RobotResult, behavior::*};
use std::{fs::File, thread::sleep, time::Duration};

fn main() -> Result<()> {
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

    let file = File::open("./examples/full_throw_trajectory.json")?;
    let traj: Vec<MotionType<7>> = serde_json::from_reader(file)?;

    robot.move_to(traj[0])?;

    let handle = robot.joint_impedance_async(
        &[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        &[50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    )?;

    for point in traj.iter() {
        if let MotionType::Joint(joint) = point {
            handle.set_target(Some(*joint));
            sleep(Duration::from_millis(1));
        }
    }

    handle.finish();

    Ok(())
}
