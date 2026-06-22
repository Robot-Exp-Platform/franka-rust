use anyhow::Result;
use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{behavior::*, controller::joint_impedance_session};
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
    let values: Vec<serde_json::Value> = serde_json::from_reader(file)?;
    let traj: Vec<[f64; 7]> = values
        .into_iter()
        .filter_map(|value| {
            let joint = value.get("Joint")?.as_array()?;
            if joint.len() != 7 {
                return None;
            }
            let mut out = [0.0; 7];
            for (idx, item) in joint.iter().enumerate() {
                out[idx] = item.as_f64()?;
            }
            Some(out)
        })
        .collect();

    robot.move_to::<JointSpace<7>>(traj[0])?;

    let (controller, handle) = joint_impedance_session::<7>(
        Some(traj[0]),
        [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    );
    robot.control_with::<TorqueControl<7>, _>(controller)?;

    for joint in traj {
        handle.set_target(Some(joint));
        sleep(Duration::from_millis(1));
    }

    handle.finish();
    robot.waiting_for_finish()?;

    Ok(())
}
