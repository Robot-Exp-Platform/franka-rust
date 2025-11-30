use std::{f64::consts::PI, thread::sleep, time::Duration};

use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{MotionType, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        upper_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        upper_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    })?;

    let mut time = Duration::ZERO;
    let mut initial_position = [0.; 7];
    robot.move_with_closure(move |state, dt| {
        if time == Duration::ZERO {
            initial_position = state.joint.unwrap();
        }
        time += dt;

        let delta_angle = PI / 8.0 * (1.0 - f64::cos(PI / 2.5 * time.as_secs_f64()));
        let output = [
            initial_position[0],
            initial_position[1],
            initial_position[2],
            initial_position[3] + delta_angle,
            initial_position[4] + delta_angle,
            initial_position[5],
            initial_position[6] + delta_angle,
        ];

        let is_finished = time > Duration::from_secs(5);

        (MotionType::Joint(output), is_finished)
    })?;

    sleep(Duration::from_secs(6));

    Ok(())
}
