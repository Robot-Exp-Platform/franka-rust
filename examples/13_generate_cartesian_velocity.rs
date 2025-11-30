use std::{thread::sleep, time::Duration};

use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{MotionType, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0],
        upper_torque_thresholds_acceleration: [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0],
        lower_torque_thresholds_nominal: [25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.],
        upper_torque_thresholds_nominal: [35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0],
        lower_force_thresholds_acceleration: [30.0, 30.0, 30.0, 25.0, 25.0, 25.0],
        upper_force_thresholds_acceleration: [40.0, 40.0, 40.0, 35.0, 35.0, 35.0],
        lower_force_thresholds_nominal: [30.0, 30.0, 30.0, 25.0, 25.0, 25.0],
        upper_force_thresholds_nominal: [40.0, 40.0, 40.0, 35.0, 35.0, 35.0],
    })?;

    let mut time = Duration::ZERO;
    robot.move_with_closure(move |_, dt| {
        time += dt;

        let time_ = time.as_secs_f64();
        let cycle = (-1.0f64).powf((time_ - time_.rem_euclid(4.0)) / 4.0);
        let velocity =
            cycle * 0.1 / 2.0 * (1.0 - f64::cos(2.0 * std::f64::consts::PI / 4.0 * time_));
        let velocity_x = f64::cos(std::f64::consts::FRAC_PI_4) * velocity;
        let velocity_z = -f64::sin(std::f64::consts::FRAC_PI_4) * velocity;

        let output = [velocity_x, 0.0, velocity_z, 0.0, 0.0, 0.0];

        let is_finished = time >= Duration::from_secs(8);
        (MotionType::CartesianVel(output), is_finished)
    })?;

    sleep(Duration::from_secs(6));

    Ok(())
}
