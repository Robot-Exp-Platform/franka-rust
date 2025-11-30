use std::f64::consts::PI;

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

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
    });

    let mut time = Duration::ZERO;
    robot.move_with_closure(move |state, dt| {
        time += dt;

        let time_ = time.as_secs_f64();
        let cycle = (-1.0f64).powf((time_ - time_.rem_euclid(1.0)) / 1.0);
        let omega = cycle * 1.0 / 2.0 * (1.0 - f64::cos(2.0 * PI / 1.0 * time_));

        let output = [0.0, 0.0, 0.0, omega, omega, omega, omega];

        let is_finished = time > Duration::from_secs(2);

        (MotionType::JointVelocity(output), is_finished)
    });

    Ok(())
}
