use std::{f64::consts::PI, time::Duration};

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    let mut time = Duration::ZERO;
    robot.control_with::<CartesianVelocityControl<7>, _>(move |_, dt| {
        time += dt;

        let t = time.as_secs_f64();
        let cycle = (-1.0f64).powf((t - t.rem_euclid(4.0)) / 4.0);
        let v = cycle * 0.05 * (1.0 - f64::cos(2.0 * PI / 4.0 * t));
        let command = [v, 0.0, -v, 0.0, 0.0, 0.0];

        (command, time >= Duration::from_secs(8))
    })
}
