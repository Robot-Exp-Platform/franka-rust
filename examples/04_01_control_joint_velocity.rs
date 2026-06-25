use std::{f64::consts::PI, time::Duration};

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    let mut time = Duration::ZERO;
    robot.control_with::<JointVelocityControl<7>, _>(move |_, dt| {
        time += dt;

        let t = time.as_secs_f64();
        let cycle = (-1.0f64).powf(t.floor());
        let omega = cycle * 0.5 * (1.0 - f64::cos(2.0 * PI * t));

        (
            [0.0, 0.0, 0.0, omega, omega, omega, omega],
            time > Duration::from_secs(2),
        )
    })
}
