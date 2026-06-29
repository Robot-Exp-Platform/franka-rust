use std::f64::consts::PI;

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot
        .with_scale(0.5)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    println!("Finished moving to initial joint configuration.");

    let mut time = 0.0;
    let omega_max = 1.0;
    let time_max = 1.0;
    robot.control_with::<JointVelocityControl<7>, _>(move |_, time_step| {
        time += time_step.as_secs_f64();

        let cycle = (-1.0f64).powf((time / time_max).floor());
        let omega = cycle * omega_max / 2.0 * (1.0 - f64::cos(2.0 * PI / time_max * time));
        let out = [0.0, 0.0, 0.0, omega, omega, omega, omega];

        if time >= 2.0 * time_max {
            println!("Finished motion, shutting down example");
            return (out, true);
        }
        (out, false)
    })
}
