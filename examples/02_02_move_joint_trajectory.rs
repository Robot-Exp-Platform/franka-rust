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

    let mut initial_position = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
    let mut time = 0.0;
    robot.control_with::<JointPositionControl<7>, _>(move |state, time_step| {
        time += time_step.as_secs_f64();
        if time == 0.0 {
            initial_position = state.des.q.unwrap_or(initial_position);
        }

        let delta_angle = PI / 8.0 * (1.0 - f64::cos(PI / 2.5 * time));
        let mut out = initial_position;
        out[3] += delta_angle;
        out[4] += delta_angle;
        out[6] += delta_angle;

        if time >= 5.0 {
            println!("Finished motion, shutting down example");
            return (out, true);
        }
        (out, false)
    })
}
