use std::{f64::consts::PI, time::Duration};

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    let mut time = Duration::ZERO;
    let mut initial = None;
    robot.control_with::<JointPositionControl<7>, _>(move |state, dt| {
        let initial = *initial.get_or_insert_with(|| state.meas.q.unwrap_or([0.0; 7]));
        time += dt;

        let delta = PI / 8.0 * (1.0 - f64::cos(PI / 2.5 * time.as_secs_f64()));
        let mut command = initial;
        command[3] += delta;
        command[4] += delta;
        command[6] += delta;

        (command, time > Duration::from_secs(5))
    })
}
