use std::{
    f64::consts::{FRAC_PI_4, PI},
    time::Duration,
};

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    let mut time = Duration::ZERO;
    let mut initial = None;
    robot.control_with::<CartesianPoseControl<7>, _>(move |state, dt| {
        let initial = *initial.get_or_insert_with(|| state.flange.meas.pose.unwrap_or_default());
        time += dt;

        let angle = FRAC_PI_4 * (1.0 - f64::cos(PI / 5.0 * time.as_secs_f64()));
        let mut pose = initial.homo();
        pose[12] += 0.15 * f64::sin(angle);
        pose[14] += 0.15 * (f64::cos(angle) - 1.0);

        (Pose::Homo(pose), time >= Duration::from_secs(10))
    })
}
