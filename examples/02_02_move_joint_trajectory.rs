use std::f64::consts::PI;

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot
        .with_scale(0.3)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    let start = FrankaEmika::JOINT_DEFAULT;
    let traj = (0..1500)
        .map(|i| {
            let s = i as f64 / 1499.0;
            let mut q = start;
            q[3] += 0.08 * (1.0 - f64::cos(2.0 * PI * s));
            q[5] -= 0.08 * (1.0 - f64::cos(2.0 * PI * s));
            q
        })
        .collect();

    robot.move_traj::<JointSpace<7>>(traj)
}
