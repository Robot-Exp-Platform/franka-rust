use std::f64::consts::PI;

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*, controller::joint_traj_impedance_control};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    let start = FrankaEmika::JOINT_DEFAULT;
    let traj = (0..1500)
        .map(|i| {
            let s = i as f64 / 1499.0;
            let mut q = start;
            q[3] += 0.04 * f64::sin(2.0 * PI * s);
            q[5] -= 0.04 * f64::sin(2.0 * PI * s);
            q
        })
        .collect();

    robot.control_with::<TorqueControl<7>, _>(joint_traj_impedance_control(
        traj,
        [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    ))
}
