#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use std::{thread::sleep, time::Duration};

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*, controller::cartesian_impedance_control};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    robot.set_default_behavior()?;
    robot.set_collision_behavior(100.0.into())?;
    robot.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.].into())?;
    robot.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.].into())?;

    let target = robot.get_endpoint();
    let model = robot.model()?;
    let controller = cartesian_impedance_control(
        model,
        target,
        [150.0, 150.0, 150.0, 10.0, 10.0, 10.0],
        [
            2.0 * 150.0_f64.sqrt(),
            2.0 * 150.0_f64.sqrt(),
            2.0 * 150.0_f64.sqrt(),
            2.0 * 10.0_f64.sqrt(),
            2.0 * 10.0_f64.sqrt(),
            2.0 * 10.0_f64.sqrt(),
        ],
    );

    robot.control_with::<ArmTorqueControl<7>, _>(controller)?;
    sleep(Duration::from_secs(25));
    robot.stop()
}
