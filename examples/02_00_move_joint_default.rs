use franka_rust::FrankaEmika;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    println!("Moving to default joint position");
    let limits = robot.motion_limits().scaled(0.3);
    robot.with_limits(limits, |robot| {
        robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)
    })
}
