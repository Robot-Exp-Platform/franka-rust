use franka_rust::FrankaEmika;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    eprintln!("[02_01] stage=initial_default");
    let limits = robot.motion_limits().scaled(0.25);
    robot.with_limits(limits, |robot| {
        robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)
    })?;

    eprintln!("[02_01] stage=relative_offset");
    robot.with_limits(limits, |robot| {
        robot.move_to::<Relative<JointSpace<7>>>([0.0, 0.0, 0.0, 0.06, 0.0, -0.06, 0.0])
    })?;

    eprintln!("[02_01] stage=return_default");
    robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;
    Ok(())
}
