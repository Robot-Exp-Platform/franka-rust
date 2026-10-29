use franka_rust::FrankaEmika;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    eprintln!("[02_01] stage=initial_default");
    robot
        .with_scale(0.25)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    robot.set_coord(Coord::Relative)?;
    eprintln!("[02_01] stage=relative_offset");

    robot
        .with_scale(0.25)
        .move_to::<JointSpace<7>>([0.0, 0.0, 0.0, 0.06, 0.0, -0.06, 0.0])?;

    robot.set_coord(Coord::OCS)?;
    eprintln!("[02_01] stage=return_default");
    robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;
    Ok(())
}
