use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot
        .with_scale(0.25)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    robot.set_coord(Coord::Relative)?;
    robot.move_to::<JointSpace<7>>([0.0, 0.0, 0.0, 0.06, 0.0, -0.06, 0.0])?;

    robot.set_coord(Coord::OCS)?;
    robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)
}
