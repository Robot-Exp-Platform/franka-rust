use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot.with_scale(0.2);
    robot.set_coord(Coord::Relative)?;
    robot.move_to::<FlangeSpace>(Pose::from([0.03, 0.0, 0.0]))?;
    robot.move_to::<FlangeSpace>(Pose::from([-0.03, 0.0, 0.0]))?;
    robot.set_coord(Coord::OCS)
}
