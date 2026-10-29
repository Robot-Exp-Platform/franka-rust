use franka_rust::FrankaEmika;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot.set_motion_limits(robot.motion_limits().scaled(0.2))?;
    robot.move_to::<Relative<FlangeSpace>>(Pose::from([0.03, 0.0, 0.0]))?;
    robot.move_to::<Relative<FlangeSpace>>(Pose::from([-0.03, 0.0, 0.0]))
}
