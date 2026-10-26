use franka_rust::FrankaEmika;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    if robot.is_moving()? {
        robot.stop()?;
    }
    robot.reset()?;
    robot.shutdown()
}
