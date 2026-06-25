use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    println!("{}", FrankaEmika::version());
    robot.init()?;
    robot.enable()?;
    robot.shutdown()
}
