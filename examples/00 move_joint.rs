use franka_rust::FrankaEmika;
use robot_behavior::{ArmParam, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.move_joint(&FrankaEmika::JOINT_DEFAULT)?;
    robot.move_joint(&FrankaEmika::JOINT_PACKED)?;

    Ok(())
}
