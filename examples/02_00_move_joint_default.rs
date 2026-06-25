use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    println!("Moving to default joint position");
    robot
        .with_scale(0.3)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)
}
