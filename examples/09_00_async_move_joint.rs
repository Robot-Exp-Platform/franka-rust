use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_io()
        .build()?;

    robot.set_default_behavior()?;
    runtime.block_on(async {
        robot
            .move_to_async::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)
            .await
    })
}
