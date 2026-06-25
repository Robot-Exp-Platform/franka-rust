use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    let path = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/examples/safe_joint_trajectory.json"
    );

    robot.set_default_behavior()?;
    robot
        .with_scale(0.3)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;
    robot.move_traj_from_file::<JointSpace<7>>(path)
}
