use franka_rust::FrankaEmika;
use roplat_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    let limits = robot.motion_limits().scaled(0.3);
    robot.with_limits(limits, |robot| {
        robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)
    })?;

    let mut mid = FrankaEmika::JOINT_DEFAULT;
    mid[3] += 0.10;
    mid[5] -= 0.10;

    robot.move_waypoints::<JointSpace<7>>(vec![
        FrankaEmika::JOINT_DEFAULT,
        mid,
        FrankaEmika::JOINT_DEFAULT,
    ])
}
