use std::time::Duration;

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*, controller::joint_impedance_control};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    let mut controller = joint_impedance_control(
        FrankaEmika::JOINT_DEFAULT,
        [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    );
    let mut elapsed = Duration::ZERO;
    robot.control_with::<TorqueControl<7>, _>(move |state, dt| {
        elapsed += dt;
        let (tau, _) = controller(state, dt);
        (tau, elapsed >= Duration::from_secs(5))
    })
}
