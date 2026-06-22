use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*, controller::joint_impedance_session};

#[tokio::main]
async fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    let (controller, handle) = joint_impedance_session::<7>(
        None,
        [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    );
    robot.control_with::<TorqueControl<7>, _>(controller)?;

    tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
    handle.finish();

    robot.waiting_for_finish()
}
