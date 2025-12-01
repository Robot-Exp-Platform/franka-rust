use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

#[tokio::main]
async fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    let (handle, mut closure) = robot.joint_impedance_control(
        &[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0],
        &[50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0],
    )?;

    let handle = async move {
        tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
        handle.finish();
    };

    let (result, _) = tokio::join!(closure(), handle);

    result
}
