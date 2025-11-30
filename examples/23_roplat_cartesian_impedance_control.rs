use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

#[tokio::main]
async fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    let (handle, mut closure) = robot.cartesian_impedance_control(
        (150.0, 10.0),
        (2.0 * 150.0_f64.sqrt(), 2.0 * 10.0_f64.sqrt()),
    )?;

    let handle = async move {
        tokio::time::sleep(tokio::time::Duration::from_secs(10)).await;
        handle.finish();
    };

    let (result, _) = tokio::join!(closure(), handle,);

    result
}
