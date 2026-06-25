use franka_rust::{FrankaEmika, model::Frame};
use robot_behavior::{RobotResult, behavior::*};
use strum::IntoEnumIterator;

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    let state = robot.read_state()?;
    let model = robot.model()?;

    for frame in Frame::iter() {
        println!("{frame}: {:?}", model.pose_from_state(&frame, &state));
    }
    Ok(())
}
