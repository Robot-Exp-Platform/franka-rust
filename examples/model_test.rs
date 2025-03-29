use std::path::Path;

use franka_rust::{
    FrankaRobot,
    types::robot_state::{RobotState, RobotStateInter},
};
use robot_behavior::RobotResult;

fn main() -> RobotResult<()> {
    let mut robot = FrankaRobot::new("172.16.0.3");

    let path = Path::new("./model.so");
    let model = robot.model_from_path(path)?;
    let robot_state: RobotState = RobotStateInter::default().into();

    println!("mass from state: {:?}", model.mass_from_state(&robot_state));
    println!(
        "coriolis from state: {:?}",
        model.coriolis_from_state(&robot_state)
    );
    println!(
        "gravity from state: {:?}",
        model.gravity_from_state(&robot_state, &[0.0, 0.0, -9.81])
    );
    Ok(())
}
