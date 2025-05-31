use robot_behavior::{RobotResult, behavior::*};

use franka_rust::FrankaRobot;

fn main() -> RobotResult<()> {
    let mut robot = FrankaRobot::new("172.16.0.3");

    robot.move_path_from_file("path/to/move/file.json")?;

    Ok(())
}
