use robot_behavior::{RobotResult, behavior::*};

use franka_rust::FrankaEmika;

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.move_path_from_file("path/to/move/file.json")?;

    Ok(())
}
