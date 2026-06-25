use std::time::Duration;

use franka_rust::FrankaGripper;
use robot_behavior::RobotResult;

fn main() -> RobotResult<()> {
    let mut gripper = FrankaGripper::new("172.16.0.3");

    gripper.homing()?;
    gripper.move_gripper(0.05, 0.1)?;

    let state = gripper.read_state()?;
    if state.max_width < 0.05 {
        println!("object is wider than current gripper limit");
        return Ok(());
    }

    gripper.grasp(0.05, 0.1, 60.0)?;
    std::thread::sleep(Duration::from_secs(5));
    println!("gripper state: {:?}", gripper.read_state()?);
    gripper.stop()?;
    Ok(())
}
