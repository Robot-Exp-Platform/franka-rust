use std::time::Duration;

use franka_rust::FrankaGripper;
use roplat_behavior::RobotResult;

fn main() -> RobotResult<()> {
    let mut gripper = FrankaGripper::new("172.16.0.3");

    println!("homing: {}", gripper.homing()?);
    println!("move 50mm: {}", gripper.move_gripper(0.05, 0.1)?);

    let state = gripper.read_state()?;
    if state.max_width < 0.05 {
        println!("object is wider than current gripper limit");
        return Ok(());
    }

    println!("grasp 50mm: {}", gripper.grasp(0.05, 0.1, 60.0)?);
    std::thread::sleep(Duration::from_secs(5));
    println!("gripper state: {:?}", gripper.read_state()?);
    println!("stop: {}", gripper.stop()?);
    Ok(())
}
