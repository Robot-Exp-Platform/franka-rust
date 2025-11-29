use std::time::Duration;

use franka_rust::FrankaGripper;
use robot_behavior::RobotResult;

fn main() -> RobotResult<()> {
    let mut gripper = FrankaGripper::new("172.16.0.3");

    gripper.homing().unwrap();

    println!("Homing done, moving gripper to 0.05m width");

    // gripper.move_gripper(0.05, 0.1)?;

    println!("Gripper moved, grasping object");

    gripper.grasp(0.05, 0.1, 1.0).unwrap();

    std::thread::sleep(Duration::from_secs(5));

    println!("Object grasped, try read state");

    let state = gripper.read_state().unwrap();
    println!("Gripper state: {state:?}");
    Ok(())
}
