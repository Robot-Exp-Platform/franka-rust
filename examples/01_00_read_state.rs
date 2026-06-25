use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    let state = robot.read_state()?;

    println!("q: {:?}", state.q);
    println!("dq: {:?}", state.dq);
    println!("mode: {:?}", state.robot_mode);
    println!(
        "control command success rate: {:.3}",
        state.control_command_success_rate
    );
    Ok(())
}
