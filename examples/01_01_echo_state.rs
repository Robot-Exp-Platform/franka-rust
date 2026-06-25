use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    for _ in 0..100 {
        let state = robot.read_state()?;
        println!(
            "q={:?}, dq={:?}, success_rate={:.3}, mode={:?}",
            state.q, state.dq, state.control_command_success_rate, state.robot_mode
        );
    }
    Ok(())
}
