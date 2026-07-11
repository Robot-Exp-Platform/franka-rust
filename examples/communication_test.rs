use std::{thread::sleep, time::Duration};

use franka_rust::FrankaEmika;
use robot_behavior::RobotResult;

fn main() -> RobotResult<()> {
    let ip = std::env::var("ROBOT_IP").unwrap_or_else(|_| "172.16.0.3".to_string());
    println!("connecting to Franka at {ip}");

    let mut robot = FrankaEmika::new(&ip);
    let mut state = robot.read_franka_state()?;
    for _ in 0..300 {
        if state.duration > Duration::ZERO {
            break;
        }
        sleep(Duration::from_millis(10));
        state = robot.read_franka_state()?;
    }
    let xyz = [
        state.pose_o_to_ee[12],
        state.pose_o_to_ee[13],
        state.pose_o_to_ee[14],
    ];

    println!("connected");
    println!("robot_mode: {:?}", state.robot_mode);
    println!("current_errors: {:?}", state.current_errors);
    println!("robot_time_ms: {}", state.duration.as_millis());
    println!("q: {:.6?}", state.q);
    println!("ee_xyz_m: {:.6?}", xyz);
    println!(
        "control_command_success_rate: {:.3}",
        state.control_command_success_rate
    );

    Ok(())
}
