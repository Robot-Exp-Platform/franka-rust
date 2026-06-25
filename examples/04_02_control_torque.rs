use std::time::Duration;

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_default_behavior()?;
    let mut time = Duration::ZERO;
    robot.control_with::<TorqueControl<7>, _>(move |state, dt| {
        time += dt;
        let tau = state
            .cmd
            .tau
            .or(state.des.tau)
            .or(state.meas.tau)
            .unwrap_or([0.0; 7]);
        (tau, time >= Duration::from_secs(2))
    })
}
