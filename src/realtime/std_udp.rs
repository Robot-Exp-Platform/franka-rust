use robot_behavior::RobotResult;
use std::time::Duration;

use crate::{
    robot_impl::FrankaRobotImpl,
    types::{robot_command::RobotCommand, robot_state::RobotStateInter, robot_types::MoveData},
};

/// Run one blocking FCI realtime session on the robot's single UDP socket.
///
/// No background forwarding thread is involved: the controller closure is
/// called directly after receiving each state packet, then its command is sent
/// back through the same socket.
pub(crate) fn control<F>(robot: &mut FrankaRobotImpl, mode: MoveData, command: F) -> RobotResult<()>
where
    F: FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    run_udp_loop(robot, mode, command)
}

fn run_udp_loop<F>(robot: &mut FrankaRobotImpl, mode: MoveData, mut command: F) -> RobotResult<()>
where
    F: FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    robot._move(mode)?;
    let session_result = loop {
        let (state, addr, _) = match robot.recv_state() {
            Ok(result) => result,
            Err(err) => break Err(err),
        };

        let next = command(state, Duration::from_millis(1));
        let done = next.motion.motion_generation_finished;
        if let Err(err) = robot.send_command(&state, addr, next) {
            break Err(err);
        }
        if done {
            break robot.finish_current_motion();
        }
    };

    if session_result.is_err() {
        let _ = robot._stop_move(());
    }

    session_result
}
