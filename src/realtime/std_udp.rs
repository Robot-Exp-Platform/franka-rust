use robot_behavior::RobotResult;
use std::time::Duration;

use crate::{
    robot_impl::FrankaRobotImpl,
    types::{
        robot_command::RobotCommand,
        robot_state::RobotStateInter,
        robot_types::{CommandIDConfig, MoveData},
    },
};

/// Run one blocking FCI realtime session on the robot's single UDP socket.
///
/// No background forwarding thread is involved: the controller closure is
/// called directly after receiving each state packet, then its command is sent
/// back through the same socket.
pub(crate) fn control<F>(
    robot: &mut FrankaRobotImpl,
    mode: MoveData,
    mut command: F,
) -> RobotResult<()>
where
    F: FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    robot._move(mode)?;
    let mut started = false;
    let mut finish_command: Option<RobotCommand> = None;
    let mut buffer = vec![0_u8; std::mem::size_of::<RobotStateInter>() * 5];
    let session_result = loop {
        let (state, addr, _) = match robot.recv_state_into(&mut buffer) {
            Ok(result) => result,
            Err(err) => break Err(err),
        };

        if let Some(mut command) = finish_command {
            if !FrankaRobotImpl::motion_started(&state, &mode) {
                break robot.finish_current_motion();
            }
            command.set_command_id(state.command_id());
            if let Err(err) = robot.send_prepared_command(addr, command) {
                break Err(err);
            }
            finish_command = Some(command);
            continue;
        }

        if !started {
            if FrankaRobotImpl::motion_started(&state, &mode) {
                started = true;
            } else {
                continue;
            }
        }

        let next = FrankaRobotImpl::prepare_command(
            &state,
            command(state, Duration::from_millis(1)),
            &mode,
        );
        let done = next.motion.motion_generation_finished;
        if let Err(err) = robot.send_prepared_command(addr, next) {
            break Err(err);
        }
        if done {
            finish_command = Some(next);
        }
    };

    if session_result.is_err() {
        let _ = robot._stop_move(());
    }

    session_result
}
