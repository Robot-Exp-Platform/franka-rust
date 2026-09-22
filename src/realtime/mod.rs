//! Realtime UDP control sessions for Franka FCI.
//!
//! The TCP command plane starts and stops a motion. The UDP session owns the
//! per-cycle path: receive state, run the user controller directly, filter the
//! command against the same state, and send it back.

pub(crate) mod std_udp;
pub(crate) mod tokio_udp;

use crate::{
    robot_impl::FrankaRobotImpl,
    types::robot_types::{MoveData, MoveStatus},
};
use robot_behavior::{RobotException, RobotResult};
// A finished handshake has already consumed Move's terminal response, even
// when that response reports an error. Do not issue a second stop in that case.
enum SessionEnd {
    Finished(RobotResult<()>),
    Cancelled,
}

fn start_session(robot: &mut FrankaRobotImpl, mode: MoveData) -> RobotResult<()> {
    match robot._move(mode)? {
        MoveStatus::MotionStarted => Ok(()),
        status => Err(RobotException::CommandException(format!(
            "move did not start: {status:?}"
        ))),
    }
}

fn finish_session(
    robot: &mut FrankaRobotImpl,
    result: RobotResult<SessionEnd>,
    was_nonblocking: bool,
) -> RobotResult<()> {
    match result {
        Ok(SessionEnd::Finished(result)) => result,
        Ok(SessionEnd::Cancelled) => robot.cancel_current_motion(was_nonblocking),
        Err(primary) => match robot.cancel_current_motion(was_nonblocking) {
            Ok(()) => Err(primary),
            Err(cleanup) => Err(RobotException::ControlSession {
                primary: Box::new(primary),
                cleanup: Box::new(cleanup),
            }),
        },
    }
}

#[cfg(test)]
mod tests;
