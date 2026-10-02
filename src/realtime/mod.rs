//! Realtime UDP control sessions for Franka FCI.
//!
//! The TCP command plane starts and stops a motion. The UDP session owns the
//! per-cycle path: receive state, run the user controller directly, filter the
//! command against the same state, and send it back.

pub(crate) mod std_udp;
pub(crate) mod tokio_udp;

use crate::types::robot_state::{ControllerMode, MotionGeneratorMode, RobotStateInter};
use crate::{
    robot_impl::FrankaRobotImpl,
    types::robot_types::{MoveData, MoveStatus},
};
use robot_behavior::{RobotException, RobotResult};
use std::time::Duration;
// A finished handshake has already consumed Move's terminal response, even
// when that response reports an error. Do not issue a second stop in that case.
enum SessionEnd {
    Finished(RobotResult<()>),
    Cancelled,
}

fn start_session(robot: &mut FrankaRobotImpl, mode: MoveData) -> RobotResult<()> {
    check_started(robot._move(mode)?)
}

fn check_started(status: MoveStatus) -> RobotResult<()> {
    match status {
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

pub(crate) const CLEANUP_TIMEOUT: Duration = Duration::from_secs(3);

pub(crate) fn cleanup_timeout() -> RobotException {
    RobotException::NetworkError(
        "control-session cleanup deadline expired; physical stop is not confirmed".into(),
    )
}

pub(crate) fn check_finished(status: MoveStatus) -> RobotResult<()> {
    if status == MoveStatus::Success {
        Ok(())
    } else {
        Err(RobotException::CommandException(format!(
            "move failed with status: {status:?}"
        )))
    }
}

pub(crate) fn check_cancelled(status: MoveStatus) -> RobotResult<()> {
    match status {
        MoveStatus::Success | MoveStatus::Preempted => Ok(()),
        status => Err(RobotException::CommandException(format!(
            "cancelled move ended with status: {status:?}"
        ))),
    }
}

pub(crate) fn motion_running(state: &RobotStateInter) -> bool {
    (state.motion_generator_mode != MotionGeneratorMode::Idle
        && state.motion_generator_mode != MotionGeneratorMode::None)
        || state.controller_mode == ControllerMode::ExternalController
}

pub(crate) fn combine_results(
    result: RobotResult<()>,
    cleanup: RobotResult<()>,
) -> RobotResult<()> {
    match (result, cleanup) {
        (Ok(()), result) | (result, Ok(())) => result,
        (Err(primary), Err(cleanup)) => Err(RobotException::ControlSession {
            primary: Box::new(primary),
            cleanup: Box::new(cleanup),
        }),
    }
}

#[cfg(test)]
mod tests;
