use robot_behavior::{RobotException, RobotResult};
use std::time::Duration;
use tokio::{net::UdpSocket, runtime::Builder};

use crate::{
    robot_impl::FrankaRobotImpl,
    types::{robot_command::RobotCommand, robot_state::RobotStateInter, robot_types::MoveData},
};

/// Async FCI realtime session whose per-cycle controller is async.
///
/// The session reuses the robot's single UDP endpoint by cloning the socket
/// handle into Tokio. No second local port is created and the robot is not
/// reconnected to a competing UDP receiver.
pub(crate) async fn control_async<F>(
    robot: &mut FrankaRobotImpl,
    mode: MoveData,
    mut command: F,
) -> RobotResult<()>
where
    F: async FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    let socket = robot.udp_socket.try_clone()?;
    socket.set_nonblocking(true)?;
    let socket = UdpSocket::from_std(socket)?;

    robot._move(mode)?;
    let session_result = run_udp_loop(robot, &socket, mode, async |state, duration| {
        command(state, duration).await
    })
    .await;

    if session_result.is_err() {
        let _ = robot._stop_move(());
    }

    session_result
}

pub(crate) fn block_on_control_async<F>(
    robot: &mut FrankaRobotImpl,
    mode: MoveData,
    command: F,
) -> RobotResult<()>
where
    F: async FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    let runtime = Builder::new_current_thread().enable_io().build()?;
    runtime.block_on(control_async(robot, mode, command))
}

pub(crate) async fn recv_state(robot: &mut FrankaRobotImpl) -> RobotResult<RobotStateInter> {
    let socket = robot.udp_socket.try_clone()?;
    socket.set_nonblocking(true)?;
    let socket = UdpSocket::from_std(socket)?;

    let mut buffer = vec![0_u8; std::mem::size_of::<RobotStateInter>() * 5];
    let (size, _) = socket.recv_from(&mut buffer).await?;
    let state: RobotStateInter = bincode::deserialize(&buffer[..size])
        .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
    state.error_result()?;

    {
        let mut latest = robot.robot_state.write().unwrap();
        *latest = state;
    }

    Ok(state)
}

async fn run_udp_loop<F>(
    robot: &mut FrankaRobotImpl,
    socket: &UdpSocket,
    mode: MoveData,
    mut command: F,
) -> RobotResult<()>
where
    F: async FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    let mut buffer = vec![0_u8; std::mem::size_of::<RobotStateInter>() * 5];
    let mut started = false;

    loop {
        let (size, addr) = socket.recv_from(&mut buffer).await?;
        let state: RobotStateInter = bincode::deserialize(&buffer[..size])
            .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
        state.error_result()?;

        {
            let mut latest = robot.robot_state.write().unwrap();
            *latest = state;
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
            command(state, Duration::from_millis(1)).await,
        );
        let done = next.motion.motion_generation_finished;
        let data = bincode::serialize(&next)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;
        socket.send_to(&data, addr).await?;

        if done {
            return robot.finish_current_motion();
        }
    }
}
