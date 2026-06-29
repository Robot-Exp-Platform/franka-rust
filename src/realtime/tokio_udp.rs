use robot_behavior::{RobotException, RobotResult};
use std::time::Duration;
use tokio::{net::UdpSocket, runtime::Builder};

use crate::{
    robot_impl::FrankaRobotImpl,
    types::{
        robot_command::RobotCommand,
        robot_state::RobotStateInter,
        robot_types::{CommandIDConfig, MoveData},
    },
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
    let mut previous_motion_time: Option<Duration> = None;
    let mut finish_command: Option<RobotCommand> = None;

    loop {
        let last_id = robot.robot_state.read().unwrap().command_id();
        let mut latest: Option<(RobotStateInter, std::net::SocketAddr)> = None;

        loop {
            match socket.try_recv_from(&mut buffer) {
                Ok((size, addr)) => {
                    let candidate: RobotStateInter = bincode::deserialize(&buffer[..size])
                        .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
                    candidate.error_result()?;
                    if candidate.command_id() > last_id
                        && latest.as_ref().map_or(true, |(state, _)| {
                            candidate.command_id() > state.command_id()
                        })
                    {
                        latest = Some((candidate, addr));
                    }
                }
                Err(err) if err.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(err) => return Err(err.into()),
            }
        }

        while latest.is_none() {
            let (size, addr) = socket.recv_from(&mut buffer).await?;
            let candidate: RobotStateInter = bincode::deserialize(&buffer[..size])
                .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
            candidate.error_result()?;
            if candidate.command_id() > last_id {
                latest = Some((candidate, addr));
            }
        }

        let (state, latest_addr) = latest.unwrap();
        {
            let mut latest = robot.robot_state.write().unwrap();
            *latest = state;
        }

        if let Some(mut command) = finish_command {
            if !FrankaRobotImpl::motion_started(&state, &mode) {
                return robot.finish_current_motion();
            }
            command.set_command_id(state.command_id());
            let data = bincode::serialize(&command)
                .map_err(|err| RobotException::CommandException(err.to_string()))?;
            socket.send_to(&data, latest_addr).await?;
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

        let motion_time = state
            .time()
            .unwrap_or_else(|| Duration::from_millis(state.command_id()));
        let period = previous_motion_time
            .replace(motion_time)
            .map_or(Duration::ZERO, |previous| {
                motion_time.saturating_sub(previous)
            });

        let next = FrankaRobotImpl::prepare_command(&state, command(state, period).await, &mode);
        let done = next.motion.motion_generation_finished;
        let data = bincode::serialize(&next)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;
        socket.send_to(&data, latest_addr).await?;

        if done {
            finish_command = Some(next);
        }
    }
}
