use robot_behavior::{RobotException, RobotResult};
use std::{
    ops::{ControlFlow, Deref},
    sync::{Arc, RwLock},
    time::Duration,
};
use tokio::{net::UdpSocket, runtime::Builder};

use crate::{
    network::AsyncNetwork,
    robot_impl::FrankaRobotImpl,
    types::{
        robot_command::RobotCommand,
        robot_state::RobotStateInter,
        robot_types::{
            CommandIDConfig, MoveData, MoveRequest, MoveResponse, MoveStatus, StopMoveRequest,
            StopMoveResponse,
        },
    },
};

/// Async FCI realtime session whose per-cycle controller is async.
///
/// The session reuses the robot's single UDP endpoint by cloning the socket
/// handle into Tokio. No second local port is created and the robot is not
/// reconnected to a competing UDP receiver.
pub(crate) async fn control_flow_async<F>(
    robot: &mut FrankaRobotImpl,
    mode: MoveData,
    mut command: F,
) -> RobotResult<()>
where
    F: async FnMut(RobotStateInter, Duration) -> ControlFlow<(), RobotCommand>,
{
    let mut session = AsyncSession::start(robot, mode).await?;
    let result = async {
        while let Some((state, duration)) = session.next_state().await? {
            let ControlFlow::Continue(command) = command(state, duration).await else {
                return Ok(LoopEnd::Cancelled);
            };
            session.send_command(&state, command).await?;
        }
        Ok(LoopEnd::Finished)
    }
    .await;
    session.finish(result).await
}

// Existing trajectory helpers always produce a valid command. Keep this
// convenience path over the same canonical, flow-aware session loop.
pub(crate) async fn control_async<F>(
    robot: &mut FrankaRobotImpl,
    mode: MoveData,
    mut command: F,
) -> RobotResult<()>
where
    F: async FnMut(RobotStateInter, Duration) -> RobotCommand,
{
    control_flow_async(robot, mode, async |state, duration| {
        ControlFlow::Continue(command(state, duration).await)
    })
    .await
}

pub(crate) fn block_on_control_flow_async<F>(
    robot: &mut FrankaRobotImpl,
    mode: MoveData,
    command: F,
) -> RobotResult<()>
where
    F: async FnMut(RobotStateInter, Duration) -> ControlFlow<(), RobotCommand>,
{
    let runtime = Builder::new_current_thread().enable_all().build()?;
    runtime.block_on(control_flow_async(robot, mode, command))
}

pub(crate) async fn recv_state(robot: &mut FrankaRobotImpl) -> RobotResult<RobotStateInter> {
    let mut socket = AsyncUdp::new(&robot.udp_socket)?;

    let mut buffer = vec![0_u8; std::mem::size_of::<RobotStateInter>() * 5];
    let (size, _) = socket.recv_from(&mut buffer).await?;
    let state: RobotStateInter = bincode::deserialize(&buffer[..size])
        .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
    state.error_result()?;

    {
        let mut latest = robot.robot_state.write().unwrap();
        *latest = state;
    }

    socket.restore()?;
    Ok(state)
}

pub(crate) enum LoopEnd {
    Finished,
    Cancelled,
}

/// Shared protocol state for both the legacy async-callback facade and the
/// native Send callback. Only invocation of the callback differs between them.
/// Socket leases are exclusive and restore the owner's blocking mode on Drop.
pub(crate) struct AsyncSession<'a> {
    network: AsyncNetwork<'a>,
    socket: AsyncUdp<'a>,
    robot_state: &'a Arc<RwLock<RobotStateInter>>,
    motion_id: &'a mut Option<u32>,
    mode: MoveData,
    buffer: Vec<u8>,
    started: bool,
    previous_motion_time: Option<Duration>,
    finish_command: Option<RobotCommand>,
    latest_addr: Option<std::net::SocketAddr>,
}

impl<'a> AsyncSession<'a> {
    pub(crate) async fn start(robot: &'a mut FrankaRobotImpl, mode: MoveData) -> RobotResult<Self> {
        if robot.motion_command_id.is_some() {
            return Err(RobotException::CommandException(
                "previous control session was not confirmed finished; reconnect before starting a new session".into()));
        }
        let socket = AsyncUdp::new(&robot.udp_socket)?;
        let network = robot.network.async_session()?;
        let mut session = Self {
            network,
            socket,
            robot_state: &robot.robot_state,
            motion_id: &mut robot.motion_command_id,
            mode,
            buffer: vec![0; std::mem::size_of::<RobotStateInter>() * 5],
            started: false,
            previous_motion_time: None,
            finish_command: None,
            latest_addr: None,
        };
        let result = session.start_motion().await;
        if let Err(error) = result {
            session.restore(Err(error))?;
        }
        Ok(session)
    }

    async fn start_motion(&mut self) -> RobotResult<()> {
        let mut request = MoveRequest::from(self.mode);
        request.set_command_id(self.network.next_command_id());
        // Mark ownership before the handshake's first await. If the future is
        // dropped or the response is lost, a later session must not reuse it.
        *self.motion_id = Some(request.command_id());
        let response: MoveResponse = self.network.send_and_recv(&request).await?;
        if response.status != MoveStatus::MotionStarted {
            *self.motion_id = None;
        }
        super::check_started(response.status)
    }

    /// Obtain one fresh running state, or finish after the final command was
    /// acknowledged by an idle state. This method never invokes user code.
    pub(crate) async fn next_state(&mut self) -> RobotResult<Option<(RobotStateInter, Duration)>> {
        loop {
            let last_id = self.robot_state.read().unwrap().command_id();
            let mut latest: Option<(RobotStateInter, std::net::SocketAddr)> = None;
            loop {
                match self.socket.try_recv_from(&mut self.buffer) {
                    Ok((size, addr)) => {
                        let candidate: RobotStateInter = bincode::deserialize(&self.buffer[..size])
                            .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
                        candidate.error_result()?;
                        if candidate.command_id() > last_id
                            && latest.as_ref().is_none_or(|(state, _)| {
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
                let (size, addr) = self.socket.recv_from(&mut self.buffer).await?;
                let candidate: RobotStateInter = bincode::deserialize(&self.buffer[..size])
                    .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
                candidate.error_result()?;
                if candidate.command_id() > last_id {
                    latest = Some((candidate, addr));
                }
            }
            let (state, latest_addr) = latest.unwrap();
            *self.robot_state.write().unwrap() = state;
            self.latest_addr = Some(latest_addr);
            if let Some(mut command) = self.finish_command {
                if !FrankaRobotImpl::motion_started(&state, &self.mode) {
                    return Ok(None);
                }
                command.set_command_id(state.command_id());
                let data = bincode::serialize(&command)
                    .map_err(|err| RobotException::CommandException(err.to_string()))?;
                self.socket.send_to(&data, latest_addr).await?;
                self.finish_command = Some(command);
                continue;
            }
            if !self.started {
                if FrankaRobotImpl::motion_started(&state, &self.mode) {
                    self.started = true;
                } else {
                    continue;
                }
            }
            let motion_time = state
                .time()
                .unwrap_or_else(|| Duration::from_millis(state.command_id()));
            let period = self
                .previous_motion_time
                .replace(motion_time)
                .map_or(Duration::ZERO, |previous| {
                    motion_time.saturating_sub(previous)
                });
            return Ok(Some((state, period)));
        }
    }

    pub(crate) async fn send_command(
        &mut self,
        state: &RobotStateInter,
        next: RobotCommand,
    ) -> RobotResult<()> {
        let next = FrankaRobotImpl::prepare_command(state, next, &self.mode);
        let done = next.motion.motion_generation_finished;
        let data = bincode::serialize(&next)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;
        self.socket
            .send_to(
                &data,
                self.latest_addr.expect("next_state precedes send_command"),
            )
            .await?;
        if done {
            self.finish_command = Some(next);
        }
        Ok(())
    }

    pub(crate) async fn finish(mut self, result: RobotResult<LoopEnd>) -> RobotResult<()> {
        let result = match result {
            Ok(LoopEnd::Finished) => {
                match receive_motion_end(&mut self.network, self.motion_id).await {
                    Ok(status) => super::check_finished(status),
                    Err(error) => Err(error),
                }
            }
            Ok(LoopEnd::Cancelled) => {
                cancel_motion(
                    &mut self.network,
                    &self.socket,
                    self.robot_state,
                    self.motion_id,
                )
                .await
            }
            Err(primary) => {
                let cleanup = cancel_motion(
                    &mut self.network,
                    &self.socket,
                    self.robot_state,
                    self.motion_id,
                )
                .await;
                super::combine_results(Err(primary), cleanup)
            }
        };
        self.restore(result)
    }

    fn restore(&mut self, result: RobotResult<()>) -> RobotResult<()> {
        let result = super::combine_results(result, self.network.restore());
        super::combine_results(result, self.socket.restore())
    }
}

async fn receive_motion_end(
    network: &mut AsyncNetwork<'_>,
    motion_id: &mut Option<u32>,
) -> RobotResult<MoveStatus> {
    let id = motion_id.ok_or_else(|| {
        RobotException::CommandException("no active control session to finish".into())
    })?;
    let response: MoveResponse = network.receive(id).await?;
    *motion_id = None;
    Ok(response.status)
}

async fn cancel_motion(
    network: &mut AsyncNetwork<'_>,
    socket: &UdpSocket,
    robot_state: &Arc<RwLock<RobotStateInter>>,
    motion_id: &mut Option<u32>,
) -> RobotResult<()> {
    // One deadline covers the Stop write, its ACK, idle UDP, and terminal Move.
    // No per-read reset can extend this network bound via fragmented traffic.
    let deadline = tokio::time::Instant::now() + super::CLEANUP_TIMEOUT;
    tokio::time::timeout_at(deadline, async {
        let mut request = StopMoveRequest::from(());
        request.set_command_id(network.next_command_id());
        let response: StopMoveResponse = network.send_and_recv(&request).await?;
        let stop: RobotResult<()> = response.status.into();
        stop?;
        let mut buffer = vec![0; std::mem::size_of::<RobotStateInter>() * 5];
        loop {
            if tokio::time::Instant::now() >= deadline {
                return Err(super::cleanup_timeout());
            }
            let (size, _) = socket.recv_from(&mut buffer).await?;
            let state: RobotStateInter = bincode::deserialize(&buffer[..size])
                .map_err(|error| RobotException::DeserializeError(error.to_string()))?;
            state.error_result()?;
            let mut latest = robot_state.write().unwrap();
            if state.command_id() <= latest.command_id() {
                continue;
            }
            *latest = state;
            if !super::motion_running(&state) {
                break;
            }
        }
        let status = receive_motion_end(network, motion_id).await?;
        super::check_cancelled(status)
    })
    .await
    .map_err(|_| super::cleanup_timeout())?
}

/// A single receiver for the existing UDP endpoint. Outside an async operation
/// the driver's owning std socket is blocking; mode changes affect all clones.
struct AsyncUdp<'a> {
    socket: UdpSocket,
    owner: &'a std::net::UdpSocket,
    restored: bool,
}
impl<'a> AsyncUdp<'a> {
    fn new(owner: &'a std::net::UdpSocket) -> RobotResult<Self> {
        tokio::runtime::Handle::try_current()
            .map_err(|error| RobotException::NetworkError(error.to_string()))?;
        let cloned = owner.try_clone()?;
        cloned.set_nonblocking(true)?;
        match UdpSocket::from_std(cloned) {
            Ok(socket) => Ok(Self { socket, owner, restored: false }),
            Err(error) => {
                owner.set_nonblocking(false)?;
                Err(error.into())
            }
        }
    }
    fn restore(&mut self) -> RobotResult<()> {
        if self.restored {
            return Ok(());
        }
        self.owner.set_nonblocking(false)?;
        self.restored = true;
        Ok(())
    }
}
impl Deref for AsyncUdp<'_> {
    type Target = UdpSocket;
    fn deref(&self) -> &Self::Target {
        &self.socket
    }
}
impl Drop for AsyncUdp<'_> {
    fn drop(&mut self) {
        let _ = self.restore();
    }
}
