use robot_behavior::{RobotException, RobotResult};
use std::{
    io::ErrorKind,
    net::{SocketAddr, UdpSocket},
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
};

use crate::{
    FRANKA_ROBOT_VERSION, PORT_ROBOT_COMMAND, PORT_ROBOT_UDP,
    network::Network,
    types::{robot_command::RobotCommand, robot_state::*, robot_types::*},
};

pub struct FrankaRobotImpl {
    pub(crate) network: Network,
    pub robot_state: Arc<RwLock<RobotStateInter>>,
    pub(crate) udp_socket: UdpSocket,
    pub(crate) motion_command_id: Option<u32>,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty; $ret_type:ty) => {
        pub(crate) fn $fn_name(&mut self, $arg_name: $arg_type) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> = self
                .network
                .tcp_send_and_recv(&mut Request::<$command, $arg_type>::from($arg_name))?;
            Ok(response.status)
        }
    };
}

impl FrankaRobotImpl {
    pub fn new(ip: &str) -> Self {
        let udp_socket = Self::bind_udp_socket(PORT_ROBOT_UDP).unwrap();
        let udp_port = udp_socket.local_addr().unwrap().port();
        let network = Network::new(ip, PORT_ROBOT_COMMAND);
        let mut robot = Self {
            network,
            robot_state: Arc::new(RwLock::new(RobotStateInter::default())),
            udp_socket,
            motion_command_id: None,
        };
        robot.connect_(udp_port).unwrap();
        robot
    }

    cmd_fn!(_connect, { Command::Connect }; data: ConnectData; ConnectStatus);
    pub(crate) fn _move(&mut self, mode: MoveData) -> RobotResult<MoveStatus> {
        if self.motion_command_id.is_some() {
            return Err(RobotException::CommandException(
                "previous control session was not confirmed finished; reconnect before starting a new session".into()));
        }
        let mut request = MoveRequest::from(mode);
        let response: RobotResult<MoveResponse> = self.network.tcp_send_and_recv(&mut request);
        // Keep the request id after an uncertain transport failure: issuing a
        // second motion cannot safely assume that the first one never started.
        self.motion_command_id = (request.command_id() != 0).then_some(request.command_id());
        match response {
            Ok(response) => {
                if response.status != MoveStatus::MotionStarted {
                    self.motion_command_id = None;
                }
                Ok(response.status)
            }
            Err(error) => Err(error),
        }
    }
    cmd_fn!(_set_collision_behavior, { Command::SetCollisionBehavior }; data: SetCollisionBehaviorData; GetterSetterStatus);
    cmd_fn!(_set_joint_impedance, { Command::SetJointImpedance }; data: SetJointImpedanceData; GetterSetterStatus);
    cmd_fn!(_set_cartesian_impedance, { Command::SetCartesianImpedance }; data: SetCartesianImpedanceData; GetterSetterStatus);
    cmd_fn!(_set_guiding_mode, { Command::SetGuidingMode }; data: SetGuidingModeData; GetterSetterStatus);
    cmd_fn!(_set_ee_to_k, { Command::SetEEToK }; data: SetEEToKData; GetterSetterStatus);
    cmd_fn!(_set_ne_to_ee, { Command::SetNEToEE }; data: SetNEToEEData; GetterSetterStatus);
    cmd_fn!(_set_load, { Command::SetLoad }; data: SetLoadData; GetterSetterStatus);
    cmd_fn!(_set_fliters, { Command::SetFilters }; data: SetFiltersData; GetterSetterStatus);
    cmd_fn!(_automatic_error_recovery, { Command::AutomaticErrorRecovery }; data: (); GetterSetterStatus);
    cmd_fn!(_stop_move, { Command::StopMove }; data: (); StopMoveStatus);
    cmd_fn!(_get_cartesian_limit, { Command::GetCartesianLimit }; data: GetCartesianLimitData; GetCartesianLimitStatus);

    fn bind_udp_socket(preferred_port: u16) -> RobotResult<UdpSocket> {
        let socket = UdpSocket::bind(("0.0.0.0", preferred_port))
            .or_else(|_| UdpSocket::bind(("0.0.0.0", 0)))?;
        Ok(socket)
    }

    fn connect_(&mut self, udp_port: u16) -> RobotResult<()> {
        let result = self._connect(ConnectData { version: FRANKA_ROBOT_VERSION, udp_port })?;
        if let ConnectStatusEnum::Success = result.status {
            Ok(())
        } else {
            Err(RobotException::IncompatibleVersionException {
                server_version: result.version as u64,
                client_version: FRANKA_ROBOT_VERSION as u64,
            })
        }
    }

    pub(crate) fn finish_current_motion(&mut self) -> RobotResult<()> {
        let response = self.receive_motion_end()?;
        if response.status == MoveStatus::Success {
            Ok(())
        } else {
            Err(RobotException::CommandException(format!(
                "move failed with status: {:?}",
                response.status
            )))
        }
    }

    /// Cancel without inventing an algorithm command. StopMove performs the
    /// device's stop protocol; wait until its running modes end, then consume
    /// the cancelled Move response so the next session starts with clean TCP state.
    pub(crate) fn cancel_current_motion(&mut self, was_nonblocking: bool) -> RobotResult<()> {
        // A network waiting bound, not a physical stopping deadline. This path
        // runs only on cancellation / failed sessions, never per normal cycle.
        const CLEANUP_TIMEOUT: Duration = Duration::from_secs(3);
        let deadline = Instant::now() + CLEANUP_TIMEOUT;
        let previous_udp_timeout = self.udp_socket.read_timeout()?;
        let previous_deadline = self.network.replace_receive_deadline(Some(deadline));
        let result = (|| {
            let stop: RobotResult<()> = self._stop_move(())?.into();
            stop?;
            self.udp_socket.set_nonblocking(false)?;
            let mut buffer = vec![0; std::mem::size_of::<RobotStateInter>() * 5];
            loop {
                let remaining = crate::network::remaining(deadline)?;
                self.udp_socket.set_read_timeout(Some(
                    previous_udp_timeout.map_or(remaining, |old| old.min(remaining)),
                ))?;
                let (size, _) = self.udp_socket.recv_from(&mut buffer)?;
                let state = Self::decode_state(&buffer[..size])?;
                let mut latest = self.robot_state.write().unwrap();
                if state.command_id() <= latest.command_id() {
                    continue;
                }
                *latest = state;
                let running = (state.motion_generator_mode != MotionGeneratorMode::Idle
                    && state.motion_generator_mode != MotionGeneratorMode::None)
                    || state.controller_mode == ControllerMode::ExternalController;
                if !running {
                    break;
                }
            }
            let response = self.receive_motion_end()?;
            match response.status {
                MoveStatus::Success | MoveStatus::Preempted => Ok(()),
                status => Err(RobotException::CommandException(format!(
                    "cancelled move ended with status: {status:?}"
                ))),
            }
        })();
        self.network.replace_receive_deadline(previous_deadline);
        let restore_timeout = self.udp_socket.set_read_timeout(previous_udp_timeout);
        let restore_mode = self.udp_socket.set_nonblocking(was_nonblocking);
        let restore: RobotResult<()> = restore_timeout.and(restore_mode).map_err(Into::into);
        match (result, restore) {
            (Ok(()), result) | (result, Ok(())) => result,
            (Err(primary), Err(cleanup)) => Err(RobotException::ControlSession {
                primary: Box::new(primary),
                cleanup: Box::new(cleanup),
            }),
        }
    }

    fn receive_motion_end(&mut self) -> RobotResult<MoveResponse> {
        let id = self.motion_command_id.ok_or_else(|| {
            RobotException::CommandException("no active control session to finish".into())
        })?;
        let response = self.network.tcp_blocking_recv::<MoveResponse>(id)?;
        self.motion_command_id = None;
        Ok(response)
    }

    pub(crate) fn recv_state(&mut self) -> RobotResult<(RobotStateInter, SocketAddr, Duration)> {
        let mut buffer = vec![0_u8; std::mem::size_of::<RobotStateInter>() * 5];
        self.recv_state_into(&mut buffer)
    }

    pub(crate) fn recv_state_into(
        &mut self,
        mut buffer: &mut [u8],
    ) -> RobotResult<(RobotStateInter, SocketAddr, Duration)> {
        let start = Instant::now();
        let last_id = self.robot_state.read().unwrap().command_id();
        let mut latest: Option<(RobotStateInter, SocketAddr)> = None;

        self.udp_socket.set_nonblocking(true)?;
        let drain_result: RobotResult<()> = loop {
            match self.udp_socket.recv_from(&mut buffer) {
                Ok((size, addr)) => {
                    let candidate = Self::decode_state(&buffer[..size])?;
                    if candidate.command_id() > last_id
                        && latest.as_ref().map_or(true, |(state, _)| {
                            candidate.command_id() > state.command_id()
                        })
                    {
                        latest = Some((candidate, addr));
                    }
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break Ok(()),
                Err(err) => break Err(err.into()),
            }
        };
        self.udp_socket.set_nonblocking(false)?;
        drain_result?;

        while latest.is_none() {
            let (size, addr) = self.udp_socket.recv_from(&mut buffer)?;
            let candidate = Self::decode_state(&buffer[..size])?;
            if candidate.command_id() > last_id {
                latest = Some((candidate, addr));
            }
        }

        let (state, latest_addr) = latest.unwrap();
        {
            let mut latest = self.robot_state.write().unwrap();
            *latest = state;
        }
        Ok((state, latest_addr, start.elapsed()))
    }

    fn decode_state(data: &[u8]) -> RobotResult<RobotStateInter> {
        let state: RobotStateInter = bincode::deserialize(data)
            .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
        state.error_result()?;
        Ok(state)
    }

    pub(crate) fn send_prepared_command(
        &mut self,
        addr: SocketAddr,
        command: RobotCommand,
    ) -> RobotResult<()> {
        let data = bincode::serialize(&command)
            .map_err(|err| RobotException::CommandException(err.to_string()))?;
        self.udp_socket.send_to(&data, addr)?;
        Ok(())
    }

    pub(crate) fn prepare_command(
        state: &RobotStateInter,
        command: RobotCommand,
        mode: &MoveData,
    ) -> RobotCommand {
        use crate::types::robot_types::CommandIDConfig;

        let mut command = command.filter_for_mode(state, mode);
        command.set_command_id(state.command_id());
        command
    }

    pub(crate) fn motion_started(state: &RobotStateInter, mode: &MoveData) -> bool {
        let expected_motion = match mode.motion_generator_mode {
            MoveMotionGeneratorMode::JointPosition => MotionGeneratorMode::JointPosition,
            MoveMotionGeneratorMode::JointVelocity => MotionGeneratorMode::JointVelocity,
            MoveMotionGeneratorMode::CartesianPosition => MotionGeneratorMode::CartesianPosition,
            MoveMotionGeneratorMode::CartesianVelocity => MotionGeneratorMode::CartesianVelocity,
        };
        let expected_controller = match mode.controller_mode {
            MoveControllerMode::JointImpedance => ControllerMode::JointImpedance,
            MoveControllerMode::CartesianImpedance => ControllerMode::CartesianImpedance,
            MoveControllerMode::ExternalController => ControllerMode::ExternalController,
        };
        state.motion_generator_mode == expected_motion
            && state.controller_mode == expected_controller
    }

    pub fn is_moving(&mut self) -> RobotResult<bool> {
        let _ = self.recv_state();
        let state = self.robot_state.read().unwrap();
        state.error_result()?;

        Ok((state.motion_generator_mode != MotionGeneratorMode::Idle
            && state.motion_generator_mode != MotionGeneratorMode::None)
            || state.controller_mode == ControllerMode::ExternalController)
    }

    pub fn waiting_for_finish(&mut self) -> RobotResult<()> {
        while self.is_moving()? {
            sleep(Duration::from_millis(1));
        }
        self.finish_current_motion()
    }
}
