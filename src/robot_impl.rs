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
        };
        robot.connect_(udp_port).unwrap();
        robot
    }

    cmd_fn!(_connect, { Command::Connect }; data: ConnectData; ConnectStatus);
    cmd_fn!(_move, { Command::Move }; data: MoveData; MoveStatus);
    cmd_fn!(_set_collision_behavior, { Command::SetCollisionBehavior }; data: SetCollisionBehaviorData; GetterSetterStatus);
    cmd_fn!(_set_joint_impedance, { Command::SetJointImpedance }; data: SetJointImpedanceData; GetterSetterStatus);
    cmd_fn!(_set_cartesian_impedance, { Command::SetCartesianImpedance }; data: SetCartesianImpedanceData; GetterSetterStatus);
    cmd_fn!(_set_guiding_mode, { Command::SetGuidingMode }; data: SetGuidingModeData; GetterSetterStatus);
    cmd_fn!(_set_ee_to_k, { Command::SetEEToK }; data: SetEEToKData; GetterSetterStatus);
    cmd_fn!(_set_ne_to_ee, { Command::SetNEToEE }; data: SetNEToEEData; GetterSetterStatus);
    cmd_fn!(_set_load, { Command::SetLoad }; data: SetLoadData; GetterSetterStatus);
    cmd_fn!(_set_fliters, { Command::SetFilters }; data: SetFiltersData; GetterSetterStatus);
    cmd_fn!(_automatic_error_recovery, { Command::AutomaticErrorRecovery }; data: (); GetterSetterStatus);
    cmd_fn!(_stop_move, { Command::StopMove }; data: (); GetterSetterStatus);
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
        let response = self.network.tcp_blocking_recv::<MoveResponse>()?;
        if response.status == MoveStatus::Success {
            Ok(())
        } else {
            Err(RobotException::CommandException(format!(
                "move failed with status: {:?}",
                response.status
            )))
        }
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
