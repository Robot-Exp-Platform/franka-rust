use robot_behavior::{RobotException, RobotResult};
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::Duration,
};

use crate::{
    FRANKA_ROBOT_VERSION, PORT_ROBOT_COMMAND, PORT_ROBOT_UDP,
    command_handle::CommandHandle,
    network::Network,
    types::{robot_command::RobotCommand, robot_state::*, robot_types::*},
};

#[derive(Clone, Default)]
pub struct FrankaRobotImpl {
    pub(crate) network: Network,
    pub(crate) command_handle: CommandHandle<RobotCommand, RobotStateInter>,
    pub robot_state: Arc<RwLock<RobotStateInter>>,
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
        let (command_handle, robot_state) = Network::spawn_udp_thread(PORT_ROBOT_UDP);
        let network = Network::new(ip, PORT_ROBOT_COMMAND);
        let mut robot = Self { network, command_handle, robot_state };
        robot.connect_().unwrap();
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

    fn connect_(&mut self) -> RobotResult<()> {
        let result =
            self._connect(ConnectData { version: FRANKA_ROBOT_VERSION, udp_port: PORT_ROBOT_UDP })?;
        if let ConnectStatusEnum::Success = result.status {
            Ok(())
        } else {
            Err(RobotException::IncompatibleVersionException {
                server_version: result.version as u64,
                client_version: FRANKA_ROBOT_VERSION as u64,
            })
        }
    }

    pub fn is_moving(&self) -> RobotResult<bool> {
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
        self.command_handle.remove_closure();
        let _ = self.network.tcp_blocking_recv::<MoveResponse>();
        Ok(())
    }
}
