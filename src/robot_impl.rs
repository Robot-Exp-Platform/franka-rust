use robot_behavior::{RobotException, RobotResult};
use std::{
    sync::{Arc, RwLock},
    thread::sleep,
    time::{Duration, Instant},
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
        Self::new_with_hook(ip, |_| {})
    }

    pub fn try_new(ip: &str) -> RobotResult<Self> {
        Self::try_new_with_hook(ip, |_| {})
    }

    pub fn new_with_hook(ip: &str, on_update: impl Fn(&RobotStateInter) + Send + 'static) -> Self {
        Self::try_new_with_hook(ip, on_update)
            .unwrap_or_else(|error| panic!("failed to connect to Franka: {error}"))
    }

    pub fn try_new_with_hook(
        ip: &str,
        on_update: impl Fn(&RobotStateInter) + Send + 'static,
    ) -> RobotResult<Self> {
        let (command_handle, robot_state, udp_port) =
            Network::spawn_udp_thread(PORT_ROBOT_UDP, on_update);
        let network = Network::new(ip, PORT_ROBOT_COMMAND);
        let mut robot = Self { network, command_handle, robot_state };
        robot.connect_(udp_port)?;
        Ok(robot)
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
    cmd_fn!(_automatic_error_recovery, { Command::AutomaticErrorRecovery }; data: (); AutomaticErrorRecoveryStatus);
    cmd_fn!(_stop_move, { Command::StopMove }; data: (); GetterSetterStatus);
    cmd_fn!(_get_cartesian_limit, { Command::GetCartesianLimit }; data: GetCartesianLimitData; GetCartesianLimitStatus);

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

    pub fn is_moving(&self) -> RobotResult<bool> {
        let state = self.robot_state.read().unwrap();
        state.error_result()?;

        Ok((state.motion_generator_mode != MotionGeneratorMode::Idle
            && state.motion_generator_mode != MotionGeneratorMode::None)
            || state.controller_mode == ControllerMode::ExternalController)
    }

    pub fn waiting_for_finish(&mut self) -> RobotResult<()> {
        let mut last_message_id = None;
        let mut last_state_update = Instant::now();
        loop {
            let (moving, message_id) = {
                let state = match self.robot_state.read() {
                    Ok(state) => state,
                    Err(_) => {
                        self.command_handle.remove_closure();
                        return Err(RobotException::CommandException(
                            "robot state lock poisoned while waiting for motion".to_string(),
                        ));
                    }
                };
                if let Err(error) = state.error_result() {
                    self.command_handle.remove_closure();
                    return Err(error);
                }
                if let Some(error) = motion_mode_error(state.robot_mode) {
                    self.command_handle.remove_closure();
                    return Err(error);
                }
                (
                    (state.motion_generator_mode != MotionGeneratorMode::Idle
                        && state.motion_generator_mode != MotionGeneratorMode::None)
                        || state.controller_mode == ControllerMode::ExternalController,
                    state.message_id,
                )
            };
            if last_message_id != Some(message_id) {
                last_message_id = Some(message_id);
                last_state_update = Instant::now();
            } else if state_stream_stalled(last_state_update.elapsed()) {
                self.command_handle.remove_closure();
                return Err(RobotException::NetworkError(
                    "robot state stopped updating for 100 ms while waiting for motion".to_string(),
                ));
            }
            if !moving {
                break;
            }
            sleep(Duration::from_millis(1));
        }
        self.command_handle.remove_closure();
        let response = self.network.tcp_blocking_recv::<MoveResponse>()?;
        let status = response.status;
        if status == MoveStatus::Success {
            Ok(())
        } else {
            Err(RobotException::CommandException(format!(
                "move failed with status: {status:?}"
            )))
        }
    }
}

const STATE_STREAM_TIMEOUT: Duration = Duration::from_millis(100);

fn state_stream_stalled(elapsed: Duration) -> bool {
    elapsed >= STATE_STREAM_TIMEOUT
}

fn motion_mode_error(robot_mode: RobotMode) -> Option<RobotException> {
    matches!(
        robot_mode,
        RobotMode::Reflux | RobotMode::UserStopped | RobotMode::AutomaticErrorRecovery
    )
    .then(|| {
        RobotException::CommandException(format!(
            "motion aborted because robot entered {robot_mode:?} mode"
        ))
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn waiting_mode_rejects_safety_and_recovery_modes() {
        assert!(motion_mode_error(RobotMode::Reflux).is_some());
        assert!(motion_mode_error(RobotMode::UserStopped).is_some());
        assert!(motion_mode_error(RobotMode::AutomaticErrorRecovery).is_some());
    }

    #[test]
    fn waiting_mode_accepts_idle_and_active_motion() {
        assert!(motion_mode_error(RobotMode::Idle).is_none());
        assert!(motion_mode_error(RobotMode::Move).is_none());
    }

    #[test]
    fn waiting_state_stream_has_a_bounded_stale_timeout() {
        assert!(!state_stream_stalled(Duration::from_millis(99)));
        assert!(state_stream_stalled(Duration::from_millis(100)));
    }
}
