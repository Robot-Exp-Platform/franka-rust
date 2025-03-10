use robot_behavior::{
    ArmBehavior, ArmBehaviorExt, ArmState, ArmStateType, ControlType, MotionType, RealtimeBehavior,
    RobotBehavior, RobotResult,
};

use crate::{
    FRANKA_EMIKA_DOF, LIBFRANKA_VERSION, PORT_ROBOT_COMMAND, PORT_ROBOT_UDP,
    control::control_mode::ControlMode,
    network::Network,
    types::{
        robot_state::{RobotState, RobotStateInter},
        robot_types::*,
    },
};

#[derive(Default)]
pub struct FrankaRobot {
    network: Network,
    control_mode: ControlMode,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty ; $ret_type:ty) => {
        fn $fn_name(&mut self, $arg_name: $arg_type) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> = self
                .network
                .tcp_send_and_recv(&mut Request::<$command, $arg_type>::from($arg_name))?;
            Ok(response.status)
        }
    };
}

impl FrankaRobot {
    pub fn new(ip: &str) -> Self {
        FrankaRobot {
            network: Network::new(ip, PORT_ROBOT_COMMAND, PORT_ROBOT_UDP),
            control_mode: ControlMode::default(),
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
    cmd_fn!(_stop_move, { Command::StopMove }; data: (); GetterSetterStatus);
    cmd_fn!(_get_cartesian_limit, { Command::GetCartesianLimit }; data: GetCartesianLimitData; GetCartesianLimitStatus);

    /// # set_collision_behavior
    /// **Set the collision behavior of the robot.**
    ///
    /// Set separate torque and force boundaries for acceleration/deceleration and constant velocity
    /// movement phases.
    ///
    /// Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
    /// Forces or torques above the upper threshold are registered as collision and cause the robot to
    /// stop moving.
    pub fn set_collision_behavior(&mut self, data: SetCollisionBehaviorData) -> RobotResult<()> {
        self._set_collision_behavior(data)?.into()
    }

    /// # set_joint_impedance
    /// **Sets the impedance for each joint in the internal controller.**
    pub fn set_joint_impedance(&mut self, data: SetJointImpedanceData) -> RobotResult<()> {
        self._set_joint_impedance(data)?.into()
    }

    /// # set_cartesian_impedance
    /// **Sets the impedance for the Cartesian internal controller.**
    pub fn set_cartesian_impedance(&mut self, data: SetCartesianImpedanceData) -> RobotResult<()> {
        self._set_cartesian_impedance(data)?.into()
    }

    /// # set_cartesian_limit
    /// **Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).**
    /// If a flag is set to true, movement is unlocked.
    pub fn set_guiding_mode(&mut self, data: SetGuidingModeData) -> RobotResult<()> {
        self._set_guiding_mode(data)?.into()
    }

    pub fn set_ee_to_k(&mut self, data: SetEEToKData) -> RobotResult<()> {
        self._set_ee_to_k(data)?.into()
    }

    pub fn set_ne_to_ee(&mut self, data: SetNEToEEData) -> RobotResult<()> {
        self._set_ne_to_ee(data)?.into()
    }

    /// # set_load
    /// **Sets the load of the robot.**
    /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    /// This is not for setting end effector parameters, which have to be set in the administrator's
    /// interface.
    pub fn set_load(&mut self, data: SetLoadData) -> RobotResult<()> {
        self._set_load(data)?.into()
    }

    #[deprecated(note = "please use `low_pass_filter` instead")]
    pub fn set_filters(&mut self, data: SetFiltersData) -> RobotResult<()> {
        self._set_fliters(data)?.into()
    }

    pub fn read_state(&mut self) -> RobotResult<RobotState> {
        self.network
            .udp_recv::<RobotStateInter>()
            .map(RobotStateInter::into)
    }

    pub fn set_default_behavior(&mut self) -> RobotResult<()> {
        self.set_collision_behavior(SetCollisionBehaviorData::default())?;
        self.set_joint_impedance(SetJointImpedanceData::default())?;
        self.set_cartesian_impedance(SetCartesianImpedanceData::default())?;
        Ok(())
    }
}

impl RobotBehavior for FrankaRobot {
    fn version(&self) -> String {
        format!("FrankaRobot v{}", LIBFRANKA_VERSION)
    }

    fn init(&mut self) -> RobotResult<()> {
        Ok(())
    }

    fn shutdown(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn enable(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn disable(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn reset(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn is_moving(&mut self) -> bool {
        unimplemented!()
    }

    fn stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn pause(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn resume(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }
}

impl ArmBehavior<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_to(&mut self, _target: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_to_async(&mut self, _target: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_rel(&mut self, _rel: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_rel_async(&mut self, _rel: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_path(&mut self, _path: Vec<MotionType<FRANKA_EMIKA_DOF>>) -> RobotResult<()> {
        unimplemented!()
    }

    fn control_with(&mut self, _control: ControlType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn read_state(&mut self, _state_type: ArmStateType) -> RobotResult<ArmState<FRANKA_EMIKA_DOF>> {
        unimplemented!()
    }
}

impl ArmBehaviorExt<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_path_prepare(&mut self, _path: Vec<MotionType<FRANKA_EMIKA_DOF>>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_path_start(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_path_prepare_from_file(&mut self, _path: &str) -> RobotResult<()> {
        unimplemented!()
    }
}

impl<C, H> RealtimeBehavior<C, H> for FrankaRobot {
    fn enter_realtime(&mut self, _realtime_config: C) -> RobotResult<H> {
        unimplemented!()
    }

    fn exit_realtime(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn quality_of_service(&self) -> f64 {
        unimplemented!()
    }
}
