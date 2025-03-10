use crate::{
    PORT_ROBOT_COMMAND,
    exception::FrankaResult,
    network::Network,
    types::{
        robot_state::{RobotState, RobotStateInter},
        robot_types::*,
    },
};

#[derive(Default)]
pub struct FrankaRobot {
    network: Network,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty ; $ret_type:ty) => {
        fn $fn_name(&mut self, $arg_name: $arg_type) -> FrankaResult<$ret_type> {
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
            network: Network::new(ip, PORT_ROBOT_COMMAND),
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

    /// # set_collision_behavior
    /// **Set the collision behavior of the robot.**
    ///
    /// Set separate torque and force boundaries for acceleration/deceleration and constant velocity
    /// movement phases.
    ///
    /// Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
    /// Forces or torques above the upper threshold are registered as collision and cause the robot to
    /// stop moving.
    pub fn set_collision_behavior(&mut self, data: SetCollisionBehaviorData) -> FrankaResult<()> {
        self._set_collision_behavior(data)?.into()
    }

    /// # set_joint_impedance
    /// **Sets the impedance for each joint in the internal controller.**
    pub fn set_joint_impedance(&mut self, data: SetJointImpedanceData) -> FrankaResult<()> {
        self._set_joint_impedance(data)?.into()
    }

    /// # set_cartesian_impedance
    /// **Sets the impedance for the Cartesian internal controller.**
    pub fn set_cartesian_impedance(&mut self, data: SetCartesianImpedanceData) -> FrankaResult<()> {
        self._set_cartesian_impedance(data)?.into()
    }

    /// # set_cartesian_limit
    /// **Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).**
    /// If a flag is set to true, movement is unlocked.
    pub fn set_guiding_mode(&mut self, data: SetGuidingModeData) -> FrankaResult<()> {
        self._set_guiding_mode(data)?.into()
    }

    pub fn set_ee_to_k(&mut self, data: SetEEToKData) -> FrankaResult<()> {
        self._set_ee_to_k(data)?.into()
    }

    pub fn set_ne_to_ee(&mut self, data: SetNEToEEData) -> FrankaResult<()> {
        self._set_ne_to_ee(data)?.into()
    }

    /// # set_load
    /// **Sets the load of the robot.**
    /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    /// This is not for setting end effector parameters, which have to be set in the administrator's
    /// interface.
    pub fn set_load(&mut self, data: SetLoadData) -> FrankaResult<()> {
        self._set_load(data)?.into()
    }

    #[deprecated(note = "please use `low_pass_filter` instead")]
    pub fn set_filters(&mut self, data: SetFiltersData) -> FrankaResult<()> {
        self._set_fliters(data)?.into()
    }

    pub fn read_state(&mut self) -> FrankaResult<RobotState> {
        self.network
            .udp_recv::<RobotStateInter>()
            .map(RobotStateInter::into)
    }
}

// impl RobotBehavior for FrankaRobot {
//     pub fn new(ip: &str) -> Self {}
// }

// impl ArmBehavior<FRANKA_EMIKA_DOF> for FrankaRobot {}
