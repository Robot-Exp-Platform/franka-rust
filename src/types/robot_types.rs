#![allow(dead_code)]

use robot_behavior::{ControlType, LoadState, MotionType, RobotException, RobotResult};
use serde::ser::SerializeStruct;
use serde::{Deserialize, Serialize, Serializer};
use serde_repr::{Deserialize_repr, Serialize_repr};
use std::f64::consts::PI;
use std::fmt;
use std::marker::ConstParamTy;
use std::time::Duration;

#[derive(Debug, ConstParamTy, PartialEq, Eq, Serialize_repr, Deserialize_repr)]
#[repr(u32)]
pub enum Command {
    Connect,
    Move,
    StopMove,
    GetCartesianLimit,
    SetCollisionBehavior,
    SetJointImpedance,
    SetCartesianImpedance,
    SetGuidingMode,
    SetEEToK,
    SetNEToEE,
    SetLoad,
    SetFilters,
    AutomaticErrorRecovery,
    LoadModelLibrary,
    GetRobotModel,
}

#[derive(Debug, Default, PartialEq, Clone, Copy)]
pub struct CommandHeader<const C: Command> {
    pub command_id: u32,
    pub size: u32,
}

#[derive(Debug, Default, Serialize, PartialEq)]
#[repr(C, packed)]
pub struct Request<const C: Command, D: Clone + Copy> {
    pub header: CommandHeader<C>,
    pub data: D,
}

#[derive(Debug, Default, Deserialize)]
#[repr(C, packed)]
pub struct Response<const C: Command, S> {
    pub header: CommandHeader<C>,
    pub status: S,
}

pub trait CommandIDConfig<T> {
    fn command_id(&self) -> T;
    fn set_command_id(&mut self, id: T);
    fn time(&self) -> Option<Duration> {
        None
    }
}

pub trait CommandFilter<S> {
    fn filter(self, state: &S) -> Self;
}

#[derive(Debug, Deserialize_repr)]
#[repr(u8)]
pub enum DefaultStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
}

#[derive(Debug, Deserialize_repr)]
#[repr(u8)]
pub enum GetterSetterStatus {
    Success,
    CommandNotPossibleRejected,
    InvalidArgumentRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
}

// ! Connect Command
pub type ConnectRequest = Request<{ Command::Connect }, ConnectData>;
pub type ConnectResponse = Response<{ Command::Connect }, ConnectStatus>;

#[derive(Debug, Default, Serialize, PartialEq, Copy, Clone)]
#[repr(C, packed)]
pub struct ConnectData {
    pub version: u16,
    pub udp_port: u16,
}

#[derive(Debug, Deserialize_repr, Serialize_repr)]
#[repr(u8)]
pub enum ConnectStatusEnum {
    Success,
    IncompatibleLibraryVersion,
}

#[derive(Deserialize)]
#[repr(C, packed)]
pub struct ConnectStatus {
    pub status: ConnectStatusEnum,
    pub version: u16,
}

// ! Move Command
pub type MoveRequest = Request<{ Command::Move }, MoveData>;
pub type MoveResponse = Response<{ Command::Move }, MoveStatus>;

impl fmt::Debug for MoveResponse {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MoveResponse")
            .field("status", &self.status)
            .finish()
    }
}

#[derive(Debug, Default, Serialize_repr, Copy, Clone)]
#[repr(u32)]
pub enum MoveControllerMode {
    #[default]
    JointImpedance,
    CartesianImpedance,
    ExternalController,
}

#[derive(Debug, Default, Serialize_repr, Copy, Clone)]
#[repr(u32)]
pub enum MoveMotionGeneratorMode {
    #[default]
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
}

#[derive(Debug, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct MoveDeviation {
    translation: f64,
    rotation: f64,
    elbow: f64,
}

impl Default for MoveDeviation {
    fn default() -> Self {
        MoveDeviation {
            translation: 10.,
            rotation: 3.12,
            elbow: 2. * PI,
        }
    }
}

#[derive(Debug, Default, Serialize, Copy, Clone)]
pub struct MoveData {
    pub controller_mode: MoveControllerMode,
    pub motion_generator_mode: MoveMotionGeneratorMode,
    pub maximum_path_deviation: MoveDeviation,
    pub maximum_goal_deviation: MoveDeviation,
}

#[derive(Debug, Deserialize_repr)]
#[repr(u8)]
pub enum MoveStatus {
    Success,
    MotionStarted,
    Preempted,
    PreemptedDueToActivatedSafetyFunctions,
    CommandRejectedDueToActivatedSafetyFunctions,
    CommandNotPossibleRejected,
    StartAtSingularPoseRejected,
    InvalidArgumentRejected,
    ReflexAborted,
    EmergencyAborted,
    InputErrorAborted,
    Aborted,
}

// ! StopMove Command
pub type StopMoveRequest = Request<{ Command::StopMove }, ()>;
pub type StopMoveResponse = Response<{ Command::StopMove }, StopMoveStatus>;

#[derive(Debug, Deserialize_repr)]
#[repr(u8)]
pub enum StopMoveStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
    EmergencyAborted,
    ReflexAborted,
    Aborted,
}

// ! GetCartesianLimit Command
pub type GetCartesianLimitRequest = Request<{ Command::GetCartesianLimit }, GetCartesianLimitData>;
pub type GetCartesianLimitResponse =
    Response<{ Command::GetCartesianLimit }, GetCartesianLimitStatus>;
pub type GetCartesianLimitStatus = DefaultStatus;

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct GetCartesianLimitData {
    id: i32,
}

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct GetCartesianLimitResponseData {
    object_world_size: [f64; 3],
    object_frame: [f64; 16],
    object_activation: bool,
}

// ! SetCollisionBehavior Command
pub type SetCollisionBehaviorRequest =
    Request<{ Command::SetCollisionBehavior }, SetCollisionBehaviorData>;
pub type SetCollisionBehaviorResponse =
    Response<{ Command::SetCollisionBehavior }, GetterSetterStatus>;

#[derive(Debug, Serialize, Copy, Clone)]
pub struct SetCollisionBehaviorData {
    pub lower_torque_thresholds_acceleration: [f64; 7],
    pub upper_torque_thresholds_acceleration: [f64; 7],
    pub lower_torque_thresholds_nominal: [f64; 7],
    pub upper_torque_thresholds_nominal: [f64; 7],
    pub lower_force_thresholds_acceleration: [f64; 6],
    pub upper_force_thresholds_acceleration: [f64; 6],
    pub lower_force_thresholds_nominal: [f64; 6],
    pub upper_force_thresholds_nominal: [f64; 6],
}

impl Default for SetCollisionBehaviorData {
    fn default() -> Self {
        SetCollisionBehaviorData {
            lower_torque_thresholds_acceleration: [20.; 7],
            upper_torque_thresholds_acceleration: [20.; 7],
            lower_torque_thresholds_nominal: [10.; 7],
            upper_torque_thresholds_nominal: [10.; 7],
            lower_force_thresholds_acceleration: [20.; 6],
            upper_force_thresholds_acceleration: [20.; 6],
            lower_force_thresholds_nominal: [10.; 6],
            upper_force_thresholds_nominal: [10.; 6],
        }
    }
}

impl From<f64> for SetCollisionBehaviorData {
    fn from(value: f64) -> Self {
        SetCollisionBehaviorData {
            lower_torque_thresholds_acceleration: [value; 7],
            upper_torque_thresholds_acceleration: [value; 7],
            lower_torque_thresholds_nominal: [value; 7],
            upper_torque_thresholds_nominal: [value; 7],
            lower_force_thresholds_acceleration: [value; 6],
            upper_force_thresholds_acceleration: [value; 6],
            lower_force_thresholds_nominal: [value; 6],
            upper_force_thresholds_nominal: [value; 6],
        }
    }
}

// ! SetJointImpedance Command
pub type SetJointImpedanceRequest = Request<{ Command::SetJointImpedance }, SetJointImpedanceData>;
pub type SetJointImpedanceResponse = Response<{ Command::SetJointImpedance }, GetterSetterStatus>;

#[derive(Debug, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetJointImpedanceData {
    k_theta: [f64; 7],
}

impl Default for SetJointImpedanceData {
    fn default() -> Self {
        SetJointImpedanceData {
            k_theta: [3000., 3000., 3000., 2500., 2500., 2000., 2000.],
        }
    }
}

impl From<[f64; 7]> for SetJointImpedanceData {
    fn from(value: [f64; 7]) -> Self {
        SetJointImpedanceData { k_theta: value }
    }
}

// ! SetCartesianImpedance Command
pub type SetCartesianImpedanceRequest =
    Request<{ Command::SetCartesianImpedance }, SetCartesianImpedanceData>;
pub type SetCartesianImpedanceResponse =
    Response<{ Command::SetCartesianImpedance }, GetterSetterStatus>;

#[derive(Debug, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetCartesianImpedanceData {
    k_x: [f64; 6],
}

impl Default for SetCartesianImpedanceData {
    fn default() -> Self {
        SetCartesianImpedanceData {
            k_x: [3000., 3000., 3000., 300., 300., 300.],
        }
    }
}

impl From<[f64; 6]> for SetCartesianImpedanceData {
    fn from(value: [f64; 6]) -> Self {
        SetCartesianImpedanceData { k_x: value }
    }
}

// ! SetGuidingMode Command
pub type SetGuidingModeRequest = Request<{ Command::SetGuidingMode }, SetGuidingModeData>;
pub type SetGuidingModeResponse = Response<{ Command::SetGuidingMode }, GetterSetterStatus>;

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetGuidingModeData {
    pub guiding_mode: [bool; 6],
    pub nullspace: bool,
}

// ! SetEEToK Command
pub type SetEEToKRequest = Request<{ Command::SetEEToK }, SetEEToKData>;
pub type SetEEToKResponse = Response<{ Command::SetEEToK }, GetterSetterStatus>;

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetEEToKData {
    pose_ee_to_k: [f64; 16],
}

impl From<[f64; 16]> for SetEEToKData {
    fn from(value: [f64; 16]) -> Self {
        SetEEToKData {
            pose_ee_to_k: value,
        }
    }
}

// ! SetNEToEE Command
pub type SetNEToEERequest = Request<{ Command::SetNEToEE }, SetNEToEEData>;
pub type SetNEToEEResponse = Response<{ Command::SetNEToEE }, GetterSetterStatus>;

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetNEToEEData {
    pose_ne_to_ee: [f64; 16],
}

impl From<[f64; 16]> for SetNEToEEData {
    fn from(value: [f64; 16]) -> Self {
        SetNEToEEData {
            pose_ne_to_ee: value,
        }
    }
}

// ! SetLoad Command
pub type SetLoadRequest = Request<{ Command::SetLoad }, SetLoadData>;
pub type SetLoadResponse = Response<{ Command::SetLoad }, GetterSetterStatus>;

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetLoadData {
    pub m_load: f64,
    pub x_load: [f64; 3],
    pub i_load: [f64; 9],
}

impl From<LoadState> for SetLoadData {
    fn from(value: LoadState) -> Self {
        SetLoadData {
            m_load: value.m,
            x_load: value.x,
            i_load: value.i,
        }
    }
}

// ! SetFilters Command
pub type SetFiltersRequest = Request<{ Command::SetFilters }, SetFiltersData>;
pub type SetFiltersResponse = Response<{ Command::SetFilters }, GetterSetterStatus>;

#[derive(Debug, Default, Serialize, Copy, Clone)]
#[repr(C, packed)]
pub struct SetFiltersData {
    joint_position_filter_frequency: f64,
    joint_velocity_filter_frequency: f64,
    cartesian_position_filter_frequency: f64,
    cartesian_velocity_filter_frequency: f64,
    controller_filter_frequency: f64,
}

// ! AutomaticErrorRecovery Command
pub type AutomaticErrorRecoveryRequest = Request<{ Command::AutomaticErrorRecovery }, ()>;
pub type AutomaticErrorRecoveryResponse =
    Response<{ Command::AutomaticErrorRecovery }, AutomaticErrorRecoveryStatus>;

#[derive(Debug, Deserialize_repr)]
#[repr(u8)]
pub enum AutomaticErrorRecoveryStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
    ManualErrorRecoveryRequiredRejected,
    ReflexAborted,
    EmergencyAborted,
    Aborted,
}

// ! LoadModelLibrary Command
pub type LoadModelLibraryRequest = Request<{ Command::LoadModelLibrary }, LoadModelLibraryData>;
pub type LoadModelLibraryResponse = Response<{ Command::LoadModelLibrary }, LoadModelLibraryStatus>;

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum LoadModelLibraryArchitecture {
    X64,
    X86,
    Arm,
    Arm64,
}

#[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
#[repr(u8)]
pub enum LoadModelLibrarySystem {
    Linux,
    Windows,
}

#[derive(Serialize, Deserialize, Debug, Copy, Clone)]
#[repr(C, packed)]
pub struct LoadModelLibraryData {
    pub architecture: LoadModelLibraryArchitecture,
    pub system: LoadModelLibrarySystem,
}

#[derive(Debug, Deserialize_repr, PartialEq)]
#[repr(u8)]
pub enum LoadModelLibraryStatus {
    Success,
    Error,
}

impl Default for LoadModelLibraryData {
    fn default() -> Self {
        let arch = if cfg!(target_arch = "x86_64") {
            LoadModelLibraryArchitecture::X64
        } else if cfg!(target_arch = "x86") {
            LoadModelLibraryArchitecture::X86
        } else if cfg!(target_arch = "aarch64") {
            LoadModelLibraryArchitecture::Arm64
        } else {
            LoadModelLibraryArchitecture::Arm
        };
        let sys = if cfg!(target_os = "linux") {
            LoadModelLibrarySystem::Linux
        } else if cfg!(target_os = "windows") {
            LoadModelLibrarySystem::Windows
        } else {
            LoadModelLibrarySystem::Linux
        };
        LoadModelLibraryData {
            architecture: arch,
            system: sys,
        }
    }
}

// ! GetRobotModel Command
pub type GetRobotModelRequest = Request<{ Command::GetRobotModel }, ()>;
pub type GetRobotModelResponse = Response<{ Command::GetRobotModel }, DefaultStatus>;

impl<const C: Command, D: Clone + Copy> Request<C, D> {
    pub fn size() -> usize {
        std::mem::size_of::<Request<C, D>>() + 4
    }
}

impl<const C: Command, S> Response<C, S> {
    pub fn size() -> usize {
        std::mem::size_of::<Response<C, S>>() + 4
    }
}

impl<const C: Command> Serialize for CommandHeader<C> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let mut state = serializer.serialize_struct("CommandHeader", 3)?;
        state.serialize_field("command", &C)?;
        state.serialize_field("command_id", &self.command_id)?;
        state.serialize_field("size", &self.size)?;
        state.end()
    }
}

impl<'de, const C: Command> Deserialize<'de> for CommandHeader<C> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct CommandHeaderInternal {
            command: Command,
            command_id: u32,
            size: u32,
        }

        let helper = CommandHeaderInternal::deserialize(deserializer)?;
        Ok(CommandHeader {
            command_id: helper.command_id,
            size: helper.size,
        })
    }
}

impl<const C: Command, D: Clone + Copy> From<D> for Request<C, D> {
    fn from(data: D) -> Self {
        Request {
            header: CommandHeader {
                command_id: 0,
                size: Self::size() as u32,
            },
            data,
        }
    }
}

impl<const C: Command, S> From<S> for Response<C, S> {
    fn from(status: S) -> Self {
        Response {
            header: CommandHeader {
                command_id: 0,
                size: Self::size() as u32,
            },
            status,
        }
    }
}

impl CommandIDConfig<u32> for () {
    fn command_id(&self) -> u32 {
        0
    }

    fn set_command_id(&mut self, _id: u32) {}
}

impl CommandIDConfig<u64> for () {
    fn command_id(&self) -> u64 {
        0
    }

    fn set_command_id(&mut self, _id: u64) {}
}

impl<const C: Command, D: Clone + Copy> CommandIDConfig<u32> for Request<C, D> {
    fn command_id(&self) -> u32 {
        self.header.command_id
    }

    fn set_command_id(&mut self, id: u32) {
        self.header.command_id = id;
    }
}

impl<const C: Command, S> CommandIDConfig<u32> for Response<C, S> {
    fn command_id(&self) -> u32 {
        self.header.command_id
    }

    fn set_command_id(&mut self, id: u32) {
        self.header.command_id = id;
    }
}

impl From<GetterSetterStatus> for RobotResult<()> {
    fn from(value: GetterSetterStatus) -> Self {
        match value {
            GetterSetterStatus::Success => Ok(()),
            GetterSetterStatus::CommandNotPossibleRejected => {
                Err(RobotException::UnprocessableInstructionError(
                    "command rejected: command not possible in current mode".to_string(),
                ))
            }
            GetterSetterStatus::InvalidArgumentRejected => Err(RobotException::InvalidInstruction(
                "command rejected: invalid argument".to_string(),
            )),
            GetterSetterStatus::CommandRejectedDueToActivatedSafetyFunctions => Err(
                RobotException::CommandException("command failed".to_string()),
            ),
        }
    }
}

impl<const N: usize> From<MotionType<N>> for MoveData {
    fn from(value: MotionType<N>) -> Self {
        MoveData {
            controller_mode: MoveControllerMode::JointImpedance,
            motion_generator_mode: match value {
                MotionType::Joint(_) => MoveMotionGeneratorMode::JointPosition,
                MotionType::JointVel(_) => MoveMotionGeneratorMode::JointVelocity,
                MotionType::Cartesian(_) => MoveMotionGeneratorMode::CartesianPosition,
                MotionType::CartesianVel(_) => MoveMotionGeneratorMode::CartesianVelocity,
                _ => MoveMotionGeneratorMode::JointPosition,
            },
            maximum_path_deviation: MoveDeviation::default(),
            maximum_goal_deviation: MoveDeviation::default(),
        }
    }
}

impl<const N: usize> From<ControlType<N>> for MoveData {
    fn from(value: ControlType<N>) -> Self {
        MoveData {
            controller_mode: match value {
                ControlType::Torque(_) => MoveControllerMode::ExternalController,
                ControlType::Zero => MoveControllerMode::JointImpedance,
            },
            motion_generator_mode: MoveMotionGeneratorMode::JointVelocity,
            maximum_path_deviation: MoveDeviation::default(),
            maximum_goal_deviation: MoveDeviation::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn type_serde_test() {
        // assert_eq!(
        //     bincode::serialize(&connect_response).unwrap(),
        //     [0u8, 0, 0, 0, 0, 0, 0, 0, 15, 0, 0, 0, 0, 5, 0]
        // );
        assert_eq!(ConnectResponse::size(), 15);
        if let Err(e) = bincode::deserialize::<ConnectResponse>(&[
            0u8, 0, 0, 0, 0, 0, 0, 0, 15, 0, 0, 0, 0, 5, 0,
        ]) {
            println!("{e:?}");
        }
        // assert!(
        //     bincode::deserialize::<ConnectResponse>(&[
        //         0u8, 0, 0, 0, 0, 0, 0, 0, 15, 0, 0, 0, 0, 5, 0
        //     ])
        //     .is_ok()
        // );
    }

    #[test]
    fn type_size() {
        println!("{}", LoadModelLibraryResponse::size());
    }
}
