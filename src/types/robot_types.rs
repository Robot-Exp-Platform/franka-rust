#![allow(dead_code)]

use robot_behavior::RobotException;
use serde::ser::SerializeStruct;
use serde::{Deserialize, Serialize, Serializer};
use serde_repr::{Deserialize_repr, Serialize_repr};
use std::marker::ConstParamTy;

use crate::exception::FrankaResult;

#[derive(ConstParamTy, PartialEq, Eq, Serialize_repr, Deserialize_repr)]
#[repr(u8)]
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

#[derive(Default, PartialEq)]
pub struct CommandHeader<const C: Command> {
    pub command_id: u32,
    pub size: u32,
}

#[derive(Default, Serialize, PartialEq)]
pub struct Request<const C: Command, D> {
    pub header: CommandHeader<C>,
    pub data: D,
}

#[derive(Default, Deserialize)]
pub struct Response<const C: Command, S> {
    pub header: CommandHeader<C>,
    pub status: S,
}

pub trait CommandIDConfig {
    fn command_id(&self) -> u32;
    fn set_command_id(&mut self, id: u32);
}

#[derive(Deserialize_repr)]
#[repr(u8)]
pub enum DefaultStatus {
    Success,
    CommandNotPossibleRejected,
    CommandRejectedDueToActivatedSafetyFunctions,
}

#[derive(Deserialize_repr)]
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

#[derive(Default, Serialize, PartialEq)]
pub struct ConnectData {
    pub version: u16,
    pub udp_port: u16,
}

#[derive(Deserialize_repr)]
#[repr(u8)]
pub enum ConnectStatus {
    Success,
    IncompatibleLibraryVersion,
}

// ! Move Command
pub type MoveRequest = Request<{ Command::Move }, MoveData>;
pub type MoveResponse = Response<{ Command::Move }, MoveStatus>;

#[derive(Serialize_repr)]
#[repr(u8)]
pub enum MoveControllerMode {
    JointImpedance,
    CartesianImpedance,
    ExternalController,
}

#[derive(Serialize_repr)]
#[repr(u8)]
pub enum MoveMotionGeneratorMode {
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
}

#[derive(Default, Serialize)]
pub struct MoveDeviation {
    translation: f64,
    rotation: f64,
    elbow: f64,
}

#[derive(Serialize)]
pub struct MoveData {
    controller_mode: MoveControllerMode,
    motion_generator_mode: MoveMotionGeneratorMode,
    maximum_path_deviation: MoveDeviation,
    maximum_goal_deviation: MoveDeviation,
}

#[derive(Deserialize_repr)]
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

#[derive(Deserialize_repr)]
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
    Response<{ Command::GetCartesianLimit }, GetCartesianLimitResponseData>;

#[derive(Default, Serialize)]
pub struct GetCartesianLimitData {
    id: i32,
}

#[derive(Default, Serialize)]
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

#[derive(Default, Serialize)]
pub struct SetCollisionBehaviorData {
    lower_torque_thresholds_acceleration: [f64; 7],
    upper_torque_thresholds_acceleration: [f64; 7],
    lower_torque_thresholds_nominal: [f64; 7],
    upper_torque_thresholds_nominal: [f64; 7],
    lower_force_thresholds_acceleration: [f64; 6],
    upper_force_thresholds_acceleration: [f64; 6],
    lower_force_thresholds_nominal: [f64; 6],
    upper_force_thresholds_nominal: [f64; 6],
}

// ! SetJointImpedance Command
pub type SetJointImpedanceRequest = Request<{ Command::SetJointImpedance }, SetJointImpedanceData>;
pub type SetJointImpedanceResponse = Response<{ Command::SetJointImpedance }, GetterSetterStatus>;

#[derive(Default, Serialize)]
pub struct SetJointImpedanceData {
    k_theta: [f64; 7],
}

// ! SetCartesianImpedance Command
pub type SetCartesianImpedanceRequest =
    Request<{ Command::SetCartesianImpedance }, SetCartesianImpedanceData>;
pub type SetCartesianImpedanceResponse =
    Response<{ Command::SetCartesianImpedance }, GetterSetterStatus>;

#[derive(Default, Serialize)]
pub struct SetCartesianImpedanceData {
    k_x: [f64; 6],
}

// ! SetGuidingMode Command
pub type SetGuidingModeRequest = Request<{ Command::SetGuidingMode }, SetGuidingModeData>;
pub type SetGuidingModeResponse = Response<{ Command::SetGuidingMode }, GetterSetterStatus>;

#[derive(Default, Serialize)]
pub struct SetGuidingModeData {
    guiding_mode: [bool; 6],
    nullspace: bool,
}

// ! SetEEToK Command
pub type SetEEToKRequest = Request<{ Command::SetEEToK }, SetEEToKData>;
pub type SetEEToKResponse = Response<{ Command::SetEEToK }, GetterSetterStatus>;

#[derive(Default, Serialize)]
pub struct SetEEToKData {
    pose_ee_to_k: [f64; 16],
}

// ! SetNEToEE Command
pub type SetNEToEERequest = Request<{ Command::SetNEToEE }, SetNEToEEData>;
pub type SetNEToEEResponse = Response<{ Command::SetNEToEE }, GetterSetterStatus>;

#[derive(Default, Serialize)]
pub struct SetNEToEEData {
    pose_ne_to_ee: [f64; 16],
}

// ! SetLoad Command
pub type SetLoadRequest = Request<{ Command::SetLoad }, SetLoadData>;
pub type SetLoadResponse = Response<{ Command::SetLoad }, GetterSetterStatus>;

#[derive(Default, Serialize)]
pub struct SetLoadData {
    m_load: f64,
    x_cload: [f64; 3],
    i_load: [f64; 9],
}

// ! SetFilters Command
pub type SetFiltersRequest = Request<{ Command::SetFilters }, SetFiltersData>;
pub type SetFiltersResponse = Response<{ Command::SetFilters }, GetterSetterStatus>;

#[derive(Default, Serialize)]
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

#[derive(Deserialize_repr)]
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

#[derive(Serialize_repr)]
#[repr(u8)]
pub enum LoadModelLibraryArchitecture {
    X64,
    X86,
    Arm,
    Arm64,
}

#[derive(Serialize_repr)]
#[repr(u8)]
pub enum LoadModelLibrarySystem {
    Linux,
    Windows,
}

#[derive(Serialize)]
pub struct LoadModelLibraryData {
    architecture: LoadModelLibraryArchitecture,
    system: LoadModelLibrarySystem,
}

#[derive(Deserialize_repr)]
#[repr(u8)]
pub enum LoadModelLibraryStatus {
    Success,
    Error,
}

// ! GetRobotModel Command
pub type GetRobotModelRequest = Request<{ Command::GetRobotModel }, ()>;
pub type GetRobotModelResponse = Response<{ Command::GetRobotModel }, DefaultStatus>;

impl<const C: Command, D> Request<C, D> {
    pub fn size() -> usize {
        std::mem::size_of::<Request<C, D>>() + 2
    }
}

impl<const C: Command, S> Response<C, S> {
    pub fn size() -> usize {
        std::mem::size_of::<Response<C, S>>() + 2
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

impl<const C: Command, D> From<D> for Request<C, D> {
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

impl<const C: Command, R> CommandIDConfig for Request<C, R> {
    fn command_id(&self) -> u32 {
        self.header.command_id
    }

    fn set_command_id(&mut self, id: u32) {
        self.header.command_id = id;
    }
}

impl<const C: Command, S> CommandIDConfig for Response<C, S> {
    fn command_id(&self) -> u32 {
        self.header.command_id
    }

    fn set_command_id(&mut self, id: u32) {
        self.header.command_id = id;
    }
}

impl Into<FrankaResult<()>> for GetterSetterStatus {
    fn into(self) -> FrankaResult<()> {
        match self {
            GetterSetterStatus::Success => Ok(()),
            GetterSetterStatus::CommandNotPossibleRejected => {
                Err(RobotException::UnprocessableInstructionError(
                    "command rejected: command not possible in current mode".to_string(),
                ))
            }
            GetterSetterStatus::InvalidArgumentRejected => Err(RobotException::InvalidInstruction(
                "command rejected: invalid argument".to_string(),
            )),
            _ => Err(RobotException::CommandException(
                "command failed".to_string(),
            )),
        }
    }
}
