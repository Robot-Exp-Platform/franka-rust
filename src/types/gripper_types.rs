#![allow(dead_code)]

use robot_behavior::{RobotException, RobotResult};
use serde::{Deserialize, Deserializer, Serialize, Serializer, ser::SerializeStruct};
use serde_repr::{Deserialize_repr, Serialize_repr};
use std::marker::ConstParamTy;

use super::robot_types::CommandIDConfig;

#[derive(Debug, ConstParamTy, PartialEq, Eq, Serialize_repr, Deserialize_repr, Copy, Clone)]
#[repr(u16)]
pub enum Command {
    Connect,
    Homing,
    Grasp,
    Move,
    Stop,
}

#[derive(Serialize_repr, Deserialize_repr, Debug)]
#[repr(u16)]
pub enum Status {
    Success,
    Fail,
    Unsuccessful,
    Aborted,
}

#[derive(Debug, Default, PartialEq)]
pub struct CommandHeader<const C: Command> {
    pub command_id: u32,
    pub size: u32,
}

#[derive(Debug, Default, Serialize, PartialEq)]
pub struct Request<const C: Command, D> {
    pub header: CommandHeader<C>,
    pub data: D,
}

#[derive(Default, Deserialize)]
pub struct Response<const C: Command, S> {
    pub header: CommandHeader<C>,
    pub status: S,
}

// ! Connect Command
pub type ConnectRequest = Request<{ Command::Connect }, ConnectData>;
pub type ConnectResponse = Response<{ Command::Connect }, ConnectStatus>;

#[derive(Debug, Serialize, Deserialize)]
#[repr(packed)]
pub struct ConnectData {
    pub version: u16,
    pub udp_port: u16,
}
#[derive(Serialize, Deserialize)]
pub struct ConnectStatus {
    pub status: Status,
    pub version: u32,
}

// ! Homing Command
pub type HomingRequest = Request<{ Command::Homing }, ()>;
pub type HomingResponse = Response<{ Command::Homing }, HomingStatus>;
pub type HomingStatus = Status;

// ! Grasp Command
pub type GraspRequest = Request<{ Command::Grasp }, GraspData>;
pub type GraspResponse = Response<{ Command::Grasp }, GraspStatus>;
pub type GraspStatus = Status;

#[derive(Debug, Serialize, Deserialize)]
#[repr(packed)]
pub struct GraspData {
    pub width: f64,
    pub epsilon: (f64, f64),
    pub speed: f64,
    pub force: f64,
}

// ! Move Command
pub type MoveRequest = Request<{ Command::Move }, MoveData>;
pub type MoveResponse = Response<{ Command::Move }, MoveStatus>;
pub type MoveStatus = Status;

#[derive(Debug, Serialize, Deserialize)]
#[repr(packed)]
pub struct MoveData {
    pub width: f64,
    pub speed: f64,
}

// ! Stop Command
pub type StopRequest = Request<{ Command::Stop }, ()>;
pub type StopResponse = Response<{ Command::Stop }, StopStatus>;
pub type StopStatus = Status;

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
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct CommandHeaderHelper {
            command_id: u32,
            size: u32,
        }

        let helper = CommandHeaderHelper::deserialize(deserializer)?;
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

impl Into<RobotResult<bool>> for Status {
    fn into(self) -> RobotResult<bool> {
        match self {
            Status::Success => Ok(true),
            Status::Fail => Err(RobotException::CommandException(
                "gripper: Command failed!".to_string(),
            )),
            Status::Unsuccessful => Ok(false),
            Status::Aborted => Err(RobotException::CommandException(
                "gripper: Command aborted!".to_string(),
            )),
        }
    }
}
