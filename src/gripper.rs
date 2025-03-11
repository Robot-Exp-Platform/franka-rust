use robot_behavior::{RobotException, RobotResult};
use std::sync::{Arc, RwLock};

use crate::{
    FRANKA_GRIPPER_VERSION, PORT_GRIPPER_COMMAND, PORT_GRIPPER_UDP,
    network::Network,
    types::{gripper_state::*, gripper_types::*},
};

/// # FrankaGripper
///
pub struct FrankaGripper {
    network: Network,
    gripper_state: Arc<RwLock<GripperStateInter>>,
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

impl FrankaGripper {
    pub fn new(ip: &str) -> Self {
        let (_, gripper_state) = Network::spawn_udp_thread::<(), _>(PORT_GRIPPER_UDP);
        let mut gripper = FrankaGripper {
            network: Network::new(ip, PORT_GRIPPER_COMMAND),
            gripper_state,
        };
        gripper.connect().unwrap();
        gripper
    }

    cmd_fn!(_connect, { Command::Connect}; data: ConnectData; ConnectStatus);
    cmd_fn!(_homing, { Command::Homing}; data: (); HomingStatus);
    cmd_fn!(_grasp, { Command::Grasp}; data: GraspData; GraspStatus);
    cmd_fn!(_move, { Command::Move}; data: MoveData; MoveStatus);
    cmd_fn!(_stop, { Command::Stop}; data: (); StopStatus);

    pub fn connect(&mut self) -> RobotResult<()> {
        let result = self._connect(ConnectData {
            version: FRANKA_GRIPPER_VERSION,
            udp_port: PORT_GRIPPER_UDP,
        })?;
        if let Status::Success = result.status {
            Ok(())
        } else {
            Err(RobotException::IncompatibleVersionException {
                server_version: result.version as u64,
                client_version: FRANKA_GRIPPER_VERSION as u64,
            })
        }
    }

    pub fn homing(&mut self) -> RobotResult<bool> {
        self._homing(())?.into()
    }

    pub fn grasp(&mut self, width: f64, speed: f64, force: f64) -> RobotResult<bool> {
        let data = GraspData {
            width,
            epsilon: (0.01, 0.01),
            speed,
            force,
        };
        self._grasp(data)?.into()
    }

    pub fn move_gripper(&mut self, width: f64, speed: f64) -> RobotResult<bool> {
        let data = MoveData { width, speed };
        self._move(data)?.into()
    }

    pub fn stop(&mut self) -> RobotResult<bool> {
        self._stop(())?.into()
    }

    pub fn read_state(&mut self) -> RobotResult<GripperState> {
        let state = self.gripper_state.read().unwrap();
        Ok((*state).into())
    }
}
