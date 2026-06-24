use robot_behavior::{RobotException, RobotResult};
use std::net::UdpSocket;

use crate::{
    FRANKA_GRIPPER_VERSION, PORT_GRIPPER_COMMAND, PORT_GRIPPER_UDP,
    network::Network,
    types::{gripper_state::*, gripper_types::*},
};

/// # FrankaGripper
///
pub struct FrankaGripper {
    network: Network,
    udp_socket: UdpSocket,
    gripper_state: GripperStateInter,
    udp_port: u16,
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
        let udp_socket = UdpSocket::bind(("0.0.0.0", PORT_GRIPPER_UDP))
            .or_else(|_| UdpSocket::bind(("0.0.0.0", 0)))
            .unwrap();
        let udp_port = udp_socket.local_addr().unwrap().port();
        let mut gripper = FrankaGripper {
            network: Network::new(ip, PORT_GRIPPER_COMMAND),
            udp_socket,
            gripper_state: GripperStateInter::default(),
            udp_port,
        };
        gripper.connect_with_udp_port(udp_port).unwrap();
        gripper
    }

    cmd_fn!(_connect, { Command::Connect}; data: ConnectData; ConnectStatus);
    cmd_fn!(_homing, { Command::Homing}; data: (); HomingStatus);
    cmd_fn!(_grasp, { Command::Grasp}; data: GraspData; GraspStatus);
    cmd_fn!(_move, { Command::Move}; data: MoveData; MoveStatus);
    cmd_fn!(_stop, { Command::Stop}; data: (); StopStatus);

    pub fn connect(&mut self) -> RobotResult<()> {
        self.connect_with_udp_port(self.udp_port)
    }

    fn connect_with_udp_port(&mut self, udp_port: u16) -> RobotResult<()> {
        let result = self._connect(ConnectData { version: FRANKA_GRIPPER_VERSION, udp_port })?;
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
        let data = GraspData { width, epsilon: (0.01, 0.01), speed, force };
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
        let mut buffer = vec![0_u8; std::mem::size_of::<GripperStateInter>() * 5];
        let (size, _) = self.udp_socket.recv_from(&mut buffer)?;
        let state: GripperStateInter = bincode::deserialize(&buffer[..size])
            .map_err(|err| RobotException::DeserializeError(err.to_string()))?;
        self.gripper_state = state;
        Ok(state.into())
    }
}
