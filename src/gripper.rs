use roplat_behavior::{Gripper, RobotException, RobotResult, msgs};
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
        Self::open(ip, PORT_GRIPPER_COMMAND)
    }

    fn open(ip: &str, tcp_port: u16) -> Self {
        let udp_socket = UdpSocket::bind(("0.0.0.0", PORT_GRIPPER_UDP))
            .or_else(|_| UdpSocket::bind(("0.0.0.0", 0)))
            .unwrap();
        let udp_port = udp_socket.local_addr().unwrap().port();
        let mut gripper = FrankaGripper {
            network: Network::new(ip, tcp_port).for_gripper(),
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

/// The shared gripper contract. A positive `force` grasps (`grasp`, with
/// libfranka's default ±0.01 m epsilon); otherwise the fingers only move. An
/// unsuccessful grasp is not an error: `read_gripper().grasped` reports it.
/// An unsuccessful move, homing or stop is, since nothing else reports it.
impl Gripper for FrankaGripper {
    fn read_gripper(&mut self) -> RobotResult<msgs::GripperState> {
        let state = self.read_state()?;
        Ok(msgs::GripperState {
            width: state.width,
            max_width: state.max_width,
            grasped: state.is_grasped as u32,
            reserved: 0,
        })
    }

    fn move_gripper(&mut self, command: msgs::GripperCommand) -> RobotResult<()> {
        if command.force > 0.0 {
            self.grasp(command.width, command.speed, command.force)?;
            Ok(())
        } else {
            let reached = FrankaGripper::move_gripper(self, command.width, command.speed)?;
            completed(reached, "move")
        }
    }

    fn home_gripper(&mut self) -> RobotResult<()> {
        let homed = self.homing()?;
        completed(homed, "homing")
    }

    fn stop_gripper(&mut self) -> RobotResult<()> {
        let stopped = self.stop()?;
        completed(stopped, "stop")
    }
}

fn completed(success: bool, what: &str) -> RobotResult<()> {
    if success {
        Ok(())
    } else {
        Err(RobotException::CommandException(format!(
            "gripper: {what} was unsuccessful"
        )))
    }
}

#[cfg(test)]
mod tests {
    use std::{
        io::{Read, Write},
        net::{TcpListener, UdpSocket},
        sync::mpsc,
        thread,
    };

    use super::*;
    use crate::types::gripper_types::Command;

    /// A gripper server speaking libfranka's packed little-endian layout:
    /// header `command: u16, command_id: u32, size: u32`, then the payload or
    /// the `u16` status (Connect adds a `u16` version). It answers each request
    /// with the next status and reports what it received.
    fn serve(statuses: Vec<u16>) -> (u16, mpsc::Receiver<(u16, Vec<u8>)>) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = listener.local_addr().unwrap().port();
        let (sender, receiver) = mpsc::channel();
        thread::spawn(move || {
            let (mut stream, _) = listener.accept().unwrap();
            for status in statuses {
                let mut header = [0u8; 10];
                stream.read_exact(&mut header).unwrap();
                let command = u16::from_le_bytes([header[0], header[1]]);
                let id = u32::from_le_bytes(header[2..6].try_into().unwrap());
                let size = u32::from_le_bytes(header[6..10].try_into().unwrap()) as usize;
                let mut payload = vec![0; size - 10];
                stream.read_exact(&mut payload).unwrap();

                let mut body = status.to_le_bytes().to_vec();
                if command == Command::Connect as u16 {
                    body.extend(FRANKA_GRIPPER_VERSION.to_le_bytes());
                    // The state stream goes to the UDP port named in Connect.
                    let udp_port = u16::from_le_bytes([payload[2], payload[3]]);
                    let mut state = 7u32.to_le_bytes().to_vec();
                    state.extend(0.042f64.to_le_bytes());
                    state.extend(0.08f64.to_le_bytes());
                    state.push(1);
                    state.extend(31u16.to_le_bytes());
                    UdpSocket::bind("127.0.0.1:0")
                        .unwrap()
                        .send_to(&state, ("127.0.0.1", udp_port))
                        .unwrap();
                }
                let mut response = command.to_le_bytes().to_vec();
                response.extend(id.to_le_bytes());
                response.extend((10 + body.len() as u32).to_le_bytes());
                response.extend(body);
                stream.write_all(&response).unwrap();
                sender.send((command, payload)).unwrap();
            }
        });
        (port, receiver)
    }

    fn f64s(payload: &[u8]) -> Vec<f64> {
        payload
            .chunks(8)
            .map(|c| f64::from_le_bytes(c.try_into().unwrap()))
            .collect()
    }

    const SUCCESS: u16 = 0;
    const FAIL: u16 = 1;
    const UNSUCCESSFUL: u16 = 2;

    #[test]
    fn gripper_contract_follows_the_wire_statuses() {
        let (port, requests) = serve(vec![
            SUCCESS,
            SUCCESS,
            SUCCESS,
            UNSUCCESSFUL,
            UNSUCCESSFUL,
            FAIL,
        ]);
        let mut gripper = FrankaGripper::open("127.0.0.1", port);
        assert_eq!(requests.recv().unwrap().0, Command::Connect as u16);

        let state = gripper.read_gripper().unwrap();
        assert_eq!(
            state,
            msgs::GripperState { width: 0.042, max_width: 0.08, grasped: 1, reserved: 0 }
        );

        gripper.home_gripper().unwrap();
        assert_eq!(requests.recv().unwrap(), (Command::Homing as u16, vec![]));

        let open = msgs::GripperCommand { width: 0.07, speed: 0.1, force: 0.0 };
        Gripper::move_gripper(&mut gripper, open).unwrap();
        let (command, payload) = requests.recv().unwrap();
        assert_eq!(
            (command, f64s(&payload)),
            (Command::Move as u16, vec![0.07, 0.1])
        );

        // An unsuccessful grasp is an outcome, not an error.
        let grasp = msgs::GripperCommand { width: 0.02, speed: 0.05, force: 40.0 };
        Gripper::move_gripper(&mut gripper, grasp).unwrap();
        let (command, payload) = requests.recv().unwrap();
        assert_eq!(command, Command::Grasp as u16);
        assert_eq!(f64s(&payload), [0.02, 0.01, 0.01, 0.05, 40.0]);

        assert!(Gripper::move_gripper(&mut gripper, open).is_err());
        assert!(gripper.stop_gripper().is_err());
        assert_eq!(requests.recv().unwrap().0, Command::Move as u16);
        assert_eq!(requests.recv().unwrap().0, Command::Stop as u16);
    }
}
