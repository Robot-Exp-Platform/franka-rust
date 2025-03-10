use robot_behavior::RobotException;

use crate::{
    FRANKA_GRIPPER_VERSION, PORT_GRIPPER_COMMAND,
    exception::FrankaResult,
    network::Network,
    types::{
        gripper_state::{GripperState, GripperStateInter},
        gripper_types::*,
    },
};

/// # FrankaGripper
///
pub struct FrankaGripper {
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

impl FrankaGripper {
    pub fn new(ip: &str) -> Self {
        let mut gripper = FrankaGripper {
            network: Network::new(ip, PORT_GRIPPER_COMMAND),
        };
        gripper.connect().unwrap();
        gripper
    }

    cmd_fn!(_connect, { Command::Connect}; data: ConnectData; ConnectStatus);
    pub fn connect(&mut self) -> FrankaResult<()> {
        let result = self._connect(ConnectData {
            version: FRANKA_GRIPPER_VERSION,
            udp_port: self.network.udp_port(),
        })?;
        println!("udp_port: {}", self.network.udp_port());
        println!("Gripper version: {}", result.version);
        if let Status::Success = result.status {
            Ok(())
        } else {
            Err(RobotException::IncompatibleVersionException {
                server_version: result.version as u64,
                client_version: FRANKA_GRIPPER_VERSION as u64,
            })
        }
    }

    cmd_fn!(_homing, { Command::Homing}; data: (); HomingStatus);
    pub fn homing(&mut self) -> FrankaResult<bool> {
        self._homing(())?.into()
    }

    cmd_fn!(_grasp, { Command::Grasp}; data: GraspData; GraspStatus);
    pub fn grasp(&mut self, width: f64, speed: f64, force: f64) -> FrankaResult<bool> {
        let data = GraspData {
            width,
            epsilon: (0.01, 0.01),
            speed,
            force,
        };
        self._grasp(data)?.into()
    }

    cmd_fn!(_move, { Command::Move}; data: MoveData; MoveStatus);
    pub fn move_gripper(&mut self, width: f64, speed: f64) -> FrankaResult<bool> {
        let data = MoveData { width, speed };
        self._move(data)?.into()
    }

    cmd_fn!(_stop, { Command::Stop}; data: (); StopStatus);
    pub fn stop(&mut self) -> FrankaResult<bool> {
        self._stop(())?.into()
    }

    pub fn read_state(&mut self) -> FrankaResult<GripperState> {
        self.network
            .udp_recv::<GripperStateInter>()
            .map(GripperStateInter::into)
    }
}
