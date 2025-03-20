use std::net::UdpSocket;

use crossbeam::epoch::Pointable;
use franka_rust::PORT_ROBOT_UDP;
use franka_rust::network::Network;
use franka_rust::types::{
    robot_command::RobotCommand,
    robot_state::{RobotState, RobotStateInter},
    robot_types::*,
};
use robot_behavior::RobotResult;

macro_rules! res_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty ; $ret_type:ty) => {
        pub fn $fn_name(network: &mut Network, $arg_name: $arg_type) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> =
                network.tcp_recv_and_send(&mut Request::<$command, $arg_type>::from($arg_name))?;
            Ok(response.status)
        }
    };
}

res_fn!(__connect, { Command::Connect }; state: ConnectStatus; ConnectData);
res_fn!(__move, { Command::Move }; state: MoveStatus; MoveData);
res_fn!(__set_collision_behavior, { Command::SetCollisionBehavior }; state: GetterSetterStatus; SetCollisionBehaviorData);
res_fn!(__set_joint_impedance, { Command::SetJointImpedance }; state: GetterSetterStatus; SetJointImpedanceData);
res_fn!(__set_cartesian_impedance, { Command::SetCartesianImpedance }; state: GetterSetterStatus; SetCartesianImpedanceData);
res_fn!(__set_guiding_mode, { Command::SetGuidingMode }; state: GetterSetterStatus; SetGuidingModeData);
res_fn!(__set_ee_to_k, { Command::SetEEToK }; state: GetterSetterStatus; SetEEToKData);
res_fn!(__set_ne_to_ee, { Command::SetNEToEE }; state: GetterSetterStatus; SetNEToEEData);
res_fn!(__set_load, { Command::SetLoad }; state: GetterSetterStatus; SetLoadData);
res_fn!(__set_fliters, { Command::SetFilters }; state: GetterSetterStatus; SetFiltersData);
res_fn!(__automatic_error_recovery, { Command::AutomaticErrorRecovery }; state: GetterSetterStatus; ());
res_fn!(__stop_move, { Command::StopMove }; state: GetterSetterStatus;());
res_fn!(__get_cartesian_limit, { Command::GetCartesianLimit }; state: GetCartesianLimitStatus;GetCartesianLimitData);

fn main() -> RobotResult<()> {
    let mut network = Network::new("0.0.0.0", 0);

    let connect_state = ConnectStatus {
        status: ConnectStatusEnum::Success,
        version: 1,
    };
    let udp_port = __connect(&mut network, connect_state)?.udp_port;

    let move_state = MoveStatus::Success;
    __move(&mut network, move_state)?;

    let udp_socket = UdpSocket::bind(format!("172.16.0.1:{}", udp_port)).unwrap();

    loop {
        let robot_state = RobotState {
            ..RobotState::default()
        };
    }

    Ok(())
}
