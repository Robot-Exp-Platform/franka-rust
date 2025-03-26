#[cfg(test)]
mod gripper_tests {
    use serde::{Deserialize, Serialize};
    use serde_repr::{Deserialize_repr, Serialize_repr};
    use std::mem::size_of;

    use franka_rust::types;

    #[derive(Serialize_repr, Deserialize_repr, Debug, Copy, Clone)]
    #[repr(u16)]
    pub enum Command {
        Connect,
        Homing,
        Grasp,
        Move,
        Stop,
    }

    #[derive(Serialize, Deserialize, Debug, Copy, Clone)]
    #[repr(C, packed)]
    pub struct CommandHeader {
        pub command: Command,
        pub command_id: u32,
        pub size: u32,
    }

    #[derive(Serialize, Deserialize, Debug, Copy, Clone)]
    #[repr(C, packed)]
    pub struct ConnectRequest {
        pub version: u16,
        pub udp_port: u16,
    }

    #[derive(Serialize, Deserialize, Debug, Copy, Clone)]
    #[repr(C, packed)]
    pub struct ConnectRequestWithHeader {
        pub header: CommandHeader,
        pub request: ConnectRequest,
    }

    #[test]
    fn type_size_test() {
        println!(
            "ConnectRequestWithHeader size: {}",
            size_of::<ConnectRequestWithHeader>()
        );

        println!(
            "ConnectRequestWithHeader serialize as bytes: {:?}",
            bincode::serialize(&ConnectRequestWithHeader {
                header: CommandHeader {
                    command: Command::Connect,
                    command_id: 0,
                    size: 14,
                },
                request: ConnectRequest {
                    version: 3,
                    udp_port: 1338,
                },
            })
            .unwrap()
        );

        println!(
            "Self ConnectRequest size: {}",
            types::gripper_types::ConnectRequest::size()
        );

        println!(
            "Self ConnectRequest serialize as bytes: {:?}",
            bincode::serialize(&types::gripper_types::ConnectRequest {
                header: types::gripper_types::CommandHeader {
                    command_id: 0,
                    size: 8,
                },
                data: types::gripper_types::ConnectData {
                    version: 1,
                    udp_port: 2,
                },
            })
        );
    }
}

#[cfg(test)]
mod robot_test {
    use franka_rust::types;
    #[test]
    fn type_size_test() {
        println!(
            "Self ConnectRequest size: {}",
            types::robot_types::ConnectRequest::size()
        );

        println!(
            "Self ConnectRequest serialize as bytes: {:?}",
            bincode::serialize(&types::robot_types::ConnectRequest {
                header: types::robot_types::CommandHeader {
                    command_id: 0,
                    size: 8,
                },
                data: types::robot_types::ConnectData {
                    version: 1,
                    udp_port: 2,
                },
            })
        );
    }
}
