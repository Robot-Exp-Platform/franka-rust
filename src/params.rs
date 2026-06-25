pub const FRANKA_GRIPPER_VERSION: u16 = 3;

// FCI robot protocol version negotiated in the Connect command. Selected by the
// `fci_v8` feature so the wire protocol stays in sync with the `MoveStatus`
// layout (see `types::robot_types::MoveStatus`).
#[cfg(not(feature = "fci_v8"))]
pub const FRANKA_ROBOT_VERSION: u16 = 5;
#[cfg(feature = "fci_v8")]
pub const FRANKA_ROBOT_VERSION: u16 = 8;

#[cfg(not(feature = "fci_v8"))]
pub const LIBFRANKA_VERSION: &str = "0.9.2";
#[cfg(feature = "fci_v8")]
pub const LIBFRANKA_VERSION: &str = "0.14.0";

pub const PORT_ROBOT_COMMAND: u16 = 1337;
pub const PORT_GRIPPER_COMMAND: u16 = 1338;
pub const PORT_ROBOT_UDP: u16 = 61337;
pub const PORT_GRIPPER_UDP: u16 = 61338;

pub const FRANKA_FREQUENCY: f64 = 1000.0;
pub const FRANKA_DOF: usize = 7;
