use std::f64::consts::PI;

pub const FRANKA_GRIPPER_VERSION: u16 = 3;
pub const FRANKA_ROBOT_VERSION: u16 = 5;
pub const LIBFRANKA_VERSION: &str = "0.14.0";

pub const FRANKA_EMIKA_DOF: usize = 7;
pub const FRANKA_FREQUENCY: f64 = 1000.0;
pub const FRANKA_ROBOT_DEFAULT_JOINT: [f64; FRANKA_EMIKA_DOF] =
    [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];

pub const PORT_ROBOT_COMMAND: u16 = 1337;
pub const PORT_GRIPPER_COMMAND: u16 = 1338;
pub const PORT_ROBOT_UDP: u16 = 61337;
pub const PORT_GRIPPER_UDP: u16 = 61338;
