use std::f64::consts::PI;

pub const FRANKA_GRIPPER_VERSION: u16 = 3;
pub const FRANKA_ROBOT_VERSION: u16 = 5;
pub const LIBFRANKA_VERSION: &str = "0.14.0";

pub const PORT_ROBOT_COMMAND: u16 = 1337;
pub const PORT_GRIPPER_COMMAND: u16 = 1338;
pub const PORT_ROBOT_UDP: u16 = 31337;
pub const PORT_GRIPPER_UDP: u16 = 31338;

pub const FRANKA_EMIKA_DOF: usize = 7;
pub const FRANKA_FREQUENCY: f64 = 1000.0;
pub const FRANKA_ROBOT_DEFAULT_JOINT: [f64; FRANKA_EMIKA_DOF] =
    [0., -PI / 4., 0., -3. * PI / 4., 0., PI / 2., PI / 4.];
pub const FRANKA_ROBOT_MIN_JOINT: [f64; FRANKA_EMIKA_DOF] = [
    -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
];
pub const FRANKA_ROBOT_MAX_JOINT: [f64; FRANKA_EMIKA_DOF] =
    [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
pub const FRANKA_ROBOT_MAX_JOINT_VEL: [f64; FRANKA_EMIKA_DOF] =
    [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
pub const FRANKA_ROBOT_MAX_JOINT_ACC: [f64; FRANKA_EMIKA_DOF] =
    [15., 7.5, 10., 12.5, 15., 20., 20.];
pub const FRANKA_ROBOT_MAX_JOINT_JERK: [f64; FRANKA_EMIKA_DOF] =
    [7500., 3750., 5000., 6250., 7500., 10000., 10000.];
pub const FRANKA_ROBOT_MAX_TORQUE: [f64; FRANKA_EMIKA_DOF] = [87., 87., 87., 87., 12., 12., 12.];
pub const FRANKA_ROBOT_MAX_TORQUE_RATE: [f64; FRANKA_EMIKA_DOF] =
    [1000., 1000., 1000., 1000., 1000., 1000., 1000.];
