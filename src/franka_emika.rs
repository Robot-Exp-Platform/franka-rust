use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use robot_behavior::behavior::*;

use crate::{FRANKA_DOF, FrankaRobot, robot::FrankaType};

#[derive(Default)]
pub struct _FrankaEmika;

impl FrankaType for _FrankaEmika {}

pub type FrankaEmika = FrankaRobot<_FrankaEmika>;

impl ArmParam<FRANKA_DOF> for FrankaEmika {
    const DH: [[f64; 4]; FRANKA_DOF] = [
        [0., 0.333, 0., 0.],
        [0., 0., 0., -FRAC_PI_2],
        [0., 0.316, 0., FRAC_PI_2],
        [0., 0., 0.0825, FRAC_PI_2],
        [0., 0.384, -0.0825, -FRAC_PI_2],
        [0., 0., 0., FRAC_PI_2],
        [0., 0., 0.088, FRAC_PI_2],
    ];
    const JOINT_DEFAULT: [f64; FRANKA_DOF] = [
        0.,
        -FRAC_PI_4,
        0.,
        -3. * FRAC_PI_4,
        0.,
        FRAC_PI_2,
        FRAC_PI_4,
    ];
    const JOINT_MIN: [f64; FRANKA_DOF] = [
        -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
    ];
    const JOINT_MAX: [f64; FRANKA_DOF] = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973];
    const JOINT_VEL_BOUND: [f64; FRANKA_DOF] =
        [2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100];
    const JOINT_ACC_BOUND: [f64; FRANKA_DOF] = [15., 7.5, 10., 12.5, 15., 20., 20.];
    const JOINT_JERK_BOUND: [f64; FRANKA_DOF] = [7500., 3750., 5000., 6250., 7500., 10000., 10000.];
    const CARTESIAN_VEL_BOUND: f64 = 1.7;
    const CARTESIAN_ACC_BOUND: f64 = 13.0;
    const CARTESIAN_JERK_BOUND: f64 = 6500.0;
    const ROTATION_VEL_BOUND: f64 = 2.5;
    const ROTATION_ACC_BOUND: f64 = 25.0;
    const ROTATION_JERK_BOUND: f64 = 12500.;
    const TORQUE_BOUND: [f64; FRANKA_DOF] = [87., 87., 87., 87., 12., 12., 12.];
    const TORQUE_DOT_BOUND: [f64; FRANKA_DOF] = [1000., 1000., 1000., 1000., 1000., 1000., 1000.];
}
