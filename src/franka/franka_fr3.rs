use std::f64::consts::{FRAC_PI_2, FRAC_PI_4};

use robot_behavior::{DhParam, behavior::*, mdh_param};

use crate::{FRANKA_DOF, FrankaRobot, robot::FrankaType};

#[derive(Default)]
pub struct _FrankaFR3;

impl FrankaType for _FrankaFR3 {}

pub type FrankaFR3 = FrankaRobot<_FrankaFR3>;

impl ArmParam<FRANKA_DOF> for _FrankaFR3 {
    const JOINT_DEFAULT: [f64; FRANKA_DOF] = [
        0.,
        -FRAC_PI_4,
        0.,
        -3. * FRAC_PI_4,
        0.,
        FRAC_PI_2,
        FRAC_PI_4,
    ];
    #[cfg(not(feature = "conservative_constraint"))]
    const JOINT_MIN: [f64; FRANKA_DOF] =
        [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159];
    #[cfg(feature = "conservative_constraint")]
    const JOINT_MIN: [f64; FrankaEmika::N] =
        [-2.3093, -1.5133, -2.4937, -2.7478, -2.4800, 0.8521, -2.6895];
    #[cfg(not(feature = "conservative_constraint"))]
    const JOINT_MAX: [f64; FRANKA_DOF] = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159];
    #[cfg(feature = "conservative_constraint")]
    const JOINT_MAX: [f64; FrankaEmika::N] =
        [2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094, 2.6895];
    #[cfg(not(feature = "conservative_constraint"))]
    const JOINT_VEL_BOUND: [f64; FRANKA_DOF] = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26];
    #[cfg(feature = "conservative_constraint")]
    const JOINT_VEL_BOUND: [f64; FRANKA_DOF] = [2., 1., 1.5, 1.25, 3., 1.5, 3.];
    const JOINT_ACC_BOUND: [f64; FRANKA_DOF] = [10.; FRANKA_DOF];
    const JOINT_JERK_BOUND: [f64; FRANKA_DOF] = [5000.; FRANKA_DOF];
    const CARTESIAN_VEL_BOUND: f64 = 3.0;
    const CARTESIAN_ACC_BOUND: f64 = 9.0;
    const CARTESIAN_JERK_BOUND: f64 = 4500.0;
    const ROTATION_VEL_BOUND: f64 = 2.5;
    const ROTATION_ACC_BOUND: f64 = 217.0;
    const ROTATION_JERK_BOUND: f64 = 8500.;
    const TORQUE_BOUND: [f64; FRANKA_DOF] = [87., 87., 87., 87., 12., 12., 12.];
    const TORQUE_DOT_BOUND: [f64; FRANKA_DOF] = [1000., 1000., 1000., 1000., 1000., 1000., 1000.];
}

impl ArmForwardKinematics<FRANKA_DOF> for FrankaFR3 {
    const DH: [DhParam; FRANKA_DOF] = [
        mdh_param!(0., 0.333, 0., 0.),
        mdh_param!(0., 0., 0., -FRAC_PI_2),
        mdh_param!(0., 0.316, 0., FRAC_PI_2),
        mdh_param!(0., 0., 0.0825, FRAC_PI_2),
        mdh_param!(0., 0.384, -0.0825, -FRAC_PI_2),
        mdh_param!(0., 0., 0., FRAC_PI_2),
        mdh_param!(0., 0., 0.088, FRAC_PI_2),
    ];
}
