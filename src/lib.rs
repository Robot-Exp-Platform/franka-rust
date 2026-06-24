#![feature(adt_const_params)]
#![allow(incomplete_features)]
#![feature(async_trait_bounds)]
#![feature(generic_const_exprs)]

mod gripper;
// mod logger;
#[cfg(feature = "ffi")]
pub mod ffi;
pub mod franka;
pub mod model;
mod network;
pub mod once;
mod params;
mod realtime;
mod robot;
mod robot_impl;
pub mod types;
pub mod utils;
mod vncuum_gripper;

pub use franka::*;
pub use gripper::FrankaGripper;
pub use params::*;
pub use robot::FrankaRobot;
pub use robot_impl::FrankaRobotImpl;
#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod franka_rust {
    #[pymodule_export]
    use super::ffi::to_py::{PyFrankaEmika, PyFrankaFR3, PyFrankaGripper, PyFrankaModel};
    #[pymodule_export]
    use robot_behavior::{
        LoadState, PyArmState, PyJointSample, PyJointState, PyMotionType, PyPose, PySpatialSample,
        PySpatialState,
    };
}
