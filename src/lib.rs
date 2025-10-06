#![feature(adt_const_params)]

mod gripper;
// mod logger;
mod command_handle;
#[cfg(feature = "ffi")]
pub mod ffi;
pub mod franka_emika;
pub mod franka_fr3;
pub mod model;
mod network;
pub mod once;
mod params;
mod robot;
pub mod types;
pub mod utils;

pub use franka_emika::*;
pub use franka_fr3::*;
pub use gripper::FrankaGripper;
pub use params::*;
pub use robot::FrankaRobot;

#[cfg(feature = "to_py")]
#[pyo3::pymodule]
mod franka_rust {
    #[pymodule_export]
    use super::ffi::to_py::{PyFrankaEmika, PyFrankaFR3, PyFrankaGripper, PyFrankaModel};
    #[pymodule_export]
    use robot_behavior::{LoadState, PyArmState, PyControlType, PyMotionType, PyPose};
}
