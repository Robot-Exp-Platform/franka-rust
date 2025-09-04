use pyo3::{PyResult, exceptions::PyException, pyclass, pymethods, types::PyAnyMethods};
use robot_behavior::{
    RobotException, behavior::*, py_arm_behavior, py_arm_param, py_arm_preplanned_motion,
    py_arm_preplanned_motion_ext, py_arm_preplanned_motion_impl, py_arm_real_time_control,
    py_arm_real_time_control_ext, py_arm_streaming_handle, py_arm_streaming_motion,
    py_robot_behavior,
};

use crate::{
    FRANKA_EMIKA_DOF, FrankaGripper, FrankaRobot, model::FrankaModel, robot::FrankaHandle,
    types::robot_types::*,
};

#[pyclass(name = "FrankaRobot")]
pub struct PyFrankaRobot(FrankaRobot);

#[pyclass(name = "FrankaHandle")]
pub struct PyFrankaHandle(FrankaHandle);

#[pyclass(name = "FrankaGripper")]
pub struct PyFrankaGripper(FrankaGripper);

#[pyclass(name = "RobotModel")]
pub struct PyFrankaModel(FrankaModel);

// #[pyclass(name = "RobotState")]
// pub struct PyRobotState(RobotState);

#[pymethods]
impl PyFrankaRobot {
    #[new]
    fn new(ip: &str) -> Self {
        PyFrankaRobot(FrankaRobot::new(ip))
    }

    fn __repr__(&self) -> String {
        "FrankaRobot".to_string()
    }

    #[allow(clippy::too_many_arguments)]
    fn set_collision_behavior(
        &mut self,
        lower_torque_thresholds_acceleration: [f64; 7],
        upper_torque_thresholds_acceleration: [f64; 7],
        lower_torque_thresholds_nominal: [f64; 7],
        upper_torque_thresholds_nominal: [f64; 7],
        lower_force_thresholds_acceleration: [f64; 6],
        upper_force_thresholds_acceleration: [f64; 6],
        lower_force_thresholds_nominal: [f64; 6],
        upper_force_thresholds_nominal: [f64; 6],
    ) -> PyResult<()> {
        let data = SetCollisionBehaviorData {
            lower_torque_thresholds_acceleration,
            upper_torque_thresholds_acceleration,
            lower_torque_thresholds_nominal,
            upper_torque_thresholds_nominal,
            lower_force_thresholds_acceleration,
            upper_force_thresholds_acceleration,
            lower_force_thresholds_nominal,
            upper_force_thresholds_nominal,
        };
        self.0.set_collision_behavior(data).map_err(map_err)
    }

    fn set_joint_impedance(&mut self, data: [f64; 7]) -> PyResult<()> {
        self.0.set_joint_impedance(data.into()).map_err(map_err)
    }

    fn set_cartesian_impedance(&mut self, data: [f64; 6]) -> PyResult<()> {
        self.0.set_cartesian_impedance(data.into()).map_err(map_err)
    }

    fn set_guiding_mode(&mut self, guiding_mode: [bool; 6], nullspace: bool) -> PyResult<()> {
        let data = SetGuidingModeData {
            guiding_mode,
            nullspace,
        };
        self.0.set_guiding_mode(data).map_err(map_err)
    }

    fn set_ee_to_k(&mut self, pose: [f64; 16]) -> PyResult<()> {
        self.0.set_ee_to_k(pose.into()).map_err(map_err)
    }

    fn set_ne_to_ee(&mut self, pose: [f64; 16]) -> PyResult<()> {
        self.0.set_ne_to_ee(pose.into()).map_err(map_err)
    }

    fn set_default_behavior(&mut self) -> PyResult<()> {
        self.0.set_default_behavior().map_err(map_err)
    }

    fn model(&mut self) -> PyResult<PyFrankaModel> {
        Ok(PyFrankaModel(self.0.model().map_err(map_err)?))
    }
}

#[pymethods]
impl PyFrankaHandle {}

impl From<FrankaHandle> for PyFrankaHandle {
    fn from(handle: FrankaHandle) -> Self {
        PyFrankaHandle(handle)
    }
}

py_robot_behavior!(PyFrankaRobot(FrankaRobot));
py_arm_behavior!(PyFrankaRobot<{7}>(FrankaRobot));
py_arm_param!(PyFrankaRobot<{7}>(FrankaRobot));
py_arm_preplanned_motion!(PyFrankaRobot<{7}>(FrankaRobot));
py_arm_preplanned_motion_impl!(PyFrankaRobot<{7}>(FrankaRobot));
py_arm_preplanned_motion_ext!(PyFrankaRobot<{7}>(FrankaRobot));
py_arm_streaming_handle!(PyFrankaHandle<{7}>(FrankaHandle));
py_arm_streaming_motion!(PyFrankaRobot<{7}>(FrankaRobot) -> PyFrankaHandle);
py_arm_real_time_control!(PyFrankaRobot<{7}>(FrankaRobot));
py_arm_real_time_control_ext!(PyFrankaRobot<{7}>(FrankaRobot));

#[pymethods]
impl PyFrankaGripper {
    #[new]
    fn new(ip: &str) -> Self {
        PyFrankaGripper(FrankaGripper::new(ip))
    }

    fn __repr__(&self) -> String {
        "FrankaGripper".to_string()
    }

    fn homing(&mut self) -> PyResult<bool> {
        self.0.homing().map_err(map_err)
    }

    fn grasp(&mut self, width: f64, speed: f64, force: f64) -> PyResult<bool> {
        self.0.grasp(width, speed, force).map_err(map_err)
    }

    fn move_gripper(&mut self, width: f64, speed: f64) -> PyResult<bool> {
        self.0.move_gripper(width, speed).map_err(map_err)
    }

    fn stop(&mut self) -> PyResult<bool> {
        self.0.stop().map_err(map_err)
    }

    // fn read_state(&mut self) -> PyResult<GripperState> {
    //     self.0.read_state().map_err(map_err)
    // }
}

#[pymethods]
impl PyFrankaModel {
    #[new]
    fn new(path: &str) -> PyResult<Self> {
        Ok(PyFrankaModel(FrankaModel::new(path).map_err(map_err)?))
    }

    fn __repr__(&self) -> String {
        "FrankaModel".to_string()
    }

    fn coriolis(
        &self,
        q: [f64; FRANKA_EMIKA_DOF],
        dq: [f64; FRANKA_EMIKA_DOF],
        m: f64,
        x: [f64; 3],
        i: [f64; 9],
    ) -> PyResult<[f64; FRANKA_EMIKA_DOF]> {
        Ok(self.0.coriolis(&q, &dq, &i, m, &x))
    }

    fn gravity(
        &self,
        q: [f64; FRANKA_EMIKA_DOF],
        m: f64,
        x: [f64; 3],
        gravity: [f64; 3],
    ) -> PyResult<[f64; FRANKA_EMIKA_DOF]> {
        Ok(self.0.gravity(&q, m, &x, &gravity))
    }
}

fn map_err(e: RobotException) -> pyo3::PyErr {
    PyException::new_err(e.to_string())
}
