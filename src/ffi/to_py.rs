use pyo3::{
    Bound, PyResult,
    exceptions::PyException,
    pyclass, pymethods, pymodule,
    types::{PyModule, PyModuleMethods},
};
use robot_behavior::{ArmBehaviorExt, RobotException, ffi::PyArmState7};

use crate::{FRANKA_EMIKA_DOF, FrankaGripper, FrankaRobot, types::robot_types::*};

#[pyclass(name = "FrankaRobot")]
pub struct PyFrankaRobot(FrankaRobot);

#[pyclass(name = "FrankaGripper")]
pub struct PyFrankaGripper(FrankaGripper);

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

    fn set_load(&mut self, m_load: f64, x_load: [f64; 3], i_load: [f64; 9]) -> PyResult<()> {
        let data = SetLoadData {
            m_load,
            x_load,
            i_load,
        };
        self.0.set_load(data).map_err(map_err)
    }

    // fn read_state(&mut self) -> PyResult<PyRobotState> {
    //     Ok(PyRobotState(self.0.read_state().map_err(map_err)?))
    // }

    fn arm_state(&mut self) -> PyResult<PyArmState7> {
        self.0.arm_state().map_err(map_err).map(PyArmState7::from)
    }

    fn set_default_behavior(&mut self) -> PyResult<()> {
        self.0.set_default_behavior().map_err(map_err)
    }

    // fn model(&mut self) -> PyResult<Model> {
    //     self.0.model().map_err(map_err)
    // }

    // fn move_to(&mut self, target: MotionType<FRANKA_EMIKA_DOF>, speed: f64) -> PyResult<()> {
    //     self.0.move_to(target, speed).map_err(map_err)
    // }

    fn move_joint(&mut self, target: [f64; FRANKA_EMIKA_DOF], speed: f64) -> PyResult<()> {
        self.0.move_joint(&target, speed).map_err(map_err)
    }

    // fn move_linear_with_quat(
    //     &mut self,
    //     target: &nalgebra::Isometry3<f64>,
    //     speed: f64,
    // ) -> PyResult<()> {
    //     self.0
    //         .move_linear_with_quat(target.into(), speed)
    //         .map_err(map_err)
    // }

    fn move_linear_with_homo(&mut self, target: [f64; 16], speed: f64) -> PyResult<()> {
        self.0
            .move_linear_with_homo(&target, speed)
            .map_err(map_err)
    }

    fn move_linear_with_euler(&mut self, target: [f64; 6], speed: f64) -> PyResult<()> {
        self.0
            .move_linear_with_euler(&target, speed)
            .map_err(map_err)
    }

    // fn control_with_closure<
    //     FC: Fn(ArmState<FRANKA_EMIKA_DOF>, Duration) -> ControlType<FRANKA_EMIKA_DOF> + Send + 'static,
    // >(
    //     &mut self,
    //     closure: FC,
    // ) -> PyResult<()> {
    //     self.0.control_with_closure(closure).map_err(map_err)
    // }
}

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

#[pymodule]
fn franka_rust(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyFrankaRobot>()?;
    m.add_class::<PyFrankaGripper>()?;
    m.add_class::<PyArmState7>()?;
    Ok(())
}

fn map_err(e: RobotException) -> pyo3::PyErr {
    PyException::new_err(e.to_string())
}
