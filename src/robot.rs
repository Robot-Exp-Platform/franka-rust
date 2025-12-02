use futures::future::BoxFuture;
use nalgebra as na;
use robot_behavior::{
    utils::path_generate::{self, cartesian_quat_simple_4th_curve},
    *,
};
use std::{
    fs::File,
    io::Write,
    marker::PhantomData,
    path::Path,
    sync::{
        Arc, Mutex,
        atomic::{AtomicBool, Ordering},
    },
    thread::sleep,
    time::Duration,
};

use crate::{
    FRANKA_DOF, FrankaRobotImpl, LIBFRANKA_VERSION,
    command_handle::CommandHandle,
    impedance::{cartesian_impedance, joint_impedance},
    model::{Frame, FrankaModel},
    once::OverrideOnce,
    types::{robot_command::RobotCommand, robot_state::*, robot_types::*},
};

pub trait FrankaType {}

#[derive(Default)]
pub struct FrankaRobot<T: FrankaType> {
    marker: PhantomData<T>,
    pub robot_impl: FrankaRobotImpl,
    // network: Network,
    // command_handle: CommandHandle<RobotCommand, RobotStateInter>,
    // pub robot_state: Arc<RwLock<RobotStateInter>>,
    is_moving: bool,
    coord: OverrideOnce<Coord>,
    max_vel: OverrideOnce<[f64; FRANKA_DOF]>,
    max_acc: OverrideOnce<[f64; FRANKA_DOF]>,
    max_jerk: OverrideOnce<[f64; FRANKA_DOF]>,
    max_cartesian_vel: OverrideOnce<f64>,
    max_cartesian_acc: OverrideOnce<f64>,
}

impl<T: FrankaType> FrankaRobot<T> {}

impl<T: FrankaType> FrankaRobot<T>
where
    FrankaRobot<T>: ArmParam<7>,
{
    pub fn new(ip: &str) -> Self {
        let mut robot = FrankaRobot {
            marker: PhantomData,
            robot_impl: FrankaRobotImpl::new(ip),
            is_moving: false,
            coord: OverrideOnce::new(Coord::OCS),
            max_vel: OverrideOnce::new(Self::JOINT_VEL_BOUND),
            max_acc: OverrideOnce::new(Self::JOINT_ACC_BOUND),
            max_jerk: OverrideOnce::new(Self::JOINT_JERK_BOUND),
            max_cartesian_vel: OverrideOnce::new(Self::CARTESIAN_VEL_BOUND),
            max_cartesian_acc: OverrideOnce::new(Self::CARTESIAN_ACC_BOUND),
        };
        let _ = robot.set_scale(0.1);

        robot
    }

    pub fn connect(&mut self, ip: &str) {
        self.robot_impl = FrankaRobotImpl::new(ip);
        self.is_moving = false;
        self.coord = OverrideOnce::new(Coord::OCS);
        self.max_vel = OverrideOnce::new(Self::JOINT_VEL_BOUND);
        self.max_acc = OverrideOnce::new(Self::JOINT_ACC_BOUND);
        self.max_jerk = OverrideOnce::new(Self::JOINT_JERK_BOUND);
        self.max_cartesian_vel = OverrideOnce::new(Self::CARTESIAN_VEL_BOUND);
        self.max_cartesian_acc = OverrideOnce::new(Self::CARTESIAN_ACC_BOUND);

        let _ = self.set_scale(0.1);
    }

    /// # set_collision_behavior
    /// **Set the collision behavior of the robot.**
    ///
    /// Set separate torque and force boundaries for acceleration/deceleration and constant velocity
    /// movement phases.
    ///
    /// Forces or torques between lower and upper threshold are shown as contacts in the RobotState.
    /// Forces or torques above the upper threshold are registered as collision and cause the robot to
    /// stop moving.
    pub fn set_collision_behavior(&mut self, data: SetCollisionBehaviorData) -> RobotResult<()> {
        self.robot_impl._set_collision_behavior(data)?.into()
    }

    /// # set_joint_impedance
    /// **Sets the impedance for each joint in the internal controller.**
    pub fn set_joint_impedance(&mut self, data: SetJointImpedanceData) -> RobotResult<()> {
        self.robot_impl._set_joint_impedance(data)?.into()
    }

    /// # set_cartesian_impedance
    /// **Sets the impedance for the Cartesian internal controller.**
    pub fn set_cartesian_impedance(&mut self, data: SetCartesianImpedanceData) -> RobotResult<()> {
        self.robot_impl._set_cartesian_impedance(data)?.into()
    }

    /// # set_cartesian_limit
    /// **Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).**
    /// If a flag is set to true, movement is unlocked.
    pub fn set_guiding_mode(&mut self, data: SetGuidingModeData) -> RobotResult<()> {
        self.robot_impl._set_guiding_mode(data)?.into()
    }

    pub fn set_ee_to_k(&mut self, data: SetEEToKData) -> RobotResult<()> {
        self.robot_impl._set_ee_to_k(data)?.into()
    }

    pub fn set_ne_to_ee(&mut self, data: SetNEToEEData) -> RobotResult<()> {
        self.robot_impl._set_ne_to_ee(data)?.into()
    }

    // /// # set_load
    // /// **Sets the load of the robot.**
    // /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    // /// This is not for setting end effector parameters, which have to be set in the administrator's
    // /// interface.
    // pub fn set_load(&mut self, data: SetLoadData) -> RobotResult<()> {
    //     self._set_load(data)?.into()
    // }

    #[deprecated(note = "please use `low_pass_filter` instead")]
    pub fn set_filters(&mut self, data: SetFiltersData) -> RobotResult<()> {
        self.robot_impl._set_fliters(data)?.into()
    }

    pub fn read_franka_state(&mut self) -> RobotResult<RobotState> {
        let state = self.robot_impl.robot_state.read().unwrap();
        Ok((*state).into())
    }

    pub fn set_default_behavior(&mut self) -> RobotResult<()> {
        self.set_collision_behavior(SetCollisionBehaviorData::default())?;
        self.set_joint_impedance(SetJointImpedanceData::default())?;
        self.set_cartesian_impedance(SetCartesianImpedanceData::default())?;
        Ok(())
    }

    fn download_library(&mut self, download_path: &Path) -> RobotResult<()> {
        let mut req: LoadModelLibraryRequest = LoadModelLibraryData::default().into();
        let (res, library_buffer) = self
            .robot_impl
            .network
            .tcp_send_and_recv_buffer::<_, LoadModelLibraryResponse>(&mut req)?;

        if res.status != LoadModelLibraryStatus::Success {
            return Err(RobotException::ModelException(
                "Error downloading model library".to_string(),
            ));
        }

        let mut file = File::create(download_path).map_err(|_| {
            RobotException::ModelException("Error writing model to disk:".to_string())
        })?;

        file.write_all(&library_buffer).map_err(|_| {
            RobotException::ModelException("Error writing model to disk:".to_string())
        })?;

        Ok(())
    }

    pub fn model(&mut self) -> RobotResult<FrankaModel> {
        let model_path = if cfg!(target_os = "linux") {
            "/tmp/model.so"
        } else if cfg!(target_os = "windows") {
            "C:\\tmp\\model.dll"
        } else {
            return Err(RobotException::ModelException(
                "Your platform is not yet supported for Downloading models.".to_string(),
            ));
        };
        let model_path = Path::new(model_path);
        if !model_path.exists() {
            self.download_library(model_path)?;
        }
        let model = FrankaModel::new(model_path)?;
        Ok(model)
    }

    pub fn model_from_path(&mut self, model_path: &Path) -> RobotResult<FrankaModel> {
        if !model_path.exists() {
            self.download_library(model_path)?;
        }
        let model = FrankaModel::new(model_path)?;
        Ok(model)
    }
}

impl<T: FrankaType> Robot for FrankaRobot<T> {
    type State = RobotState;
    fn version() -> String {
        format!("FrankaRobot v{LIBFRANKA_VERSION}")
    }

    fn init(&mut self) -> RobotResult<()> {
        Ok(())
    }

    fn shutdown(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn enable(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn disable(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn reset(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn is_moving(&mut self) -> RobotResult<bool> {
        self.robot_impl.is_moving()
    }

    fn waiting_for_finish(&mut self) -> RobotResult<()> {
        self.robot_impl.waiting_for_finish()?;
        self.is_moving = false;
        Ok(())
    }

    fn stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn pause(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn resume(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        let state = self.robot_impl.robot_state.read().unwrap();
        Ok((*state).into())
    }
}

impl<T: FrankaType> Arm<7> for FrankaRobot<T>
where
    FrankaRobot<T>: ArmParam<7>,
{
    fn state(&mut self) -> RobotResult<ArmState<FRANKA_DOF>> {
        let state = self.robot_impl.robot_state.read().unwrap();
        Ok((*state).into())
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        self.robot_impl._set_load(load.into()).map(|_| ())
    }
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }
    fn set_scale(&mut self, scale: f64) -> RobotResult<()> {
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_jerk.set(Self::JOINT_JERK_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * scale);
        Ok(())
    }

    fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }
    fn with_scale(&mut self, scale: f64) -> &mut Self {
        self.max_vel.once(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.once(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_jerk
            .once(Self::JOINT_JERK_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .once(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .once(Self::CARTESIAN_ACC_BOUND * scale);
        self
    }
    fn with_velocity(&mut self, joint_vel: &[f64; FRANKA_DOF]) -> &mut Self {
        self.max_vel.once(*joint_vel);
        self
    }
    fn with_acceleration(&mut self, joint_acc: &[f64; FRANKA_DOF]) -> &mut Self {
        self.max_acc.once(*joint_acc);
        self
    }
    fn with_jerk(&mut self, joint_jerk: &[f64; FRANKA_DOF]) -> &mut Self {
        self.max_jerk.once(*joint_jerk);
        self
    }
    fn with_cartesian_velocity(&mut self, cartesian_vel: f64) -> &mut Self {
        self.max_cartesian_vel.once(cartesian_vel);
        self
    }
    fn with_cartesian_acceleration(&mut self, cartesian_acc: f64) -> &mut Self {
        self.max_cartesian_acc.once(cartesian_acc);
        self
    }
    fn with_cartesian_jerk(&mut self, _cartesian_jerk: f64) -> &mut Self {
        self
    }
    fn with_rotation_velocity(&mut self, rotation_vel: f64) -> &mut Self {
        self.max_cartesian_vel.once(rotation_vel);
        self
    }
    fn with_rotation_acceleration(&mut self, rotation_acc: f64) -> &mut Self {
        self.max_cartesian_acc.once(rotation_acc);
        self
    }
    fn with_rotation_jerk(&mut self, _rotation_jerk: f64) -> &mut Self {
        self
    }
}

impl<T: FrankaType> ArmParam<7> for FrankaRobot<T>
where
    T: ArmParam<7>,
{
    const JOINT_DEFAULT: [f64; FRANKA_DOF] = T::JOINT_DEFAULT;
    const JOINT_MIN: [f64; FRANKA_DOF] = T::JOINT_MIN;
    const JOINT_MAX: [f64; FRANKA_DOF] = T::JOINT_MAX;
    const JOINT_VEL_BOUND: [f64; FRANKA_DOF] = T::JOINT_VEL_BOUND;
    const JOINT_ACC_BOUND: [f64; FRANKA_DOF] = T::JOINT_ACC_BOUND;
    const JOINT_JERK_BOUND: [f64; FRANKA_DOF] = T::JOINT_JERK_BOUND;
    const CARTESIAN_VEL_BOUND: f64 = T::CARTESIAN_VEL_BOUND;
    const CARTESIAN_ACC_BOUND: f64 = T::CARTESIAN_ACC_BOUND;
    const CARTESIAN_JERK_BOUND: f64 = T::CARTESIAN_JERK_BOUND;
    const ROTATION_VEL_BOUND: f64 = T::ROTATION_VEL_BOUND;
    const ROTATION_ACC_BOUND: f64 = T::ROTATION_ACC_BOUND;
    const ROTATION_JERK_BOUND: f64 = T::ROTATION_JERK_BOUND;
    const TORQUE_BOUND: [f64; FRANKA_DOF] = T::TORQUE_BOUND;
    const TORQUE_DOT_BOUND: [f64; FRANKA_DOF] = T::TORQUE_DOT_BOUND;
}

impl<T: FrankaType> ArmPreplannedMotion<7> for FrankaRobot<T>
where
    FrankaRobot<T>: ArmParam<7>,
{
    fn move_joint(&mut self, target: &[f64; FRANKA_DOF]) -> RobotResult<()> {
        self.move_joint_async(target)?;

        self.waiting_for_finish()
    }
    fn move_joint_async(&mut self, target: &[f64; FRANKA_DOF]) -> RobotResult<()> {
        self.is_moving = true;
        self.robot_impl._move(MotionType::Joint(*target).into())?;
        sleep(Duration::from_millis(2));

        let state = self.robot_impl.robot_state.read().unwrap();
        let joint = state.q_d;
        drop(state);
        let target = match self.coord.get() {
            Coord::Relative => {
                let mut result = [0.0; FRANKA_DOF];
                for i in 0..FRANKA_DOF {
                    result[i] = joint[i] + target[i];
                }
                result
            }
            _ => *target,
        };

        let (v_max, a_max, j_max) = (self.max_vel.get(), self.max_acc.get(), self.max_jerk.get());
        let (path_generate, t_max) =
            path_generate::joint_s_curve(&joint, &target, v_max, a_max, j_max);
        // let path_generate = path_generate::joint_trapezoid(&joint, &target, v_max, a_max);

        let mut duration = Duration::from_millis(0);
        self.robot_impl.command_handle.set_closure(move |_, d| {
            duration += d;
            (MotionType::Joint(path_generate(duration)), duration > t_max).into()
        });
        Ok(())
    }
    fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()> {
        self.move_cartesian_async(target)?;

        self.waiting_for_finish()
    }
    fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()> {
        self.is_moving = true;
        self.robot_impl
            ._move(MotionType::<7>::Cartesian(*target).into())?;
        sleep(Duration::from_millis(1));

        let target = *target;
        let state = self.robot_impl.robot_state.read().unwrap();
        let pose: Pose = state.O_T_EE.into();
        drop(state);

        let target = match self.coord.get() {
            Coord::Relative => pose * target,
            &Coord::Inertial => Pose::Position(pose.position()) * target,
            _ => target,
        };

        let (v_max, a_max) = (self.max_cartesian_vel.get(), self.max_cartesian_acc.get());
        let (path_generate, t_max) =
            cartesian_quat_simple_4th_curve(pose.quat(), target.quat(), *v_max, *a_max);

        let mut duration = Duration::from_millis(0);
        self.robot_impl.command_handle.set_closure(move |_, d| {
            duration += d;
            (
                MotionType::Cartesian(path_generate(duration).into()),
                duration > t_max,
            )
                .into()
        });
        Ok(())
    }
}

impl<T: FrankaType> ArmPreplannedPath<7> for FrankaRobot<T>
where
    Self: ArmParam<7>,
{
    fn move_traj(&mut self, path: Vec<MotionType<FRANKA_DOF>>) -> RobotResult<()> {
        self.move_traj_async(path)?;

        self.waiting_for_finish()
    }
    fn move_traj_async(&mut self, path: Vec<MotionType<FRANKA_DOF>>) -> RobotResult<()> {
        self.is_moving = true;
        let coord = self.coord.get().to_owned();
        let state = self.state()?;
        self.with_coord(coord);

        self.move_to(path[0])?;
        let last = path.last().cloned().unwrap_or(MotionType::Stop);
        let mut path = path.into_iter();
        self.robot_impl.command_handle.set_closure(move |_, _| {
            if let Some(next) = path.next() {
                (next.with_coord(&coord, &state), false)
            } else {
                (last, true)
            }
            .into()
        });
        Ok(())
    }
}

pub struct FrankaHandle {
    command_handle: CommandHandle<RobotCommand, RobotStateInter>,
    last_motion: Option<MotionType<FRANKA_DOF>>,
    last_control: Option<ControlType<FRANKA_DOF>>,
}

impl ArmStreamingHandle<FRANKA_DOF> for FrankaHandle {
    fn move_to(&mut self, target: MotionType<FRANKA_DOF>) -> RobotResult<()> {
        self.command_handle.set_target((target, false));
        Ok(())
    }
    fn control_with(&mut self, control: ControlType<FRANKA_DOF>) -> RobotResult<()> {
        self.command_handle.set_target((control, false));
        Ok(())
    }

    fn last_control(&self) -> Option<ControlType<FRANKA_DOF>> {
        self.last_control
    }
    fn last_motion(&self) -> Option<MotionType<FRANKA_DOF>> {
        self.last_motion
    }
}

impl<T: FrankaType> ArmStreamingMotion<7> for FrankaRobot<T>
where
    Self: ArmParam<7>,
{
    type Handle = FrankaHandle;
    fn start_streaming(&mut self) -> RobotResult<Self::Handle> {
        self.is_moving = true;
        self.robot_impl._move(MotionType::Joint([0.0; 7]).into())?;
        sleep(Duration::from_millis(2));
        let state = self.robot_impl.robot_state.read().unwrap();
        self.robot_impl
            .command_handle
            .set_target((MotionType::Joint(state.q_d), false));

        Ok(FrankaHandle {
            command_handle: self.robot_impl.command_handle.clone(),
            last_motion: None,
            last_control: None,
        })
    }
    fn end_streaming(&mut self) -> RobotResult<()> {
        self.stop()
    }
    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<FRANKA_DOF>>>> {
        unimplemented!()
    }
    fn control_with_target(&mut self) -> Arc<Mutex<Option<ControlType<FRANKA_DOF>>>> {
        unimplemented!()
    }
}

impl<T: FrankaType> ArmImpedance<7> for FrankaRobot<T>
where
    Self: ArmParam<7>,
{
    fn joint_impedance_async(
        &mut self,
        stiffness: &[f64; 7],
        damping: &[f64; 7],
    ) -> RobotResult<JointImpedanceHandle<7>> {
        let handle = JointImpedanceHandle {
            stiffness: Arc::new(Mutex::new(*stiffness)),
            damping: Arc::new(Mutex::new(*damping)),
            target: Arc::new(Mutex::new(None)),
            is_finished: Arc::new(AtomicBool::new(false)),
        };

        let stiffness_clone = handle.stiffness.clone();
        let damping_clone = handle.damping.clone();
        let target_clone = handle.target.clone();
        let is_finished_clone = handle.is_finished.clone();
        let _ = self.control_with_closure(move |state, _| {
            let q = state.joint.unwrap();
            let dq = state.joint_vel.unwrap();
            let target = {
                let t = target_clone.lock().unwrap();
                t.unwrap_or(state.joint.unwrap())
            };
            let stiffness = {
                let s = stiffness_clone.lock().unwrap();
                *s
            };
            let damping = {
                let d = damping_clone.lock().unwrap();
                *d
            };
            let torque = joint_impedance(&stiffness, &damping, target, q, dq);
            (
                ControlType::Torque(torque),
                is_finished_clone.load(Ordering::SeqCst),
            )
        });

        Ok(handle)
    }
    fn cartesian_impedance_async(
        &mut self,
        stiffness: (f64, f64),
        damping: (f64, f64),
    ) -> RobotResult<CartesianImpedanceHandle> {
        let handle = CartesianImpedanceHandle {
            stiffness: Arc::new(Mutex::new(stiffness)),
            damping: Arc::new(Mutex::new(damping)),
            target: Arc::new(Mutex::new(None)),
            is_finished: Arc::new(AtomicBool::new(false)),
        };

        let stiffness_clone = handle.stiffness.clone();
        let damping_clone = handle.damping.clone();
        let target_clone = handle.target.clone();
        let is_finished_clone = handle.is_finished.clone();
        let model = self.model()?;

        let _ = self.control_with_closure(move |state, _| {
            let pose = state.pose_o_to_ee.unwrap();
            let dq = state.joint_vel.unwrap();
            let target = {
                let t = target_clone.lock().unwrap();
                t.unwrap_or(pose)
            };
            let stiffness = {
                let s = stiffness_clone.lock().unwrap();
                *s
            };
            let damping = {
                let d = damping_clone.lock().unwrap();
                *d
            };
            let jacobian = na::SMatrix::<f64, 6, 7>::from_column_slice(
                &model.zero_jacobian_from_arm_state(&Frame::EndEffector, &state),
            );
            let force_torque =
                cartesian_impedance(stiffness, damping, target.quat(), jacobian, dq, pose.quat());
            (
                ControlType::Torque(force_torque),
                is_finished_clone.load(Ordering::SeqCst),
            )
        });

        Ok(handle)
    }
    fn joint_impedance_control(
        &mut self,
        stiffness: &[f64; 7],
        damping: &[f64; 7],
    ) -> RobotResult<(
        JointImpedanceHandle<7>,
        Box<dyn FnMut() -> BoxFuture<'static, RobotResult<()>> + Send + 'static>,
    )> {
        let handle = self.joint_impedance_async(stiffness, damping)?;
        let is_finished = handle.is_finished.clone();
        let robot = self.robot_impl.clone();

        let closure = Box::new(move || {
            let mut robot = robot.clone();
            let is_finished = is_finished.clone();
            Box::pin(async move {
                let mut stop_sent = false;
                // Wait for robot to start moving
                let start_wait = std::time::Instant::now();
                while !robot.is_moving()? {
                    if start_wait.elapsed() > Duration::from_secs(2) {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }

                while robot.is_moving()? {
                    if is_finished.load(Ordering::SeqCst) && !stop_sent {
                        let _ = robot._stop_move(());
                        stop_sent = true;
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
                robot.command_handle.remove_closure();
                let _ = robot.network.tcp_blocking_recv::<MoveResponse>();
                Ok(())
            }) as BoxFuture<'static, RobotResult<()>>
        });
        Ok((handle, closure))
    }

    fn cartesian_impedance_control(
        &mut self,
        stiffness: (f64, f64),
        damping: (f64, f64),
    ) -> RobotResult<(
        CartesianImpedanceHandle,
        Box<dyn FnMut() -> BoxFuture<'static, RobotResult<()>> + Send + 'static>,
    )> {
        let handle = self.cartesian_impedance_async(stiffness, damping)?;
        let robot = self.robot_impl.clone();
        let is_finished = handle.is_finished.clone();

        let closure = Box::new(move || {
            let mut robot = robot.clone();
            let is_finished = is_finished.clone();
            Box::pin(async move {
                let mut stop_sent = false;
                // Wait for robot to start moving
                let start_wait = std::time::Instant::now();
                while !robot.is_moving()? {
                    if start_wait.elapsed() > Duration::from_secs(2) {
                        break;
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
                while robot.is_moving()? {
                    if is_finished.load(Ordering::SeqCst) && !stop_sent {
                        let _ = robot._stop_move(());
                        stop_sent = true;
                    }
                    tokio::time::sleep(Duration::from_millis(1)).await;
                }
                robot.command_handle.remove_closure();
                let _ = robot.network.tcp_blocking_recv::<MoveResponse>();
                Ok(())
            }) as BoxFuture<'static, RobotResult<()>>
        });
        Ok((handle, closure))
    }
}

impl<T: FrankaType> Realtime for FrankaRobot<T> {}

impl<T: FrankaType> ArmRealtimeControl<7> for FrankaRobot<T>
where
    Self: ArmParam<7>,
{
    fn move_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<FRANKA_DOF>, Duration) -> (MotionType<FRANKA_DOF>, bool)
            + Send
            + 'static,
    {
        self.is_moving = true;
        let example = ArmState::<FRANKA_DOF>::default();
        self.robot_impl
            ._move(closure(example, Duration::from_millis(0)).0.into())?;
        self.robot_impl
            .command_handle
            .set_closure(move |state, duration| closure((*state).into(), duration).into());
        Ok(())
    }

    fn control_with_closure<FC>(&mut self, mut closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<FRANKA_DOF>, Duration) -> (ControlType<FRANKA_DOF>, bool)
            + Send
            + 'static,
    {
        self.is_moving = true;
        let example = ArmState::<FRANKA_DOF>::default();
        self.robot_impl
            ._move(closure(example, Duration::from_millis(0)).0.into())?;
        self.robot_impl
            .command_handle
            .set_closure(move |state, duration| closure((*state).into(), duration).into());
        Ok(())
    }
}

impl<T: FrankaType> ArmRealtimeControlExt<FRANKA_DOF> for FrankaRobot<T> where
    Self: ArmParam<FRANKA_DOF>
{
}
