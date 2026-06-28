use robot_behavior::{utils::path_generate, *};
use std::{
    fs::File,
    io::Write,
    marker::PhantomData,
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{
    FRANKA_DOF, FrankaRobotImpl, LIBFRANKA_VERSION,
    model::FrankaModel,
    once::OverrideOnce,
    types::{robot_state::*, robot_types::*},
};

pub trait FrankaType {
    const JOINT_NAMES: [&'static str; FRANKA_DOF];
}

type FrankaControlObservers = Arc<Mutex<Vec<ControlObserver<RobotState>>>>;

fn control_observers() -> FrankaControlObservers {
    Arc::new(Mutex::new(Vec::new()))
}

fn has_control_observers(observers: &FrankaControlObservers) -> bool {
    !observers
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner())
        .is_empty()
}

fn notify_control_observers(
    observers: &FrankaControlObservers,
    state: &RobotState,
    duration: Duration,
) {
    let mut observers = observers
        .lock()
        .unwrap_or_else(|poisoned| poisoned.into_inner());
    for observer in observers.iter_mut() {
        observer(state, duration);
    }
}

pub struct FrankaRobot<T: FrankaType> {
    marker: PhantomData<T>,
    pub robot_impl: FrankaRobotImpl,
    is_moving: bool,
    before_observers: FrankaControlObservers,
    after_observers: FrankaControlObservers,
    coord: OverrideOnce<Coord>,
    scale: OverrideOnce<f64>,
    max_vel: OverrideOnce<[f64; FRANKA_DOF]>,
    max_acc: OverrideOnce<[f64; FRANKA_DOF]>,
    max_jerk: OverrideOnce<[f64; FRANKA_DOF]>,
    max_cartesian_vel: OverrideOnce<f64>,
    max_cartesian_acc: OverrideOnce<f64>,
}

impl<T: FrankaType> FrankaRobot<T>
where
    FrankaRobot<T>: Joints<7> + EndPoint,
{
    pub fn new(ip: &str) -> Self {
        let mut robot = FrankaRobot {
            marker: PhantomData,
            robot_impl: FrankaRobotImpl::new(ip),
            is_moving: false,
            before_observers: control_observers(),
            after_observers: control_observers(),
            coord: OverrideOnce::new(Coord::OCS),
            scale: OverrideOnce::new(0.1),
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
        self.scale = OverrideOnce::new(0.1);
        self.max_vel = OverrideOnce::new(Self::JOINT_VEL_BOUND.map(|x| x * 0.1));
        self.max_acc = OverrideOnce::new(Self::JOINT_ACC_BOUND.map(|x| x * 0.1));
        self.max_jerk = OverrideOnce::new(Self::JOINT_JERK_BOUND.map(|x| x * 0.1));
        self.max_cartesian_vel = OverrideOnce::new(Self::CARTESIAN_VEL_BOUND * 0.1);
        self.max_cartesian_acc = OverrideOnce::new(Self::CARTESIAN_ACC_BOUND * 0.1);

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

    /// # automatic_error_recovery
    /// **Performs automatic error recovery after a collision or safety stop.**
    pub fn automatic_error_recovery(&mut self) -> RobotResult<()> {
        self.robot_impl._automatic_error_recovery(())?.into()
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
        let (state, _, _) = self.robot_impl.recv_state()?;
        Ok(state.into())
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
    const CONTROL_PERIOD: f64 = 1e-3;

    fn version() -> String {
        format!("FrankaRobot v{LIBFRANKA_VERSION}")
    }

    fn init(&mut self) -> RobotResult<()> {
        Ok(())
    }

    fn shutdown(&mut self) -> RobotResult<()> {
        if self.robot_impl.is_moving().unwrap_or(false) {
            self.stop()?;
        }
        Ok(())
    }

    fn enable(&mut self) -> RobotResult<()> {
        Ok(())
    }

    fn disable(&mut self) -> RobotResult<()> {
        self.stop()
    }

    fn reset(&mut self) -> RobotResult<()> {
        self.robot_impl._automatic_error_recovery(())?.into()
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
        let result: RobotResult<()> = self.robot_impl._stop_move(())?.into();
        self.is_moving = false;
        result
    }

    fn pause(&mut self) -> RobotResult<()> {
        Err(RobotException::UnprocessableInstructionError(
            "Franka FCI does not expose a generic pause operation; use stop()".to_string(),
        ))
    }

    fn resume(&mut self) -> RobotResult<()> {
        Err(RobotException::UnprocessableInstructionError(
            "Franka FCI does not expose a generic resume operation".to_string(),
        ))
    }

    fn emergency_stop(&mut self) -> RobotResult<()> {
        self.stop()
    }

    fn clear_emergency_stop(&mut self) -> RobotResult<()> {
        self.robot_impl._automatic_error_recovery(())?.into()
    }

    fn read_state(&mut self) -> RobotResult<Self::State> {
        let (state, _, _) = self.robot_impl.recv_state()?;
        Ok(state.into())
    }
}

impl<T: FrankaType> ControlObservation for FrankaRobot<T> {
    fn before<H>(&mut self, observer: H) -> &mut Self
    where
        H: FnMut(&Self::State, Duration) + Send + 'static,
    {
        self.before_observers
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(Box::new(observer));
        self
    }

    fn after<H>(&mut self, observer: H) -> &mut Self
    where
        H: FnMut(&Self::State, Duration) + Send + 'static,
    {
        self.after_observers
            .lock()
            .unwrap_or_else(|poisoned| poisoned.into_inner())
            .push(Box::new(observer));
        self
    }
}

impl<T: FrankaType> Arm<7> for FrankaRobot<T>
where
    FrankaRobot<T>: Joints<7> + EndPoint + MoveTo<JointSpace<7>> + MoveTo<FlangeSpace>,
{
    fn state(&mut self) -> RobotResult<ArmState<FRANKA_DOF>> {
        let (state, _, _) = self.robot_impl.recv_state()?;
        Ok(state.into())
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        self.robot_impl._set_load(load.into()).map(|_| ())
    }

    fn get_joint(&self) -> [f64; FRANKA_DOF] {
        self.robot_impl.robot_state.read().unwrap().q
    }

    fn with_joint_vel(mut self, vel_bound: [f64; FRANKA_DOF]) -> Self {
        self.max_vel.once(vel_bound);
        self
    }

    fn with_joint_acc(mut self, acc_bound: [f64; FRANKA_DOF]) -> Self {
        self.max_acc.once(acc_bound);
        self
    }

    fn with_joint_jerk(mut self, jerk_bound: [f64; FRANKA_DOF]) -> Self {
        self.max_jerk.once(jerk_bound);
        self
    }

    fn with_torque(self, _torque_bound: [f64; FRANKA_DOF]) -> Self {
        self
    }

    fn with_torque_dot(self, _torque_dot_bound: [f64; FRANKA_DOF]) -> Self {
        self
    }

    fn get_endpoint(&self) -> Pose {
        Pose::Homo(self.robot_impl.robot_state.read().unwrap().O_T_EE)
    }

    fn with_cartesian_vel(mut self, vel_bound: f64) -> Self {
        self.max_cartesian_vel.once(vel_bound);
        self
    }

    fn with_cartesian_acc(mut self, acc_bound: f64) -> Self {
        self.max_cartesian_acc.once(acc_bound);
        self
    }

    fn with_cartesian_jerk(self, _jerk_bound: f64) -> Self {
        self
    }

    fn with_rotation_vel(mut self, vel_bound: f64) -> Self {
        self.max_cartesian_vel.once(vel_bound);
        self
    }

    fn with_rotation_acc(mut self, acc_bound: f64) -> Self {
        self.max_cartesian_acc.once(acc_bound);
        self
    }

    fn with_rotation_jerk(self, _jerk_bound: f64) -> Self {
        self
    }
}

impl<T: FrankaType> FrankaRobot<T>
where
    Self: Joints<7> + EndPoint,
{
    pub fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }

    pub fn set_scale(&mut self, scale: f64) -> RobotResult<()> {
        self.scale.set(scale);
        self.max_vel.set(Self::JOINT_VEL_BOUND.map(|v| v * scale));
        self.max_acc.set(Self::JOINT_ACC_BOUND.map(|v| v * scale));
        self.max_jerk.set(Self::JOINT_JERK_BOUND.map(|v| v * scale));
        self.max_cartesian_vel
            .set(Self::CARTESIAN_VEL_BOUND * scale);
        self.max_cartesian_acc
            .set(Self::CARTESIAN_ACC_BOUND * scale);
        Ok(())
    }

    pub fn get_scale(&self) -> f64 {
        *self.scale.get()
    }

    pub fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }

    pub fn with_scale(&mut self, scale: f64) -> &mut Self {
        self.scale.once(scale);
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
}

const PID_K: [f64; 7] = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 300.0];
const PID_D: [f64; 7] = [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 6.0];
const PID_I: [f64; 7] = [6.0, 6.0, 6.0, 6.0, 2.0, 1.5, 500.01];

fn sample_joint_trajectory<const N: usize>(
    path: impl Fn(Duration) -> [f64; N],
    t_max: Duration,
) -> Vec<[f64; N]> {
    const DT: Duration = Duration::from_millis(1);
    let steps = (t_max.as_secs_f64() / DT.as_secs_f64()).ceil() as usize;
    let mut traj = Vec::with_capacity(steps + 2);
    for i in 0..=steps {
        traj.push(path((DT * i as u32).min(t_max)));
    }
    traj.push(path(t_max));
    traj
}

impl<T: FrankaType> MoveTo<JointSpace<7>> for FrankaRobot<T>
where
    Self: Joints<7>,
{
    fn move_to(&mut self, target: [f64; FRANKA_DOF]) -> RobotResult<()> {
        let (state, _, _) = self.robot_impl.recv_state()?;
        let joint = state.q_d;
        let target = match self.coord.get() {
            Coord::Relative => {
                let mut result = [0.0; FRANKA_DOF];
                for i in 0..FRANKA_DOF {
                    result[i] = joint[i] + target[i];
                }
                result
            }
            _ => target,
        };

        let (v_max, a_max, j_max) = (self.max_vel.get(), self.max_acc.get(), self.max_jerk.get());

        let (path_generate, t_max) =
            path_generate::joint_s_curve(&joint, &target, v_max, a_max, j_max);

        let traj = sample_joint_trajectory(path_generate.as_ref(), t_max);
        let controller =
            robot_behavior::controller::joint_traj_pid_control(traj, PID_K, PID_I, PID_D);

        <Self as Control>::control_with::<TorqueControl<7>, _>(self, controller)
    }

    fn move_to_async(
        &mut self,
        target: [f64; FRANKA_DOF],
    ) -> impl std::future::Future<Output = RobotResult<()>> {
        async move {
            let state = crate::realtime::tokio_udp::recv_state(&mut self.robot_impl).await?;
            let joint = state.q_d;
            let target = match self.coord.get() {
                Coord::Relative => {
                    let mut result = [0.0; FRANKA_DOF];
                    for i in 0..FRANKA_DOF {
                        result[i] = joint[i] + target[i];
                    }
                    result
                }
                _ => target,
            };

            let (v_max, a_max, j_max) =
                (self.max_vel.get(), self.max_acc.get(), self.max_jerk.get());
            let (path_generate, t_max) =
                path_generate::joint_s_curve(&joint, &target, v_max, a_max, j_max);

            // Same torque-space realisation as the synchronous path: pre-sample
            // the trajectory and run the PID controller per cycle.
            let traj = sample_joint_trajectory(path_generate.as_ref(), t_max);
            let mut controller =
                robot_behavior::controller::joint_traj_pid_control(traj, PID_K, PID_I, PID_D);

            self.is_moving = true;
            let result = crate::realtime::tokio_udp::control_async(
                &mut self.robot_impl,
                ControlType::Torque([0.0; FRANKA_DOF]).into(),
                async move |state, d| {
                    let arm: ArmState<FRANKA_DOF> = state.into();
                    let (torque, done) = controller(arm.joint, d);
                    (ControlType::Torque(torque), done).into()
                },
            )
            .await;
            self.is_moving = false;
            result
        }
    }
}

impl<T: FrankaType> MoveTo<FlangeSpace> for FrankaRobot<T>
where
    Self: EndPoint + ArmInverseKinematics<7>,
    [(); 8]:,
{
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        let (state, _, _) = self.robot_impl.recv_state()?;
        let pose: Pose = state.O_T_EE.into();
        let q_start = state.q_d;

        let target = match self.coord.get() {
            Coord::Relative => pose * target,
            &Coord::Inertial => Pose::Position(pose.position()) * target,
            _ => target,
        };

        let traj = robot_behavior::utils::trajectory::plan_cartesian_line_traj_via_time_scaling::<
            Self,
            _,
            7,
        >(pose, target, q_start, self.get_scale(), |pose, seed| {
            let q0 = JVec::<7>::from_row_slice(&seed);
            let q = <Self as ArmInverseKinematics<7>>::ik_solve(&q0, &pose, Default::default());
            Ok(q.as_slice().try_into().unwrap())
        })?;

        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, traj)
    }

    fn move_to_async(
        &mut self,
        target: Pose,
    ) -> impl std::future::Future<Output = RobotResult<()>> {
        async move {
            let state = crate::realtime::tokio_udp::recv_state(&mut self.robot_impl).await?;
            let pose: Pose = state.O_T_EE.into();
            let q_start = state.q_d;

            let target = match self.coord.get() {
                Coord::Relative => pose * target,
                &Coord::Inertial => Pose::Position(pose.position()) * target,
                _ => target,
            };

            let traj =
                robot_behavior::utils::trajectory::plan_cartesian_line_traj_via_time_scaling::<
                    Self,
                    _,
                    7,
                >(pose, target, q_start, self.get_scale(), |pose, seed| {
                    let q0 = JVec::<7>::from_row_slice(&seed);
                    let q =
                        <Self as ArmInverseKinematics<7>>::ik_solve(&q0, &pose, Default::default());
                    Ok(q.as_slice().try_into().unwrap())
                })?;

            if traj.is_empty() {
                return Ok(());
            }

            let last = *traj.last().unwrap();
            let mut traj = traj.into_iter();
            self.is_moving = true;
            let result = crate::realtime::tokio_udp::control_async(
                &mut self.robot_impl,
                MotionType::<FRANKA_DOF>::Joint(q_start).into(),
                async move |_, _| {
                    if let Some(next) = traj.next() {
                        (MotionType::Joint(next), false).into()
                    } else {
                        (MotionType::Joint(last), true).into()
                    }
                },
            )
            .await;
            self.is_moving = false;
            result
        }
    }
}

impl<T: FrankaType> MoveTraj<JointSpace<7>> for FrankaRobot<T>
where
    Self: Joints<7> + EndPoint,
{
    fn move_traj(&mut self, path: Vec<[f64; FRANKA_DOF]>) -> RobotResult<()> {
        if path.is_empty() {
            return Ok(());
        }

        let last = *path.last().unwrap();
        let mut path = path.into_iter();

        <Self as Control>::control_with::<JointPositionControl<7>, _>(self, move |_, _| {
            if let Some(next) = path.next() {
                (next, false)
            } else {
                (last, true)
            }
        })
    }

    fn move_traj_sync(&mut self, path: Vec<[f64; FRANKA_DOF]>) -> RobotResult<()> {
        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, path)
    }

    fn move_path<F>(&mut self, path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; FRANKA_DOF]>,
    {
        let traj = robot_behavior::utils::trajectory::plan_path_traj_via_copp::<Self, F, FRANKA_DOF>(
            &path,
            self.get_scale(),
        )?;
        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, traj)
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; FRANKA_DOF]>) -> RobotResult<()> {
        let traj = robot_behavior::utils::trajectory::plan_waypoints_traj_via_copp::<
            Self,
            FRANKA_DOF,
        >(&waypoints, self.get_scale())?;
        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, traj)
    }
}

impl<T: FrankaType> ControlWith<JointPositionControl<7>> for FrankaRobot<T>
where
    Self: Joints<7>,
{
    fn hold_command(state: &JointState<FRANKA_DOF>) -> [f64; FRANKA_DOF] {
        state
            .cmd
            .q
            .or(state.des.q)
            .or(state.meas.q)
            .unwrap_or([0.0; FRANKA_DOF])
    }

    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let mode_selector = self.robot_impl.robot_state.read().unwrap().q_d;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control(
            &mut self.robot_impl,
            MotionType::Joint(mode_selector).into(),
            move |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let joint_state: JointState<FRANKA_DOF> = state.into();
                let (joint, done) = closure(joint_state, duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::Joint(joint), done).into()
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(JointState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let mode_selector = self.robot_impl.robot_state.read().unwrap().q_d;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_async(
            &mut self.robot_impl,
            MotionType::Joint(mode_selector).into(),
            async |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let joint_state: JointState<FRANKA_DOF> = state.into();
                let (joint, done) = closure(joint_state, duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::Joint(joint), done).into()
            },
        );
        self.is_moving = false;
        result
    }
}

impl<T: FrankaType> ControlWith<JointVelocityControl<7>> for FrankaRobot<T> {
    fn hold_command(_state: &JointState<FRANKA_DOF>) -> [f64; FRANKA_DOF] {
        [0.0; FRANKA_DOF]
    }

    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control(
            &mut self.robot_impl,
            MotionType::JointVel([0.0; FRANKA_DOF]).into(),
            move |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let joint_state: JointState<FRANKA_DOF> = state.into();
                let (velocity, done) = closure(joint_state, duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::JointVel(velocity), done).into()
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(JointState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_async(
            &mut self.robot_impl,
            MotionType::JointVel([0.0; FRANKA_DOF]).into(),
            async |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let joint_state: JointState<FRANKA_DOF> = state.into();
                let (velocity, done) = closure(joint_state, duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::JointVel(velocity), done).into()
            },
        );
        self.is_moving = false;
        result
    }
}

impl<T: FrankaType> ControlWith<CartesianVelocityControl<7>> for FrankaRobot<T> {
    fn hold_command(_state: &ArmState<FRANKA_DOF>) -> [f64; 6] {
        [0.0; 6]
    }

    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<FRANKA_DOF>, Duration) -> ([f64; 6], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control(
            &mut self.robot_impl,
            MotionType::<FRANKA_DOF>::CartesianVel([0.0; 6]).into(),
            move |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let (velocity, done) = closure(state.into(), duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::<FRANKA_DOF>::CartesianVel(velocity), done).into()
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(ArmState<FRANKA_DOF>, Duration) -> ([f64; 6], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_async(
            &mut self.robot_impl,
            MotionType::<FRANKA_DOF>::CartesianVel([0.0; 6]).into(),
            async |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let (velocity, done) = closure(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::<FRANKA_DOF>::CartesianVel(velocity), done).into()
            },
        );
        self.is_moving = false;
        result
    }
}

impl<T: FrankaType> ControlWith<CartesianPoseControl<7>> for FrankaRobot<T> {
    fn hold_command(state: &ArmState<FRANKA_DOF>) -> Pose {
        state
            .flange
            .cmd
            .pose
            .or(state.flange.des.pose)
            .or(state.flange.meas.pose)
            .unwrap_or_default()
    }

    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<FRANKA_DOF>, Duration) -> (Pose, bool),
    {
        self.is_moving = true;
        let mode_selector = Pose::Homo(self.robot_impl.robot_state.read().unwrap().O_T_EE);
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control(
            &mut self.robot_impl,
            MotionType::<FRANKA_DOF>::Cartesian(mode_selector).into(),
            move |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let (pose, done) = closure(state.into(), duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::<FRANKA_DOF>::Cartesian(pose), done).into()
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(ArmState<FRANKA_DOF>, Duration) -> (Pose, bool),
    {
        self.is_moving = true;
        let mode_selector = Pose::Homo(self.robot_impl.robot_state.read().unwrap().O_T_EE);
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_async(
            &mut self.robot_impl,
            MotionType::<FRANKA_DOF>::Cartesian(mode_selector).into(),
            async |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let (pose, done) = closure(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (MotionType::<FRANKA_DOF>::Cartesian(pose), done).into()
            },
        );
        self.is_moving = false;
        result
    }
}

impl<T: FrankaType> ControlWith<TorqueControl<7>> for FrankaRobot<T> {
    fn hold_command(state: &JointState<FRANKA_DOF>) -> [f64; FRANKA_DOF] {
        state
            .cmd
            .tau
            .or(state.des.tau)
            .or(state.meas.tau)
            .unwrap_or([0.0; FRANKA_DOF])
    }

    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control(
            &mut self.robot_impl,
            ControlType::Torque([0.0; FRANKA_DOF]).into(),
            move |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let joint_state: JointState<FRANKA_DOF> = state.into();
                let (torque, done) = closure(joint_state, duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (ControlType::Torque(torque), done).into()
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(JointState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_async(
            &mut self.robot_impl,
            ControlType::Torque([0.0; FRANKA_DOF]).into(),
            async |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let joint_state: JointState<FRANKA_DOF> = state.into();
                let (torque, done) = closure(joint_state, duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (ControlType::Torque(torque), done).into()
            },
        );
        self.is_moving = false;
        result
    }
}

impl<T: FrankaType> ControlWith<ArmTorqueControl<7>> for FrankaRobot<T> {
    fn hold_command(state: &ArmState<FRANKA_DOF>) -> [f64; FRANKA_DOF] {
        state
            .joint
            .cmd
            .tau
            .or(state.joint.des.tau)
            .or(state.joint.meas.tau)
            .unwrap_or([0.0; FRANKA_DOF])
    }

    fn control_with<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control(
            &mut self.robot_impl,
            ControlType::Torque([0.0; FRANKA_DOF]).into(),
            move |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let (torque, done) = closure(state.into(), duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (ControlType::Torque(torque), done).into()
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(ArmState<FRANKA_DOF>, Duration) -> ([f64; FRANKA_DOF], bool),
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_async(
            &mut self.robot_impl,
            ControlType::Torque([0.0; FRANKA_DOF]).into(),
            async |state, duration| {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let (torque, done) = closure(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                (ControlType::Torque(torque), done).into()
            },
        );
        self.is_moving = false;
        result
    }
}
