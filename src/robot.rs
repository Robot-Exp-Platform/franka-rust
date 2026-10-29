use roplat_behavior::*;
use roplat_kinematics::{ArmForwardKinematics, ArmInverseKinematics};
use roplat_trajectory::{JointTrajectoryFollower, path_generate, points_from_samples};
use std::{
    fs::File,
    io::Write,
    marker::PhantomData,
    ops::ControlFlow,
    path::Path,
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{
    FRANKA_DOF, FrankaRobotImpl, LIBFRANKA_VERSION,
    model::FrankaModel,
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

pub(crate) fn flange_command_to_ee(pose: Pose, state: &RobotStateInter) -> Pose {
    let f_t_ee = state.F_T_EE;
    Pose::from_isometry(pose.quat() * crate::utils::array_to_isometry(&f_t_ee))
}

pub(crate) fn flange_velocity_to_ee(mut twist: [f64; 6], state: &RobotStateInter) -> [f64; 6] {
    let (f_t_ee, o_t_ee) = (state.F_T_EE, state.O_T_EE);
    let tool = crate::utils::array_to_isometry(&f_t_ee);
    let ee = crate::utils::array_to_isometry(&o_t_ee);
    let r = (ee.rotation * tool.rotation.inverse()) * tool.translation.vector;
    let v = nalgebra::Vector3::new(twist[0], twist[1], twist[2])
        + nalgebra::Vector3::new(twist[3], twist[4], twist[5]).cross(&r);
    twist[..3].copy_from_slice(v.as_slice());
    twist
}

pub struct FrankaRobot<T: FrankaType> {
    marker: PhantomData<T>,
    pub robot_impl: FrankaRobotImpl,
    is_moving: bool,
    before_observers: FrankaControlObservers,
    after_observers: FrankaControlObservers,
    /// Limits the arm's own motions plan with; `scale` scales their velocity,
    /// acceleration and jerk.
    limits: MotionLimits<FRANKA_DOF>,
}

impl<T: FrankaType> FrankaRobot<T>
where
    FrankaRobot<T>: Joints<7> + EndPoint,
{
    /// Connects to the robot at `ip`. Motions use the declared limits at a
    /// conservative 10 % speed until [`Arm::set_motion_limits`] changes them.
    pub fn new(ip: &str) -> Self {
        FrankaRobot {
            marker: PhantomData,
            robot_impl: FrankaRobotImpl::new(ip),
            is_moving: false,
            before_observers: control_observers(),
            after_observers: control_observers(),
            limits: Self::default_limits(),
        }
    }

    fn default_limits() -> MotionLimits<FRANKA_DOF> {
        MotionLimits::envelope::<Self>().scaled(0.1)
    }

    #[cfg(test)]
    pub(crate) fn from_test_impl(robot_impl: FrankaRobotImpl) -> Self {
        Self {
            marker: PhantomData,
            robot_impl,
            is_moving: false,
            before_observers: control_observers(),
            after_observers: control_observers(),
            limits: Self::default_limits(),
        }
    }

    #[cfg(test)]
    pub(crate) fn session_flag(&self) -> bool {
        self.is_moving
    }

    pub fn connect(&mut self, ip: &str) {
        self.robot_impl = FrankaRobotImpl::new(ip);
        self.is_moving = false;
        self.limits = Self::default_limits();
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

    /// Loads the robot's model library from [`FrankaModel::cache_path`],
    /// downloading it there first if it is missing. The cache is shared by
    /// every robot; [`model_from_path`](Self::model_from_path) keeps one per
    /// robot.
    pub fn model(&mut self) -> RobotResult<FrankaModel> {
        self.model_from_path(&FrankaModel::cache_path()?)
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
    const CONTROL_PERIOD: Duration = Duration::from_millis(1);

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
    FrankaRobot<T>:
        Joints<7> + EndPoint + EndPointPose + MoveTo<JointSpace<7>> + MoveTo<FlangeSpace>,
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

    fn motion_limits(&self) -> MotionLimits<FRANKA_DOF> {
        self.limits
    }

    fn set_motion_limits(&mut self, limits: MotionLimits<FRANKA_DOF>) -> RobotResult<()> {
        limits.within(&MotionLimits::envelope::<Self>())?;
        self.limits = limits;
        Ok(())
    }
}

/// The flange pose in a state: the end effector's pose, `O_T_EE`, without
/// the flange-to-end-effector transform, `F_T_EE`.
fn flange_pose_of(state: &RobotStateInter) -> Pose {
    let ee = Pose::from_homo(state.O_T_EE).quat();
    Pose::from_isometry(ee * Pose::from_homo(state.F_T_EE).quat().inverse())
}

impl<T: FrankaType> FrankaRobot<T> {
    /// The flange pose in the last state received.
    pub(crate) fn flange_pose(&self) -> Pose {
        flange_pose_of(&self.robot_impl.robot_state.read().unwrap())
    }

    /// The flange pose in a state read from the robot now.
    pub(crate) fn read_flange_pose(&mut self) -> RobotResult<Pose> {
        let (state, _, _) = self.robot_impl.recv_state()?;
        Ok(flange_pose_of(&state))
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
        let limits = self.limits.scaled_joint();
        let mut path_generate =
            path_generate::JointSCurve::new(limits.dq_max, limits.ddq_max, limits.dddq_max);
        let mut planned = false;
        let mut elapsed = Duration::ZERO;

        <Self as Control>::control_with::<JointPositionControl<7>, _>(self, move |state, dt| {
            if !planned {
                let joint = state
                    .des
                    .q()
                    .or(state.meas.q())
                    .unwrap_or([0.0; FRANKA_DOF]);
                path_generate.plan(joint, target);
                planned = true;
            }

            elapsed += dt;
            path_generate.apply(elapsed)
        })
    }

    fn move_to_async(
        &mut self,
        target: [f64; FRANKA_DOF],
    ) -> impl std::future::Future<Output = RobotResult<()>> {
        async move {
            let state = crate::realtime::tokio_udp::recv_state(&mut self.robot_impl).await?;
            let joint = state.q_d;

            let limits = self.limits.scaled_joint();
            let (path_generate, t_max) = path_generate::joint_s_curve(
                &joint,
                &target,
                &limits.dq_max,
                &limits.ddq_max,
                &limits.dddq_max,
            );

            // Retain this API's existing torque-space realisation: pre-sample
            // the trajectory and run the PID controller per cycle.
            let traj = sample_joint_trajectory(path_generate.as_ref(), t_max);
            let mut controller = roplat_control::joint_traj_pid_control(traj, PID_K, PID_I, PID_D);

            self.is_moving = true;
            let _moving = MovingSession(&mut self.is_moving);
            crate::realtime::tokio_udp::control_async(
                &mut self.robot_impl,
                ControlType::Torque([0.0; FRANKA_DOF]).into(),
                async move |state, d| {
                    let arm: ArmState<FRANKA_DOF> = state.into();
                    let (torque, done) = controller(arm.joint, d);
                    (ControlType::Torque(torque), done).into()
                },
            )
            .await
        }
    }
}

impl<T: FrankaType> FrankaRobot<T>
where
    Self: Joints<7> + EndPoint + ArmInverseKinematics<7>,
{
    /// A straight flange line from the pose at `q_start` to `target`, within
    /// the motion limits, sampled every control period.
    pub(crate) fn plan_flange_motion(
        &self,
        q_start: [f64; 7],
        target: Pose,
    ) -> RobotResult<Vec<[f64; 7]>> {
        roplat_trajectory::plan_cartesian_line_traj_via_time_scaling(
            Self::fk_end_pose(&q_start),
            target,
            q_start,
            |pose, seed: [f64; 7]| {
                let seed = JVec::from_row_slice(&seed);
                Self::ik_solve_converged(&seed, &pose, Default::default()).map(Into::into)
            },
            &self.limits.scaled_joint(),
            &self.limits.scaled_cartesian(),
            Self::CONTROL_PERIOD,
            Default::default(),
        )
        .map_err(RobotException::from_algorithm)
    }
}

impl<T: FrankaType> MoveTo<FlangeSpace> for FrankaRobot<T>
where
    Self: EndPoint + ArmInverseKinematics<7>,
{
    fn move_to(&mut self, target: Pose) -> RobotResult<()> {
        let (state, _, _) = self.robot_impl.recv_state()?;
        let traj = self.plan_flange_motion(state.q_d, target)?;

        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, traj)
    }

    fn move_to_async(
        &mut self,
        target: Pose,
    ) -> impl std::future::Future<Output = RobotResult<()>> {
        async move {
            let state = crate::realtime::tokio_udp::recv_state(&mut self.robot_impl).await?;
            let q_start = state.q_d;
            let traj = self.plan_flange_motion(q_start, target)?;

            if traj.is_empty() {
                return Ok(());
            }

            let mut follower = cycle_timed_follower(&traj)?;
            let mut failure = None;
            self.is_moving = true;
            let _moving = MovingSession(&mut self.is_moving);
            crate::realtime::tokio_udp::control_flow_async(
                &mut self.robot_impl,
                MotionType::<FRANKA_DOF>::Joint(q_start).into(),
                async |state, dt| {
                    let joint = ArmState::<FRANKA_DOF>::from(state).joint;
                    follow_step(&mut follower, dt, &joint, &mut failure)
                        .map_continue(|(next, done)| (MotionType::Joint(next), done).into())
                },
            )
            .await?;
            failure.map_or(Ok(()), |error| Err(RobotException::from_algorithm(error)))
        }
    }
}

/// Follows a dense trajectory sampled every control period, timed by the
/// cycle durations the control session reports.
fn cycle_timed_follower(
    traj: &[[f64; FRANKA_DOF]],
) -> RobotResult<JointTrajectoryFollower<msgs::ManualClock, FRANKA_DOF>> {
    let period = Duration::from_millis(1);
    JointTrajectoryFollower::cycle_timed(points_from_samples(traj, period))
        .map_err(RobotException::from_algorithm)
}

/// One cycle of `follower`. A tracking error ends the session; it is kept in
/// `failure` for the caller to return.
fn follow_step(
    follower: &mut JointTrajectoryFollower<msgs::ManualClock, FRANKA_DOF>,
    dt: Duration,
    joint: &JointState<FRANKA_DOF>,
    failure: &mut Option<roplat_trajectory::TrackingError>,
) -> ControlStep<[f64; FRANKA_DOF]> {
    match follower.step_by(dt, joint) {
        Ok(step) => ControlFlow::Continue(step),
        Err(error) => {
            *failure = Some(error);
            ControlFlow::Break(())
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

        let mut follower = cycle_timed_follower(&path)?;
        let mut failure = None;
        <Self as ControlWith<JointPositionControl<7>>>::control_with_flow(self, |joint, dt| {
            follow_step(&mut follower, dt, &joint, &mut failure)
        })?;
        failure.map_or(Ok(()), |error| Err(RobotException::from_algorithm(error)))
    }

    fn move_traj_sync(&mut self, path: Vec<[f64; FRANKA_DOF]>) -> RobotResult<()> {
        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, path)
    }

    fn move_path<F>(&mut self, path: F) -> RobotResult<()>
    where
        F: Fn(f64) -> Option<[f64; FRANKA_DOF]>,
    {
        let limits = self.limits.scaled_joint();
        let traj = roplat_trajectory::plan_path_traj_via_copp(&path, &limits, Self::CONTROL_PERIOD)
            .map_err(RobotException::from_algorithm)?;
        <Self as MoveTraj<JointSpace<7>>>::move_traj(self, traj)
    }

    fn move_waypoints(&mut self, waypoints: Vec<[f64; FRANKA_DOF]>) -> RobotResult<()> {
        let limits = self.limits.scaled_joint();
        let traj = roplat_trajectory::plan_waypoints_traj_via_copp(
            &waypoints,
            &limits,
            Self::CONTROL_PERIOD,
        )
        .map_err(RobotException::from_algorithm)?;
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
            .q()
            .or(state.des.q())
            .or(state.meas.q())
            .unwrap_or([0.0; FRANKA_DOF])
    }

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let mode_selector = self.robot_impl.robot_state.read().unwrap().q_d;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control_flow(
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
                let step = closure(joint_state, duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (MotionType::Joint(command), done).into())
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(JointState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let mode_selector = self.robot_impl.robot_state.read().unwrap().q_d;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_flow_async(
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
                let step = closure(joint_state, duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (MotionType::Joint(command), done).into())
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

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control_flow(
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
                let step = closure(joint_state, duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (MotionType::JointVel(command), done).into())
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(JointState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_flow_async(
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
                let step = closure(joint_state, duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (MotionType::JointVel(command), done).into())
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

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<FRANKA_DOF>, Duration) -> ControlStep<[f64; 6]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control_flow(
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
                let step = closure(state.into(), duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| {
                    (
                        MotionType::<FRANKA_DOF>::CartesianVel(flange_velocity_to_ee(
                            command, &state,
                        )),
                        done,
                    )
                        .into()
                })
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(ArmState<FRANKA_DOF>, Duration) -> ControlStep<[f64; 6]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_flow_async(
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
                let step = closure(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| {
                    (
                        MotionType::<FRANKA_DOF>::CartesianVel(flange_velocity_to_ee(
                            command, &state,
                        )),
                        done,
                    )
                        .into()
                })
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
            .pose()
            .or(state.flange.des.pose())
            .or(state.flange.meas.pose())
            .unwrap_or_default()
    }

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<FRANKA_DOF>, Duration) -> ControlStep<Pose>,
    {
        self.is_moving = true;
        let mode_selector = Pose::from_homo(self.robot_impl.robot_state.read().unwrap().O_T_EE);
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control_flow(
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
                let step = closure(state.into(), duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| {
                    (
                        MotionType::<FRANKA_DOF>::Cartesian(flange_command_to_ee(command, &state)),
                        done,
                    )
                        .into()
                })
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(ArmState<FRANKA_DOF>, Duration) -> ControlStep<Pose>,
    {
        self.is_moving = true;
        let mode_selector = Pose::from_homo(self.robot_impl.robot_state.read().unwrap().O_T_EE);
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_flow_async(
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
                let step = closure(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| {
                    (
                        MotionType::<FRANKA_DOF>::Cartesian(flange_command_to_ee(command, &state)),
                        done,
                    )
                        .into()
                })
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
            .tau()
            .or(state.des.tau())
            .or(state.meas.tau())
            .unwrap_or([0.0; FRANKA_DOF])
    }

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(JointState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control_flow(
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
                let step = closure(joint_state, duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (ControlType::Torque(command), done).into())
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(JointState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_flow_async(
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
                let step = closure(joint_state, duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (ControlType::Torque(command), done).into())
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
            .tau()
            .or(state.joint.des.tau())
            .or(state.joint.meas.tau())
            .unwrap_or([0.0; FRANKA_DOF])
    }

    fn control_with_flow<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: FnMut(ArmState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::std_udp::control_flow(
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
                let step = closure(state.into(), duration);
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (ControlType::Torque(command), done).into())
            },
        );
        self.is_moving = false;
        result
    }

    fn control_with_flow_async<F>(&mut self, mut closure: F) -> RobotResult<()>
    where
        F: async FnMut(ArmState<FRANKA_DOF>, Duration) -> ControlStep<[f64; FRANKA_DOF]>,
    {
        self.is_moving = true;
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        let result = crate::realtime::tokio_udp::block_on_control_flow_async(
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
                let step = closure(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                step.map_continue(|(command, done)| (ControlType::Torque(command), done).into())
            },
        );
        self.is_moving = false;
        result
    }
}

// This borrowed flag is reset on return and on a dropped session future. A
// still-active protocol remains protected by robot_impl.motion_command_id.
struct MovingSession<'a>(&'a mut bool);
impl Drop for MovingSession<'_> {
    fn drop(&mut self) {
        *self.0 = false;
    }
}

impl<T: FrankaType + Send> FrankaRobot<T> {
    async fn native_session<C, Obs, Command, Map>(
        &mut self,
        mode: MoveData,
        callback: &mut C,
        map: Map,
    ) -> RobotResult<()>
    where
        C: AsyncControlCallback<Obs, Command>,
        Obs: From<RobotStateInter> + Send,
        Command: Send,
        Map: Fn(Command, bool, &RobotStateInter) -> crate::types::robot_command::RobotCommand
            + Send
            + Sync,
    {
        self.is_moving = true;
        let _moving = MovingSession(&mut self.is_moving);
        let before_observers = self.before_observers.clone();
        let after_observers = self.after_observers.clone();
        let observe_before = has_control_observers(&before_observers);
        let observe_after = has_control_observers(&after_observers);
        use crate::realtime::tokio_udp::{AsyncSession, LoopEnd};
        let mut session = AsyncSession::start(&mut self.robot_impl, mode).await?;
        let result = async {
            while let Some((state, duration)) = session.next_state().await? {
                let robot_state =
                    (observe_before || observe_after).then(|| RobotState::from(state));
                if observe_before {
                    notify_control_observers(
                        &before_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let step = callback.call(state.into(), duration).await;
                if observe_after {
                    notify_control_observers(
                        &after_observers,
                        robot_state.as_ref().unwrap(),
                        duration,
                    );
                }
                let std::ops::ControlFlow::Continue((command, done)) = step else {
                    return Ok(LoopEnd::Cancelled);
                };
                session
                    .send_command(&state, map(command, done, &state))
                    .await?;
            }
            Ok(LoopEnd::Finished)
        }
        .await;
        session.finish(result).await
    }
}

impl<T: FrankaType + Send> AsyncControlWith<JointPositionControl<7>> for FrankaRobot<T> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<JointState<FRANKA_DOF>, [f64; FRANKA_DOF]>,
    {
        let mode = MotionType::Joint(self.robot_impl.robot_state.read().unwrap().q_d).into();
        self.native_session(mode, callback, |command, done, _state| {
            (MotionType::Joint(command), done).into()
        })
        .await
    }
}

impl<T: FrankaType + Send> AsyncControlWith<JointVelocityControl<7>> for FrankaRobot<T> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<JointState<FRANKA_DOF>, [f64; FRANKA_DOF]>,
    {
        let mode = MotionType::JointVel([0.0; FRANKA_DOF]).into();
        self.native_session(mode, callback, |command, done, _state| {
            (MotionType::JointVel(command), done).into()
        })
        .await
    }
}

impl<T: FrankaType + Send> AsyncControlWith<CartesianVelocityControl<7>> for FrankaRobot<T> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<ArmState<FRANKA_DOF>, [f64; 6]>,
    {
        let mode = MotionType::<FRANKA_DOF>::CartesianVel([0.0; 6]).into();
        self.native_session(mode, callback, |command, done, _state| {
            (
                MotionType::<FRANKA_DOF>::CartesianVel(flange_velocity_to_ee(command, _state)),
                done,
            )
                .into()
        })
        .await
    }
}

impl<T: FrankaType + Send> AsyncControlWith<CartesianPoseControl<7>> for FrankaRobot<T> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<ArmState<FRANKA_DOF>, Pose>,
    {
        let mode = MotionType::<FRANKA_DOF>::Cartesian(Pose::from_homo(
            self.robot_impl.robot_state.read().unwrap().O_T_EE,
        ))
        .into();
        self.native_session(mode, callback, |command, done, _state| {
            (
                MotionType::<FRANKA_DOF>::Cartesian(flange_command_to_ee(command, _state)),
                done,
            )
                .into()
        })
        .await
    }
}

impl<T: FrankaType + Send> AsyncControlWith<TorqueControl<7>> for FrankaRobot<T> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<JointState<FRANKA_DOF>, [f64; FRANKA_DOF]>,
    {
        let mode = ControlType::Torque([0.0; FRANKA_DOF]).into();
        self.native_session(mode, callback, |command, done, _state| {
            (ControlType::Torque(command), done).into()
        })
        .await
    }
}

impl<T: FrankaType + Send> AsyncControlWith<ArmTorqueControl<7>> for FrankaRobot<T> {
    async fn control_native_async<C>(&mut self, callback: &mut C) -> RobotResult<()>
    where
        C: AsyncControlCallback<ArmState<FRANKA_DOF>, [f64; FRANKA_DOF]>,
    {
        let mode = ControlType::Torque([0.0; FRANKA_DOF]).into();
        self.native_session(mode, callback, |command, done, _state| {
            (ControlType::Torque(command), done).into()
        })
        .await
    }
}
