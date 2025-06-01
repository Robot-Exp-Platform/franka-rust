use robot_behavior::{
    utils::path_generate::{self, cartesian_quat_simple_4th_curve},
    *,
};
use std::{
    f64::consts::FRAC_PI_2,
    fs::File,
    io::Write,
    path::Path,
    sync::{Arc, RwLock},
    thread::sleep,
    time::Duration,
};

use crate::{
    FRANKA_EMIKA_DOF, FRANKA_ROBOT_DEFAULT_JOINT, FRANKA_ROBOT_MAX_CARTESIAN_ACC,
    FRANKA_ROBOT_MAX_CARTESIAN_VEL, FRANKA_ROBOT_MAX_JOINT, FRANKA_ROBOT_MAX_JOINT_ACC,
    FRANKA_ROBOT_MAX_JOINT_JERK, FRANKA_ROBOT_MAX_JOINT_VEL, FRANKA_ROBOT_MAX_TORQUE,
    FRANKA_ROBOT_MAX_TORQUE_RATE, FRANKA_ROBOT_MIN_JOINT, FRANKA_ROBOT_VERSION, LIBFRANKA_VERSION,
    PORT_ROBOT_COMMAND, PORT_ROBOT_UDP,
    command_handle::CommandHandle,
    model::{self, FrankaModel},
    network::Network,
    once::OverrideOnce,
    types::{
        robot_command::RobotCommand,
        robot_state::{RobotState, RobotStateInter},
        robot_types::*,
    },
};

pub struct FrankaRobot {
    network: Network,
    command_handle: CommandHandle<RobotCommand, RobotStateInter>,
    robot_state: Arc<RwLock<RobotStateInter>>,
    is_moving: bool,
    coord: OverrideOnce<Coord>,
    max_vel: OverrideOnce<[f64; FRANKA_EMIKA_DOF]>,
    max_acc: OverrideOnce<[f64; FRANKA_EMIKA_DOF]>,
    max_jerk: OverrideOnce<[f64; FRANKA_EMIKA_DOF]>,
    max_cartesian_vel: OverrideOnce<f64>,
    max_cartesian_acc: OverrideOnce<f64>,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty; $ret_type:ty) => {
        fn $fn_name(&mut self, $arg_name: $arg_type) -> RobotResult<$ret_type> {
            let response: Response<$command, $ret_type> = self
                .network
                .tcp_send_and_recv(&mut Request::<$command, $arg_type>::from($arg_name))?;
            Ok(response.status)
        }
    };
}

impl FrankaRobot {
    pub fn new(ip: &str) -> Self {
        let (command_handle, robot_state) = Network::spawn_udp_thread(PORT_ROBOT_UDP);
        let mut robot = FrankaRobot {
            network: Network::new(ip, PORT_ROBOT_COMMAND),
            command_handle,
            robot_state,
            is_moving: false,
            coord: OverrideOnce::new(Coord::OCS),
            max_vel: OverrideOnce::new(FRANKA_ROBOT_MAX_JOINT_VEL),
            max_acc: OverrideOnce::new(FRANKA_ROBOT_MAX_JOINT_ACC),
            max_jerk: OverrideOnce::new(FRANKA_ROBOT_MAX_JOINT_JERK),
            max_cartesian_vel: OverrideOnce::new(FRANKA_ROBOT_MAX_CARTESIAN_VEL[0]),
            max_cartesian_acc: OverrideOnce::new(FRANKA_ROBOT_MAX_CARTESIAN_ACC[0]),
        };
        robot.connect().unwrap();
        let _ = robot.set_speed(0.1);

        robot
    }

    cmd_fn!(_connect, { Command::Connect }; data: ConnectData; ConnectStatus);
    cmd_fn!(_move, { Command::Move }; data: MoveData; MoveStatus);
    cmd_fn!(_set_collision_behavior, { Command::SetCollisionBehavior }; data: SetCollisionBehaviorData; GetterSetterStatus);
    cmd_fn!(_set_joint_impedance, { Command::SetJointImpedance }; data: SetJointImpedanceData; GetterSetterStatus);
    cmd_fn!(_set_cartesian_impedance, { Command::SetCartesianImpedance }; data: SetCartesianImpedanceData; GetterSetterStatus);
    cmd_fn!(_set_guiding_mode, { Command::SetGuidingMode }; data: SetGuidingModeData; GetterSetterStatus);
    cmd_fn!(_set_ee_to_k, { Command::SetEEToK }; data: SetEEToKData; GetterSetterStatus);
    cmd_fn!(_set_ne_to_ee, { Command::SetNEToEE }; data: SetNEToEEData; GetterSetterStatus);
    cmd_fn!(_set_load, { Command::SetLoad }; data: SetLoadData; GetterSetterStatus);
    cmd_fn!(_set_fliters, { Command::SetFilters }; data: SetFiltersData; GetterSetterStatus);
    cmd_fn!(_automatic_error_recovery, { Command::AutomaticErrorRecovery }; data: (); GetterSetterStatus);
    cmd_fn!(_stop_move, { Command::StopMove }; data: (); GetterSetterStatus);
    cmd_fn!(_get_cartesian_limit, { Command::GetCartesianLimit }; data: GetCartesianLimitData; GetCartesianLimitStatus);

    fn connect(&mut self) -> RobotResult<()> {
        let result = self._connect(ConnectData {
            version: FRANKA_ROBOT_VERSION,
            udp_port: PORT_ROBOT_UDP,
        })?;
        if let ConnectStatusEnum::Success = result.status {
            Ok(())
        } else {
            Err(RobotException::IncompatibleVersionException {
                server_version: result.version as u64,
                client_version: FRANKA_ROBOT_VERSION as u64,
            })
        }
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
        self._set_collision_behavior(data)?.into()
    }

    /// # set_joint_impedance
    /// **Sets the impedance for each joint in the internal controller.**
    pub fn set_joint_impedance(&mut self, data: SetJointImpedanceData) -> RobotResult<()> {
        self._set_joint_impedance(data)?.into()
    }

    /// # set_cartesian_impedance
    /// **Sets the impedance for the Cartesian internal controller.**
    pub fn set_cartesian_impedance(&mut self, data: SetCartesianImpedanceData) -> RobotResult<()> {
        self._set_cartesian_impedance(data)?.into()
    }

    /// # set_cartesian_limit
    /// **Locks or unlocks guiding mode movement in (x, y, z, roll, pitch, yaw).**
    /// If a flag is set to true, movement is unlocked.
    pub fn set_guiding_mode(&mut self, data: SetGuidingModeData) -> RobotResult<()> {
        self._set_guiding_mode(data)?.into()
    }

    pub fn set_ee_to_k(&mut self, data: SetEEToKData) -> RobotResult<()> {
        self._set_ee_to_k(data)?.into()
    }

    pub fn set_ne_to_ee(&mut self, data: SetNEToEEData) -> RobotResult<()> {
        self._set_ne_to_ee(data)?.into()
    }

    /// # set_load
    /// **Sets the load of the robot.**
    /// The transformation matrix is represented as a vectorized 4x4 matrix in column-major format.
    /// This is not for setting end effector parameters, which have to be set in the administrator's
    /// interface.
    pub fn set_load(&mut self, data: SetLoadData) -> RobotResult<()> {
        self._set_load(data)?.into()
    }

    #[deprecated(note = "please use `low_pass_filter` instead")]
    pub fn set_filters(&mut self, data: SetFiltersData) -> RobotResult<()> {
        self._set_fliters(data)?.into()
    }

    pub fn read_franka_state(&mut self) -> RobotResult<RobotState> {
        let state = self.robot_state.read().unwrap();
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
        let model = model::FrankaModel::new(model_path)?;
        Ok(model)
    }

    pub fn model_from_path(&mut self, model_path: &Path) -> RobotResult<FrankaModel> {
        if !model_path.exists() {
            self.download_library(model_path)?;
        }
        let model = model::FrankaModel::new(model_path)?;
        Ok(model)
    }
}

impl RobotBehavior for FrankaRobot {
    type State = RobotState;
    fn version(&self) -> String {
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

    fn is_moving(&mut self) -> bool {
        unimplemented!()
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
        let state = self.robot_state.read().unwrap();
        Ok((*state).into())
    }
}

impl ArmBehavior<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn state(&mut self) -> RobotResult<ArmState<FRANKA_EMIKA_DOF>> {
        let state = self.robot_state.read().unwrap();
        Ok((*state).into())
    }
    fn set_load(&mut self, load: LoadState) -> RobotResult<()> {
        self._set_load(load.into()).map(|_| ())
    }
    fn set_coord(&mut self, coord: Coord) -> RobotResult<()> {
        self.coord.set(coord);
        Ok(())
    }
    fn set_speed(&mut self, speed: f64) -> RobotResult<()> {
        self.max_vel
            .set(FRANKA_ROBOT_MAX_JOINT_VEL.map(|v| v * speed));
        self.max_acc
            .set(FRANKA_ROBOT_MAX_JOINT_ACC.map(|v| v * speed));
        self.max_jerk
            .set(FRANKA_ROBOT_MAX_JOINT_JERK.map(|v| v * speed));
        self.max_cartesian_vel
            .set(FRANKA_ROBOT_MAX_CARTESIAN_VEL[0] * speed);
        self.max_cartesian_acc
            .set(FRANKA_ROBOT_MAX_CARTESIAN_ACC[0] * speed);
        Ok(())
    }

    fn with_coord(&mut self, coord: Coord) -> &mut Self {
        self.coord.once(coord);
        self
    }
    fn with_speed(&mut self, speed: f64) -> &mut Self {
        self.max_vel
            .once(FRANKA_ROBOT_MAX_JOINT_VEL.map(|v| v * speed));
        self.max_acc
            .once(FRANKA_ROBOT_MAX_JOINT_ACC.map(|v| v * speed));
        self.max_jerk
            .once(FRANKA_ROBOT_MAX_JOINT_JERK.map(|v| v * speed));
        self.max_cartesian_vel
            .once(FRANKA_ROBOT_MAX_CARTESIAN_VEL[0] * speed);
        self.max_cartesian_acc
            .once(FRANKA_ROBOT_MAX_CARTESIAN_ACC[0] * speed);
        self
    }
    fn with_velocity(&mut self, joint_vel: &[f64; FRANKA_EMIKA_DOF]) -> &mut Self {
        self.max_vel.once(*joint_vel);
        self
    }
    fn with_acceleration(&mut self, joint_acc: &[f64; FRANKA_EMIKA_DOF]) -> &mut Self {
        self.max_acc.once(*joint_acc);
        self
    }
    fn with_jerk(&mut self, joint_jerk: &[f64; FRANKA_EMIKA_DOF]) -> &mut Self {
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

impl ArmParam<FRANKA_EMIKA_DOF> for FrankaRobot {
    const DH: [[f64; 4]; FRANKA_EMIKA_DOF] = [
        [0., 0.333, 0., 0.],
        [0., 0., 0., -FRAC_PI_2],
        [0., 0.316, 0., FRAC_PI_2],
        [0., 0., 0.0825, FRAC_PI_2],
        [0., 0.384, -0.0825, -FRAC_PI_2],
        [0., 0., 0., FRAC_PI_2],
        [0., 0., 0.088, FRAC_PI_2],
    ];
    const JOINT_DEFAULT: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_DEFAULT_JOINT;
    const JOINT_MIN: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MIN_JOINT;
    const JOINT_MAX: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MAX_JOINT;
    const JOINT_VEL_BOUND: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MAX_JOINT_VEL;
    const JOINT_ACC_BOUND: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MAX_JOINT_ACC;
    const JOINT_JERK_BOUND: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MAX_JOINT_JERK;
    const CARTESIAN_VEL_BOUND: f64 = 1.7;
    const CARTESIAN_ACC_BOUND: f64 = 13.0;
    const TORQUE_BOUND: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MAX_TORQUE;
    const TORQUE_DOT_BOUND: [f64; FRANKA_EMIKA_DOF] = FRANKA_ROBOT_MAX_TORQUE_RATE;
}

impl ArmPreplannedMotionImpl<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_joint(&mut self, target: &[f64; FRANKA_EMIKA_DOF]) -> RobotResult<()> {
        self.move_joint_async(target)?;

        loop {
            let state = self.robot_state.read().unwrap();
            let joint = state.q_d;
            drop(state);

            if target
                .iter()
                .zip(joint.into_iter())
                .fold(true, |acc, (t, j)| acc && (t - j).abs() < 0.01)
            {
                break;
            }
            sleep(Duration::from_millis(1));
        }
        let _ = self._stop_move(());
        self.is_moving = false;
        Ok(())
    }
    fn move_joint_async(&mut self, target: &[f64; FRANKA_EMIKA_DOF]) -> RobotResult<()> {
        self.is_moving = true;
        self._move(MotionType::Joint(*target).into())?;
        sleep(Duration::from_millis(2));

        let state = self.robot_state.read().unwrap();
        let joint = state.q_d;
        drop(state);
        let target = *target;

        let (v_max, a_max, j_max) = (self.max_vel.get(), self.max_acc.get(), self.max_jerk.get());
        let path_generate = path_generate::joint_s_curve(&joint, &target, v_max, a_max, j_max);

        self.command_handle.set_closure(move |state, duration| {
            let finished = target
                .iter()
                .zip(state.q_d.into_iter())
                .fold(true, |acc, (t, j)| acc && (t - j).abs() < 0.01);
            (MotionType::Joint(path_generate(duration)), finished).into()
        });
        Ok(())
    }
    fn move_cartesian(&mut self, target: &Pose) -> RobotResult<()> {
        self.move_cartesian_async(target)?;

        loop {
            let state = self.robot_state.read().unwrap();
            let pose: Pose = state.O_T_EE.into();
            drop(state);

            if (*target / pose) < 0.01 {
                break;
            } else {
                sleep(Duration::from_millis(1));
            }
        }
        let _ = self._stop_move(());
        self.is_moving = false;
        Ok(())
    }
    fn move_cartesian_async(&mut self, target: &Pose) -> RobotResult<()> {
        self.is_moving = true;
        self._move(MotionType::<7>::Cartesian(*target).into())?;
        sleep(Duration::from_millis(1));

        let target = *target;
        let state = self.robot_state.read().unwrap();
        let pose: Pose = state.O_T_EE.into();
        drop(state);

        let (v_max, a_max) = (self.max_cartesian_vel.get(), self.max_cartesian_acc.get());
        let path_generate =
            cartesian_quat_simple_4th_curve(pose.quat(), target.quat(), *v_max, *a_max);

        self.command_handle.set_closure(move |state, duration| {
            (
                MotionType::Cartesian(path_generate(duration).into()),
                (target / Into::<Pose>::into(state.O_T_EE)) < 0.01,
            )
                .into()
        });
        Ok(())
    }
}

impl ArmPreplannedMotion<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_path(&mut self, path: Vec<MotionType<FRANKA_EMIKA_DOF>>) -> RobotResult<()> {
        self.move_path_async(path)?;

        loop {
            let state = self.robot_state.read().unwrap();
            let vel = state.dq_d;
            drop(state);

            if vel.into_iter().sum::<f64>() < 0.01 {
                break;
            } else {
                sleep(Duration::from_millis(1));
            }
        }

        self.is_moving = false;
        Ok(())
    }
    fn move_path_async(&mut self, path: Vec<MotionType<FRANKA_EMIKA_DOF>>) -> RobotResult<()> {
        self.is_moving = true;

        self.move_to(path[0])?;
        let mut path = path.into_iter();

        self.command_handle.set_closure(move |_, _| {
            if let Some(next) = path.next() {
                (next, false).into()
            } else {
                (MotionType::Stop, true).into()
            }
        });
        Ok(())
    }
    fn move_path_prepare(&mut self, _path: Vec<MotionType<FRANKA_EMIKA_DOF>>) -> RobotResult<()> {
        unimplemented!()
    }
    fn move_path_start(&mut self, _start: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }
}

impl ArmRealtimeControl<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_with_closure<FM>(&mut self, mut closure: FM) -> RobotResult<()>
    where
        FM: FnMut(ArmState<FRANKA_EMIKA_DOF>, Duration) -> (MotionType<FRANKA_EMIKA_DOF>, bool)
            + Send
            + 'static,
    {
        self.is_moving = true;
        let example = ArmState::<FRANKA_EMIKA_DOF>::default();
        self._move(closure(example, Duration::from_millis(1)).0.into())?;
        self.command_handle
            .set_closure(move |state, duration| closure((*state).into(), duration).into());
        Ok(())
    }

    fn control_with_closure<FC>(&mut self, mut closure: FC) -> RobotResult<()>
    where
        FC: FnMut(ArmState<FRANKA_EMIKA_DOF>, Duration) -> (ControlType<FRANKA_EMIKA_DOF>, bool)
            + Send
            + 'static,
    {
        self.is_moving = true;
        let example = ArmState::<FRANKA_EMIKA_DOF>::default();
        self._move(closure(example, Duration::from_millis(1)).0.into())?;
        self.command_handle
            .set_closure(move |state, duration| closure((*state).into(), duration).into());
        Ok(())
    }
}
