use crossbeam::queue::ArrayQueue;
use nalgebra as na;
use robot_behavior::{
    ArmBehavior, ArmBehaviorExt, ArmRealtimeBehavior, ArmRealtimeBehaviorExt, ArmState,
    ControlType, MotionType, RobotBehavior, RobotException, RobotResult, utils::path_generate,
};
use std::{
    sync::{Arc, Mutex, RwLock},
    thread::{self, sleep},
    time::{Duration, Instant},
};

use crate::{
    FRANKA_EMIKA_DOF, FRANKA_ROBOT_MAX_JOINT_ACC, FRANKA_ROBOT_MAX_JOINT_VEL, FRANKA_ROBOT_VERSION,
    LIBFRANKA_VERSION, PORT_ROBOT_COMMAND, PORT_ROBOT_UDP,
    network::Network,
    types::{
        robot_command::RobotCommand,
        robot_state::{RobotState, RobotStateInter},
        robot_types::*,
    },
};

pub struct FrankaRobot {
    network: Network,
    control_queue: Arc<ArrayQueue<RobotCommand>>,
    robot_state: Arc<RwLock<RobotStateInter>>,
    is_moving: bool,
    control_handle: Option<thread::JoinHandle<()>>,
}

macro_rules! cmd_fn {
    ($fn_name:ident, $command:expr; $arg_name:ident: $arg_type:ty ; $ret_type:ty) => {
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
        let (control_queue, robot_state) = Network::spawn_udp_thread(PORT_ROBOT_UDP);
        let mut robot = FrankaRobot {
            network: Network::new(ip, PORT_ROBOT_COMMAND),
            control_queue,
            robot_state,
            is_moving: false,
            control_handle: None,
        };
        robot.connect().unwrap();
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

    pub fn read_state(&mut self) -> RobotResult<RobotState> {
        let state = self.robot_state.read().unwrap();
        Ok((*state).into())
    }

    pub fn set_default_behavior(&mut self) -> RobotResult<()> {
        self.set_collision_behavior(SetCollisionBehaviorData::default())?;
        self.set_joint_impedance(SetJointImpedanceData::default())?;
        self.set_cartesian_impedance(SetCartesianImpedanceData::default())?;
        Ok(())
    }
}

impl RobotBehavior for FrankaRobot {
    fn version(&self) -> String {
        format!("FrankaRobot v{}", LIBFRANKA_VERSION)
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
}

impl ArmBehavior<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_to(&mut self, target: MotionType<FRANKA_EMIKA_DOF>, speed: f64) -> RobotResult<()> {
        match target {
            MotionType::Joint(target) => self.move_joint(&target, speed),
            _ => unimplemented!(),
        }
    }

    fn move_to_async(
        &mut self,
        target: MotionType<FRANKA_EMIKA_DOF>,
        _speed: f64,
    ) -> RobotResult<()> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let robot_state = self.robot_state.clone();
        self._move(target.into())?;
        self.control_handle = Some(thread::spawn(move || {
            loop {
                let start_time = Instant::now();

                let state = robot_state.read().unwrap();
                let mut motion: RobotCommand = target.into();
                motion.set_command_id(state.message_id as u32);
                let state: ArmState<7> = (*state).into();
                if state == target {
                    break;
                }
                control_queue.force_push(motion);

                let end_time = Instant::now();
                let elapsed = end_time - start_time;
                if elapsed < Duration::from_millis(1) {
                    sleep(Duration::from_millis(1) - elapsed);
                }
            }
        }));
        Ok(())
    }

    fn move_rel(&mut self, _rel: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_rel_async(&mut self, _rel: MotionType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_path(
        &mut self,
        _path: Vec<MotionType<FRANKA_EMIKA_DOF>>,
        _speed: f64,
    ) -> RobotResult<()> {
        unimplemented!()
    }

    fn control_with(&mut self, _control: ControlType<FRANKA_EMIKA_DOF>) -> RobotResult<()> {
        unimplemented!()
    }

    fn read_state(&mut self) -> RobotResult<ArmState<FRANKA_EMIKA_DOF>> {
        unimplemented!()
    }
}

impl ArmBehaviorExt<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_joint(&mut self, target: &[f64; FRANKA_EMIKA_DOF], speed: f64) -> RobotResult<()> {
        self.is_moving = true;

        let state = self.robot_state.read().unwrap();
        let joint = state.q;
        let v_max: Vec<f64> = FRANKA_ROBOT_MAX_JOINT_VEL
            .iter()
            .map(|x| x * speed)
            .collect();
        let v_max: [f64; 7] = v_max.try_into().expect("slice with incorrect length");
        let a_max = FRANKA_ROBOT_MAX_JOINT_ACC;
        drop(state);

        let path_generate = path_generate::joint_trapezoid(&joint, &target, &v_max, &a_max);

        self._move(MotionType::Joint(*target).into())?;
        {
            let time = Instant::now();
            loop {
                let start_time = Instant::now();

                let state = self.robot_state.read().unwrap();
                let joint = path_generate(start_time - time);
                let mut motion: RobotCommand = MotionType::Joint(joint).into();
                motion.set_command_id(state.message_id as u32);
                if let Some(error) = state.get_error() {
                    return Err(RobotException::InvalidInstruction(format!("{}", error)));
                }

                let state: ArmState<7> = (*state).into();
                if state.joint.unwrap() == *target {
                    motion.motion.motion_generation_finished = true;
                    self.control_queue.force_push(motion);
                    break;
                }
                self.control_queue.force_push(motion);

                let end_time = Instant::now();
                let elapsed = end_time - start_time;
                if elapsed < Duration::from_millis(1) {
                    sleep(Duration::from_millis(1) - elapsed);
                }
            }
        }
        self.is_moving = false;
        Ok(())
    }

    fn move_path_prepare(&mut self, _path: Vec<MotionType<FRANKA_EMIKA_DOF>>) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_path_start(&mut self) -> RobotResult<()> {
        unimplemented!()
    }

    fn move_path_prepare_from_file(&mut self, _path: &str) -> RobotResult<()> {
        unimplemented!()
    }
}

impl ArmRealtimeBehavior<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_with_closure<
        FM: Fn(ArmState<FRANKA_EMIKA_DOF>, Duration) -> MotionType<FRANKA_EMIKA_DOF> + Send + 'static,
    >(
        &mut self,
        closure: FM,
    ) -> RobotResult<()> {
        self.is_moving = true;
        let robot_state = self.robot_state.clone();
        let control_queue = self.control_queue.clone();
        let example = ArmState::<FRANKA_EMIKA_DOF>::default();
        self._move(closure(example, Duration::from_millis(1)).into())?;
        self.control_handle = Some(thread::spawn(move || {
            loop {
                let start_time = Instant::now();

                let state = robot_state.read().unwrap();
                let mut motion: RobotCommand =
                    closure((*state).into(), Duration::from_millis(1)).into();
                motion.set_command_id(state.message_id as u32);
                control_queue.force_push(motion);

                let end_time = Instant::now();
                let elapsed = end_time - start_time;
                if elapsed < Duration::from_millis(1) {
                    sleep(Duration::from_millis(1) - elapsed);
                }
            }
        }));
        Ok(())
    }
    fn control_with_closure<
        FC: Fn(ArmState<FRANKA_EMIKA_DOF>, Duration) -> ControlType<FRANKA_EMIKA_DOF> + Send + 'static,
    >(
        &mut self,
        closure: FC,
    ) -> RobotResult<()> {
        self.is_moving = true;
        let robot_state = self.robot_state.clone();
        let control_queue = self.control_queue.clone();
        let example = ArmState::<FRANKA_EMIKA_DOF>::default();
        self._move(closure(example, Duration::from_millis(1)).into())?;
        self.control_handle = Some(thread::spawn(move || {
            loop {
                let start_time = Instant::now();

                let state = robot_state.read().unwrap();
                let mut control: RobotCommand =
                    closure((*state).into(), Duration::from_millis(1)).into();
                control.set_command_id(state.message_id as u32);
                control_queue.force_push(control);

                let end_time = Instant::now();
                let elapsed = end_time - start_time;
                if elapsed < Duration::from_millis(1) {
                    sleep(Duration::from_millis(1) - elapsed);
                }
            }
        }));
        Ok(())
    }

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<FRANKA_EMIKA_DOF>>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<MotionType<7>>>> = target.clone();
        // TODO send move command
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(motion) = target.lock().unwrap().take() {
                    control_queue.force_push(motion.into());
                }

                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }

    fn control_to_target(&mut self) -> Arc<Mutex<Option<ControlType<FRANKA_EMIKA_DOF>>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<ControlType<7>>>> = target.clone();
        // TODO send move command
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(control) = target.lock().unwrap().take() {
                    control_queue.force_push(control.into());
                }
                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }
}

impl ArmRealtimeBehaviorExt<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> = target.clone();
        let move_data = MoveData {
            motion_generator_mode: MoveMotionGeneratorMode::JointPosition,
            ..MoveData::default()
        };
        self._move(move_data).unwrap();
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(joint) = target.lock().unwrap().take() {
                    control_queue.force_push(MotionType::Joint(joint).into());
                }

                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }

    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> = target.clone();
        let move_data = MoveData {
            motion_generator_mode: MoveMotionGeneratorMode::JointVelocity,
            ..MoveData::default()
        };
        self._move(move_data).unwrap();
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(joint_vel) = target.lock().unwrap().take() {
                    control_queue.force_push(MotionType::JointVel(joint_vel).into());
                }

                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }

    fn move_joint_acc_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        unimplemented!()
    }

    fn move_cartesian_euler_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        unimplemented!()
    }

    fn move_cartesian_quat_target(&mut self) -> Arc<Mutex<Option<na::Isometry3<f64>>>> {
        unimplemented!()
    }

    fn move_cartesian_homo_target(&mut self) -> Arc<Mutex<Option<[f64; 16]>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<[f64; 16]>>> = target.clone();
        let move_data = MoveData {
            motion_generator_mode: MoveMotionGeneratorMode::CartesianPosition,
            controller_mode: MoveControllerMode::CartesianImpedance,
            ..MoveData::default()
        };
        self._move(move_data).unwrap();
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(cartesian_homo) = target.lock().unwrap().take() {
                    control_queue.force_push(MotionType::CartesianHomo(cartesian_homo).into());
                }

                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }

    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<[f64; 6]>>> = target.clone();
        let move_data = MoveData {
            motion_generator_mode: MoveMotionGeneratorMode::CartesianVelocity,
            controller_mode: MoveControllerMode::CartesianImpedance,
            ..MoveData::default()
        };
        self._move(move_data).unwrap();
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(cartesian_vel) = target.lock().unwrap().take() {
                    control_queue.force_push(MotionType::CartesianVel(cartesian_vel).into());
                }

                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }

    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        self.is_moving = true;
        let control_queue = self.control_queue.clone();
        let target = Arc::new(Mutex::new(None));
        let target_clone: Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> = target.clone();
        let move_data = MoveData {
            controller_mode: MoveControllerMode::ExternalController,
            ..MoveData::default()
        };
        self._move(move_data).unwrap();
        self.control_handle = Some(thread::spawn(move || {
            loop {
                if let Some(force) = target.lock().unwrap().take() {
                    control_queue.force_push(ControlType::Force(force).into());
                }

                sleep(Duration::from_millis(1));
            }
        }));
        target_clone
    }
}
