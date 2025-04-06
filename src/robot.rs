use nalgebra as na;
use robot_behavior::{
    ArmBehavior, ArmBehaviorExt, ArmRealtimeBehavior, ArmRealtimeBehaviorExt, ArmState,
    ControlType, MotionType, Pose, RobotBehavior, RobotException, RobotResult,
    utils::path_generate,
};
use std::{
    fs::File,
    io::Write,
    path::Path,
    sync::{Arc, Mutex, RwLock},
    thread::sleep,
    time::Duration,
};

use crate::{
    FRANKA_EMIKA_DOF, FRANKA_ROBOT_MAX_CARTESIAN_ACC, FRANKA_ROBOT_MAX_CARTESIAN_VEL,
    FRANKA_ROBOT_MAX_JOINT_ACC, FRANKA_ROBOT_MAX_JOINT_VEL, FRANKA_ROBOT_VERSION,
    LIBFRANKA_VERSION, PORT_ROBOT_COMMAND, PORT_ROBOT_UDP,
    command_handle::CommandHandle,
    model::{self, FrankaModel},
    network::Network,
    types::{
        robot_command::RobotCommand,
        robot_state::{RobotState, RobotStateInter},
        robot_types::*,
    },
    utils::array_to_isometry,
};

pub struct FrankaRobot {
    network: Network,
    command_handle: CommandHandle<RobotCommand, RobotStateInter>,
    robot_state: Arc<RwLock<RobotStateInter>>,
    is_moving: bool,
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
            MotionType::CartesianQuat(target) => self.move_linear_with_quat(&target, speed),
            _ => unimplemented!(),
        }
    }

    fn move_to_async(
        &mut self,
        _target: MotionType<FRANKA_EMIKA_DOF>,
        _speed: f64,
    ) -> RobotResult<()> {
        unimplemented!()
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
        let state = self.robot_state.read().unwrap();
        Ok((*state).into())
    }
}

impl ArmBehaviorExt<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_joint(&mut self, target: &[f64; FRANKA_EMIKA_DOF], speed: f64) -> RobotResult<()> {
        self.is_moving = true;
        self._move(MotionType::Joint(*target).into())?;
        sleep(Duration::from_millis(2));

        let state = self.robot_state.read().unwrap();
        let joint = state.q_d;
        drop(state);
        let mut v_max = FRANKA_ROBOT_MAX_JOINT_VEL;
        let mut a_max = FRANKA_ROBOT_MAX_JOINT_ACC;
        // let mut jeck = FRANKA_ROBOT_MAX_JOINT_JERK;
        for i in 0..FRANKA_EMIKA_DOF {
            v_max[i] *= speed;
            a_max[i] *= speed;
        }

        let path_generate = path_generate::joint_simple_4th_curve(&joint, target, &v_max, &a_max);

        self.command_handle
            .set_closure(move |_, duration| MotionType::Joint(path_generate(duration)).into());

        loop {
            let state = self.robot_state.read().unwrap();
            let joint = state.q_d;
            drop(state);
            let mut is_done = true;
            for i in 0..FRANKA_EMIKA_DOF {
                if (joint[i] - target[i]).abs() > 0.01 {
                    is_done = false;
                    break;
                }
            }
            if is_done {
                break;
            }
            sleep(Duration::from_millis(1));
        }
        let _ = self._stop_move(());
        self.is_moving = false;
        Ok(())
    }

    fn move_linear_with_quat(
        &mut self,
        target: &nalgebra::Isometry3<f64>,
        speed: f64,
    ) -> RobotResult<()> {
        self.is_moving = true;
        self._move(
            MotionType::<7>::CartesianHomo(target.to_homogeneous().as_slice().try_into().unwrap())
                .into(),
        )?;
        sleep(Duration::from_millis(1));

        let state = self.robot_state.read().unwrap();
        let arm_state: ArmState<FRANKA_EMIKA_DOF> = (*state).into();
        drop(state);
        let pose = if let Some(Pose::Homo(pose)) = arm_state.pose_o_to_ee {
            array_to_isometry(&pose)
        } else {
            unreachable!()
        };

        let v_max = FRANKA_ROBOT_MAX_CARTESIAN_VEL[0] * speed;
        let a_max = FRANKA_ROBOT_MAX_CARTESIAN_ACC[0] * speed;

        let path_generate =
            path_generate::cartesian_quat_simple_4th_curve(pose, *target, v_max, a_max);

        self.command_handle.set_closure(move |_, duration| {
            MotionType::CartesianHomo(
                path_generate(duration)
                    .to_homogeneous()
                    .as_slice()
                    .try_into()
                    .unwrap(),
            )
            .into()
        });

        loop {
            let state = self.robot_state.read().unwrap();
            let arm_state: ArmState<FRANKA_EMIKA_DOF> = (*state).into();
            drop(state);
            let pose = if let Some(Pose::Homo(pose)) = arm_state.pose_o_to_ee {
                array_to_isometry(&pose)
            } else {
                unreachable!()
            };

            if (target.translation.vector - pose.translation.vector).norm() < 0.01 {
                break;
            } else {
                sleep(Duration::from_millis(1));
            }
        }
        let _ = self._stop_move(());
        self.is_moving = false;
        Ok(())
    }

    fn move_linear_with_homo(&mut self, target: &[f64; 16], speed: f64) -> RobotResult<()> {
        self.move_linear_with_quat(&array_to_isometry(target), speed)
    }

    fn move_linear_with_euler(&mut self, target: &[f64; 6], speed: f64) -> RobotResult<()> {
        let translation = na::Translation3::new(target[0], target[1], target[2]);
        let rotation =
            na::Unit::<na::Quaternion<f64>>::from_euler_angles(target[3], target[4], target[5]);
        let target = na::Isometry3::from_parts(translation, rotation);
        self.move_linear_with_quat(&target, speed)
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
        let example = ArmState::<FRANKA_EMIKA_DOF>::default();
        self._move(closure(example, Duration::from_millis(1)).into())?;
        self.command_handle
            .set_closure(move |state, duration| closure((*state).into(), duration).into());
        Ok(())
    }

    fn control_with_closure<
        FC: Fn(ArmState<FRANKA_EMIKA_DOF>, Duration) -> ControlType<FRANKA_EMIKA_DOF> + Send + 'static,
    >(
        &mut self,
        closure: FC,
    ) -> RobotResult<()> {
        self.is_moving = true;
        let example = ArmState::<FRANKA_EMIKA_DOF>::default();
        self._move(closure(example, Duration::from_millis(1)).into())?;
        sleep(Duration::from_millis(1));
        self.command_handle.set_closure(move |state, duration| {
            state.error_result().unwrap();
            closure((*state).into(), duration).into()
        });
        Ok(())
    }

    fn move_to_target(&mut self) -> Arc<Mutex<Option<MotionType<FRANKA_EMIKA_DOF>>>> {
        unimplemented!()
    }

    fn control_to_target(&mut self) -> Arc<Mutex<Option<ControlType<FRANKA_EMIKA_DOF>>>> {
        unimplemented!()
    }
}

impl ArmRealtimeBehaviorExt<FRANKA_EMIKA_DOF> for FrankaRobot {
    fn move_joint_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        unimplemented!()
    }

    fn move_joint_vel_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        unimplemented!()
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
        unimplemented!()
    }

    fn move_cartesian_vel_target(&mut self) -> Arc<Mutex<Option<[f64; 6]>>> {
        unimplemented!()
    }

    fn control_tau_target(&mut self) -> Arc<Mutex<Option<[f64; FRANKA_EMIKA_DOF]>>> {
        unimplemented!()
    }
}
