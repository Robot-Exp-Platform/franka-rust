use nalgebra as na;
use robot_behavior::{
    ArmState, JointSample, LoadState, Pose, RobotResult, SpatialSample, StateView,
};
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;
use serde_repr::{Deserialize_repr, Serialize_repr};
use std::{fmt::Display, time::Duration};

use super::{
    robot_error::{ErrorFlag, FrankaError},
    robot_types::CommandIDConfig,
};

#[derive(Debug, Default, Serialize_repr, Deserialize_repr, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum MotionGeneratorMode {
    #[default]
    Idle,
    JointPosition,
    JointVelocity,
    CartesianPosition,
    CartesianVelocity,
    None,
}

#[derive(Debug, Default, Serialize_repr, Deserialize_repr, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum ControllerMode {
    #[default]
    JointImpedance,
    CartesianImpedance,
    ExternalController,
    Other,
}

#[derive(Debug, Default, Serialize_repr, Deserialize_repr, PartialEq, Clone, Copy)]
#[repr(u8)]
pub enum RobotMode {
    Other,
    #[default]
    Idle,
    Move,
    Guiding,
    Reflux,
    UserStopped,
    AutomaticErrorRecovery,
}

/// # RobotState
/// 鏈哄櫒浜虹姸鎬佺粨鏋勪綋
pub struct RobotState {
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// Measured end effector pose **in base frame**.\
    /// 鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑浣嶅Э
    pub pose_o_to_ee: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// last desired end effector pose of motion generation **in base frame**.\
    /// 涓婁竴娆¤繍鍔ㄧ敓鎴愬櫒鎻愪緵鐨勬寚浠わ紝鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑浣嶅Э
    pub pose_o_to_ee_d: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// End effortor frame pose **in flange frame**.\
    /// 鏈鎵ц鍣ㄥ湪**娉曞叞鍧愭爣绯?*涓嬬殑浣嶅Э
    pub pose_f_to_ee: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// Nominal end effector frame pose **in flange frame**.\
    /// 鏍囩О鏈鎵ц鍣ㄥ湪**娉曞叞鍧愭爣绯?*涓嬬殑浣嶅Э
    pub pose_f_to_ne: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// Nominal end effector frame pose **in end effector frame**.\
    /// 鏍囩О鏈鎵ц鍣ㄥ湪**鏍囩О鏈鎵ц鍣ㄥ潗鏍囩郴**涓嬬殑浣嶅Э
    pub pose_ne_to_ee: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// Stiffness frame pose **in end effector frame**.\
    /// 鍒氬害鍧愭爣绯诲湪**鏈鎵ц鍣ㄥ潗鏍囩郴**涓嬬殑浣嶅Э
    pub pose_ee_to_k: [f64; 16],
    /// Configured mass of the end effector.\
    /// 鏈鎵ц鍣ㄧ殑璐ㄩ噺
    pub m_ee: f64,
    /// Configured center of mass of the end effector load with respect to the flange frame.\
    /// 鏈鎵ц鍣ㄨ礋杞界浉瀵逛簬娉曞叞鍧愭爣绯荤殑璐ㄥ績
    pub x_ee: [f64; 3],
    /// Configured inertia of the end effector.\
    /// 鏈鎵ц鍣ㄧ殑鎯噺
    pub i_ee: [f64; 9],
    /// Configured mass of the external load.
    pub m_load: f64,
    /// Configured center of mass of the external load with respect to the flange frame.
    pub x_load: [f64; 3],
    /// Configured inertia matrix of the external load with respect to the flange frame.
    pub i_load: [f64; 9],
    /// Sum of the mass of the end effector and the external load.
    pub m_total: f64,
    /// Combined center of mass of the end effector and the external load.
    pub x_total: [f64; 3],
    /// Combined rotational inertia matrix of the end effector and the external load.
    pub i_total: [f64; 9],
    /// Elbow configuration.\
    /// 鑲橀儴閰嶇疆
    ///
    /// elbow_d\[0\]  Position of the 3rd joint in $rad$.
    /// elbow_d\[1\]  Flip direction of the elbow (4th joint)
    /// - +1 if $q_4 > q_{elbow-flip}$
    /// - 0  if $q_4 = q_{elbow-flip}$
    /// - -1 if $q_4 < q_{elbow-flip}$
    pub elbow: [f64; 2],
    /// Desired elbow configuration.\
    /// 鏈熸湜鑲橀儴閰嶇疆
    ///
    /// elbow_d\[0\]  Position of the 3rd joint in $rad$.
    /// elbow_d\[1\]  Flip direction of the elbow (4th joint)
    /// - +1 if $q_4 > q_{elbow-flip}$
    /// - 0  if $q_4 = q_{elbow-flip}$
    /// - -1 if $q_4 < q_{elbow-flip}$
    pub elbow_d: [f64; 2],
    /// Commanded elbow configuration.\
    /// 鎸囦护鑲橀儴閰嶇疆
    ///
    /// `elbow_d`\[0\]  Position of the 3rd joint in $rad$.
    /// `elbow_d`\[1\]  Flip direction of the elbow (4th joint)
    /// - +1 if $q_4 > q_{elbow-flip}$
    /// - 0  if $q_4 = q_{elbow-flip}$
    /// - -1 if $q_4 < q_{elbow-flip}$
    pub elbow_c: [f64; 2],
    /// Commanded elbow velocity.\
    /// 鎸囦护鑲橀儴閫熷害
    ///
    /// delbow_c[0] Velocity of the 3rd joint in $rad/s$.
    /// delbow_c[1] is always zero.
    pub delbow_c: [f64; 2],
    /// Commanded elbow acceleration.\
    /// 鎸囦护鑲橀儴鍔犻€熷害
    ///
    /// `ddelbow_c`\[0\] Acceleration of the 3rd joint in $\frac{rad}{s^2}$.
    /// `ddelbow_c`\[1\] is always zero.
    pub ddelbow_c: [f64; 2],
    /// Measured joint torques.\
    /// 娴嬮噺鍏宠妭鍔涚煩
    pub tau_j: [f64; 7],
    /// Desired joint torques.\
    /// 鏈熸湜鍏宠妭鍔涚煩
    pub tau_j_d: [f64; 7],
    /// Derivative of measured joint torques in $\frac{Nm}{s}$.
    pub dtau_j: [f64; 7],
    /// Measured joint positions.\
    /// 娴嬮噺鍏宠妭浣嶇疆
    pub q: [f64; 7],
    /// Desired joint positions.\
    /// 鏈熸湜鍏宠妭浣嶇疆
    pub q_d: [f64; 7],
    /// Measured joint velocities in $\frac{rad}{s}$.
    /// 娴嬮噺鍏宠妭閫熷害
    pub dq: [f64; 7],
    /// Desired joint velocities in $\frac{rad}{s}$.
    /// 鏈熸湜鍏宠妭閫熷害
    pub dq_d: [f64; 7],
    /// Desired joint accelerations in $frac{rad}{s^2}$.
    /// 鏈熸湜鍏宠妭鍔犻€熷害
    pub ddq_d: [f64; 7],
    /// Indicate which contact level is activated in which joint.\
    /// After contact disappears, value turns to zero.\
    /// 鎸囩ず鍏宠妭婵€娲讳簡鍝釜鎺ヨЕ绾у埆. 鎺ヨЕ娑堝け鍚庯紝璇ュ€煎彉涓洪浂
    /// ## Use
    /// - [`Robot::set_Collision_behavior`](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.
    pub joint_contact: [f64; 7],
    /// Indicate which contact level is activated in which Cartesian direction $x, y, z, R, P, Y$.\
    /// After contact disappears, the value turns to zero.\
    /// 鎸囩ず绗涘崱灏旀柟鍚?$x, y, z, R, P, Y$ 婵€娲讳簡鍝釜鎺ヨЕ绾у埆. 鎺ヨЕ娑堝け鍚庯紝璇ュ€煎彉涓洪浂
    /// ## Use
    /// - [`Robot::set_collision_behavior`](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.\
    pub cartesian_contact: [f64; 6],
    /// Indicate which collision level is activated in which joint.\
    /// After contact disappears, the value stays the same until a reset command is sent.\
    /// 鎸囩ず鍏宠妭婵€娲讳簡鍝釜纰版挒绾у埆. 鎺ヨЕ娑堝け鍚庯紝璇ュ€间繚鎸佷笉鍙橈紝鐩村埌鍙戦€侀噸缃懡浠?    /// ## Use
    /// - [`Robot::set_collision_behavior`](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.
    /// - [`Robot::automatic_error_recovery`](crate::robot::Robot::automatic_error_recovery) for performing a reset after a collision.
    pub joint_collision: [f64; 7],
    /// Indicate which collision level is activated in which Cartesian direction $x, y, z, R, P, Y$.\
    /// After contact disappears, the value stays the same until a reset command is sent.\
    /// 鎸囩ず绗涘崱灏旀柟鍚?$x, y, z, R, P, Y$ 婵€娲讳簡鍝釜纰版挒绾у埆. 鎺ヨЕ娑堝け鍚庯紝璇ュ€间繚鎸佷笉鍙橈紝鐩村埌鍙戦€侀噸缃懡浠?    /// ## Use
    /// - [`Robot::set_collision_behavior`](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.
    /// - [`Robot::automatic_error_recovery`](crate::robot::Robot::automatic_error_recovery) for performing a reset after a collision.
    pub cartesian_collision: [f64; 6],
    /// low-pass filtered torques generated by the external forces on the joint.\
    /// It does not include configured end-effector and load nor the mass and dynamics of the robot.
    /// `tau_ext_hat_filtered` is the error between `tau_J` and the expected torques given by the robot model.
    /// 鍏宠妭涓婂閮ㄥ姏浜х敓鐨勪綆閫氭护娉㈠姏鐭? 瀹冧笉鍖呮嫭閰嶇疆鐨勬湯绔墽琛屽櫒鍜岃礋杞斤紝涔熶笉鍖呮嫭鏈哄櫒浜虹殑璐ㄩ噺鍜屽姩鍔涘.
    /// `tau_ext_hat_filtered` 鏄?`tau_J` 鍜屾満鍣ㄤ汉妯″瀷缁欏嚭鐨勬湡鏈涘姏鐭╀箣闂寸殑璇樊
    pub tau_ext_hat_filtered: [f64; 7],
    /// Estimated external wrench(force and torque) actiong on stiffness frame ,
    /// expressed relative in the base frame.\
    /// 鐩稿浜庡熀搴ф爣绯讳綔鐢ㄥ湪鍒氬害鍧愭爣绯讳笂棰勪及鐨勫閮ㄥ箍涔夊姏(鍔涘拰鍔涚煩)
    ///
    /// Forces applied by the robot to the environment are positive,
    /// while forces applied by the environment on the robot are negative.
    /// Becomes
    /// \[0,0,0,0,0,0\] when near or in a singularity.
    pub force_ext_in_o: [f64; 6],
    /// Estimated external wrench(force and torque) actiong on stiffness frame in $\[N,N,Nm,Nm,Nm,Nm\]$ ,
    /// expressed relative in the stiffness frame.\
    /// 鐩稿浜庡垰搴﹀潗鏍囩郴浣滅敤鍦ㄥ垰搴﹀潗鏍囩郴涓婇浼扮殑澶栭儴骞夸箟鍔?鍔涘拰鍔涚煩)
    ///
    /// Forces applied by the robot to the environment are positive,
    /// while forces applied by the environment on the robot are negative.
    /// Becomes
    /// \[0,0,0,0,0,0\] when near or in a singularity.
    pub force_ext_in_k: [f64; 6],
    /// Desired end effector twist **in base frame**.\
    /// 鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑閫熷害铻烘棆
    pub dpose_o_to_ee_d: [f64; 6],
    /// Desired end effector acceleration **in base frame** in $\[\frac{m}{s^2},\frac{m}{s^2},\frac{m}{s^2},\frac{rad}{s^2},\frac{rad}{s^2},\frac{rad}{s^2}\]$.\
    /// 鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑鏈熸湜鍔犻€熷害铻烘棆
    pub ddpose_o_to_ee: [f64; 3],
    /// A 4x4 homogeneous transformation matrix in column-major format.\
    /// Last commanded end effector pose of motion generation **in base frame**.\
    /// 涓婁竴娆¤繍鍔ㄧ敓鎴愬櫒鎻愪緵鐨勬寚浠わ紝鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑浣嶅Э
    pub pose_o_to_ee_c: [f64; 16],
    /// Last commanded end effector twist of motion generation **in base frame**.\
    /// 涓婁竴娆¤繍鍔ㄧ敓鎴愬櫒鎻愪緵鐨勬寚浠わ紝鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑閫熷害铻烘棆
    pub dpose_o_to_ee_c: [f64; 6],
    /// Last commanded end effector acceleration of motion generation **in base frame**.\
    /// 涓婁竴娆¤繍鍔ㄧ敓鎴愬櫒鎻愪緵鐨勬寚浠わ紝鏈鎵ц鍣ㄥ湪**鍩哄潗鏍囩郴**涓嬬殑鏈熸湜鍔犻€熷害铻烘棆
    pub ddpose_o_to_ee_c: [f64; 6],
    /// Motor positions.\
    /// 鐢垫満浣嶇疆
    pub theta: [f64; 7],
    /// Motor velocities in $\frac{rad}{s}$.
    /// 鐢垫満閫熷害
    pub dtheta: [f64; 7],
    /// Current error state.
    pub current_errors: Option<FrankaError>,
    /// contain the errors that aborted the previous motion.\
    /// 鍖呭惈涓涓婁竴娆¤繍鍔ㄧ殑閿欒
    pub last_motion_errors: Option<FrankaError>,
    /// Percentage of the last 100 control commands that were successfully received by the robot.\
    /// Shows a value of zero if no control or motion generator loop is currently running.\
    /// 鏈€鍚?00涓帶鍒跺懡浠や腑鎴愬姛鎺ユ敹鍒扮殑鐧惧垎姣? 濡傛灉褰撳墠娌℃湁鎺у埗鎴栬繍鍔ㄧ敓鎴愬櫒寰幆姝ｅ湪杩愯锛屽垯鏄剧ず涓洪浂
    ///
    /// **Range**: \[0, 1\]
    pub control_command_success_rate: f64,
    /// Current robot mode.
    pub robot_mode: RobotMode,
    /// Strictly monotonically increasing timestamp since robot start.
    ///   
    /// 鏈哄櫒浜哄惎鍔ㄤ互鏉ヤ弗鏍煎崟璋冮€掑鐨勬椂闂存埑
    ///
    /// Inside of control loops [`time_step`] parameter of [`Robot::control`] can be used instead.
    pub duration: Duration,
}

#[derive(Debug, Serialize, Deserialize, Clone, Copy)]
#[allow(non_snake_case)]
#[repr(C, packed)]
pub struct RobotStateInter {
    pub message_id: u64,
    pub O_T_EE: [f64; 16],
    pub O_T_EE_d: [f64; 16],
    pub F_T_EE: [f64; 16],
    pub EE_T_K: [f64; 16],
    pub F_T_NE: [f64; 16],
    pub NE_T_EE: [f64; 16],
    pub m_ee: f64,
    pub I_ee: [f64; 9],
    pub F_x_Cee: [f64; 3],
    pub m_load: f64,
    pub I_load: [f64; 9],
    pub F_x_Cload: [f64; 3],
    pub elbow: [f64; 2],
    pub elbow_d: [f64; 2],
    pub tau_J: [f64; 7],
    pub tau_J_d: [f64; 7],
    pub dtau_J: [f64; 7],
    pub q: [f64; 7],
    pub q_d: [f64; 7],
    pub dq: [f64; 7],
    pub dq_d: [f64; 7],
    pub ddq_d: [f64; 7],
    pub joint_contact: [f64; 7],
    pub cartesian_contact: [f64; 6],
    pub joint_collision: [f64; 7],
    pub cartesian_collision: [f64; 6],
    pub tau_ext_hat_filtered: [f64; 7],
    pub O_F_ext_hat_K: [f64; 6],
    pub K_F_ext_hat_K: [f64; 6],
    pub O_dP_EE_d: [f64; 6],
    pub O_ddP_O: [f64; 3],
    pub elbow_c: [f64; 2],
    pub delbow_c: [f64; 2],
    pub ddelbow_c: [f64; 2],
    pub O_T_EE_c: [f64; 16],
    pub O_dP_EE_c: [f64; 6],
    pub O_ddP_EE_c: [f64; 6],
    pub theta: [f64; 7],
    pub dtheta: [f64; 7],
    pub motion_generator_mode: MotionGeneratorMode,
    pub controller_mode: ControllerMode,
    #[serde(with = "BigArray")]
    pub errors: [bool; 41],
    #[serde(with = "BigArray")]
    pub reflex_reason: [bool; 41],
    pub robot_mode: RobotMode,
    pub control_command_success_rate: f64,
}

impl RobotStateInter {
    pub fn error_result(&self) -> RobotResult<()> {
        let flags: ErrorFlag = self.errors.into();
        if flags.is_empty() {
            Ok(())
        } else {
            Err(robot_behavior::RobotException::CommandException(format!(
                "{flags}"
            )))
        }
    }
}

impl From<RobotStateInter> for RobotState {
    fn from(val: RobotStateInter) -> Self {
        let (m_total, x_total, i_total) = combine_ee_load(
            val.m_ee,
            val.F_x_Cee,
            val.I_ee,
            val.m_load,
            val.F_x_Cload,
            val.I_load,
        );
        let currect_error: ErrorFlag = val.errors.into();
        let last_motion_error: ErrorFlag = val.reflex_reason.into();
        RobotState {
            pose_o_to_ee: val.O_T_EE,
            pose_o_to_ee_d: val.O_T_EE_d,
            pose_f_to_ee: val.F_T_EE,
            pose_f_to_ne: val.F_T_NE,
            pose_ne_to_ee: val.NE_T_EE,
            pose_ee_to_k: val.EE_T_K,
            m_ee: val.m_ee,
            x_ee: val.F_x_Cee,
            i_ee: val.I_ee,
            m_load: val.m_load,
            i_load: val.I_load,
            x_load: val.F_x_Cload,
            m_total,
            i_total,
            x_total,
            elbow: val.elbow,
            elbow_d: val.elbow_d,
            tau_j: val.tau_J,
            tau_j_d: val.tau_J_d,
            dtau_j: val.dtau_J,
            q: val.q,
            q_d: val.q_d,
            dq: val.dq,
            dq_d: val.dq_d,
            ddq_d: val.ddq_d,
            joint_contact: val.joint_contact,
            cartesian_contact: val.cartesian_contact,
            joint_collision: val.joint_collision,
            cartesian_collision: val.cartesian_collision,
            tau_ext_hat_filtered: val.tau_ext_hat_filtered,
            force_ext_in_o: val.O_F_ext_hat_K,
            force_ext_in_k: val.K_F_ext_hat_K,
            dpose_o_to_ee_d: val.O_dP_EE_d,
            ddpose_o_to_ee: val.O_ddP_O,
            elbow_c: val.elbow_c,
            delbow_c: val.delbow_c,
            ddelbow_c: val.ddelbow_c,
            pose_o_to_ee_c: val.O_T_EE_c,
            dpose_o_to_ee_c: val.O_dP_EE_c,
            ddpose_o_to_ee_c: val.O_ddP_EE_c,
            theta: val.theta,
            dtheta: val.dtheta,
            current_errors: currect_error.into(),
            last_motion_errors: last_motion_error.into(),
            control_command_success_rate: val.control_command_success_rate,
            robot_mode: val.robot_mode,
            duration: Duration::from_millis(val.message_id),
        }
    }
}

impl From<RobotStateInter> for ArmState<7> {
    fn from(val: RobotStateInter) -> Self {
        let (m, x, i) = combine_ee_load(
            val.m_ee,
            val.F_x_Cee,
            val.I_ee,
            val.m_load,
            val.F_x_Cload,
            val.I_load,
        );
        ArmState {
            joint: StateView {
                meas: JointSample {
                    q: Some(val.q),
                    dq: Some(val.dq),
                    ddq: None,
                    tau: Some(val.tau_J),
                    dtau: Some(val.dtau_J),
                },
                des: JointSample {
                    q: Some(val.q_d),
                    dq: Some(val.dq_d),
                    ddq: Some(val.ddq_d),
                    tau: Some(val.tau_J_d),
                    dtau: None,
                },
                cmd: JointSample {
                    q: Some(val.theta),
                    dq: Some(val.dtheta),
                    ddq: None,
                    tau: None,
                    dtau: None,
                },
            },
            flange: StateView {
                meas: SpatialSample {
                    pose: Some(Pose::Homo(val.O_T_EE)),
                    vel: None,
                    acc: None,
                    wrench: Some(val.O_F_ext_hat_K),
                },
                des: SpatialSample {
                    pose: Some(Pose::Homo(val.O_T_EE_d)),
                    vel: Some(val.O_dP_EE_d),
                    acc: Some([
                        val.O_ddP_O[0],
                        val.O_ddP_O[1],
                        val.O_ddP_O[2],
                        0.0,
                        0.0,
                        0.0,
                    ]),
                    wrench: None,
                },
                cmd: SpatialSample {
                    pose: Some(Pose::Homo(val.O_T_EE_c)),
                    vel: Some(val.O_dP_EE_c),
                    acc: Some(val.O_ddP_EE_c),
                    wrench: None,
                },
            },
            tcp: None,
            stiffness: Some(StateView::from_meas(SpatialSample {
                pose: Some(Pose::Homo(val.EE_T_K)),
                vel: None,
                acc: None,
                wrench: Some(val.K_F_ext_hat_K),
            })),
            load: Some(LoadState { m, x, i }),
        }
    }
}

fn combine_ee_load(
    m_ee: f64,
    x_ee: [f64; 3],
    i_ee: [f64; 9],
    m_load: f64,
    x_load: [f64; 3],
    i_load: [f64; 9],
) -> (f64, [f64; 3], [f64; 9]) {
    let x_ee = na::Vector3::from_column_slice(&x_ee);
    let x_load = na::Vector3::from_column_slice(&x_load);
    let i_ee = na::Matrix3::from_column_slice(&i_ee);
    let i_load = na::Matrix3::from_column_slice(&i_load);

    let m_total = m_ee + m_load;
    let x_total = (m_ee * x_ee + m_load * x_load) / m_total;
    let i_total = i_ee
        + i_load
        + m_ee
            * (na::Matrix3::from_diagonal_element((x_ee.transpose() * x_ee)[(0, 0)])
                - x_ee * x_ee.transpose())
        + m_load
            * (na::Matrix3::from_diagonal_element((x_load.transpose() * x_load)[(0, 0)])
                - x_load * x_load.transpose());

    let x_total = x_total.as_slice().try_into().unwrap();
    let i_total = i_total.as_slice().try_into().unwrap();

    (m_total, x_total, i_total)
}

impl Default for RobotStateInter {
    fn default() -> Self {
        RobotStateInter {
            message_id: 0,
            O_T_EE: [0.0; 16],
            O_T_EE_d: [0.0; 16],
            F_T_EE: [0.0; 16],
            EE_T_K: [0.0; 16],
            F_T_NE: [0.0; 16],
            NE_T_EE: [0.0; 16],
            m_ee: 0.0,
            I_ee: [0.0; 9],
            F_x_Cee: [0.0; 3],
            m_load: 0.0,
            I_load: [0.0; 9],
            F_x_Cload: [0.0; 3],
            elbow: [0.0; 2],
            elbow_d: [0.0; 2],
            tau_J: [0.0; 7],
            tau_J_d: [0.0; 7],
            dtau_J: [0.0; 7],
            q: [0.0; 7],
            q_d: [0.0; 7],
            dq: [0.0; 7],
            dq_d: [0.0; 7],
            ddq_d: [0.0; 7],
            joint_contact: [0.0; 7],
            cartesian_contact: [0.0; 6],
            joint_collision: [0.0; 7],
            cartesian_collision: [0.0; 6],
            tau_ext_hat_filtered: [0.0; 7],
            O_F_ext_hat_K: [0.0; 6],
            K_F_ext_hat_K: [0.0; 6],
            O_dP_EE_d: [0.0; 6],
            O_ddP_O: [0.0; 3],
            elbow_c: [0.0; 2],
            delbow_c: [0.0; 2],
            ddelbow_c: [0.0; 2],
            O_T_EE_c: [0.0; 16],
            O_dP_EE_c: [0.0; 6],
            O_ddP_EE_c: [0.0; 6],
            theta: [0.0; 7],
            dtheta: [0.0; 7],
            motion_generator_mode: MotionGeneratorMode::Idle,
            controller_mode: ControllerMode::JointImpedance,
            errors: [false; 41],
            reflex_reason: [false; 41],
            robot_mode: RobotMode::Idle,
            control_command_success_rate: 0.0,
        }
    }
}

impl CommandIDConfig<u64> for RobotStateInter {
    fn set_command_id(&mut self, id: u64) {
        self.message_id = id;
    }
    fn command_id(&self) -> u64 {
        self.message_id
    }
    fn time(&self) -> Option<Duration> {
        Some(Duration::from_millis(self.message_id))
    }
}

impl Display for RobotStateInter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let robot_state: RobotState = (*self).into();
        let message_id = self.message_id;
        let success_rate = self.control_command_success_rate;
        let errors = self.error_result();
        write!(
            f,
            r"robot state:
    | message_id: {message_id}, success_rate: {success_rate}, errors: {errors:?},
    | q: {:?},
    | q_d: {:?},
    | dq_d: {:?},
    | ddq_d: {:?},
    | tau: {:?},
    | tau_j: {:?}",
            robot_state.q,
            robot_state.q_d,
            robot_state.dq_d,
            robot_state.ddq_d,
            robot_state.tau_j,
            robot_state.tau_j_d
        )
    }
}

#[cfg(test)]
mod test {
    use std::mem::offset_of;

    use super::*;

    #[test]
    fn test_combine_ee_load() {
        let m_ee = 1.0;
        let x_ee = [1.0, 2.0, 3.0];
        let i_ee = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        let m_load = 2.0;
        let x_load = [2.0, 3.0, 4.0];
        let i_load = [2.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 2.0];

        let (m_total, x_total, i_total) = combine_ee_load(m_ee, x_ee, i_ee, m_load, x_load, i_load);

        assert_eq!(m_total, 3.0);
        assert_eq!(x_total, [1.5, 2.5, 3.5]);
        assert_eq!(i_total, [3.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 3.0]);
    }

    #[derive(Serialize, Default, Deserialize, Debug, Copy, Clone)]
    #[allow(non_snake_case)]
    #[repr(C, packed)]
    pub struct RobotStateIntern {
        pub message_id: u64,
        pub O_T_EE: [f64; 16],
        pub O_T_EE_d: [f64; 16],
        pub F_T_EE: [f64; 16],
        pub EE_T_K: [f64; 16],
        pub F_T_NE: [f64; 16],
        pub NE_T_EE: [f64; 16],
        pub m_ee: f64,
        pub I_ee: [f64; 9],
        pub F_x_Cee: [f64; 3],
        pub m_load: f64,
        pub I_load: [f64; 9],
        pub F_x_Cload: [f64; 3],
        pub elbow: [f64; 2],
        pub elbow_d: [f64; 2],
        pub tau_J: [f64; 7],
        pub tau_J_d: [f64; 7],
        pub dtau_J: [f64; 7],
        pub q: [f64; 7],
        pub q_d: [f64; 7],
        pub dq: [f64; 7],
        pub dq_d: [f64; 7],
        pub ddq_d: [f64; 7],
        pub joint_contact: [f64; 7],
        pub cartesian_contact: [f64; 6],
        pub joint_collision: [f64; 7],
        pub cartesian_collision: [f64; 6],
        pub tau_ext_hat_filtered: [f64; 7],
        pub O_F_ext_hat_K: [f64; 6],
        pub K_F_ext_hat_K: [f64; 6],
        pub O_dP_EE_d: [f64; 6],
        pub O_ddP_O: [f64; 3],
        pub elbow_c: [f64; 2],
        pub delbow_c: [f64; 2],
        pub ddelbow_c: [f64; 2],
        pub O_T_EE_c: [f64; 16],
        pub O_dP_EE_c: [f64; 6],
        pub O_ddP_EE_c: [f64; 6],
        pub theta: [f64; 7],
        pub dtheta: [f64; 7],
        pub motion_generator_mode: MotionGeneratorMode,
        pub controller_mode: ControllerMode,
        // pub errors: [bool; 37], reenable when const generics arrive
        // pub reflex_reason: [bool; 37], reenable when const generics arrive
        pub errors: RoboErrorHelperStruct,
        pub robot_mode: RobotMode,
        pub control_command_success_rate: f64,
    }

    #[derive(Default, Serialize, Deserialize, Debug, Copy, Clone, PartialEq)]
    #[repr(C, packed)]
    pub struct RoboErrorHelperStruct {
        pub errors1: [bool; 32],
        pub errors2: [bool; 9],
        pub reflex_reason1: [bool; 32],
        pub reflex_reason2: [bool; 9],
    }

    #[test]
    fn state_inter_size() {
        println!(
            "RobotStateInter size: {}",
            std::mem::size_of::<RobotStateInter>()
        );
        let robot_state_inter = RobotStateInter::default();
        let robot_state_intern = RobotStateIntern::default();

        assert_eq!(
            std::mem::size_of::<RobotStateInter>(),
            std::mem::size_of::<RobotStateIntern>()
        );
        assert_eq!(
            bincode::serialize(&robot_state_inter).unwrap().len(),
            bincode::serialize(&robot_state_intern).unwrap().len()
        );

        let data_1 = bincode::serialize(&robot_state_inter.robot_mode).unwrap();
        let data_2 = bincode::serialize(&robot_state_intern.robot_mode).unwrap();

        assert_eq!(data_1, data_2);
    }

    #[test]
    fn test_offect() {
        println!("q_d offset: {}", offset_of!(RobotStateInter, dtau_J));
    }

    #[test]
    fn display_robot_state() {
        let robot_state_inter = RobotStateInter::default();
        println!("{robot_state_inter}");
    }
}
