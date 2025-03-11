use nalgebra as na;
use robot_behavior::ArmState;
use serde::{Deserialize, Serialize};
use serde_big_array::BigArray;
use serde_repr::{Deserialize_repr, Serialize_repr};
use std::time::Duration;

#[derive(Default, Serialize_repr, Deserialize_repr, Clone, Copy)]
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

#[derive(Default, Serialize_repr, Deserialize_repr, Clone, Copy)]
#[repr(u8)]
pub enum ControllerMode {
    #[default]
    JointImpedance,
    CartesianImpedance,
    ExternalController,
    Other,
}

#[derive(Default, Serialize_repr, Deserialize_repr, Clone, Copy)]
#[repr(u8)]
pub enum RobotMode {
    Other,
    Idle,
    Move,
    Guiding,
    Reflux,
    #[default]
    UserStopped,
    AutomaticErrorRecovery,
}

/// # RobotState
/// 机器人状态结构体
pub struct RobotState {
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// Measured end effector pose **in base frame**.  
    /// 末端执行器在**基坐标系**下的位姿
    pub pose_o_to_ee: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// last desired end effector pose of motion generation **in base frame**.  
    /// 上一次运动生成器提供的指令，末端执行器在**基坐标系**下的位姿
    pub pose_o_to_ee_d: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// End effortor frame pose **in flange frame**.  
    /// 末端执行器在**法兰坐标系**下的位姿
    pub pose_f_to_ee: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// Nominal end effector frame pose **in flange frame**.  
    /// 标称末端执行器在**法兰坐标系**下的位姿
    pub pose_f_to_ne: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// Nominal end effector frame pose **in end effector frame**.  
    /// 标称末端执行器在**标称末端执行器坐标系**下的位姿
    pub pose_ne_to_ee: [f64; 16],
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// Stiffness frame pose **in end effector frame**.  
    /// 刚度坐标系在**末端执行器坐标系**下的位姿
    pub pose_ee_to_k: [f64; 16],
    /// Configured mass of the end effector.  
    /// 末端执行器的质量
    pub m_ee: f64,
    /// Configured center of mass of the end effector load with respect to the flange frame.  
    /// 末端执行器负载相对于法兰坐标系的质心
    pub x_ee: [f64; 3],
    /// Configured inertia of the end effector.  
    /// 末端执行器的惯量
    pub i_ee: [f64; 9],
    /// Configured mass of the External load.  
    /// 外部负载的质量
    pub m_load: f64,
    /// Configured center of mass of the External load with respect to the flange frame.  
    /// 外部负载相对于法兰坐标系的质心
    pub x_load: [f64; 3],
    /// Configured inertia matrix of the External load with respect to the flange frame.  
    /// 外部负载相对于法兰坐标系的惯量矩阵
    pub i_load: [f64; 9],
    /// Sum of the mass of the end effector and the external load.  
    /// 末端执行器和外部负载的质量之和
    pub m_total: f64,
    /// Combined center of mass of the end effector and the external load with respect to the flange frame.  
    /// 末端执行器和外部负载相对于法兰坐标系的组合质心
    pub x_total: [f64; 3],
    /// Combined rotational inertia matrix of the end effector and the external load with respect to the flange frame.  
    /// 末端执行器和外部负载相对于法兰坐标系的组合旋转惯量矩阵
    pub i_total: [f64; 9],
    /// Elbow configuration.  
    /// 肘部配置
    ///
    /// elbow_d\[0\]  Position of the 3rd joint in $rad$.
    /// elbow_d\[1\]  Flip direction of the elbow (4th joint)
    /// - +1 if $q_4 > q_{elbow-flip}$
    /// - 0  if $q_4 = q_{elbow-flip}$
    /// - -1 if $q_4 < q_{elbow-flip}$
    pub elbow: [f64; 2],
    /// Desired elbow configuration.  
    /// 期望肘部配置
    ///
    /// elbow_d\[0\]  Position of the 3rd joint in $rad$.
    /// elbow_d\[1\]  Flip direction of the elbow (4th joint)
    /// - +1 if $q_4 > q_{elbow-flip}$
    /// - 0  if $q_4 = q_{elbow-flip}$
    /// - -1 if $q_4 < q_{elbow-flip}$
    pub elbow_d: [f64; 2],
    /// Commanded elbow configuration.  
    /// 指令肘部配置
    ///
    /// elbow_d\[0\]  Position of the 3rd joint in $rad$.
    /// elbow_d\[1\]  Flip direction of the elbow (4th joint)
    /// - +1 if $q_4 > q_{elbow-flip}$
    /// - 0  if $q_4 = q_{elbow-flip}$
    /// - -1 if $q_4 < q_{elbow-flip}$
    pub elbow_c: [f64; 2],
    /// Commanded elbow velocity.  
    /// 指令肘部速度
    ///
    /// delbow_c[0] Velocity of the 3rd joint in $rad/s$.
    /// delbow_c[1] is always zero.
    pub delbow_c: [f64; 2],
    /// Commanded elbow acceleration.  
    /// 指令肘部加速度
    ///
    /// ddelbow_c\[0\] Acceleration of the 3rd joint in $\frac{rad}{s^2}$.
    /// ddelbow_c\[1\] is always zero.
    pub ddelbow_c: [f64; 2],
    /// Measured joint torques.  
    /// 测量关节力矩
    pub tau_j: [f64; 7],
    /// Desired joint torques.  
    /// 期望关节力矩
    pub tau_j_d: [f64; 7],
    /// Derivative of Measured joint torques in $\frac{Nm}{s}$.
    /// 测量关节力矩的导数
    pub dtau_j: [f64; 7],
    /// Measured joint positions.  
    /// 测量关节位置
    pub q: [f64; 7],
    /// Desired joint positions.  
    /// 期望关节位置
    pub q_d: [f64; 7],
    /// Measured joint velocities in $\frac{rad}{s}$.
    /// 测量关节速度
    pub dq: [f64; 7],
    /// Desired joint velocities in $\frac{rad}{s}$.
    /// 期望关节速度
    pub dq_d: [f64; 7],
    /// Desired joint accelerations in $frac{rad}{s^2}$.
    /// 期望关节加速度
    pub ddq_d: [f64; 7],
    /// Indicate which contact level is activated in which joint.  
    /// After contact disappears, value turns to zero.  
    /// 指示关节激活了哪个接触级别. 接触消失后，该值变为零
    /// ## Use
    /// - [Robot::set_Collision_behavior](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.
    pub joint_contact: [f64; 7],
    /// Indicate which contact level is activated in which Cartesian direction $x, y, z, R, P, Y$.  
    /// After contact disappears, the value turns to zero.  
    /// 指示笛卡尔方向 $x, y, z, R, P, Y$ 激活了哪个接触级别. 接触消失后，该值变为零
    /// ## Use
    /// - [Robot::set_Collision_behavior](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.  
    pub cartesian_contact: [f64; 6],
    /// Indicate which collision level is activated in which joint.  
    /// After contact disappears, the value stays the same until a reset command is sent.  
    /// 指示关节激活了哪个碰撞级别. 接触消失后，该值保持不变，直到发送重置命令
    /// ## Use
    /// - [Robot::set_collision_behavior](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.
    /// - [Robot::automatic_error_recovery](crate::robot::Robot::automatic_error_recovery) for performing a reset after a collision.
    pub joint_collision: [f64; 7],
    /// Indicate which collision level is activated in which Cartesian direction $x, y, z, R, P, Y$.  
    /// After contact disappears, the value stays the same until a reset command is sent.  
    /// 指示笛卡尔方向 $x, y, z, R, P, Y$ 激活了哪个碰撞级别. 接触消失后，该值保持不变，直到发送重置命令
    /// ## Use
    /// - [Robot::set_collision_behavior](crate::robot::Robot::set_collision_behavior) for setting sensitivity values.
    /// - [Robot::automatic_error_recovery](crate::robot::Robot::automatic_error_recovery) for performing a reset after a collision.
    pub cartesian_collision: [f64; 6],
    /// low-pass filtered torques generated by the external forces on the joint.  
    /// It does not include configured end-effector and load nor the mass and dynamics of the robot.
    /// tau_ext_hat_filtered is the error between tau_J and the expected torques given by the robot model.
    /// 关节上外部力产生的低通滤波力矩. 它不包括配置的末端执行器和负载，也不包括机器人的质量和动力学.
    /// tau_ext_hat_filtered 是 tau_J 和机器人模型给出的期望力矩之间的误差
    pub tau_ext_hat_filtered: [f64; 7],
    /// Estimated external wrench(force and torque) actiong on stiffness frame ,
    /// expressed relative in the base frame.  
    /// 相对于基座标系作用在刚度坐标系上预估的外部广义力(力和力矩)
    ///
    /// Forces applied by the robot to the environment are positive,
    /// while forces applied by the environment on the robot are negative.
    /// Becomes
    /// \[0,0,0,0,0,0\] when near or in a singularity.
    pub force_ext_in_o: [f64; 6],
    /// Estimated external wrench(force and torque) actiong on stiffness frame in $\[N,N,Nm,Nm,Nm,Nm\]$ ,
    /// expressed relative in the stiffness frame.  
    /// 相对于刚度坐标系作用在刚度坐标系上预估的外部广义力(力和力矩)
    ///
    /// Forces applied by the robot to the environment are positive,
    /// while forces applied by the environment on the robot are negative.
    /// Becomes
    /// \[0,0,0,0,0,0\] when near or in a singularity.
    pub force_ext_in_k: [f64; 6],
    /// Desired end effector twist **in base frame**.  
    /// 末端执行器在**基坐标系**下的速度螺旋
    pub dpose_o_to_ee_d: [f64; 6],
    /// Desired end effector acceleration **in base frame** in $\[\frac{m}{s^2},\frac{m}{s^2},\frac{m}{s^2},\frac{rad}{s^2},\frac{rad}{s^2},\frac{rad}{s^2}\]$.  
    /// 末端执行器在**基坐标系**下的期望加速度螺旋
    pub ddpose_o_to_ee: [f64; 3],
    /// A 4x4 homogeneous transformation matrix in column-major format.  
    /// Last commanded end effector pose of motion generation **in base frame**.  
    /// 上一次运动生成器提供的指令，末端执行器在**基坐标系**下的位姿
    pub pose_o_to_ee_c: [f64; 16],
    /// Last commanded end effector twist of motion generation **in base frame**.  
    /// 上一次运动生成器提供的指令，末端执行器在**基坐标系**下的速度螺旋
    pub dpose_o_to_ee_c: [f64; 6],
    /// Last commanded end effector acceleration of motion generation **in base frame**.  
    /// 上一次运动生成器提供的指令，末端执行器在**基坐标系**下的期望加速度螺旋
    pub ddpose_o_to_ee_c: [f64; 6],
    /// Motor positions.  
    /// 电机位置
    pub theta: [f64; 7],
    /// Motor velocities in $\frac{rad}{s}$.
    /// 电机速度
    pub dtheta: [f64; 7],
    /// Currect error state.  
    /// 当前错误状态
    pub currect_errors: [bool; 41],
    /// contain the errirs that aborted the previous motion.  
    /// 包含中止上一次运动的错误
    pub last_motion_errors: [bool; 41],
    /// Percentage of the last 100 control commands that were successfully received by the robot.  
    /// Shows a value of zero if no control or motion generator loop is currently running.
    /// 最后100个控制命令中成功接收到的百分比, 如果当前没有控制或运动生成器循环正在运行，则显示为零
    ///
    /// **Range**: \[0, 1\]
    pub control_command_success_rate: f64,
    /// Current robot mode.  
    /// 当前机器人模式
    pub robot_mode: RobotMode,
    /// Strictly monotonically increasing timestamp since robot start.  
    /// 机器人启动以来严格单调递增的时间戳
    ///
    /// Inside of control loops [time_step] parameter of [Robot::control] can be used instead.
    pub duration: Duration,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
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

impl Into<RobotState> for RobotStateInter {
    fn into(self) -> RobotState {
        let (m_total, x_total, i_total) = combine_ee_load(
            self.m_ee,
            self.F_x_Cee,
            self.I_ee,
            self.m_load,
            self.F_x_Cload,
            self.I_load,
        );
        RobotState {
            pose_o_to_ee: self.O_T_EE,
            pose_o_to_ee_d: self.O_T_EE_d,
            pose_f_to_ee: self.F_T_EE,
            pose_f_to_ne: self.F_T_NE,
            pose_ne_to_ee: self.NE_T_EE,
            pose_ee_to_k: self.EE_T_K,
            m_ee: self.m_ee,
            x_ee: self.F_x_Cee,
            i_ee: self.I_ee,
            m_load: self.m_load,
            i_load: self.I_load,
            x_load: self.F_x_Cload,
            m_total,
            i_total,
            x_total,
            elbow: self.elbow,
            elbow_d: self.elbow_d,
            tau_j: self.tau_J,
            tau_j_d: self.tau_J_d,
            dtau_j: self.dtau_J,
            q: self.q,
            q_d: self.q_d,
            dq: self.dq,
            dq_d: self.dq_d,
            ddq_d: self.ddq_d,
            joint_contact: self.joint_contact,
            cartesian_contact: self.cartesian_contact,
            joint_collision: self.joint_collision,
            cartesian_collision: self.cartesian_collision,
            tau_ext_hat_filtered: self.tau_ext_hat_filtered,
            force_ext_in_o: self.O_F_ext_hat_K,
            force_ext_in_k: self.K_F_ext_hat_K,
            dpose_o_to_ee_d: self.O_dP_EE_d,
            ddpose_o_to_ee: self.O_ddP_O,
            elbow_c: self.elbow_c,
            delbow_c: self.delbow_c,
            ddelbow_c: self.ddelbow_c,
            pose_o_to_ee_c: self.O_T_EE_c,
            dpose_o_to_ee_c: self.O_dP_EE_c,
            ddpose_o_to_ee_c: self.O_ddP_EE_c,
            theta: self.theta,
            dtheta: self.dtheta,
            currect_errors: self.errors,
            last_motion_errors: self.reflex_reason,
            control_command_success_rate: self.control_command_success_rate,
            robot_mode: self.robot_mode,
            duration: Duration::from_millis(self.message_id),
        }
    }
}

impl Into<ArmState<7>> for RobotStateInter {
    fn into(self) -> ArmState<7> {
        ArmState {
            joint: Some(self.q),
            joint_vel: Some(self.dq),
            joint_acc: None,
            tau: Some(self.tau_J),
            cartisian_euler: None,
            cartisian_quat: None,
            cartisian_homo: Some(self.O_T_EE),
            cartisian_vel: None,
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
#[cfg(test)]
mod test {
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
}
