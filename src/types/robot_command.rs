use robot_behavior::{ControlType, MotionType};
use serde::{Deserialize, Serialize};

use super::robot_types::CommandIDConfig;

/// This struct is a command of the motion generator, including joint angle generator,
/// joint velocity generator, Cartesian space generator, and Cartesian space velocity generator
///
/// 运动生成器指令结构体，包含 关节角度生成器、关节速度生成器、笛卡尔空间生成器、笛卡尔空间速度生成器的指令
#[derive(Default, Serialize, Deserialize, Clone, Copy)]
#[repr(C, packed)]
pub struct MotionGeneratorCommand {
    /// joint angle command
    /// 关节角度指令
    pub q_c: [f64; 7],
    /// joint velocity command
    /// 关节速度指令
    pub dq_c: [f64; 7],
    /// Measured end effector pose in base frame.
    /// 末端执行器在基坐标系下的位姿
    pub pose_o_to_ee_c: [f64; 16],
    /// Measured end effector velocity in base frame.
    /// 末端执行器在基坐标系下的速度
    pub dpose_o_to_ee_c: [f64; 6],
    /// elbow configuration.
    pub elbow_c: [f64; 2],
    pub valid_elbow: bool,
    pub motion_generation_finished: bool,
}
/// This struct is a command of the controller, including joint torque command
/// 控制器指令结构体，包含关节力矩指令
#[derive(Default, Serialize, Deserialize, Clone, Copy)]
#[repr(C, packed)]
pub struct ControllerCommand {
    /// joint torque command
    pub tau_j_d: [f64; 7],
    pub torque_command_finished: bool,
}

#[derive(Default, Serialize, Deserialize, Clone, Copy)]
#[repr(C, packed)]
pub struct RobotCommand {
    pub message_id: u64,
    /// Motion generator command
    /// 运动生成指令，用于控制关节角度生成器、关节速度生成器、笛卡尔空间生成器、笛卡尔空间速度生成器
    pub motion: MotionGeneratorCommand,
    /// Controller command
    /// 控制器指令，用于控制关节力矩
    pub control: ControllerCommand,
}

impl CommandIDConfig for RobotCommand {
    fn command_id(&self) -> u32 {
        self.message_id as u32
    }
    fn set_command_id(&mut self, id: u32) {
        self.message_id = id as u64;
    }
}

impl From<MotionType<7>> for RobotCommand {
    fn from(value: MotionType<7>) -> Self {
        RobotCommand {
            message_id: 0,
            motion: match value {
                MotionType::Joint(joint) => MotionGeneratorCommand {
                    q_c: joint,
                    ..MotionGeneratorCommand::default()
                },
                MotionType::JointVel(joint_vel) => MotionGeneratorCommand {
                    dq_c: joint_vel,
                    ..MotionGeneratorCommand::default()
                },
                MotionType::CartesianHomo(pose) => MotionGeneratorCommand {
                    pose_o_to_ee_c: pose,
                    ..MotionGeneratorCommand::default()
                },
                MotionType::CartesianVel(pose_vel) => MotionGeneratorCommand {
                    dpose_o_to_ee_c: pose_vel,
                    ..MotionGeneratorCommand::default()
                },
                _ => MotionGeneratorCommand::default(),
            },
            control: ControllerCommand::default(),
        }
    }
}

impl From<ControlType<7>> for RobotCommand {
    fn from(value: ControlType<7>) -> Self {
        RobotCommand {
            message_id: 0,
            motion: MotionGeneratorCommand::default(),
            control: match value {
                ControlType::Force(tau) => ControllerCommand {
                    tau_j_d: tau,
                    ..ControllerCommand::default()
                },
                _ => ControllerCommand::default(),
            },
        }
    }
}
