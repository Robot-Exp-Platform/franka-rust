use robot_behavior::{ControlType, MotionType};
use serde::{Deserialize, Serialize};

use super::robot_types::CommandIDConfig;

/// This struct is a command of the motion generator, including joint angle generator,
/// joint velocity generator, Cartesian space generator, and Cartesian space velocity generator
///
/// 运动生成器指令结构体，包含 关节角度生成器、关节速度生成器、笛卡尔空间生成器、笛卡尔空间速度生成器的指令
#[derive(Debug, Default, Serialize, Deserialize, Clone, Copy)]
#[repr(packed)]
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
#[derive(Debug, Default, Serialize, Deserialize, Clone, Copy)]
#[repr(packed)]
pub struct ControllerCommand {
    /// joint torque command
    pub tau_j_d: [f64; 7],
}

#[derive(Debug, Default, Serialize, Deserialize, Clone, Copy)]
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

#[cfg(test)]
mod test {
    use super::*;

    #[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
    #[allow(non_snake_case)]
    #[repr(packed)]
    pub struct MotionGeneratorCommandPacked {
        pub q_c: [f64; 7],
        pub dq_c: [f64; 7],
        pub O_T_EE_c: [f64; 16],
        pub O_dP_EE_c: [f64; 6],
        pub elbow_c: [f64; 2],
        pub valid_elbow: bool,
        pub motion_generation_finished: bool,
    }

    #[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
    #[allow(non_snake_case)]
    #[repr(packed)]
    pub struct ControllerCommandPacked {
        pub tau_J_d: [f64; 7],
    }

    #[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
    #[repr(packed)]
    pub struct RobotCommandPacked {
        pub message_id: u64,
        pub motion: MotionGeneratorCommandPacked,
        pub control: ControllerCommandPacked,
    }

    #[test]
    fn test_robot_command() {
        assert_eq!(
            bincode::serialize(&RobotCommandPacked::default()).unwrap(),
            bincode::serialize(&RobotCommand::default()).unwrap()
        );
        println!(
            "size of RobotCommand: {}",
            std::mem::size_of::<RobotCommand>()
        );
        println!(
            "size of RobotCommandPacked: {}",
            std::mem::size_of::<RobotCommandPacked>()
        );

        assert_eq!(
            bincode::serialize(&RobotCommandPacked::default()).unwrap(),
            bincode::serialize(&RobotCommand::default()).unwrap()
        );

        let robot_command_packed = RobotCommandPacked {
            message_id: 1,
            motion: MotionGeneratorCommandPacked {
                q_c: [1.0; 7],
                dq_c: [2.0; 7],
                O_T_EE_c: [3.0; 16],
                O_dP_EE_c: [4.0; 6],
                elbow_c: [5.0; 2],
                valid_elbow: true,
                motion_generation_finished: true,
            },
            control: ControllerCommandPacked { tau_J_d: [6.0; 7] },
        };
        let robot_command = RobotCommand {
            message_id: 1,
            motion: MotionGeneratorCommand {
                q_c: [1.0; 7],
                dq_c: [2.0; 7],
                pose_o_to_ee_c: [3.0; 16],
                dpose_o_to_ee_c: [4.0; 6],
                elbow_c: [5.0; 2],
                valid_elbow: true,
                motion_generation_finished: true,
            },
            control: ControllerCommand { tau_j_d: [6.0; 7] },
        };

        assert_eq!(
            bincode::serialize(&robot_command_packed).unwrap(),
            bincode::serialize(&robot_command).unwrap()
        );
    }
}
