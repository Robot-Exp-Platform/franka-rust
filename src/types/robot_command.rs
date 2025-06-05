use std::fmt::Display;

use robot_behavior::{ControlType, MotionType};
use serde::{Deserialize, Serialize};

use crate::utils::{
    franka_limit_rate_cartesian_pose, franka_limit_rate_cartesian_velocity,
    franka_limit_rate_joint_positions, franka_limit_rate_joint_velocities,
    franka_limit_rate_torques,
};

use super::{
    robot_state::{RobotState, RobotStateInter},
    robot_types::{CommandFilter, CommandIDConfig},
};

/// This struct is a command of the motion generator, including joint angle generator,
/// joint velocity generator, Cartesian space generator, and Cartesian space velocity generator
///
/// 运动生成器指令结构体，包含 关节角度生成器、关节速度生成器、笛卡尔空间生成器、笛卡尔空间速度生成器的指令
#[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
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
#[derive(Debug, Default, Serialize, Deserialize, Clone, Copy)]
#[repr(C, packed)]
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

impl CommandIDConfig<u64> for RobotCommand {
    fn command_id(&self) -> u64 {
        self.message_id
    }
    fn set_command_id(&mut self, id: u64) {
        self.message_id = id;
    }
}

impl CommandFilter<RobotStateInter> for MotionGeneratorCommand {
    fn filter(self, state: &RobotStateInter) -> Self {
        let state: RobotState = (*state).into();
        let q_c = self.q_c;
        let dq_c = self.dq_c;
        let pose_o_to_ee_c = self.pose_o_to_ee_c;
        let dpose_o_to_ee_c = self.dpose_o_to_ee_c;
        let q_c = franka_limit_rate_joint_positions(&q_c, &state.q_d, &state.dq_d, &state.ddq_d);
        let dq_c = franka_limit_rate_joint_velocities(&dq_c, &state.dq, &state.ddq_d);
        let pose_o_to_ee_c = franka_limit_rate_cartesian_pose(
            &pose_o_to_ee_c,
            &state.pose_o_to_ee_c,
            &state.dpose_o_to_ee_c,
            &state.ddpose_o_to_ee_c,
        );
        let dpose_o_to_ee_c = franka_limit_rate_cartesian_velocity(
            &dpose_o_to_ee_c,
            &state.ddpose_o_to_ee_c,
            &state.ddpose_o_to_ee_c,
        );
        MotionGeneratorCommand {
            q_c,
            dq_c,
            pose_o_to_ee_c,
            dpose_o_to_ee_c,
            ..self
        }
    }
}

impl CommandFilter<RobotStateInter> for ControllerCommand {
    fn filter(self, state: &RobotStateInter) -> Self {
        let tau_j_d = self.tau_j_d;
        let tau_j = state.tau_J_d;
        let tau_j_d = franka_limit_rate_torques(&tau_j_d, &tau_j);
        ControllerCommand { tau_j_d }
    }
}

impl CommandFilter<RobotStateInter> for RobotCommand {
    fn filter(self, state: &RobotStateInter) -> Self {
        RobotCommand {
            message_id: self.message_id,
            motion: self.motion.filter(state),
            control: self.control.filter(state),
        }
    }
}

impl From<(MotionType<7>, bool)> for RobotCommand {
    fn from(value: (MotionType<7>, bool)) -> Self {
        RobotCommand {
            message_id: 0,
            motion: match value.0 {
                MotionType::Joint(joint) => MotionGeneratorCommand {
                    q_c: joint,
                    motion_generation_finished: value.1,
                    ..MotionGeneratorCommand::default()
                },
                MotionType::JointVel(joint_vel) => MotionGeneratorCommand {
                    dq_c: joint_vel,
                    motion_generation_finished: value.1,
                    ..MotionGeneratorCommand::default()
                },
                MotionType::Cartesian(pose) => MotionGeneratorCommand {
                    pose_o_to_ee_c: pose.homo(),
                    motion_generation_finished: value.1,
                    ..MotionGeneratorCommand::default()
                },
                MotionType::CartesianVel(pose_vel) => MotionGeneratorCommand {
                    dpose_o_to_ee_c: pose_vel,
                    motion_generation_finished: value.1,
                    ..MotionGeneratorCommand::default()
                },
                _ => MotionGeneratorCommand::default(),
            },
            control: ControllerCommand::default(),
        }
    }
}

impl From<(ControlType<7>, bool)> for RobotCommand {
    fn from(value: (ControlType<7>, bool)) -> Self {
        RobotCommand {
            message_id: 0,
            motion: MotionGeneratorCommand::default(),
            control: match value.0 {
                ControlType::Torque(tau) => ControllerCommand { tau_j_d: tau },
                _ => ControllerCommand::default(),
            },
        }
    }
}

impl Display for RobotCommand {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let message_id = self.message_id;
        let motion_generation_finished = self.motion.motion_generation_finished;
        let q = self.motion.q_c;
        let pose = self.motion.pose_o_to_ee_c;
        let tau = self.control.tau_j_d;
        write!(
            f,
            r#"robot command:
    | message_id: {message_id},
    | finished: {motion_generation_finished},
    | motion:
        | q_c: {q:?},
        | pose: {pose:?},
    | control:
        | tau: {tau:?}"#
        )
    }
}

#[cfg(test)]
mod test {
    use std::mem::offset_of;

    use super::*;

    #[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
    #[allow(non_snake_case)]
    #[repr(C, packed)]
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
    #[repr(C, packed)]
    pub struct ControllerCommandPacked {
        pub tau_J_d: [f64; 7],
    }

    #[derive(Default, Serialize, Deserialize, Debug, Copy, Clone)]
    #[repr(C, packed)]
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
                q_c: [1.1111; 7],
                dq_c: [2.2222; 7],
                O_T_EE_c: [3.3333; 16],
                O_dP_EE_c: [4.4444; 6],
                elbow_c: [5.5555; 2],
                valid_elbow: true,
                motion_generation_finished: true,
            },
            control: ControllerCommandPacked {
                tau_J_d: [6.6666; 7],
            },
        };
        let robot_command = RobotCommand {
            message_id: 1,
            motion: MotionGeneratorCommand {
                q_c: [1.1111; 7],
                dq_c: [2.2222; 7],
                pose_o_to_ee_c: [3.3333; 16],
                dpose_o_to_ee_c: [4.4444; 6],
                elbow_c: [5.5555; 2],
                valid_elbow: true,
                motion_generation_finished: true,
            },
            control: ControllerCommand {
                tau_j_d: [6.6666; 7],
            },
        };

        assert_eq!(
            bincode::serialize(&robot_command_packed).unwrap(),
            bincode::serialize(&robot_command).unwrap()
        );

        assert_eq!(
            bincode::serialize(
                &bincode::deserialize::<RobotCommandPacked>(
                    &bincode::serialize(&robot_command).unwrap()
                )
                .unwrap()
            )
            .unwrap(),
            bincode::serialize(&robot_command_packed).unwrap()
        );

        println!(
            "offset of message_id: {}",
            offset_of!(RobotCommandPacked, message_id)
        );
        println!(
            "offset of motion: {}",
            offset_of!(RobotCommandPacked, motion)
        );
        println!(
            "offset of control: {}",
            offset_of!(RobotCommandPacked, control)
        );
        println!(
            "offset of q_c: {}",
            offset_of!(MotionGeneratorCommandPacked, q_c)
        );
        println!(
            "offset of dq_c: {}",
            offset_of!(MotionGeneratorCommandPacked, dq_c)
        );
        println!(
            "offset of O_T_EE_c: {}",
            offset_of!(MotionGeneratorCommandPacked, O_T_EE_c)
        );
        println!(
            "offset of O_dP_EE_c: {}",
            offset_of!(MotionGeneratorCommandPacked, O_dP_EE_c)
        );
        println!(
            "offset of elbow_c: {}",
            offset_of!(MotionGeneratorCommandPacked, elbow_c)
        );
        println!(
            "offset of valid_elbow: {}",
            offset_of!(MotionGeneratorCommandPacked, valid_elbow)
        );
        println!(
            "offset of motion_generation_finished: {}",
            offset_of!(MotionGeneratorCommandPacked, motion_generation_finished)
        );
        println!(
            "offset of tau_J_d: {}",
            offset_of!(ControllerCommandPacked, tau_J_d)
        );

        println!(
            "offset of message_id: {}",
            offset_of!(RobotCommand, message_id)
        );
        println!("offset of motion: {}", offset_of!(RobotCommand, motion));
        println!("offset of control: {}", offset_of!(RobotCommand, control));
        println!("offset of q_c: {}", offset_of!(MotionGeneratorCommand, q_c));
        println!(
            "offset of dq_c: {}",
            offset_of!(MotionGeneratorCommand, dq_c)
        );
        println!(
            "offset of pose_o_to_ee_c: {}",
            offset_of!(MotionGeneratorCommand, pose_o_to_ee_c)
        );
        println!(
            "offset of dpose_o_to_ee_c: {}",
            offset_of!(MotionGeneratorCommand, dpose_o_to_ee_c)
        );
        println!(
            "offset of elbow_c: {}",
            offset_of!(MotionGeneratorCommand, elbow_c)
        );
        println!(
            "offset of valid_elbow: {}",
            offset_of!(MotionGeneratorCommand, valid_elbow)
        );
        println!(
            "offset of motion_generation_finished: {}",
            offset_of!(MotionGeneratorCommand, motion_generation_finished)
        );
        println!(
            "offset of tau_j_d: {}",
            offset_of!(ControllerCommand, tau_j_d)
        );

        let command_u8 = [
            166, 92, 188, 19, 0, 0, 0, 0, 255, 210, 11, 192, 118, 132, 2, 63, 251, 11, 184, 68, 51,
            34, 233, 191, 166, 4, 234, 87, 157, 131, 59, 63, 117, 226, 69, 93, 1, 219, 2, 192, 107,
            33, 122, 73, 170, 108, 18, 63, 168, 0, 226, 73, 54, 34, 249, 63, 136, 169, 62, 96, 4,
            33, 233, 63, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        ];

        let command = RobotCommand {
            message_id: 331111590,
            motion: MotionGeneratorCommand {
                q_c: [
                    3.5319208114536684e-5,
                    -0.7854248373190552,
                    0.0004198321475215684,
                    -2.356936196036538,
                    7.028378819929285e-5,
                    1.5708525548980479,
                    0.7852804069309949,
                ],
                dq_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                pose_o_to_ee_c: [
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                ],
                dpose_o_to_ee_c: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                elbow_c: [0.0, 0.0],
                valid_elbow: false,
                motion_generation_finished: false,
            },
            control: ControllerCommand {
                tau_j_d: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            },
        };

        println!(
            "command packed {:?}",
            bincode::deserialize::<RobotCommandPacked>(&command_u8).unwrap()
        );
        println!("command {:?}", command);
    }

    #[test]
    fn display_robot_command() {
        let command = RobotCommand::default();
        println!("{}", command);
    }
}
