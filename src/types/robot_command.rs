use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Copy)]
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

/// This struct is a command of the motion generator, including joint angle generator,
/// joint velocity generator, Cartesian space generator, and Cartesian space velocity generator
///
/// 运动生成器指令结构体，包含 关节角度生成器、关节速度生成器、笛卡尔空间生成器、笛卡尔空间速度生成器的指令
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

#[derive(Serialize, Deserialize, Clone, Copy)]
#[allow(non_snake_case)]
#[repr(C, packed)]
pub struct ControllerCommandPacked {
    pub tau_J_d: [f64; 7],
    pub torque_command_finished: bool,
}

/// This struct is a command of the controller, including joint torque command
/// 控制器指令结构体，包含关节力矩指令
pub struct ControllerCommand {
    /// joint torque command
    pub tau_j_d: [f64; 7],
    pub torque_command_finished: bool,
}

#[derive(Serialize, Deserialize, Clone, Copy)]
#[allow(non_snake_case)]
#[repr(C, packed)]
pub struct RobotCommandPacked {
    pub message_id: u64,
    pub motion: MotionGeneratorCommandPacked,
    pub control: ControllerCommandPacked,
}

#[allow(non_snake_case)]
pub struct RobotCommand {
    pub message_id: u64,
    /// Motion generator command
    /// 运动生成指令，用于控制关节角度生成器、关节速度生成器、笛卡尔空间生成器、笛卡尔空间速度生成器
    pub motion: MotionGeneratorCommand,
    /// Controller command
    /// 控制器指令，用于控制关节力矩
    pub control: ControllerCommand,
}

impl From<MotionGeneratorCommandPacked> for MotionGeneratorCommand {
    fn from(packed: MotionGeneratorCommandPacked) -> Self {
        Self {
            q_c: packed.q_c,
            dq_c: packed.dq_c,
            pose_o_to_ee_c: packed.O_T_EE_c,
            dpose_o_to_ee_c: packed.O_dP_EE_c,
            elbow_c: packed.elbow_c,
            valid_elbow: packed.valid_elbow,
            motion_generation_finished: packed.motion_generation_finished,
        }
    }
}

impl From<ControllerCommandPacked> for ControllerCommand {
    fn from(packed: ControllerCommandPacked) -> Self {
        Self {
            tau_j_d: packed.tau_J_d,
            torque_command_finished: packed.torque_command_finished,
        }
    }
}

impl From<RobotCommandPacked> for RobotCommand {
    fn from(packed: RobotCommandPacked) -> Self {
        Self {
            message_id: packed.message_id,
            motion: MotionGeneratorCommand::from(packed.motion),
            control: ControllerCommand::from(packed.control),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn check_packed_size() {
        use std::mem::size_of;

        assert_ne!(
            size_of::<MotionGeneratorCommandPacked>(),
            size_of::<MotionGeneratorCommand>()
        );
        assert_ne!(
            size_of::<ControllerCommandPacked>(),
            size_of::<ControllerCommand>()
        );
        assert_ne!(size_of::<RobotCommandPacked>(), size_of::<RobotCommand>());
    }
}
