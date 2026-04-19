use crate::{FRANKA_DOF, FrankaRobot, robot::FrankaType};

#[derive(Default)]
pub struct _FrankaFP3;

impl FrankaType for _FrankaFP3 {
    const JOINT_NAMES: [&'static str; FRANKA_DOF] = [
        "fp3_joint1",
        "fp3_joint2",
        "fp3_joint3",
        "fp3_joint4",
        "fp3_joint5",
        "fp3_joint6",
        "fp3_joint7",
    ];
}

pub type FrankaFP3 = FrankaRobot<_FrankaFP3>;
