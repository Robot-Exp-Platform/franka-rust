use crate::{FrankaRobot, robot::FrankaType};

#[derive(Default)]
pub struct _FrankaFP3;

impl FrankaType for _FrankaFP3 {}

pub type FrankaFP3 = FrankaRobot<_FrankaFP3>;
