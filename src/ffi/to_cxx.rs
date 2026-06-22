use robot_behavior::{Arm, FlangeSpace, JointSpace, LoadState, MoveTo, Pose, Robot, RobotResult};

struct FrankaEmika(crate::FrankaEmika);
struct FrankaFR3(crate::FrankaFR3);

#[cxx::bridge]
mod franka_cxx {
    enum CxxPoseKind {
        Euler,
        Quat,
        Homo,
        AxisAngle,
        Position,
    }

    struct CxxPoseData {
        kind: CxxPoseKind,
        values: Vec<f64>,
    }

    extern "Rust" {
        type FrankaEmika;

        fn franka_emika_attach(ip: &str) -> Box<FrankaEmika>;
        fn version(&self) -> String;
        fn init(&mut self) -> Result<()>;
        fn shutdown(&mut self) -> Result<()>;
        fn enable(&mut self) -> Result<()>;
        fn disable(&mut self) -> Result<()>;
        fn reset(&mut self) -> Result<()>;
        fn stop(&mut self) -> Result<()>;
        fn emergency_stop(&mut self) -> Result<()>;
        fn clear_emergency_stop(&mut self) -> Result<()>;
        fn is_moving(&mut self) -> Result<bool>;
        fn state(&mut self) -> Result<String>;
        fn set_load(&mut self, m: f64, x: [f64; 3], i: [f64; 9]) -> Result<()>;
        fn get_joint(&self) -> [f64; 7];
        fn get_endpoint(&self) -> CxxPoseData;
        fn move_joint(&mut self, target: [f64; 7]) -> Result<()>;
        fn move_joint_sync(&mut self, target: [f64; 7]) -> Result<()>;
        fn move_flange(&mut self, target: CxxPoseData) -> Result<()>;
        fn move_flange_sync(&mut self, target: CxxPoseData) -> Result<()>;
    }

    extern "Rust" {
        type FrankaFR3;

        fn franka_fr3_attach(ip: &str) -> Box<FrankaFR3>;
        fn version(&self) -> String;
        fn init(&mut self) -> Result<()>;
        fn shutdown(&mut self) -> Result<()>;
        fn enable(&mut self) -> Result<()>;
        fn disable(&mut self) -> Result<()>;
        fn reset(&mut self) -> Result<()>;
        fn stop(&mut self) -> Result<()>;
        fn emergency_stop(&mut self) -> Result<()>;
        fn clear_emergency_stop(&mut self) -> Result<()>;
        fn is_moving(&mut self) -> Result<bool>;
        fn state(&mut self) -> Result<String>;
        fn set_load(&mut self, m: f64, x: [f64; 3], i: [f64; 9]) -> Result<()>;
        fn get_joint(&self) -> [f64; 7];
        fn get_endpoint(&self) -> CxxPoseData;
        fn move_joint(&mut self, target: [f64; 7]) -> Result<()>;
        fn move_joint_sync(&mut self, target: [f64; 7]) -> Result<()>;
        fn move_flange(&mut self, target: CxxPoseData) -> Result<()>;
        fn move_flange_sync(&mut self, target: CxxPoseData) -> Result<()>;
    }
}

pub use franka_cxx::*;

fn franka_emika_attach(ip: &str) -> Box<FrankaEmika> {
    Box::new(FrankaEmika(crate::FrankaEmika::new(ip)))
}

fn franka_fr3_attach(ip: &str) -> Box<FrankaFR3> {
    Box::new(FrankaFR3(crate::FrankaFR3::new(ip)))
}

macro_rules! impl_franka_cxx_robot {
    ($wrapper:ident, $inner:ty) => {
        impl $wrapper {
            fn version(&self) -> String {
                <$inner as Robot>::version()
            }

            fn init(&mut self) -> RobotResult<()> {
                <$inner as Robot>::init(&mut self.0)
            }

            fn shutdown(&mut self) -> RobotResult<()> {
                <$inner as Robot>::shutdown(&mut self.0)
            }

            fn enable(&mut self) -> RobotResult<()> {
                <$inner as Robot>::enable(&mut self.0)
            }

            fn disable(&mut self) -> RobotResult<()> {
                <$inner as Robot>::disable(&mut self.0)
            }

            fn reset(&mut self) -> RobotResult<()> {
                <$inner as Robot>::reset(&mut self.0)
            }

            fn stop(&mut self) -> RobotResult<()> {
                <$inner as Robot>::stop(&mut self.0)
            }

            fn emergency_stop(&mut self) -> RobotResult<()> {
                <$inner as Robot>::emergency_stop(&mut self.0)
            }

            fn clear_emergency_stop(&mut self) -> RobotResult<()> {
                <$inner as Robot>::clear_emergency_stop(&mut self.0)
            }

            fn is_moving(&mut self) -> RobotResult<bool> {
                <$inner as Robot>::is_moving(&mut self.0)
            }

            fn state(&mut self) -> RobotResult<String> {
                Ok(format!("{:?}", <$inner as Arm<7>>::state(&mut self.0)?))
            }

            fn set_load(&mut self, m: f64, x: [f64; 3], i: [f64; 9]) -> RobotResult<()> {
                <$inner as Arm<7>>::set_load(&mut self.0, LoadState { m, x, i })
            }

            fn get_joint(&self) -> [f64; 7] {
                <$inner as Arm<7>>::get_joint(&self.0)
            }

            fn get_endpoint(&self) -> CxxPoseData {
                pose_to_cxx(<$inner as Arm<7>>::get_endpoint(&self.0))
            }

            fn move_joint(&mut self, target: [f64; 7]) -> RobotResult<()> {
                <$inner as MoveTo<JointSpace<7>>>::move_to(&mut self.0, target)
            }

            fn move_joint_sync(&mut self, target: [f64; 7]) -> RobotResult<()> {
                <$inner as MoveTo<JointSpace<7>>>::move_to_sync(&mut self.0, target)
            }

            fn move_flange(&mut self, target: CxxPoseData) -> RobotResult<()> {
                <$inner as MoveTo<FlangeSpace>>::move_to(&mut self.0, cxx_to_pose(target)?)
            }

            fn move_flange_sync(&mut self, target: CxxPoseData) -> RobotResult<()> {
                <$inner as MoveTo<FlangeSpace>>::move_to_sync(&mut self.0, cxx_to_pose(target)?)
            }
        }
    };
}

impl_franka_cxx_robot!(FrankaEmika, crate::FrankaEmika);
impl_franka_cxx_robot!(FrankaFR3, crate::FrankaFR3);

fn pose_to_cxx(pose: Pose) -> CxxPoseData {
    match pose {
        Pose::Euler(tran, rot) => {
            let mut values = Vec::with_capacity(6);
            values.extend_from_slice(&tran);
            values.extend_from_slice(&rot);
            CxxPoseData { kind: CxxPoseKind::Euler, values }
        }
        Pose::Quat(pose) => {
            let mut values = Vec::with_capacity(7);
            values.extend_from_slice(pose.translation.vector.as_slice());
            values.extend_from_slice(pose.rotation.coords.as_slice());
            CxxPoseData { kind: CxxPoseKind::Quat, values }
        }
        Pose::Homo(values) => CxxPoseData { kind: CxxPoseKind::Homo, values: values.to_vec() },
        Pose::AxisAngle(tran, axis, angle) => {
            let mut values = Vec::with_capacity(7);
            values.extend_from_slice(&tran);
            values.extend_from_slice(&axis);
            values.push(angle);
            CxxPoseData { kind: CxxPoseKind::AxisAngle, values }
        }
        Pose::Position(tran) => CxxPoseData { kind: CxxPoseKind::Position, values: tran.to_vec() },
    }
}

fn cxx_to_pose(data: CxxPoseData) -> RobotResult<Pose> {
    match (data.kind, data.values.len()) {
        (CxxPoseKind::Euler, 6) => Ok(Pose::Euler(
            data.values[..3].try_into().unwrap(),
            data.values[3..6].try_into().unwrap(),
        )),
        (CxxPoseKind::Quat, 7) => Ok(Pose::from([
            data.values[0],
            data.values[1],
            data.values[2],
            data.values[3],
            data.values[4],
            data.values[5],
            data.values[6],
        ])),
        (CxxPoseKind::Homo, 16) => Ok(Pose::Homo(data.values.try_into().unwrap())),
        (CxxPoseKind::AxisAngle, 7) => Ok(Pose::AxisAngle(
            data.values[..3].try_into().unwrap(),
            data.values[3..6].try_into().unwrap(),
            data.values[6],
        )),
        (CxxPoseKind::Position, 3) => Ok(Pose::Position(data.values.try_into().unwrap())),
        _ => Err(robot_behavior::RobotException::InvalidFFIData(
            "invalid CxxPoseData length for pose kind".into(),
        )),
    }
}
