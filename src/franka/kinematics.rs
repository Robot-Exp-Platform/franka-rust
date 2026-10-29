//! The seven-row DH chain ends at Joint7; Franka's flange is 107 mm beyond it.
use roplat_behavior::{JVec, Jaco, Pose, RobotException, RobotResult};
use roplat_kinematics::{ArmForwardKinematics, ArmInverseKinematics, CommonStop};

pub(crate) fn flange_pose<R: ArmForwardKinematics<7>>(q: &[f64; 7]) -> Pose {
    R::fk_end_pose(q) * Pose::from_position([0.0, 0.0, 0.107])
}

pub(crate) fn flange_jacobian<R: ArmForwardKinematics<7>>(q: &[f64; 7]) -> Jaco<7> {
    let cache = R::kine_cache(q, &[0.0; 7]);
    let r = cache.end_effector_pose().rotation * nalgebra::Vector3::new(0.0, 0.0, 0.107);
    let mut j = cache.jacobian();
    for col in 0..7 {
        let shift = j.fixed_view::<3, 1>(3, col).into_owned().cross(&r);
        for row in 0..3 {
            j[(row, col)] += shift[row];
        }
    }
    j
}

pub(crate) fn solve_flange_ik<R: ArmInverseKinematics<7>>(
    seed: [f64; 7],
    target: Pose,
) -> RobotResult<[f64; 7]> {
    let joint7_target = target * Pose::from_position([0.0, 0.0, -0.107]);
    let q = R::ik_solve(
        &JVec::from_row_slice(&seed),
        &joint7_target,
        Default::default(),
    );
    let (error, _) = R::task_error_and_jacobian(&q, &joint7_target);
    let stop = CommonStop::default();
    if !error.iter().all(|v| v.is_finite())
        || error.fixed_rows::<3>(0).norm() > stop.pos_tol
        || error.fixed_rows::<3>(3).norm() > stop.rot_tol
    {
        return Err(RobotException::UnprocessableInstructionError(
            "Franka Cartesian IK did not converge to the requested flange pose".into(),
        ));
    }
    Ok(q.as_slice().try_into().unwrap())
}

macro_rules! franka_flange_kinematics {
    ($ty:ty) => {
        impl roplat_behavior::SpaceMap<roplat_behavior::JointSpace<7>, roplat_behavior::FlangeSpace>
            for $ty
        {
            type Input = [f64; 7];
            type Output = roplat_behavior::Pose;
            fn map(&self, q: Self::Input) -> roplat_behavior::RobotResult<Self::Output> {
                Ok(crate::franka::kinematics::flange_pose::<Self>(&q))
            }
        }
        impl
            roplat_behavior::SpaceMap<
                roplat_behavior::JointSpace<7>,
                roplat_behavior::JacobianSpace<7>,
            > for $ty
        {
            type Input = [f64; 7];
            type Output = roplat_behavior::Jaco<7>;
            fn map(&self, q: Self::Input) -> roplat_behavior::RobotResult<Self::Output> {
                Ok(crate::franka::kinematics::flange_jacobian::<Self>(&q))
            }
        }
        impl roplat_behavior::SpaceMap<roplat_behavior::FlangeSpace, roplat_behavior::JointSpace<7>>
            for $ty
        {
            type Input = roplat_behavior::Pose;
            type Output = [f64; 7];
            fn map(&self, pose: Self::Input) -> roplat_behavior::RobotResult<Self::Output> {
                crate::franka::kinematics::solve_flange_ik::<Self>(
                    <Self as roplat_behavior::Joints<7>>::JOINT_DEFAULT,
                    pose,
                )
            }
        }
    };
}
pub(crate) use franka_flange_kinematics;
