#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use std::{thread::sleep, time::Duration};

use franka_rust::{FrankaRobot, model::Frame, utils::array_to_isometry};
use nalgebra as na;
use robot_behavior::{ArmRealtimeBehavior, ArmState, ControlType, Pose, RobotResult};

fn main() -> RobotResult<()> {
    let translational_stiffness: f64 = 150.;
    let rotational_stiffness: f64 = 10.;
    let stiffness = na::Matrix6::from_partial_diagonal(&combine_vec(
        &[translational_stiffness; 3],
        &[rotational_stiffness; 3],
    ));
    let damping = na::Matrix6::from_partial_diagonal(&combine_vec(
        &[2. * translational_stiffness.sqrt(); 3],
        &[2. * rotational_stiffness.sqrt(); 3],
    ));
    let mut robot = FrankaRobot::new("172.16.0.3");
    let model = robot.model()?;
    robot.set_collision_behavior(100.0.into())?;
    robot.set_joint_impedance([3000., 3000., 3000., 2500., 2500., 2000., 2000.].into())?;
    robot.set_cartesian_impedance([3000., 3000., 3000., 300., 300., 300.].into())?;

    let state = robot.read_franka_state()?;
    let initial_pose = array_to_isometry(&state.pose_o_to_ee);
    let position_d = initial_pose.translation.vector;
    let orientation_d = initial_pose.rotation;

    robot.control_with_closure(move |state: ArmState<7>, _: Duration| {
        let coriolis: na::SVector<f64, 7> = model.coriolis_from_arm_state(&state).into();
        let jacobian = na::SMatrix::<f64, 6, 7>::from_column_slice(
            &model.zero_jacobian_from_arm_state(&Frame::EndEffector, &state),
        );
        println!("coriolis: {}", coriolis);
        println!("Jacobian: {:?}", jacobian);
        let dq: na::SVector<f64, 7> = state.joint_vel.unwrap().into();
        let transform = if let Some(Pose::Homo(pose)) = state.pose_o_to_ee {
            array_to_isometry(&pose)
        } else {
            na::Isometry3::default()
        };
        let position = transform.translation.vector;
        let mut orientation = *transform.rotation.quaternion();
        let error_head: [f64; 3] = (position - position_d).into();

        if orientation_d.coords.dot(&orientation.coords) < 0. {
            orientation.coords = -orientation.coords;
        }
        let orientation = na::UnitQuaternion::new_normalize(orientation);
        let error_quaternion: na::UnitQuaternion<f64> = orientation.inverse() * orientation_d;
        let error_tail: [f64; 3] = (-1.
            * (transform.rotation.to_rotation_matrix()
                * na::Vector3::new(error_quaternion.i, error_quaternion.j, error_quaternion.k)))
        .into();

        let error = na::Vector6::<f64>::from_column_slice(&combine_vec(&error_head, &error_tail));

        let tau_task = jacobian.transpose() * (-stiffness * error - damping * (jacobian * dq));
        let tau_d = tau_task + coriolis;

        ControlType::Force(tau_d.into())
    })?;

    sleep(Duration::from_secs(25));

    Ok(())
}

fn combine_vec<const N: usize, const M: usize>(v1: &[f64; N], v2: &[f64; M]) -> [f64; N + M] {
    let mut result = [0.0; N + M];
    result[..N].copy_from_slice(v1);
    result[N..].copy_from_slice(v2);
    result
}
