use nalgebra as na;

pub fn joint_impedance<const N: usize>(
    stiffness: &[f64; N],
    damping: &[f64; N],
    target: [f64; N],
    q: [f64; N],
    dq: [f64; N],
) -> [f64; N] {
    let mut output = [0.0f64; N];
    for i in 0..N {
        let torque_stiffness = stiffness[i] * (target[i] - q[i]);
        let torque_damping = damping[i] * (-dq[i]);
        output[i] = torque_stiffness + torque_damping;
    }
    output
}

pub fn cartesian_impedance<const N: usize>(
    stiffness: (f64, f64),
    damping: (f64, f64),
    target: na::Isometry3<f64>,
    jacobian: na::SMatrix<f64, 6, N>,
    dq: [f64; N],
    pose: na::Isometry3<f64>,
) -> [f64; N] {
    // Compliance parameters
    let (trans_stiffness, rot_stiffness) = stiffness;
    let (trans_damping, rot_damping) = damping;

    let stiffness_mat = na::Matrix6::from_diagonal(&na::Vector6::new(
        trans_stiffness,
        trans_stiffness,
        trans_stiffness,
        rot_stiffness,
        rot_stiffness,
        rot_stiffness,
    ));

    let damping_mat = na::Matrix6::from_diagonal(&na::Vector6::new(
        trans_damping,
        trans_damping,
        trans_damping,
        rot_damping,
        rot_damping,
        rot_damping,
    ));

    // equilibrium point
    let position_d = target.translation.vector;
    let orientation_d = target.rotation;

    // current state
    let position = pose.translation.vector;
    let mut orientation = *pose.rotation.quaternion();

    // compute error to desired equilibrium pose
    // position error
    let error_head = position - position_d;

    // orientation error
    // "difference" quaternion
    if orientation_d.coords.dot(&orientation.coords) < 0.0 {
        orientation.coords = -orientation.coords;
    }
    let orientation = na::UnitQuaternion::new_normalize(orientation);
    // "difference" quaternion
    let error_quaternion = orientation.inverse() * orientation_d;
    // Transform to base frame
    let error_tail = -1.0 * (pose.rotation * error_quaternion.vector());

    // compute control
    let mut error = na::Vector6::zeros();
    error.fixed_view_mut::<3, 1>(0, 0).copy_from(&error_head);
    error.fixed_view_mut::<3, 1>(3, 0).copy_from(&error_tail);

    let dq_vec = na::SVector::<f64, N>::from_column_slice(&dq);

    // Spring damper system with damping ratio=1
    let tau_task =
        jacobian.transpose() * (-stiffness_mat * error - damping_mat * (jacobian * dq_vec));

    let mut output = [0.0; N];
    output.copy_from_slice(tau_task.as_slice());
    output
}
