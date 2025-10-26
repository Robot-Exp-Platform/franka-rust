// Copyright (c) 2021 Marco Boneberger
// Licensed under the EUPL-1.2-or-later

//! Contains functions for limiting the rate of torques, Cartesian pose, Cartesian velocity,
//! joint position and joint velocity.

use nalgebra::{
    Isometry3, Matrix3, Matrix3x1, Matrix6x1, Rotation3, Translation3, UnitQuaternion, Vector3,
};

use crate::utils::{array_to_isometry, is_homogeneous_from_slice};

///Sample time constant
pub static DELTA_T: f64 = 1e-3;
///Epsilon value for checking limits
pub static LIMIT_EPS: f64 = 1e-3;
/// Epsilon value for limiting Cartesian accelerations/jerks or not
pub static NORM_EPS: f64 = f64::EPSILON;
/// Number of packets lost considered for the definition of velocity limits.
/// When a packet is lost, FCI assumes a constant acceleration model
pub static TOL_NUMBER_PACKETS_LOST: f64 = 1e-3;
/// Factor for the definition of rotational limits using the Cartesian Pose interface
pub static FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE: f64 = 0.99;
/// Maximum torque rate
pub static MAX_TORQUE_RATE: [f64; 7] = [1000. - LIMIT_EPS; 7];
/// Maximum joint jerk
pub static MAX_JOINT_JERK: [f64; 7] = [
    7500.0 - LIMIT_EPS,
    3750.0 - LIMIT_EPS,
    5000.0 - LIMIT_EPS,
    6250.0 - LIMIT_EPS,
    7500.0 - LIMIT_EPS,
    10000.0 - LIMIT_EPS,
    10000.0 - LIMIT_EPS,
];
/// Maximum joint acceleration
pub static MAX_JOINT_ACCELERATION: [f64; 7] = [
    15.0000 - LIMIT_EPS,
    7.500 - LIMIT_EPS,
    10.0000 - LIMIT_EPS,
    12.5000 - LIMIT_EPS,
    15.0000 - LIMIT_EPS,
    20.0000 - LIMIT_EPS,
    20.0000 - LIMIT_EPS,
];
/// Maximum joint velocity
pub static MAX_JOINT_VELOCITY: [f64; 7] = [
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[0],
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[1],
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[2],
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[3],
    2.6100 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[4],
    2.6100 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[5],
    2.6100 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_JOINT_ACCELERATION[6],
];
///  Maximum translational jerk
pub static MAX_TRANSLATIONAL_JERK: f64 = 6500.0 - LIMIT_EPS;
/// Maximum translational acceleration
pub static MAX_TRANSLATIONAL_ACCELERATION: f64 = 13.0000 - LIMIT_EPS;
/// Maximum translational velocity
pub static MAX_TRANSLATIONAL_VELOCITY: f64 =
    2.0000 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_TRANSLATIONAL_ACCELERATION;
/// Maximum rotational jerk
pub static MAX_ROTATIONAL_JERK: f64 = 12500.0 - LIMIT_EPS;
/// Maximum rotational acceleration
pub static MAX_ROTATIONAL_ACCELERATION: f64 = 25.0000 - LIMIT_EPS;
/// Maximum rotational velocity
pub static MAX_ROTATIONAL_VELOCITY: f64 =
    2.5000 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_ROTATIONAL_ACCELERATION;
/// Maximum elbow jerk
pub static MAX_ELBOW_JERK: f64 = 5000. - LIMIT_EPS;
/// Maximum elbow acceleration
pub static MAX_ELBOW_ACCELERATION: f64 = 10.0000 - LIMIT_EPS;
/// Maximum elbow velocity
pub static MAX_ELBOW_VELOCITY: f64 =
    2.1750 - LIMIT_EPS - TOL_NUMBER_PACKETS_LOST * DELTA_T * MAX_ELBOW_ACCELERATION;

/// Limits the rate of a desired joint position considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Per-joint maximum allowed velocity.
/// * `max_acceleration` - Per-joint maximum allowed acceleration.
/// * `max_jerk` - Per-joint maximum allowed jerk.
/// * `commanded_positions` - Commanded joint positions of the current time step.
/// * `last_commanded_velocities` - Commanded joint positions of the previous time step.
/// * `last_commanded_velocities` - Commanded joint velocities of the previous time step.
/// * `last_commanded_accelerations` - Commanded joint accelerations of the previous time step.
/// # Panics
/// * if `commanded_positions` are infinite or NaN.
/// # Return
/// Rate-limited vector of desired joint positions.
pub fn limit_rate_joint_positions(
    max_velocity: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded_positions: &[f64; 7],
    last_commanded_positions: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> [f64; 7] {
    for x in commanded_positions {
        assert!(x.is_finite());
    }
    let mut limited_commanded_positions = [0.; 7];
    for i in 0..7 {
        limited_commanded_positions[i] = limit_rate_position(
            max_velocity[i],
            max_acceleration[i],
            max_jerk[i],
            commanded_positions[i],
            last_commanded_positions[i],
            last_commanded_velocities[i],
            last_commanded_accelerations[i],
        );
    }
    limited_commanded_positions
}

pub fn franka_limit_rate_joint_positions(
    commanded_positions: &[f64; 7],
    last_commanded_positions: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> [f64; 7] {
    if commanded_positions.iter().sum::<f64>() == 0.0 {
        return *commanded_positions;
    }
    limit_rate_joint_positions(
        &MAX_JOINT_VELOCITY,
        &MAX_JOINT_ACCELERATION,
        &MAX_JOINT_JERK,
        commanded_positions,
        last_commanded_positions,
        last_commanded_velocities,
        last_commanded_accelerations,
    )
}

/// Limits the rate of a desired joint position considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Maximum allowed velocity.
/// * `max_acceleration` - Maximum allowed acceleration.
/// * `max_jerk` - Maximum allowed jerk.
/// * `commanded_position` - Commanded joint position of the current time step.
/// * `last_commanded_velocity` - Commanded joint velocity of the previous time step.
/// * `last_commanded_acceleration` - Commanded joint acceleration of the previous time step.
/// # Panics
/// * if `commanded_values` are infinite or NaN.
/// # Return
/// Rate-limited desired joint position.
pub fn limit_rate_position(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_position: f64,
    last_commanded_position: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    assert!(commanded_position.is_finite());
    last_commanded_position
        + limit_rate_velocity(
            max_velocity,
            max_acceleration,
            max_jerk,
            (commanded_position - last_commanded_position) / DELTA_T,
            last_commanded_velocity,
            last_commanded_acceleration,
        ) * DELTA_T
}

/// Limits the rate of a desired joint velocity considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Maximum allowed velocity.
/// * `max_acceleration` - Maximum allowed acceleration.
/// * `max_jerk` - Maximum allowed jerk.
/// * `commanded_velocity` - Commanded joint velocity of the current time step.
/// * `last_commanded_velocity` - Commanded joint velocity of the previous time step.
/// * `last_commanded_acceleration` - Commanded joint acceleration of the previous time step.
/// # Panics
/// * if `commanded_values` are infinite or NaN.
/// # Return
/// Rate-limited desired joint velocity.
fn limit_rate_velocity(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: f64,
    last_commanded_velocity: f64,
    last_commanded_acceleration: f64,
) -> f64 {
    assert!(commanded_velocity.is_finite());
    let commanded_jerk = (((commanded_velocity - last_commanded_velocity) / DELTA_T)
        - last_commanded_acceleration)
        / DELTA_T;
    let commanded_acceleration = last_commanded_acceleration
        + f64::max(f64::min(commanded_jerk, max_jerk), -max_jerk) * DELTA_T;
    let safe_max_acceleration = f64::min(
        (max_jerk / max_acceleration) * (max_velocity - last_commanded_velocity),
        max_acceleration,
    );
    let safe_min_acceleration = f64::max(
        (max_jerk / max_acceleration) * (-max_velocity - last_commanded_velocity),
        -max_acceleration,
    );
    last_commanded_velocity
        + f64::max(
            f64::min(commanded_acceleration, safe_max_acceleration),
            safe_min_acceleration,
        ) * DELTA_T
}

/// Limits the rate of an input vector of per-joint commands considering the maximum allowed
/// time derivatives.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_derivatives` - Per-joint maximum allowed time derivative.
/// * `commanded_values` - Commanded values of the current time step.
/// * `last_commanded_values` - Commanded values of the previous time step.
/// # Panics
/// * if `commanded_values` are infinite or NaN.
/// # Return
/// Rate-limited vector of desired values.
pub fn limit_rate_torques(
    max_derivatives: &[f64; 7],
    commanded_values: &[f64; 7],
    last_commanded_values: &[f64; 7],
) -> [f64; 7] {
    for x in commanded_values {
        assert!(x.is_finite());
    }
    let mut limited_values = [0.; 7];
    for i in 0..7 {
        let commanded_derivative = (commanded_values[i] - last_commanded_values[i]) / DELTA_T;
        limited_values[i] = last_commanded_values[i]
            + f64::max(
                f64::min(commanded_derivative, max_derivatives[i]),
                -max_derivatives[i],
            ) * DELTA_T;
    }
    limited_values
}

pub fn franka_limit_rate_torques(
    commanded_values: &[f64; 7],
    last_commanded_values: &[f64; 7],
) -> [f64; 7] {
    if commanded_values.iter().sum::<f64>() == 0.0 {
        return *commanded_values;
    }
    limit_rate_torques(&MAX_TORQUE_RATE, commanded_values, last_commanded_values)
}

/// Limits the rate of a desired joint velocity considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_velocity` - Per-joint maximum allowed velocity.
/// * `max_acceleration` - Per-joint maximum allowed acceleration.
/// * `max_jerk` - Per-joint maximum allowed jerk.
/// * `commanded_velocities` - Commanded joint velocity of the current time step.
/// * `last_commanded_velocities` - Commanded joint velocities of the previous time step.
/// * `last_commanded_accelerations` - Commanded joint accelerations of the previous time step.
/// # Panics
/// * if `commanded_velocities` are infinite or NaN.
/// # Return
/// Rate-limited vector of desired joint velocities.
pub fn limit_rate_joint_velocities(
    max_velocity: &[f64; 7],
    max_acceleration: &[f64; 7],
    max_jerk: &[f64; 7],
    commanded_velocities: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> [f64; 7] {
    for x in commanded_velocities {
        assert!(x.is_finite());
    }
    let mut limited_commanded_velocities = [0.; 7];
    for i in 0..7 {
        limited_commanded_velocities[i] = limit_rate_velocity(
            max_velocity[i],
            max_acceleration[i],
            max_jerk[i],
            commanded_velocities[i],
            last_commanded_velocities[i],
            last_commanded_accelerations[i],
        );
    }
    limited_commanded_velocities
}

pub fn franka_limit_rate_joint_velocities(
    commanded_velocities: &[f64; 7],
    last_commanded_velocities: &[f64; 7],
    last_commanded_accelerations: &[f64; 7],
) -> [f64; 7] {
    if commanded_velocities.iter().sum::<f64>() == 0.0 {
        return *commanded_velocities;
    }
    limit_rate_joint_velocities(
        &MAX_JOINT_VELOCITY,
        &MAX_JOINT_ACCELERATION,
        &MAX_JOINT_JERK,
        commanded_velocities,
        last_commanded_velocities,
        last_commanded_accelerations,
    )
}

/// Limits the rate of a desired Cartesian pose considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_translational_velocity` - Maximum translational velocity.
/// * `max_translational_acceleration` - Maximum translational acceleration.
/// * `max_translational_jerk` - Maximum translational jerk.
/// * `max_rotational_velocity` - Maximum rotational velocity.
/// * `max_rotational_acceleration` - Maximum rotational acceleration.
/// * `max_rotational_jerk` - Maximum rotational jerk.
/// * `O_T_EE_c` - Commanded pose of the current time step.
/// * `last_O_T_EE_c` - Commanded pose of the previous time step.
/// * `last_O_dP_EE_c` - Commanded end effector twist of the previous time step.
/// * `last_O_ddP_EE_c` - Commanded end effector acceleration of the previous time step.
/// # Panics
/// * if an element of `O_T_EE_c` is infinite or NaN.
/// # Return
/// Rate-limited desired pose.
#[allow(non_snake_case, clippy::too_many_arguments)]
pub fn limit_rate_cartesian_pose(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    O_T_EE_c: &[f64; 16],
    last_O_T_EE_c: &[f64; 16],
    last_O_dP_EE_c: &[f64; 6],
    last_O_ddP_EE_c: &[f64; 6],
) -> [f64; 16] {
    for x in O_T_EE_c {
        assert!(x.is_finite());
    }
    assert!(is_homogeneous_from_slice(O_T_EE_c));

    let commanded_pose = array_to_isometry(O_T_EE_c);
    let mut limited_commanded_pose = Isometry3::<f64>::identity();
    let last_commanded_pose = array_to_isometry(last_O_T_EE_c);

    let dx_head =
        (commanded_pose.translation.vector - last_commanded_pose.translation.vector) / DELTA_T;

    let mut rot_diff: Rotation3<f64> = commanded_pose.rotation.to_rotation_matrix()
        * last_commanded_pose
            .rotation
            .to_rotation_matrix()
            .transpose();
    rot_diff.renormalize();
    let dx_tail = rot_diff.scaled_axis() / DELTA_T;

    let mut commanded_O_dP_EE_c = [0.; 6];
    for i in 0..3 {
        commanded_O_dP_EE_c[i] = dx_head[i];
    }
    for i in 0..3 {
        commanded_O_dP_EE_c[i + 3] = dx_tail[i];
    }
    commanded_O_dP_EE_c = limit_rate_cartesian_velocity(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_velocity,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_acceleration,
        FACTOR_CARTESIAN_ROTATION_POSE_INTERFACE * max_rotational_jerk,
        &commanded_O_dP_EE_c,
        last_O_dP_EE_c,
        last_O_ddP_EE_c,
    );
    let dx: Matrix6x1<f64> = Matrix6x1::<f64>::from_column_slice(&commanded_O_dP_EE_c);
    limited_commanded_pose.translation = Translation3::from(
        last_commanded_pose.translation.vector + Vector3::new(dx[0], dx[1], dx[2]) * DELTA_T,
    );
    limited_commanded_pose.rotation = last_commanded_pose.rotation;
    let dx_tail = dx.remove_row(0).remove_row(0).remove_row(0);
    if dx_tail.norm() > NORM_EPS {
        let w_norm = dx_tail.normalize();
        let theta = DELTA_T * dx_tail.norm();
        let omega_skew = Matrix3::new(
            0., -w_norm[2], w_norm[1], w_norm[2], 0., -w_norm[0], -w_norm[1], w_norm[0], 0.,
        );
        let R = Matrix3::identity()
            + f64::sin(theta) * omega_skew
            + (1. - f64::cos(theta)) * (omega_skew * omega_skew);
        limited_commanded_pose.rotation =
            UnitQuaternion::from_matrix(&(R * last_commanded_pose.rotation.to_rotation_matrix()));
    }
    let mut limited_values = [0.; 16];
    for (i, &x) in limited_commanded_pose.to_homogeneous().iter().enumerate() {
        limited_values[i] = x;
    }
    limited_values
}

pub fn franka_limit_rate_cartesian_pose(
    pose_o_to_ee_c: &[f64; 16],
    last_pose_o_to_ee_c: &[f64; 16],
    last_dpose_o_to_ee_c: &[f64; 6],
    last_ddpose_o_to_ee_c: &[f64; 6],
) -> [f64; 16] {
    if pose_o_to_ee_c.iter().sum::<f64>() == 0.0 {
        return *pose_o_to_ee_c;
    }
    limit_rate_cartesian_pose(
        MAX_TRANSLATIONAL_VELOCITY,
        MAX_TRANSLATIONAL_ACCELERATION,
        MAX_TRANSLATIONAL_JERK,
        MAX_ROTATIONAL_VELOCITY,
        MAX_ROTATIONAL_ACCELERATION,
        MAX_ROTATIONAL_JERK,
        pose_o_to_ee_c,
        last_pose_o_to_ee_c,
        last_dpose_o_to_ee_c,
        last_ddpose_o_to_ee_c,
    )
}

/// Limits the rate of a desired Cartesian velocity considering the limits provided.
/// # Note
/// FCI filters must be deactivated to work properly.
/// # Arguments
/// * `max_translational_velocity` - Maximum translational velocity.
/// * `max_translational_acceleration` - Maximum translational acceleration.
/// * `max_translational_jerk` - Maximum translational jerk.
/// * `max_rotational_velocity` - Maximum rotational velocity.
/// * `max_rotational_acceleration` - Maximum rotational acceleration.
/// * `max_rotational_jerk` - Maximum rotational jerk.
/// * `O_dP_EE_c` - Commanded pose of the current time step.
/// * `last_O_dP_EE_c` - Commanded end effector twist of the previous time step.
/// * `last_O_ddP_EE_c` - Commanded end effector acceleration of the previous time step.
/// # Panics
/// * if an element of `O_dP_EE_c` is infinite or NaN.
/// # Return
/// Rate-limited desired end effector twist.
#[allow(non_snake_case, clippy::too_many_arguments)]
pub fn limit_rate_cartesian_velocity(
    max_translational_velocity: f64,
    max_translational_acceleration: f64,
    max_translational_jerk: f64,
    max_rotational_velocity: f64,
    max_rotational_acceleration: f64,
    max_rotational_jerk: f64,
    O_dP_EE_c: &[f64; 6],
    last_O_dP_EE_c: &[f64; 6],
    last_O_ddP_EE_c: &[f64; 6],
) -> [f64; 6] {
    for x in O_dP_EE_c {
        assert!(x.is_finite());
    }
    let dx: Matrix6x1<f64> = Matrix6x1::from_column_slice(O_dP_EE_c);
    let last_dx = Matrix6x1::from_column_slice(last_O_dP_EE_c);
    let last_ddx = Matrix6x1::from_column_slice(last_O_ddP_EE_c);
    let dx_head = limit_rate_single_cartesian_velocity(
        max_translational_velocity,
        max_translational_acceleration,
        max_translational_jerk,
        &Vector3::from(dx.fixed_view::<3, 1>(0, 0)),
        &Vector3::from(last_dx.fixed_view::<3, 1>(0, 0)),
        &Vector3::from(last_ddx.fixed_view::<3, 1>(0, 0)),
    );
    let dx_tail = limit_rate_single_cartesian_velocity(
        max_rotational_velocity,
        max_rotational_acceleration,
        max_rotational_jerk,
        &Vector3::from(dx.fixed_view::<3, 1>(3, 0)),
        &Vector3::from(last_dx.fixed_view::<3, 1>(3, 0)),
        &Vector3::from(last_ddx.fixed_view::<3, 1>(3, 0)),
    );

    let mut limited_values = [0.; 6];
    for i in 0..3 {
        limited_values[i] = dx_head[i];
    }
    for i in 0..3 {
        limited_values[i + 3] = dx_tail[i];
    }
    limited_values
}

pub fn franka_limit_rate_cartesian_velocity(
    dpose_o_to_ee_c: &[f64; 6],
    last_dpose_o_to_ee_c: &[f64; 6],
    last_ddpose_o_to_ee_c: &[f64; 6],
) -> [f64; 6] {
    if dpose_o_to_ee_c.iter().sum::<f64>() == 0.0 {
        return *dpose_o_to_ee_c;
    }
    limit_rate_cartesian_velocity(
        MAX_TRANSLATIONAL_VELOCITY,
        MAX_TRANSLATIONAL_ACCELERATION,
        MAX_TRANSLATIONAL_JERK,
        MAX_ROTATIONAL_VELOCITY,
        MAX_ROTATIONAL_ACCELERATION,
        MAX_ROTATIONAL_JERK,
        dpose_o_to_ee_c,
        last_dpose_o_to_ee_c,
        last_ddpose_o_to_ee_c,
    )
}

fn limit_rate_single_cartesian_velocity(
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
    commanded_velocity: &Vector3<f64>,
    last_commanded_velocity: &Vector3<f64>,
    last_commanded_acceleration: &Vector3<f64>,
) -> Vector3<f64> {
    let commanded_jerk: Vector3<f64> = (((commanded_velocity - last_commanded_velocity) / DELTA_T)
        - last_commanded_acceleration)
        / DELTA_T;
    let mut commanded_acceleration = last_commanded_acceleration.clone_owned();
    if commanded_jerk.norm() > NORM_EPS {
        commanded_acceleration += commanded_jerk.normalize()
            * f64::max(f64::min(commanded_jerk.norm(), max_jerk), -max_jerk)
            * DELTA_T;
    }
    let unit_commanded_acceleration: Matrix3x1<f64> = commanded_acceleration.normalize();
    let dot_product = unit_commanded_acceleration.dot(last_commanded_velocity);
    let distance_to_max_velocity = -dot_product
        + f64::sqrt(
            f64::powf(dot_product, 2.) - last_commanded_velocity.norm_squared()
                + f64::powf(max_velocity, 2.),
        );
    let safe_max_acceleration = f64::min(
        (max_jerk / max_acceleration) * distance_to_max_velocity,
        max_acceleration,
    );
    let mut limited_commanded_velocity = last_commanded_velocity.clone_owned();
    if commanded_acceleration.norm() > NORM_EPS {
        limited_commanded_velocity += unit_commanded_acceleration
            * f64::min(commanded_acceleration.norm(), safe_max_acceleration)
            * DELTA_T;
    }
    limited_commanded_velocity
}
