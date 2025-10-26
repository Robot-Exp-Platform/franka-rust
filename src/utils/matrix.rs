use nalgebra as na;

pub fn array_to_isometry(array: &[f64; 16]) -> na::Isometry3<f64> {
    let rot = na::Rotation3::from_matrix(
        &na::Matrix4::from_column_slice(array)
            .remove_column(3)
            .remove_row(3),
    );
    na::Isometry3::from_parts(
        na::Vector3::new(array[12], array[13], array[14]).into(),
        rot.into(),
    )
}

/// Determines whether the given elbow configuration is valid or not.
///
/// 判断给定的手肘配置是否有效。
pub fn is_valid_elbow(elbow: &[f64; 2]) -> bool {
    elbow[1] == -1.0 || elbow[1] == 1.0
}

pub fn is_homogeneous_from_pose(_: na::Isometry3<f64>) -> bool {
    true
}

/// Determines whether the given array represents a valid homogeneous transformation matrix.
pub fn is_homogeneous_from_slice(slice: &[f64; 16]) -> bool {
    const EPSILON: f64 = 1e-5;
    if (slice[3] != 0.0) || (slice[7] != 0.0) || (slice[11] != 0.0) || (slice[15] != 1.0) {
        return false;
    }
    for i in 0..3 {
        let norm: f64 = slice[i * 4..(i + 1) * 4].iter().map(|&x| x * x).sum();
        if (norm - 1.0).abs() > EPSILON {
            return false;
        }
    }
    true
}

/// Determines whether the given array represents a valid homogeneous transformation matrix.
pub fn is_homogeneous_from_matrix(matrix: na::Matrix4<f64>) -> bool {
    is_homogeneous_from_slice(
        matrix
            .as_slice()
            .try_into()
            .expect("slice with incorrect length"),
    )
}

pub fn check_finite(value: &[f64]) -> bool {
    value.iter().all(|&x| x.is_finite())
}
