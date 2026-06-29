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
    if slice[3].abs() > EPSILON
        || slice[7].abs() > EPSILON
        || slice[11].abs() > EPSILON
        || (slice[15] - 1.0).abs() > EPSILON
    {
        return false;
    }
    for col in 0..3 {
        let norm =
            (slice[col * 4].powi(2) + slice[col * 4 + 1].powi(2) + slice[col * 4 + 2].powi(2))
                .sqrt();
        if (norm - 1.0).abs() > EPSILON {
            return false;
        }
    }

    for row in 0..3 {
        let norm = (slice[row].powi(2) + slice[4 + row].powi(2) + slice[8 + row].powi(2)).sqrt();
        if (norm - 1.0).abs() > EPSILON {
            return false;
        }
    }
    true
}

pub fn canonicalize_homogeneous_from_slice(slice: &[f64; 16]) -> Option<[f64; 16]> {
    if is_homogeneous_from_slice(slice) {
        return Some(*slice);
    }

    let mut transposed = [0.0; 16];
    for row in 0..4 {
        for col in 0..4 {
            transposed[col * 4 + row] = slice[row * 4 + col];
        }
    }

    is_homogeneous_from_slice(&transposed).then_some(transposed)
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

#[cfg(test)]
mod tests {
    use super::{canonicalize_homogeneous_from_slice, is_homogeneous_from_slice};

    #[test]
    fn canonicalizes_row_major_homogeneous_pose() {
        let row_major = [
            1.0, 0.0, 0.0, 0.3, 0.0, 1.0, 0.0, -0.2, 0.0, 0.0, 1.0, 0.5, 0.0, 0.0, 0.0, 1.0,
        ];

        assert!(!is_homogeneous_from_slice(&row_major));

        let column_major = canonicalize_homogeneous_from_slice(&row_major).unwrap();
        assert!(is_homogeneous_from_slice(&column_major));
        assert_eq!(column_major[12], 0.3);
        assert_eq!(column_major[13], -0.2);
        assert_eq!(column_major[14], 0.5);
    }
}
