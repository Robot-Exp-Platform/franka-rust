use std::path::Path;

use super::Pose;
use nalgebra as na;

/// Determines whether the given elbow configuration is valid or not.  
/// 判断给定的手肘配置是否有效。
pub fn is_valid_elbow(elbow: &[f64; 2]) -> bool {
    elbow[1] == -1.0 || elbow[1] == 1.0
}

pub fn is_homogeneous_from_pose(_: Pose) -> bool {
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

/// On Linux, this checks for the existence of `/sys/kernel/realtime`.
/// On Windows, this always returns true.
pub fn has_realtime_kernel() -> bool {
    if cfg!(target_os = "linux") {
        Path::new("/sys/kernel/realtime").exists()
    } else if cfg!(target_os = "windows") {
        true
    } else {
        println!("Unknown OS, assuming realtime kernel.");
        true
    }
}

pub fn check_finite(value: &[f64]) -> bool {
    value.iter().all(|&x| x.is_finite())
}
