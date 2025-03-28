use std::f64::consts::PI;

use crate::utils::array_to_isometry;

// pub fn low_pass_filter(sample_time: f64, cutoff_frequency: f64, y_last: f64, y: f64) -> f64 {
//     let alpha = 1.0 / (1.0 + (2.0 * PI * cutoff_frequency * sample_time).exp());
//     y_last + alpha * (y - y_last)
// }

pub fn low_pass_filter(sample_time: f64, cutoff_frequency: f64, y_last: f64, y: f64) -> f64 {
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));
    gain * y + (1. - gain) * y_last
}

pub fn joint_low_pass_filter(
    sample_time: f64,
    y: &[f64; 6],
    y_last: &[f64; 6],
    cutoff_frequency: f64,
) -> [f64; 6] {
    assert!(sample_time.is_sign_positive() && sample_time.is_finite());
    assert!(cutoff_frequency.is_sign_positive() && cutoff_frequency.is_finite());
    y.iter()
        .zip(y_last.iter())
        .for_each(|(i, j)| assert!(i.is_finite() && j.is_finite()));
    let mut out = [0.; 6];
    for i in 0..6 {
        out[i] = low_pass_filter(sample_time, cutoff_frequency, y_last[i], y[i]);
    }
    out
}

pub fn cartesian_low_pass_filter(
    sample_time: f64,
    y: &[f64; 16],
    y_last: &[f64; 16],
    cutoff_frequency: f64,
) -> [f64; 16] {
    assert!(sample_time.is_sign_positive() && sample_time.is_finite());
    assert!(cutoff_frequency.is_sign_positive() && cutoff_frequency.is_finite());
    y.iter()
        .zip(y_last.iter())
        .for_each(|(i, j)| assert!(i.is_finite() && j.is_finite()));
    let mut transform = array_to_isometry(y);
    let transform_last = array_to_isometry(y_last);
    let gain = sample_time / (sample_time + (1.0 / (2.0 * PI * cutoff_frequency)));
    transform.translation.vector =
        gain * transform.translation.vector + (1. - gain) * transform_last.translation.vector;
    transform.rotation = transform_last.rotation.slerp(&transform.rotation, gain);

    let mut out = [0.; 16];
    for (i, &x) in transform.to_homogeneous().iter().enumerate() {
        out[i] = x;
    }
    out
}
