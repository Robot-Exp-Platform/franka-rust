use std::time::Duration;

use roplat_control::filter::{CartesianLowPass, LowPass};

use crate::utils::array_to_isometry;

/// Same command low-pass cutoff as libfranka 0.9.2.
pub const DEFAULT_CUTOFF_FREQUENCY: f64 = 100.0;

/// The shared first-order low-pass law, constructed for one call. libfranka
/// filters a command against the value the robot reports back, so the state
/// is the robot's, not the filter's.
fn low_pass<const N: usize>(sample_time: f64, cutoff_frequency: f64) -> LowPass<N> {
    assert!(sample_time.is_sign_positive() && sample_time.is_finite());
    assert!(cutoff_frequency.is_sign_positive() && cutoff_frequency.is_finite());
    LowPass::new(cutoff_frequency, Duration::from_secs_f64(sample_time))
        .expect("checked above: a finite positive period and cutoff")
}

pub fn low_pass_filter(sample_time: f64, cutoff_frequency: f64, y_last: f64, y: f64) -> f64 {
    low_pass::<1>(sample_time, cutoff_frequency).blend(&[y_last], &[y])[0]
}

pub fn joint_low_pass_filter<const N: usize>(
    sample_time: f64,
    y: &[f64; N],
    y_last: &[f64; N],
    cutoff_frequency: f64,
) -> [f64; N] {
    y.iter()
        .zip(y_last.iter())
        .for_each(|(i, j)| assert!(i.is_finite() && j.is_finite()));
    low_pass::<N>(sample_time, cutoff_frequency).blend(y_last, y)
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
    let filter = CartesianLowPass::new(cutoff_frequency, Duration::from_secs_f64(sample_time))
        .expect("checked above: a finite positive period and cutoff");
    let transform = filter.blend(&array_to_isometry(y_last), &array_to_isometry(y));

    let mut out = [0.; 16];
    for (i, &x) in transform.to_homogeneous().iter().enumerate() {
        out[i] = x;
    }
    out
}
