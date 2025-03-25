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
