use franka_rust::{
    FrankaEmika,
    model::{Frame, FrankaModel},
};
use roplat_behavior::{RobotResult, behavior::*};
use strum::IntoEnumIterator;

fn main() -> RobotResult<()> {
    // The library `FrankaEmika::model` downloaded from the robot earlier.
    let model = FrankaModel::new(FrankaModel::cache_path()?)?;
    let q = FrankaEmika::JOINT_DEFAULT;
    let dq = [0.0; 7];
    let mut pose = [0.0; 16];
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;
    pose[15] = 1.0;

    let mass = 0.5;
    let center_of_mass = [0.1; 3];
    let inertia = [0.1; 9];

    for frame in Frame::iter() {
        println!("{frame}>");
        println!("  pose: {:?}", model.pose(&frame, &q, &pose, &pose));
        println!(
            "  body jacobian: {:?}",
            model.body_jacobian(&frame, &q, &pose, &pose)
        );
        println!(
            "  zero jacobian: {:?}",
            model.zero_jacobian(&frame, &q, &pose, &pose)
        );
    }
    println!(
        "mass: {:?}",
        model.mass(&q, &inertia, mass, &center_of_mass)
    );
    println!(
        "coriolis: {:?}",
        model.coriolis(&q, &dq, &inertia, mass, &center_of_mass)
    );
    println!(
        "gravity: {:?}",
        model.gravity(&q, mass, &center_of_mass, &[0.0, 0.0, -9.81])
    );
    Ok(())
}
