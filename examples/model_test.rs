use franka_rust::{
    FRANKA_ROBOT_DEFAULT_JOINT,
    model::{Frame, FrankaModel},
};
use robot_behavior::RobotResult;
use strum::IntoEnumIterator;

fn main() -> RobotResult<()> {
    let path = if cfg!(target_os = "linux") {
        "/tmp/model.so"
    } else if cfg!(target_os = "windows") {
        "C:\\tmp\\model.dll"
    } else {
        "/tmp/model.dylib"
    };

    let model = FrankaModel::new(path)?;

    let q = FRANKA_ROBOT_DEFAULT_JOINT;
    let dq = [0.; 7];
    let mut pose = [0.; 16];
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;

    let m = 0.5;
    let x = [0.1; 3];
    let i = [0.1; 9];

    for frame in Frame::iter() {
        println!("{}>", frame);
        println!("   pose: {:?}", model.pose(&frame, &q, &pose, &pose));
        println!(
            "   body: {:?}",
            model.body_jacobian(&frame, &q, &pose, &pose)
        );
        println!(
            "   zero: {:?}",
            model.zero_jacobian(&frame, &q, &pose, &pose)
        );
    }
    println!("\nmass: {:?}", model.mass(&q, &i, m, &x));
    println!("coriolis: {:?}", model.coriolis(&q, &dq, &i, m, &x));
    println!("gravity: {:?}", model.gravity(&q, m, &x, &[0., 0., -0.98]));

    Ok(())
}
