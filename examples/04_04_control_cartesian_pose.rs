use std::f64::consts::PI;

use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    let _ = robot.automatic_error_recovery();
    robot.set_default_behavior()?;
    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        upper_torque_thresholds_acceleration: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        lower_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        upper_torque_thresholds_nominal: [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0],
        lower_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        upper_force_thresholds_acceleration: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        lower_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
        upper_force_thresholds_nominal: [20.0, 20.0, 20.0, 25.0, 25.0, 25.0],
    })?;
    robot
        .with_scale(0.5)
        .move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    println!("Finished moving to initial joint configuration.");

    let mut initial = None;
    let mut time = 0.0;
    robot.control_with::<CartesianPoseControl<7>, _>(move |state, time_step| {
        let initial = *initial.get_or_insert_with(|| {
            state
                .flange
                .cmd
                .pose
                .or(state.flange.des.pose)
                .or(state.flange.meas.pose)
                .expect("first control frame did not contain a valid Cartesian pose")
                .homo()
        });
        time += time_step.as_secs_f64();

        let radius = 0.2;
        let angle = PI / 4.0 * (1.0 - f64::cos(PI / 5.0 * time));
        let delta_x = radius * f64::sin(angle);
        let delta_z = radius * (f64::cos(angle) - 1.0);
        let mut out = initial;
        out[12] += delta_x;
        out[14] += delta_z;

        if time >= 10.0 {
            println!("Finished motion, shutting down example");
            return (Pose::Homo(out), true);
        }
        (Pose::Homo(out), false)
    })
}
