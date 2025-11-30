use std::{
    f64::consts::{FRAC_PI_4, PI},
    thread::sleep,
    time::Duration,
};

use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{MotionType, Pose, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

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

    let mut time = Duration::ZERO;
    let mut initial_position = Pose::Homo([0.; 16]);
    robot.move_with_closure(move |state, dt| {
        if time == Duration::ZERO {
            initial_position = state.pose_o_to_ee.unwrap();
        }
        time += dt;

        let radius = 0.3;
        let angle = FRAC_PI_4 * (1.0 - f64::cos(PI / 5.0 * time.as_secs_f64()));
        let delta_x = radius * f64::sin(angle);
        let delta_z = radius * (f64::cos(angle) - 1.0);

        let mut new_pose = initial_position.homo();
        new_pose[12] += delta_x;
        new_pose[14] += delta_z;

        let is_finished = time >= Duration::from_secs(10);
        (MotionType::Cartesian(Pose::Homo(new_pose)), is_finished)
    })?;

    sleep(Duration::from_secs(6));

    Ok(())
}
