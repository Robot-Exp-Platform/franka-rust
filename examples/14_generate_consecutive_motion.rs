use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{MotionType, RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0],
        upper_torque_thresholds_acceleration: [10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0],
        lower_torque_thresholds_nominal: [10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0],
        upper_torque_thresholds_nominal: [10.0, 10.0, 9.0, 9.0, 8.0, 7.0, 6.0],
        lower_force_thresholds_acceleration: [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
        upper_force_thresholds_acceleration: [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
        lower_force_thresholds_nominal: [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
        upper_force_thresholds_nominal: [10.0, 10.0, 10.0, 12.5, 12.5, 12.5],
    })?;

    for _ in 0..5 {
        println!("Executing motion.");
        let time_max = 4.0;
        let omega_max = 0.2;
        let mut time = 0.0;
        robot.move_with_closure(move |_, period| {
            time += period.as_secs_f64();

            let cycle = (-1.0f64).powf((time - time.rem_euclid(time_max)) / time_max);
            let omega = cycle * omega_max / 2.0
                * (1.0 - f64::cos(2.0 * std::f64::consts::PI / time_max * time));

            let velocities = [0.0, 0.0, omega, 0.0, 0.0, 0.0, 0.0];

            if time >= 2.0 * time_max {
                println!("\nFinished motion.");
                (MotionType::JointVel(velocities), true)
            } else {
                (MotionType::JointVel(velocities), false)
            }
        })?;

        robot.waiting_for_finish()?;
    }

    Ok(())
}
