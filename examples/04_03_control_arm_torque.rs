use std::time::Duration;

use franka_rust::{FrankaEmika, model::Frame, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    robot.set_default_behavior()?;
    let model = robot.model()?;

    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [100.0; 7],
        upper_torque_thresholds_acceleration: [100.0; 7],
        lower_torque_thresholds_nominal: [100.0; 7],
        upper_torque_thresholds_nominal: [100.0; 7],
        lower_force_thresholds_acceleration: [100.0; 6],
        upper_force_thresholds_acceleration: [100.0; 6],
        lower_force_thresholds_nominal: [100.0; 6],
        upper_force_thresholds_nominal: [100.0; 6],
    })?;

    let initial = robot.state()?;
    let initial_position = initial.flange.meas.pose.unwrap_or_default().position();
    let initial_gravity = model.gravity_from_arm_state(&initial, None::<&[f64; 3]>);
    let initial_tau = initial.joint.meas.tau.unwrap_or([0.0; 7]);
    let initial_tau_ext: [f64; 7] = std::array::from_fn(|i| initial_tau[i] - initial_gravity[i]);

    let mut elapsed = Duration::ZERO;
    let mut desired_mass = 0.0;
    let mut integral = [0.0; 7];
    robot.control_with::<ArmTorqueControl<7>, _>(move |state, dt| {
        elapsed += dt;

        let position = state.flange.meas.pose.unwrap_or_default().position();
        let drift = position
            .iter()
            .zip(initial_position)
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        if drift > 0.01 || elapsed > Duration::from_secs(10) {
            return ([0.0; 7], true);
        }

        let jacobian = model.zero_jacobian_from_arm_state(&Frame::EndEffector, &state);
        let gravity = model.gravity_from_arm_state(&state, None::<&[f64; 3]>);
        let tau_meas = state.joint.meas.tau.unwrap_or([0.0; 7]);
        let mut tau_d = [0.0; 7];
        for joint in 0..7 {
            tau_d[joint] = jacobian[2 + 6 * joint] * desired_mass * -9.81;
        }

        let mut tau_cmd = [0.0; 7];
        for joint in 0..7 {
            let tau_ext = tau_meas[joint] - gravity[joint] - initial_tau_ext[joint];
            let error = tau_d[joint] - tau_ext;
            integral[joint] += dt.as_secs_f64() * error;
            tau_cmd[joint] = tau_d[joint] + error + 2.0 * integral[joint];
        }

        desired_mass = 0.001 + 0.999 * desired_mass;
        (tau_cmd, false)
    })
}
