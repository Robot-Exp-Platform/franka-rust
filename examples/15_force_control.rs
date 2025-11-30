use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::RobotResult;

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");

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

    // Eigen::VectorXd initial_tau_ext(7);
    // Eigen::VectorXd tau_error_integral(7);
    // // Bias torque sensor
    // std::array<double, 7> gravity_array = model.gravity(initial_state);
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
    // Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
    // initial_tau_ext = initial_tau_measured - initial_gravity;

    // // init integrator
    // tau_error_integral.setZero();

    // auto force_control_callback = [&](const franka::RobotState& robot_state,
    //                                   franka::Duration period) -> franka::Torques {
    //   time += period.toSec();

    //   if (time == 0.0) {
    //     initial_position = get_position(robot_state);
    //   }

    //   if (time > 0 && (get_position(robot_state) - initial_position).norm() > 0.01) {
    //     throw std::runtime_error("Aborting; too far away from starting pose!");
    //   }

    //   // get state variables
    //   std::array<double, 42> jacobian_array =
    //       model.zeroJacobian(franka::Frame::kEndEffector, robot_state);

    //   Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    //   Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data());
    //   Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());

    //   Eigen::VectorXd tau_d(7);
    //   Eigen::VectorXd desired_force_torque(6);
    //   Eigen::VectorXd tau_cmd(7);
    //   Eigen::VectorXd tau_ext(7);
    //   desired_force_torque.setZero();
    //   desired_force_torque(2) = desired_mass * -9.81;
    //   tau_ext << tau_measured - gravity - initial_tau_ext;
    //   tau_d << jacobian.transpose() * desired_force_torque;
    //   tau_error_integral += period.toSec() * (tau_d - tau_ext);
    //   // FF + PI control
    //   tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;

    //   // Smoothly update the mass to reach the desired target value
    //   desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;

    //   std::array<double, 7> tau_d_array{};
    //   Eigen::VectorXd::Map(tau_d_array.data(), 7) = tau_cmd;
    //   return tau_d_array;
    // };

    Ok(())
}
