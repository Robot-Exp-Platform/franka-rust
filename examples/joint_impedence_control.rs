use franka_rust::{
    FrankaRobot,
    types::{robot_state::RobotState, robot_types::SetCollisionBehaviorData},
};
use nalgebra as na;
use robot_behavior::{ControlType, RobotResult, behavior::*};
use std::{thread::sleep, time::Duration};

fn main() -> RobotResult<()> {
    let mut robot = FrankaRobot::new("172.16.0.3");
    robot.set_default_behavior()?;
    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [10.0; 7],
        upper_torque_thresholds_acceleration: [100.0; 7],
        lower_torque_thresholds_nominal: [10.0; 7],
        upper_torque_thresholds_nominal: [100.0; 7],
        lower_force_thresholds_acceleration: [10.0; 6],
        upper_force_thresholds_acceleration: [100.0; 6],
        lower_force_thresholds_nominal: [10.0; 6],
        upper_force_thresholds_nominal: [100.0; 6],
    })?;

    // let k_gains = [600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0];
    // let d_gains = [50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0];
    // let i_gains = [0.; 7];
    let k_gains = na::SMatrix::<f64, 7, 7>::from_diagonal(
        &[600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0].into(),
    );
    // let d_gains =
    // na::SMatrix::<f64, 7, 7>::from_diagonal(&[50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0].into());
    let d_gains = na::SMatrix::<f64, 7, 7>::from_diagonal(&[0.; 7].into());
    let i_gains = na::SMatrix::<f64, 7, 7>::from_diagonal(&[0.; 7].into());
    let mut int = na::SVector::<f64, 7>::zeros();

    let model = robot.model()?;
    let state = robot.read_state()?;
    let target: na::SVector<f64, 7> = state.q.into();
    // let mut path = [[0.; 7]; 48].iter();

    let state = robot.robot_state.clone();

    robot.control_with_closure(move |_, _| {
        let state_lock = state.read().unwrap();
        let q: na::SVector<f64, 7> = state_lock.q.into();
        let dq: na::SVector<f64, 7> = state_lock.dq.into();
        let robot_state: RobotState = (*state_lock).into();
        let coriolis: na::SVector<f64, 7> = model.coriolis_from_state(&robot_state).into();

        // let target = path.next().unwrap_or(&[0.; 7]);

        int += target - q;
        let tau_d = k_gains * (target - q) / 10. + d_gains * (-dq) + i_gains * int + coriolis;

        (ControlType::Torque(tau_d.into()), false)
    })?;

    sleep(Duration::from_secs(10));

    Ok(())
}
