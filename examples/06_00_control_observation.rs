use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    let stats = Arc::new(Mutex::new((0usize, 0usize, 0.0f64)));

    let before_stats = stats.clone();
    robot.before(move |state, _| {
        let mut stats = before_stats.lock().unwrap();
        stats.0 += 1;
        stats.2 += state.control_command_success_rate;
    });

    let after_stats = stats.clone();
    robot.after(move |_, _| {
        after_stats.lock().unwrap().1 += 1;
    });

    let mut elapsed = Duration::ZERO;
    let mut target = None;
    robot.control_with::<JointPositionControl<7>, _>(move |state, dt| {
        elapsed += dt;
        let target = *target.get_or_insert_with(|| state.meas.q.unwrap_or([0.0; 7]));
        (target, elapsed >= Duration::from_secs(3))
    })?;

    let (before, after, success_sum) = *stats.lock().unwrap();
    if before > 0 {
        println!("before samples: {before}");
        println!("after samples: {after}");
        println!(
            "average command success rate: {:.3}",
            success_sum / before as f64
        );
    }
    Ok(())
}
