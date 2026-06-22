use std::{
    io::Read,
    net::TcpListener,
    sync::{Arc, Mutex},
};

use franka_rust::FrankaEmika;
use robot_behavior::behavior::*;

fn main() -> anyhow::Result<()> {
    let mut robot = FrankaEmika::new("172.16.0.2");
    let tcp_listener = TcpListener::bind("127.0.0.1:8080")?;
    let (mut stream, _addr) = tcp_listener.accept()?;

    let target = Arc::new(Mutex::new(robot.get_joint()));
    let control_target = target.clone();
    robot.control_with::<JointPositionControl<7>, _>(move |state, _| {
        let joint = control_target
            .lock()
            .map(|target| *target)
            .unwrap_or_else(|_| {
                state
                    .cmd
                    .q
                    .or(state.des.q)
                    .or(state.meas.q)
                    .unwrap_or([0.0; 7])
            });
        (joint, false)
    })?;

    loop {
        let mut buffer = [0; 1024];
        let bytes_read = stream.read(&mut buffer)?;
        if bytes_read == 0 {
            break;
        }

        let joint: [f64; 7] = serde_json::from_slice(&buffer[..bytes_read])?;
        *target.lock().unwrap() = joint;
    }

    robot.stop()?;
    Ok(())
}
