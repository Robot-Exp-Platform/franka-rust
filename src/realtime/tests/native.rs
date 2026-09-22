//! Exercise the public native capability on an actual single-thread Tokio
//! executor. The mock TCP/UDP peer runs on loopback, never a physical device.
use super::*;
use crate::{FrankaEmika, FrankaRobot};
use robot_behavior::{
    AsyncControlCallback, AsyncControlWith, ControlStep, ControlWith, JointPositionControl,
    JointState,
};
use std::sync::atomic::{AtomicUsize, Ordering};

fn runtime() -> tokio::runtime::Runtime {
    tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap()
}
fn public_robot(robot: FrankaRobotImpl) -> FrankaEmika {
    FrankaRobot::from_test_impl(robot)
}
fn assert_send<T: Send>(_: &T) {}

struct Callback {
    calls: usize,
    cancel: bool,
}
impl AsyncControlCallback<JointState<7>, [f64; 7]> for Callback {
    async fn call(&mut self, _: JointState<7>, _: Duration) -> ControlStep<[f64; 7]> {
        // Lending controller state stays available after Pending and return.
        tokio::task::yield_now().await;
        self.calls += 1;
        if self.cancel {
            ControlFlow::Break(())
        } else {
            ControlFlow::Continue(([0.0; 7], true))
        }
    }
}

#[test]
fn public_native_session_borrows_callback_and_restores_sockets() {
    run_bounded(|| {
        let (robot, server) = fixture(false, 0);
        let mut robot = public_robot(robot);
        let mut callback = Callback { calls: 0, cancel: false };
        runtime().block_on(async {
            let future = <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                &mut robot,
                &mut callback,
            );
            assert_send(&future);
            future.await.unwrap();
        });
        assert_eq!(callback.calls, 1);
        assert!(!robot.session_flag());
        assert!(robot.robot_impl.motion_command_id.is_none());
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        assert_eq!(server.join().unwrap(), 1);
    });
}

#[test]
fn sync_native_blocking_async_and_sync_share_one_connection() {
    run_bounded(|| {
        let (robot, server) = fixture_sessions(true, 0, 4, Omit::Nothing);
        let mut robot = public_robot(robot);
        <_ as ControlWith<JointPositionControl<7>>>::control_with_flow(&mut robot, |_, _| {
            ControlFlow::Break(())
        })
        .unwrap();
        let mut callback = Callback { calls: 0, cancel: true };
        runtime()
            .block_on(
                <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                    &mut robot,
                    &mut callback,
                ),
            )
            .unwrap();
        <_ as ControlWith<JointPositionControl<7>>>::control_with_flow_async(
            &mut robot,
            async |_, _| ControlFlow::Break(()),
        )
        .unwrap();
        <_ as ControlWith<JointPositionControl<7>>>::control_with_flow(&mut robot, |_, _| {
            ControlFlow::Break(())
        })
        .unwrap();
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        assert_eq!(callback.calls, 1);
        assert_eq!(server.join().unwrap(), 0);
    });
}

#[test]
fn native_terminal_error_is_consumed_without_an_extra_stop() {
    run_bounded(|| {
        let (robot, server) = fixture(false, 2);
        let mut robot = public_robot(robot);
        let mut callback = Callback { calls: 0, cancel: false };
        let result = runtime().block_on(
            <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                &mut robot,
                &mut callback,
            ),
        );
        assert!(matches!(result, Err(RobotException::CommandException(_))));
        assert!(!robot.session_flag());
        assert!(robot.robot_impl.motion_command_id.is_none());
        assert_eq!(server.join().unwrap(), 1);
    });
}

// Each network stage waits for a sibling Tokio task to release it. A blocking
// TCP read/UDP receive on the current_thread executor would deadlock until the
// peer's bounded timeout, and therefore fail instead of accidentally passing.
fn gated_fixture(
    cancel: bool,
) -> (
    FrankaRobotImpl,
    thread::JoinHandle<()>,
    Arc<AtomicUsize>,
    std::sync::mpsc::Sender<usize>,
) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
    let udp_socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    let local_addr = udp_socket.local_addr().unwrap();
    let stage = Arc::new(AtomicUsize::new(0));
    let server_stage = stage.clone();
    let (release, wait) = std::sync::mpsc::channel();
    let server = thread::spawn(move || {
        let gate = |n| {
            server_stage.store(n, Ordering::SeqCst);
            assert_eq!(wait.recv_timeout(Duration::from_secs(2)).unwrap(), n);
        };
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
        peer.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
        let (_, move_id) = read_request(&mut stream);
        let start = response(Command::Move, move_id, 1);
        stream.write_all(&start[..5]).unwrap();
        gate(1);
        stream.write_all(&start[5..]).unwrap();
        gate(2);
        let mut state = RobotStateInter {
            message_id: 1,
            motion_generator_mode: MotionGeneratorMode::JointPosition,
            controller_mode: ControllerMode::JointImpedance,
            ..Default::default()
        };
        peer.send_to(&bincode::serialize(&state).unwrap(), local_addr)
            .unwrap();
        if cancel {
            let (command, stop_id) = read_request(&mut stream);
            assert_eq!(command, Command::StopMove as u32);
            let stop = response(Command::StopMove, stop_id, 0);
            stream.write_all(&stop[..5]).unwrap();
            gate(3);
            stream.write_all(&stop[5..]).unwrap();
            gate(4);
        } else {
            let mut bytes = [0; 4096];
            let (len, _) = peer.recv_from(&mut bytes).unwrap();
            let cmd: RobotCommand = bincode::deserialize(&bytes[..len]).unwrap();
            assert!(cmd.motion.motion_generation_finished);
        }
        state.message_id = 2;
        state.motion_generator_mode = MotionGeneratorMode::Idle;
        state.controller_mode = ControllerMode::Other;
        peer.send_to(&bincode::serialize(&state).unwrap(), local_addr)
            .unwrap();
        let terminal = response(Command::Move, move_id, if cancel { 2 } else { 0 });
        stream.write_all(&terminal[..5]).unwrap();
        gate(if cancel { 5 } else { 3 });
        stream.write_all(&terminal[5..]).unwrap();
    });
    (
        FrankaRobotImpl {
            network,
            udp_socket,
            robot_state: Arc::new(RwLock::new(RobotStateInter::default())),
            motion_command_id: None,
        },
        server,
        stage,
        release,
    )
}

fn progress_at_every_boundary(cancel: bool) {
    let (robot, server, stage, release) = gated_fixture(cancel);
    let mut robot = public_robot(robot);
    let mut callback = Callback { calls: 0, cancel };
    runtime().block_on(async {
        let sibling = tokio::spawn(async move {
            let mut last = 0;
            loop {
                tokio::time::sleep(Duration::from_millis(1)).await;
                let next = stage.load(Ordering::SeqCst);
                if next != last {
                    release.send(next).unwrap();
                    last = next;
                }
                if next == if cancel { 5 } else { 3 } {
                    return next;
                }
            }
        });
        <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
            &mut robot,
            &mut callback,
        )
        .await
        .unwrap();
        assert_eq!(sibling.await.unwrap(), if cancel { 5 } else { 3 });
    });
    assert_eq!(callback.calls, 1);
    assert_udp_mode(&robot.robot_impl.udp_socket, false);
    server.join().unwrap();
}
#[test]
fn native_start_udp_and_normal_finish_allow_same_executor_progress() {
    run_bounded(|| progress_at_every_boundary(false));
}
#[test]
fn native_start_cancel_ack_idle_and_terminal_allow_same_executor_progress() {
    run_bounded(|| progress_at_every_boundary(true));
}

fn bare_fixture(
    serve: impl FnOnce(TcpStream, std::net::SocketAddr) + Send + 'static,
) -> (FrankaRobotImpl, thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
    let udp_socket = UdpSocket::bind("127.0.0.1:0").unwrap();
    let local_addr = udp_socket.local_addr().unwrap();
    let server = thread::spawn(move || {
        let (stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(2)))
            .unwrap();
        serve(stream, local_addr);
    });
    (
        FrankaRobotImpl {
            network,
            udp_socket,
            robot_state: Arc::new(RwLock::new(RobotStateInter::default())),
            motion_command_id: None,
        },
        server,
    )
}

#[test]
fn dropped_partial_start_restores_mode_and_forbids_reuse() {
    run_bounded(|| {
        let stage = Arc::new(AtomicUsize::new(0));
        let peer_stage = stage.clone();
        let (release, wait) = std::sync::mpsc::channel();
        let (robot, server) = bare_fixture(move |mut stream, _| {
            let (_, id) = read_request(&mut stream);
            stream
                .write_all(&response(Command::Move, id, 1)[..5])
                .unwrap();
            peer_stage.store(1, Ordering::SeqCst);
            wait.recv_timeout(Duration::from_secs(2)).unwrap();
        });
        let mut robot = public_robot(robot);
        let mut callback = Callback { calls: 0, cancel: false };
        runtime().block_on(async {
            let result = tokio::time::timeout(
                Duration::from_millis(30),
                <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                    &mut robot,
                    &mut callback,
                ),
            )
            .await;
            assert!(result.is_err());
        });
        assert_eq!(stage.load(Ordering::SeqCst), 1);
        assert_eq!(callback.calls, 0);
        assert!(!robot.session_flag());
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        let retry =
            <_ as ControlWith<JointPositionControl<7>>>::control_with_flow(&mut robot, |_, _| {
                panic!("uncertain start must not run")
            });
        assert!(retry.unwrap_err().to_string().contains("reconnect"));
        // Any unrelated TCP operation must also refuse the incomplete frame.
        let mut stop = crate::types::robot_types::StopMoveRequest::from(());
        let result: RobotResult<crate::types::robot_types::StopMoveResponse> =
            robot.robot_impl.network.tcp_send_and_recv(&mut stop);
        assert!(matches!(result, Err(error) if error.to_string().contains("reconnect")));
        release.send(()).unwrap();
        server.join().unwrap();
    });
}

#[test]
fn rejected_start_clears_session_without_running_callback() {
    run_bounded(|| {
        let (robot, server) = bare_fixture(|mut stream, _| {
            // A known, rejected start is safe to retry on the same connection.
            for _ in 0..2 {
                let (command, id) = read_request(&mut stream);
                assert_eq!(command, Command::Move as u32);
                stream.write_all(&response(Command::Move, id, 0)).unwrap();
            }
        });
        let mut robot = public_robot(robot);
        let mut callback = Callback { calls: 0, cancel: false };
        let runtime = runtime();
        for _ in 0..2 {
            let error = runtime
                .block_on(
                    <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                        &mut robot,
                        &mut callback,
                    ),
                )
                .unwrap_err();
            assert!(error.to_string().contains("did not start"));
            assert!(robot.robot_impl.motion_command_id.is_none());
            assert!(!robot.session_flag());
        }
        assert_eq!(callback.calls, 0);
        server.join().unwrap();
    });
}

#[test]
fn startup_eof_leaves_uncertain_session_unreusable() {
    run_bounded(|| {
        let (robot, server) = bare_fixture(|mut stream, _| {
            read_request(&mut stream);
        });
        let mut robot = public_robot(robot);
        let mut callback = Callback { calls: 0, cancel: false };
        assert!(
            runtime()
                .block_on(
                    <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                        &mut robot,
                        &mut callback
                    )
                )
                .is_err()
        );
        assert!(robot.robot_impl.motion_command_id.is_some());
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        assert!(!robot.session_flag());
        assert_eq!(callback.calls, 0);
        server.join().unwrap();
    });
}

#[test]
fn malformed_udp_and_failed_cleanup_retain_both_errors() {
    run_bounded(|| {
        let (robot, server) = bare_fixture(|mut stream, local_addr| {
            let (_, move_id) = read_request(&mut stream);
            stream
                .write_all(&response(Command::Move, move_id, 1))
                .unwrap();
            let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
            peer.send_to(&[0], local_addr).unwrap();
            let (command, stop_id) = read_request(&mut stream);
            assert_eq!(command, Command::StopMove as u32);
            stream
                .write_all(&response(Command::StopMove, stop_id, 1))
                .unwrap();
        });
        let mut robot = public_robot(robot);
        let mut callback = Callback { calls: 0, cancel: false };
        let error = runtime()
            .block_on(
                <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                    &mut robot,
                    &mut callback,
                ),
            )
            .unwrap_err();
        let RobotException::ControlSession { primary, cleanup } = error else {
            panic!("both errors must survive");
        };
        assert!(matches!(*primary, RobotException::DeserializeError(_)));
        assert!(matches!(*cleanup, RobotException::CommandException(_)));
        assert_eq!(callback.calls, 0);
        assert!(!robot.session_flag());
        assert!(robot.robot_impl.motion_command_id.is_some());
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        server.join().unwrap();
    });
}

#[test]
fn native_cleanup_deadline_keeps_sibling_running_and_requires_reconnect() {
    run_bounded(|| {
        let (robot, server) = fixture_sessions(true, 0, 1, Omit::IdleState);
        let mut robot = public_robot(robot);
        robot.robot_impl.udp_socket.set_read_timeout(None).unwrap();
        let mut callback = Callback { calls: 0, cancel: true };
        let start = Instant::now();
        runtime().block_on(async {
            let sibling = tokio::spawn(async {
                tokio::time::sleep(Duration::from_millis(20)).await;
            });
            let result = <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                &mut robot,
                &mut callback,
            )
            .await;
            assert!(result.unwrap_err().to_string().contains("deadline"));
            assert!(sibling.is_finished());
            sibling.await.unwrap();
        });
        assert!(start.elapsed() >= Duration::from_millis(2500));
        assert!(start.elapsed() < Duration::from_millis(3250));
        assert!(robot.robot_impl.motion_command_id.is_some());
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        assert_eq!(robot.robot_impl.udp_socket.read_timeout().unwrap(), None);
        server.join().unwrap();
    });
}

mod performance;

mod system;

#[test]
fn legacy_async_callback_still_accepts_non_send_borrowed_state() {
    run_bounded(|| {
        let (robot, server) = fixture(false, 0);
        let mut robot = public_robot(robot);
        let calls = std::rc::Rc::new(std::cell::Cell::new(0));
        <_ as ControlWith<JointPositionControl<7>>>::control_with_flow_async(
            &mut robot,
            async |_, _| {
                tokio::task::yield_now().await;
                calls.set(calls.get() + 1);
                ControlFlow::Continue(([0.0; 7], true))
            },
        )
        .unwrap();
        assert_eq!(calls.get(), 1);
        assert_eq!(server.join().unwrap(), 1);
    });
}

#[test]
fn existing_move_to_async_awaits_state_start_and_finish_on_callers_runtime() {
    run_bounded(|| {
        let stage = Arc::new(AtomicUsize::new(0));
        let server_stage = stage.clone();
        let (release, wait) = std::sync::mpsc::channel();
        let (robot, server) = bare_fixture(move |mut tcp, local_addr| {
            let gate = |n| {
                server_stage.store(n, Ordering::SeqCst);
                assert_eq!(wait.recv_timeout(Duration::from_secs(2)).unwrap(), n);
            };
            let udp = UdpSocket::bind("127.0.0.1:0").unwrap();
            udp.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
            gate(1);
            let mut state = RobotStateInter { message_id: 1, ..Default::default() };
            udp.send_to(&bincode::serialize(&state).unwrap(), local_addr)
                .unwrap();
            let (command, id) = read_request(&mut tcp);
            assert_eq!(command, Command::Move as u32);
            let start = response(Command::Move, id, 1);
            tcp.write_all(&start[..5]).unwrap();
            gate(2);
            tcp.write_all(&start[5..]).unwrap();
            state.controller_mode = ControllerMode::ExternalController;
            state.motion_generator_mode = MotionGeneratorMode::JointVelocity;
            let mut finished = false;
            for cycle in 2..12 {
                state.message_id = cycle;
                udp.send_to(&bincode::serialize(&state).unwrap(), local_addr)
                    .unwrap();
                let mut bytes = [0; 4096];
                let (size, _) = udp.recv_from(&mut bytes).unwrap();
                let command: RobotCommand = bincode::deserialize(&bytes[..size]).unwrap();
                if command.motion.motion_generation_finished {
                    finished = true;
                    break;
                }
            }
            assert!(
                finished,
                "zero-distance trajectory must finish in a bounded number of cycles"
            );
            state.message_id += 1;
            state.controller_mode = ControllerMode::Other;
            state.motion_generator_mode = MotionGeneratorMode::Idle;
            udp.send_to(&bincode::serialize(&state).unwrap(), local_addr)
                .unwrap();
            let finish = response(Command::Move, id, 0);
            tcp.write_all(&finish[..5]).unwrap();
            gate(3);
            tcp.write_all(&finish[5..]).unwrap();
        });
        let mut robot = public_robot(robot);
        runtime().block_on(async {
            let sibling = tokio::spawn(async move {
                let mut last = 0;
                loop {
                    tokio::time::sleep(Duration::from_millis(1)).await;
                    let next = stage.load(Ordering::SeqCst);
                    if next != last {
                        release.send(next).unwrap();
                        last = next;
                    }
                    if next == 3 {
                        break;
                    }
                }
            });
            <_ as robot_behavior::MoveTo<robot_behavior::JointSpace<7>>>::move_to_async(
                &mut robot, [0.0; 7],
            )
            .await
            .unwrap();
            sibling.await.unwrap();
        });
        assert!(!robot.session_flag());
        assert_udp_mode(&robot.robot_impl.udp_socket, false);
        server.join().unwrap();
    });
}
