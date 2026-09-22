//! Offline protocol fixtures. Only ephemeral loopback ports are used.
use super::*;
use crate::{
    network::Network,
    types::{
        robot_command::RobotCommand,
        robot_state::{ControllerMode, MotionGeneratorMode, RobotStateInter},
        robot_types::{Command, CommandIDConfig},
    },
};
use std::{
    io::{Read, Write},
    net::{TcpListener, TcpStream, UdpSocket},
    ops::ControlFlow,
    sync::{Arc, RwLock},
    thread,
    time::{Duration, Instant},
};

fn read_request(stream: &mut TcpStream) -> (u32, u32) {
    let mut header = [0; 12];
    stream.read_exact(&mut header).unwrap();
    let command = u32::from_le_bytes(header[0..4].try_into().unwrap());
    let id = u32::from_le_bytes(header[4..8].try_into().unwrap());
    let size = u32::from_le_bytes(header[8..12].try_into().unwrap());
    let mut body = vec![0; size as usize - 12];
    stream.read_exact(&mut body).unwrap();
    (command, id)
}

fn response(command: Command, id: u32, status: u8) -> Vec<u8> {
    let mut frame = Vec::new();
    frame.extend_from_slice(&(command as u32).to_le_bytes());
    frame.extend_from_slice(&id.to_le_bytes());
    frame.extend_from_slice(&13_u32.to_le_bytes());
    frame.push(status);
    frame
}

#[derive(Clone, Copy, PartialEq)]
enum Omit {
    Nothing,
    IdleState,
    TerminalResponse,
}

fn fixture(cancel: bool, stop_status: u8) -> (FrankaRobotImpl, thread::JoinHandle<usize>) {
    fixture_sessions(cancel, stop_status, 1, Omit::Nothing)
}

fn fixture_sessions(
    cancel: bool,
    stop_status: u8,
    sessions: u64,
    omit: Omit,
) -> (FrankaRobotImpl, thread::JoinHandle<usize>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
    let local_udp = UdpSocket::bind("127.0.0.1:0").unwrap();
    local_udp
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let local_addr = local_udp.local_addr().unwrap();
    let server_udp = UdpSocket::bind("127.0.0.1:0").unwrap();
    server_udp
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let mut total_commands = 0;
        for session in 0..sessions {
            server_udp.set_nonblocking(false).unwrap();
            let (command, move_id) = read_request(&mut stream);
            assert_eq!(command, Command::Move as u32);
            let start = response(Command::Move, move_id, 1);
            // Deliberately split the header: TCP reads need not match messages.
            stream.write_all(&start[..5]).unwrap();
            stream.write_all(&start[5..]).unwrap();
            let mut state = RobotStateInter {
                message_id: session * 2 + 1,
                motion_generator_mode: MotionGeneratorMode::JointPosition,
                controller_mode: ControllerMode::JointImpedance,
                ..Default::default()
            };
            server_udp
                .send_to(&bincode::serialize(&state).unwrap(), local_addr)
                .unwrap();
            let business_commands = if cancel {
                let (command, stop_id) = read_request(&mut stream);
                assert_eq!(command, Command::StopMove as u32);
                // Terminal Move precedes StopMove in a single TCP write. It must
                // remain available to cleanup rather than being dropped or decoded
                // as the StopMove acknowledgement.
                let mut replies = if omit == Omit::TerminalResponse {
                    Vec::new()
                } else {
                    response(Command::Move, move_id, 2)
                };
                replies.extend(response(Command::StopMove, stop_id, stop_status));
                stream.write_all(&replies).unwrap();
                0
            } else {
                let mut buffer = [0; 4096];
                let (size, _) = server_udp.recv_from(&mut buffer).unwrap();
                let command: RobotCommand = bincode::deserialize(&buffer[..size]).unwrap();
                assert!(command.motion.motion_generation_finished);
                assert_eq!(command.command_id(), session * 2 + 1);
                stream
                    .write_all(&response(Command::Move, move_id, stop_status))
                    .unwrap();
                1
            };
            state.message_id = session * 2 + 2;
            state.motion_generator_mode = MotionGeneratorMode::Idle;
            state.controller_mode = ControllerMode::Other;
            if omit != Omit::IdleState {
                server_udp
                    .send_to(&bincode::serialize(&state).unwrap(), local_addr)
                    .unwrap();
            }
            // Keep the connection alive past the production 3 s deadline. EOF must
            // not be the reason that cancellation returns.
            if omit != Omit::Nothing {
                thread::sleep(Duration::from_millis(3300));
            }
            server_udp.set_nonblocking(true).unwrap();
            let mut buffer = [0; 4096];
            assert_eq!(
                server_udp.recv_from(&mut buffer).unwrap_err().kind(),
                std::io::ErrorKind::WouldBlock,
                "Break must not produce a UDP command, and done must send exactly one"
            );
            total_commands += business_commands;
        }
        total_commands
    });
    (
        FrankaRobotImpl {
            network,
            robot_state: Arc::new(RwLock::new(RobotStateInter::default())),
            udp_socket: local_udp,
            motion_command_id: None,
        },
        server,
    )
}

// The public async-callback facade deliberately blocks. Bound the test itself
// so a protocol regression cannot hang the suite while waiting for UDP.
fn run_bounded(work: impl FnOnce() + Send + 'static) {
    let (sender, receiver) = std::sync::mpsc::sync_channel(1);
    let worker = thread::spawn(move || {
        work();
        let _ = sender.send(());
    });
    receiver
        .recv_timeout(Duration::from_secs(5))
        .expect("loopback control session did not finish");
    worker.join().unwrap();
}

#[test]
fn sync_break_sends_stop_without_command_and_drains_move() {
    run_bounded(|| {
        let (mut robot, server) = fixture(true, 0);
        let mut calls = 0;
        std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            calls += 1;
            ControlFlow::Break(())
        })
        .unwrap();
        assert_eq!(calls, 1);
        assert_eq!(server.join().unwrap(), 0);
    });
}

#[test]
fn async_break_sends_stop_without_command_and_drains_move() {
    run_bounded(|| {
        let (mut robot, server) = fixture(true, 0);
        let mut calls = 0;
        tokio_udp::block_on_control_flow_async(&mut robot, MoveData::default(), async |_, _| {
            calls += 1;
            ControlFlow::Break(())
        })
        .unwrap();
        assert_eq!(calls, 1);
        assert_eq!(server.join().unwrap(), 0);
    });
}

#[test]
fn sync_done_sends_final_command_without_stop_request() {
    run_bounded(|| {
        let (mut robot, server) = fixture(false, 0);
        let mut calls = 0;
        std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            calls += 1;
            let mut command = RobotCommand::default();
            command.motion.motion_generation_finished = true;
            ControlFlow::Continue(command)
        })
        .unwrap();
        assert_eq!(calls, 1);
        assert_eq!(server.join().unwrap(), 1);
    });
}

#[test]
fn async_done_sends_final_command_without_stop_request() {
    run_bounded(|| {
        let (mut robot, server) = fixture(false, 0);
        let mut calls = 0;
        tokio_udp::block_on_control_flow_async(&mut robot, MoveData::default(), async |_, _| {
            calls += 1;
            let mut command = RobotCommand::default();
            command.motion.motion_generation_finished = true;
            ControlFlow::Continue(command)
        })
        .unwrap();
        assert_eq!(calls, 1);
        assert_eq!(server.join().unwrap(), 1);
    });
}

#[test]
fn stop_rejection_is_an_error_without_a_business_command() {
    run_bounded(|| {
        let (mut robot, server) = fixture(true, 1);
        let result = std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            ControlFlow::Break(())
        });
        assert!(matches!(result, Err(RobotException::CommandException(_))));
        let retry = std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            panic!("unconfirmed session must not restart")
        });
        assert!(retry.unwrap_err().to_string().contains("reconnect"));
        assert_eq!(server.join().unwrap(), 0);
    });
}

#[test]
fn finished_handshake_error_does_not_issue_a_second_stop() {
    run_bounded(|| {
        let (mut robot, server) = fixture(false, 2);
        let result = std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            let mut command = RobotCommand::default();
            command.motion.motion_generation_finished = true;
            ControlFlow::Continue(command)
        });
        assert!(matches!(result, Err(RobotException::CommandException(_))));
        assert_eq!(server.join().unwrap(), 1);
    });
}

#[test]
fn two_cancelled_sessions_consume_their_own_move_responses() {
    run_bounded(|| {
        let (mut robot, server) = fixture_sessions(true, 0, 2, Omit::Nothing);
        std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            ControlFlow::Break(())
        })
        .unwrap();
        assert!(robot.motion_command_id.is_none());
        tokio_udp::block_on_control_flow_async(&mut robot, MoveData::default(), async |_, _| {
            ControlFlow::Break(())
        })
        .unwrap();
        assert!(robot.motion_command_id.is_none());
        assert_udp_mode(&robot.udp_socket, true);
        assert_eq!(server.join().unwrap(), 0);
    });
}

fn assert_udp_mode(socket: &UdpSocket, nonblocking: bool) {
    #[cfg(unix)]
    {
        use std::os::fd::AsRawFd;
        // The socket owns a live descriptor for this read-only flag query.
        let flags = unsafe { libc::fcntl(socket.as_raw_fd(), libc::F_GETFL) };
        assert_ne!(flags, -1);
        assert_eq!(flags & libc::O_NONBLOCK != 0, nonblocking);
    }
    #[cfg(not(unix))]
    let _ = (socket, nonblocking);
}

fn assert_deadline(omit: Omit, use_async: bool) {
    let (mut robot, server) = fixture_sessions(true, 0, 1, omit);
    // Production sockets have no configured read timeout. Do not let the mock
    // fixture's guard accidentally make an unbounded implementation pass.
    robot.udp_socket.set_read_timeout(None).unwrap();
    let start = Instant::now();
    let result = if use_async {
        tokio_udp::block_on_control_flow_async(&mut robot, MoveData::default(), async |_, _| {
            ControlFlow::Break(())
        })
    } else {
        std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
            ControlFlow::Break(())
        })
    };
    let elapsed = start.elapsed();
    assert!(result.is_err());
    assert!(elapsed >= Duration::from_millis(2500));
    assert!(
        elapsed < Duration::from_millis(3250),
        "server EOF must not terminate cleanup: {elapsed:?}"
    );
    assert_eq!(robot.udp_socket.read_timeout().unwrap(), None);
    assert_udp_mode(&robot.udp_socket, use_async);
    assert!(robot.motion_command_id.is_some());
    let retry = std_udp::control_flow(&mut robot, MoveData::default(), |_, _| {
        panic!("unconfirmed session must not restart")
    });
    assert!(retry.unwrap_err().to_string().contains("reconnect"));
    assert_eq!(server.join().unwrap(), 0);
}

#[test]
fn missing_idle_udp_returns_error_by_total_deadline() {
    run_bounded(|| assert_deadline(Omit::IdleState, false));
}

#[test]
fn missing_terminal_tcp_returns_error_by_total_deadline() {
    run_bounded(|| assert_deadline(Omit::TerminalResponse, true));
}
