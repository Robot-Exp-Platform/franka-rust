//! Finite loopback CPU/network benchmark; run the ignored test in release mode.
//! This is not a robot-cycle deadline or hardware latency measurement.
use super::*;

#[derive(Clone, Copy)]
enum Backend {
    Std,
    BlockingAsync,
    Native,
}
impl Backend {
    fn label(self) -> &'static str {
        match self {
            Self::Std => "std",
            Self::BlockingAsync => "blocking_async",
            Self::Native => "native_async",
        }
    }
}
struct ReadyController {
    remaining: usize,
}
impl AsyncControlCallback<JointState<7>, [f64; 7]> for ReadyController {
    fn call(
        &mut self,
        _: JointState<7>,
        _: Duration,
    ) -> impl std::future::Future<Output = ControlStep<[f64; 7]>> + Send {
        self.remaining -= 1;
        std::future::ready(ControlFlow::Continue(([0.0; 7], self.remaining == 0)))
    }
}

fn measure(
    backend: Backend,
    cycles: usize,
    runtime: &tokio::runtime::Runtime,
) -> (Duration, Vec<Duration>) {
    let (robot, server) = {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
        let udp_socket = UdpSocket::bind("127.0.0.1:0").unwrap();
        udp_socket
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let local_addr = udp_socket.local_addr().unwrap();
        let server = thread::spawn(move || {
            let (mut tcp, _) = listener.accept().unwrap();
            tcp.set_nodelay(true).unwrap();
            tcp.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
            let udp = UdpSocket::bind("127.0.0.1:0").unwrap();
            udp.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
            let (_, move_id) = read_request(&mut tcp);
            tcp.write_all(&response(Command::Move, move_id, 1)).unwrap();
            let mut state = RobotStateInter {
                motion_generator_mode: MotionGeneratorMode::JointPosition,
                controller_mode: ControllerMode::JointImpedance,
                ..Default::default()
            };
            let mut durations = Vec::with_capacity(cycles);
            let mut bytes = [0; 4096];
            for cycle in 1..=cycles {
                state.message_id = cycle as u64;
                let packet = bincode::serialize(&state).unwrap();
                let start = Instant::now();
                udp.send_to(&packet, local_addr).unwrap();
                let (size, _) = udp.recv_from(&mut bytes).unwrap();
                durations.push(start.elapsed());
                let command: RobotCommand = bincode::deserialize(&bytes[..size]).unwrap();
                assert_eq!(command.command_id(), cycle as u64);
                assert_eq!(command.motion.motion_generation_finished, cycle == cycles);
            }
            state.message_id += 1;
            state.motion_generator_mode = MotionGeneratorMode::Idle;
            state.controller_mode = ControllerMode::Other;
            udp.send_to(&bincode::serialize(&state).unwrap(), local_addr)
                .unwrap();
            tcp.write_all(&response(Command::Move, move_id, 0)).unwrap();
            durations
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
    };
    let mut robot = public_robot(robot);
    let start = Instant::now();
    match backend {
        Backend::Std => {
            let mut remaining = cycles;
            <_ as ControlWith<JointPositionControl<7>>>::control_with_flow(&mut robot, |_, _| {
                remaining -= 1;
                ControlFlow::Continue(([0.0; 7], remaining == 0))
            })
            .unwrap();
        }
        Backend::BlockingAsync => {
            let mut remaining = cycles;
            <_ as ControlWith<JointPositionControl<7>>>::control_with_flow_async(
                &mut robot,
                async |_, _| {
                    remaining -= 1;
                    ControlFlow::Continue(([0.0; 7], remaining == 0))
                },
            )
            .unwrap();
        }
        Backend::Native => {
            let mut callback = ReadyController { remaining: cycles };
            runtime
                .block_on(
                    <_ as AsyncControlWith<JointPositionControl<7>>>::control_native_async(
                        &mut robot,
                        &mut callback,
                    ),
                )
                .unwrap();
        }
    }
    let elapsed = start.elapsed();
    (elapsed, server.join().unwrap())
}

#[test]
#[ignore = "finite performance measurement; run alone in release mode"]
fn native_loopback_performance() {
    let rounds = std::env::var("FRANKA_PERF_ROUNDS")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(11);
    let cycles = std::env::var("FRANKA_PERF_CYCLES")
        .ok()
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(1000);
    assert!(rounds > 0 && cycles > 1);
    let runtime = runtime(); // Native callers already own an executor; do not count constructing it per session.
    let backends = [Backend::Std, Backend::BlockingAsync, Backend::Native];
    println!(
        "# loopback only; current-thread Tokio; public JointPosition channel; no pacing; peer RTT includes OS scheduling and codec/filter work"
    );
    println!(
        "# state_bytes={},command_bytes={}",
        bincode::serialized_size(&RobotStateInter::default()).unwrap(),
        bincode::serialized_size(&RobotCommand::default()).unwrap()
    );
    println!("backend,cycles,round,session_ns,cycle_p50_ns,cycle_p99_ns");
    for count in [1, cycles] {
        for backend in backends {
            for _ in 0..3 {
                let _ = measure(backend, count, &runtime);
            }
        }
        for round in 0..rounds {
            for offset in 0..3 {
                let backend = backends[(round + offset) % 3];
                let (elapsed, mut samples) = measure(backend, count, &runtime);
                samples.sort_unstable();
                let p50 = samples[(samples.len() - 1) / 2].as_nanos();
                let p99 = samples[(samples.len() * 99).div_ceil(100).saturating_sub(1)].as_nanos();
                println!(
                    "{},{},{},{},{},{}",
                    backend.label(),
                    count,
                    round,
                    elapsed.as_nanos(),
                    p50,
                    p99
                );
            }
        }
    }
}
