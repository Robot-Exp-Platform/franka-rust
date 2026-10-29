//! Exercise all public Cartesian-pose entries against a real loopback wire peer.
use super::*;
use crate::FrankaEmika;
use roplat_behavior::{
    ArmState, AsyncControlWith, CartesianPoseControl, CartesianVelocityControl, ControlWith, Pose,
};

fn fixture(velocity: bool) -> (FrankaEmika, thread::JoinHandle<()>) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
    let client = UdpSocket::bind("127.0.0.1:0").unwrap();
    client
        .set_read_timeout(Some(Duration::from_secs(3)))
        .unwrap();
    let addr = client.local_addr().unwrap();
    let peer = UdpSocket::bind("127.0.0.1:0").unwrap();
    peer.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
    let ee = Pose::from_position([0.4, 0.1, 0.7]).homo();
    let mut state = RobotStateInter {
        message_id: 1,
        O_T_EE: ee,
        O_T_EE_d: ee,
        O_T_EE_c: ee,
        F_T_EE: Pose::from_position([0.0, 0.0, 0.1]).homo(),
        EE_T_K: Pose::IDENTITY.homo(),
        motion_generator_mode: MotionGeneratorMode::CartesianPosition,
        controller_mode: ControllerMode::JointImpedance,
        ..Default::default()
    };
    if velocity {
        state.motion_generator_mode = MotionGeneratorMode::CartesianVelocity;
        state.O_dP_EE_c = [0.1, 0.0, 0.0, 0.0, 1.0, 0.0];
    }
    let mut cached = state;
    cached.message_id = 0; // The next wire state must be newer than the cache.
    let server = thread::spawn(move || {
        let (mut stream, _) = listener.accept().unwrap();
        stream
            .set_read_timeout(Some(Duration::from_secs(3)))
            .unwrap();
        let (command, id) = read_request(&mut stream);
        assert_eq!(command, Command::Move as u32);
        stream.write_all(&response(Command::Move, id, 1)).unwrap();
        peer.send_to(&bincode::serialize(&state).unwrap(), addr)
            .unwrap();
        let mut buffer = [0; 4096];
        let (len, _) = peer.recv_from(&mut buffer).unwrap();
        let cmd: RobotCommand = bincode::deserialize(&buffer[..len]).unwrap();
        assert!(cmd.motion.motion_generation_finished);
        if velocity {
            let actual = cmd.motion.dpose_o_to_ee_c;
            let expected = [0.1, 0.0, 0.0, 0.0, 1.0, 0.0];
            for i in 0..6 {
                assert!(
                    (actual[i] - expected[i]).abs() < 1e-12,
                    "velocity wire element {i}"
                );
            }
        } else {
            let actual = cmd.motion.pose_o_to_ee_c;
            for i in 0..16 {
                assert!((actual[i] - ee[i]).abs() < 1e-12, "pose wire element {i}");
            }
        }
        state.message_id = 2;
        state.motion_generator_mode = MotionGeneratorMode::Idle;
        state.controller_mode = ControllerMode::Other;
        peer.send_to(&bincode::serialize(&state).unwrap(), addr)
            .unwrap();
        stream.write_all(&response(Command::Move, id, 0)).unwrap();
    });
    let robot = FrankaEmika::from_test_impl(FrankaRobotImpl {
        network,
        udp_socket: client,
        robot_state: Arc::new(RwLock::new(cached)),
        motion_command_id: None,
    });
    (robot, server)
}

#[test]
fn all_cartesian_entries_preserve_tool_frame_on_wire() {
    run_bounded(|| {
        for entry in 0..6 {
            let velocity = entry >= 3;
            let (mut robot, server) = fixture(velocity);
            if velocity {
                let hold = |state: ArmState<7>| {
                    let twist: [f64; 6] = state.flange.cmd.twist().unwrap().into();
                    ControlFlow::Continue((twist, true))
                };
                match entry - 3 {
                    0 => <_ as ControlWith<CartesianVelocityControl<7>>>::control_with_flow(
                        &mut robot,
                        |state, _| hold(state),
                    )
                    .unwrap(),
                    1 => <_ as ControlWith<CartesianVelocityControl<7>>>::control_with_flow_async(
                        &mut robot,
                        async |state, _| hold(state),
                    )
                    .unwrap(),
                    _ => {
                        let mut callback = |state: ArmState<7>, _: Duration| async move {
                            let twist: [f64; 6] = state.flange.cmd.twist().unwrap().into();
                            ControlFlow::Continue((twist, true))
                        };
                        tokio::runtime::Builder::new_current_thread().enable_all().build().unwrap().block_on(<_ as AsyncControlWith<CartesianVelocityControl<7>>>::control_native_async(&mut robot, &mut callback)).unwrap();
                    }
                }
                server.join().unwrap();
                continue;
            }
            let hold = |state: ArmState<7>| {
                ControlFlow::Continue((state.flange.meas.pose().unwrap(), true))
            };
            match entry {
                0 => <_ as ControlWith<CartesianPoseControl<7>>>::control_with_flow(
                    &mut robot,
                    |state, _| hold(state),
                )
                .unwrap(),
                1 => <_ as ControlWith<CartesianPoseControl<7>>>::control_with_flow_async(
                    &mut robot,
                    async |state, _| hold(state),
                )
                .unwrap(),
                _ => {
                    let mut callback = |state: ArmState<7>, _: Duration| async move {
                        ControlFlow::Continue((state.flange.meas.pose().unwrap(), true))
                    };
                    tokio::runtime::Builder::new_current_thread()
                        .enable_all()
                        .build()
                        .unwrap()
                        .block_on(
                            <_ as AsyncControlWith<CartesianPoseControl<7>>>::control_native_async(
                                &mut robot,
                                &mut callback,
                            ),
                        )
                        .unwrap();
                }
            }
            server.join().unwrap();
        }
    });
}
