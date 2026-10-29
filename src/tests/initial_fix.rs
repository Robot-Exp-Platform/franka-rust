//! Offline regressions for defects reproduced against 912fb63. No device access.
use crate::{
    FrankaEmika, FrankaRobotImpl,
    network::Network,
    robot::*,
    types::{
        robot_command::{ControllerCommand, MotionGeneratorCommand},
        robot_state::RobotStateInter,
        robot_types::{MoveControllerMode, MoveMotionGeneratorMode},
    },
    utils::*,
};
use roplat_behavior::*;
use std::{
    net::{TcpListener, TcpStream, UdpSocket},
    sync::{Arc, RwLock},
};

fn eye() -> [f64; 16] {
    Pose::IDENTITY.homo()
}
fn qdiff(a: [f64; 7], b: [f64; 7]) -> f64 {
    a.into_iter()
        .zip(b)
        .map(|(a, b)| (a - b).abs())
        .fold(0.0, f64::max)
}
fn robot() -> (FrankaEmika, TcpStream) {
    let listener = TcpListener::bind("127.0.0.1:0").unwrap();
    let network = Network::new("127.0.0.1", listener.local_addr().unwrap().port());
    let (peer, _) = listener.accept().unwrap();
    let state = RobotStateInter {
        q: FrankaEmika::JOINT_DEFAULT,
        q_d: FrankaEmika::JOINT_DEFAULT,
        F_T_EE: eye(),
        EE_T_K: eye(),
        ..Default::default()
    };
    let mut robot = FrankaEmika::from_test_impl(FrankaRobotImpl {
        network,
        udp_socket: UdpSocket::bind("127.0.0.1:0").unwrap(),
        robot_state: Arc::new(RwLock::new(state)),
        motion_command_id: None,
    });
    robot.with_scale(0.2);
    robot.set_coord(Coord::Relative).unwrap();
    (robot, peer)
}
fn assert_joint_limits(traj: &[[f64; 7]], v: f64, a: f64) {
    for w in traj.windows(2) {
        assert!(qdiff(w[1], w[0]) / 0.001 <= v + 1e-8);
    }
    for w in traj.windows(3) {
        for k in 0..7 {
            assert!(((w[2][k] - 2.0 * w[1][k] + w[0][k]) / 1e-6).abs() <= a + 1e-6);
        }
    }
}

#[test]
fn zero_sum_commands_and_zero_velocity_do_not_bypass_limits() {
    let q0 = FrankaEmika::JOINT_DEFAULT;
    let q = [1.0, -1.0, 0.0, -2.0, 0.0, 2.0, 0.0];
    let out = franka_limit_rate_joint_positions(&q, &q0, &[0.0; 7], &[0.0; 7]);
    assert!(qdiff(out, q0) < 1e-4);
    let v = franka_limit_rate_joint_velocities(&[0.0; 7], &[0.1; 7], &[0.0; 7]);
    assert!(v.iter().all(|x| *x > 0.07 && *x < 0.1));
    let v =
        franka_limit_rate_cartesian_velocity(&[0.0; 6], &[0.1, 0.0, 0.0, 0.0, 0.0, 0.0], &[0.0; 6]);
    assert!(v[0] > 0.09 && v[0] < 0.1);
}

#[test]
fn lowpass_is_applied_before_rate_limiting_in_all_five_modes() {
    let state = RobotStateInter {
        O_T_EE: eye(),
        O_T_EE_d: eye(),
        O_T_EE_c: eye(),
        ..Default::default()
    };
    let small = 1e-7;
    let expected = low_pass_filter(0.001, 100.0, 0.0, small);
    for (mode, command) in [
        (
            MoveMotionGeneratorMode::JointPosition,
            MotionGeneratorCommand {
                q_c: [small; 7],
                ..Default::default()
            },
        ),
        (
            MoveMotionGeneratorMode::JointVelocity,
            MotionGeneratorCommand {
                dq_c: [small; 7],
                ..Default::default()
            },
        ),
        (
            MoveMotionGeneratorMode::CartesianVelocity,
            MotionGeneratorCommand {
                dpose_o_to_ee_c: [small; 6],
                ..Default::default()
            },
        ),
    ] {
        let out = command.filter_for_mode(&state, mode);
        let actual = match mode {
            MoveMotionGeneratorMode::JointPosition => out.q_c[0],
            MoveMotionGeneratorMode::JointVelocity => out.dq_c[0],
            _ => out.dpose_o_to_ee_c[0],
        };
        assert!((actual - expected).abs() < 1e-13);
    }
    let mut pose = eye();
    pose[12] = 1e-9;
    let out = MotionGeneratorCommand {
        pose_o_to_ee_c: pose,
        ..Default::default()
    }
    .filter_for_mode(&state, MoveMotionGeneratorMode::CartesianPosition);
    assert!((out.pose_o_to_ee_c[12] - low_pass_filter(0.001, 100.0, 0.0, 1e-9)).abs() < 1e-13);
    let out = ControllerCommand {
        tau_j_d: [small; 7],
    }
    .filter_for_mode(&state, MoveControllerMode::ExternalController);
    assert!((out.tau_j_d[0] - expected).abs() < 1e-13);
    // The same vector helper also supports Panda's seven joints.
    assert_eq!(
        joint_low_pass_filter(0.001, &[small; 7], &[0.0; 7], 100.0),
        [expected; 7]
    );
}

#[test]
fn flange_fk_and_jacobian_match_saved_official_model() {
    let f: serde_json::Value =
        serde_json::from_str(include_str!("franka_default_model.json")).unwrap();
    let q = FrankaEmika::JOINT_DEFAULT;
    let actual = crate::franka::kinematics::flange_pose::<FrankaEmika>(&q).homo();
    for (i, x) in actual.iter().enumerate() {
        assert!((x - f["pose"][i].as_f64().unwrap()).abs() < 1e-9);
    }
    let j = crate::franka::kinematics::flange_jacobian::<FrankaEmika>(&q);
    for (i, x) in j.iter().enumerate() {
        assert!(
            (x - f["zero_jacobian"][i].as_f64().unwrap()).abs() < 1e-9,
            "element {i}"
        );
    }
}

#[test]
fn zero_cartesian_offset_holds_start_and_unreachable_target_is_rejected() {
    let (mut r, _peer) = robot();
    let q = FrankaEmika::JOINT_DEFAULT;
    let t = r.plan_flange_motion(q, Pose::IDENTITY).unwrap();
    assert!(t.iter().all(|p| qdiff(*p, q) < 1e-10));
    r.set_coord(Coord::OCS).unwrap();
    assert!(
        r.plan_flange_motion(q, Pose::from_position([10.0, 0.0, 0.0]))
            .is_err()
    );
}

#[test]
fn cartesian_motion_uses_instance_joint_limits_and_has_no_start_step() {
    let (r, _peer) = robot();
    let q = FrankaEmika::JOINT_DEFAULT;
    let r = r
        .with_joint_vel([0.01; 7])
        .with_joint_acc([0.05; 7])
        .with_joint_jerk([1.0; 7]);
    let delta = Pose::from_position([0.03, 0.0, 0.0]);
    assert_eq!(r.get_joint_vel_bound(), [0.01; 7]);
    let t = r.plan_flange_motion(q, delta).unwrap();
    assert!(qdiff(t[0], q) < 1e-9);
    assert_joint_limits(&t, 0.01, 0.05);
    let actual = crate::franka::kinematics::flange_pose::<FrankaEmika>(t.last().unwrap()).quat();
    let expected = (crate::franka::kinematics::flange_pose::<FrankaEmika>(&q) * delta).quat();
    assert!((actual.translation.vector - expected.translation.vector).norm() < 1e-4);
    assert!(actual.rotation.angle_to(&expected.rotation) < 1e-4);
}

#[test]
fn cartesian_linear_and_rotation_overrides_are_independent() {
    let (r, _peer) = robot();
    let q = FrankaEmika::JOINT_DEFAULT;
    let r = r
        .with_cartesian_vel(0.003)
        .with_cartesian_acc(0.03)
        .with_cartesian_jerk(0.3)
        .with_rotation_vel(0.01)
        .with_rotation_acc(0.02)
        .with_rotation_jerk(0.1);
    assert_eq!(r.get_cartesian_vel_bound(), 0.003);
    assert_eq!(r.get_rotation_vel_bound(), 0.01);
    let t = r
        .plan_flange_motion(q, Pose::from_position([0.03, 0.0, 0.0]))
        .unwrap();
    let mut vmax = 0.0f64;
    for w in t.windows(2) {
        let a = crate::franka::kinematics::flange_pose::<FrankaEmika>(&w[0]).quat();
        let b = crate::franka::kinematics::flange_pose::<FrankaEmika>(&w[1]).quat();
        vmax = vmax.max((a.translation.vector - b.translation.vector).norm() / 0.001);
    }
    assert!(vmax <= 0.003 * 1.01, "linear velocity {vmax}");
    let rotation = Pose::from_isometry(nalgebra::Isometry3::new(
        nalgebra::Vector3::zeros(),
        nalgebra::Vector3::new(0.0, 0.0, 0.05),
    ));
    let t = r.plan_flange_motion(q, rotation).unwrap();
    let mut wmax = 0.0f64;
    for w in t.windows(2) {
        let a = crate::franka::kinematics::flange_pose::<FrankaEmika>(&w[0]).quat();
        let b = crate::franka::kinematics::flange_pose::<FrankaEmika>(&w[1]).quat();
        wmax = wmax.max(a.rotation.angle_to(&b.rotation) / 0.001);
    }
    assert!(wmax <= 0.01 * 1.01, "angular velocity {wmax}");
}

#[test]
fn unified_frames_and_native_cartesian_hold_agree_with_tool_offset() {
    let mut ee = eye();
    ee[12] = 0.4;
    ee[13] = 0.1;
    ee[14] = 0.7;
    let mut tool = eye();
    tool[14] = 0.1;
    let mut k = eye();
    k[12] = 0.02;
    let state = RobotStateInter {
        O_T_EE: ee,
        O_T_EE_d: ee,
        O_T_EE_c: ee,
        F_T_EE: tool,
        EE_T_K: k,
        q_d: FrankaEmika::JOINT_DEFAULT,
        theta: [1.0; 7],
        O_dP_EE_d: [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
        O_F_ext_hat_K: [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
        O_ddP_O: [0.0, 0.0, -9.81],
        ..Default::default()
    };
    let arm: ArmState<7> = state.into();
    let flange = arm.flange.meas.pose().unwrap();
    let p = flange.position();
    assert!((p[0] - 0.4).abs() < 1e-12 && (p[2] - 0.6).abs() < 1e-12);
    assert!((arm.tcp.as_ref().unwrap().meas.pose().unwrap().position()[2] - 0.7).abs() < 1e-12);
    assert!(
        (arm.stiffness
            .as_ref()
            .unwrap()
            .meas
            .pose()
            .unwrap()
            .position()[0]
            - 0.42)
            .abs()
            < 1e-12
    );
    let twist: [f64; 6] = arm.flange.des.twist().unwrap().into();
    assert!((twist[0] + 0.1).abs() < 1e-12);
    assert!(arm.flange.des.accel().is_none());
    assert!(arm.joint.cmd.q().is_none());
    let hold = <FrankaEmika as ControlWith<JointPositionControl<7>>>::hold_command(&arm.joint);
    assert!(qdiff(hold, FrankaEmika::JOINT_DEFAULT) < 1e-12);
    let wrench: [f64; 6] = arm.flange.meas.wrench().unwrap().into();
    assert!((wrench[3] + 0.1).abs() < 1e-12 && (wrench[5] - 0.02).abs() < 1e-12);
    let mut rotating = state;
    rotating.O_dP_EE_c = [0.0, 0.0, 0.0, 0.0, 1.0, 0.0];
    let rotating: ArmState<7> = rotating.into();
    let accel: [f64; 6] = rotating.flange.cmd.accel().unwrap().into();
    assert!((accel[2] - 0.1).abs() < 1e-12);
    let native = flange_command_to_ee(flange, &state).homo();
    for i in 0..16 {
        assert!((native[i] - ee[i]).abs() < 1e-12);
    }
    let velocity = flange_velocity_to_ee([0.0, 0.0, 0.0, 0.0, 1.0, 0.0], &state);
    assert!((velocity[0] - 0.1).abs() < 1e-12);
    let output = MotionGeneratorCommand {
        pose_o_to_ee_c: native,
        ..Default::default()
    }
    .filter_for_mode(&state, MoveMotionGeneratorMode::CartesianPosition);
    for i in 0..16 {
        assert!((output.pose_o_to_ee_c[i] - ee[i]).abs() < 1e-12);
    }
}

#[test]
fn original_planner_entrypoint_matches_explicit_default_joint_limits() {
    use roplat_trajectory::*;
    let q = FrankaEmika::JOINT_DEFAULT;
    let start = crate::franka::kinematics::flange_pose::<FrankaEmika>(&q);
    let target = start * Pose::from_position([0.03, 0.0, 0.0]);
    let solve = |p, seed| crate::franka::kinematics::solve_flange_ik::<FrankaEmika>(seed, p);
    let original = plan_cartesian_line_traj_via_time_scaling::<FrankaEmika, _, 7>(
        start, target, q, 0.2, solve,
    )
    .unwrap();
    let (a, b) = (start.quat(), target.quat());
    let explicit = plan_cartesian_path_traj_via_time_scaling_with_limits::<FrankaEmika, _, _, 7>(
        |s| Pose::from_isometry(a.lerp_slerp(&b, s.clamp(0.0, 1.0))),
        q,
        solve,
        CartesianTimeScalingLimits::from_robot::<FrankaEmika>(0.2),
        Default::default(),
    )
    .unwrap();
    assert_eq!(original, explicit);
}
