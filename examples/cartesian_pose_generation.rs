use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{Arm, ArmPreplannedMotion, MotionType, Pose, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    robot.set_default_behavior()?;

    robot.set_collision_behavior(SetCollisionBehaviorData {
        lower_torque_thresholds_acceleration: [20., 20., 18., 18., 16., 14., 12.],
        upper_torque_thresholds_acceleration: [20., 20., 18., 18., 16., 14., 12.],
        lower_torque_thresholds_nominal: [20., 20., 18., 18., 16., 14., 12.],
        upper_torque_thresholds_nominal: [20., 20., 18., 18., 16., 14., 12.],
        lower_force_thresholds_acceleration: [20., 20., 20., 25., 25., 25.],
        upper_force_thresholds_acceleration: [20., 20., 20., 25., 25., 25.],
        lower_force_thresholds_nominal: [20., 20., 20., 25., 25., 25.],
        upper_force_thresholds_nominal: [20., 20., 20., 25., 25., 25.],
    })?;

    let pose = Pose::Homo([
        0.999_988_717_111_279,
        -0.001_715_819_976_483_054_3,
        0.000_607_052_843_200_716_5,
        0.0,
        -0.001_715_979_358_026_773_2,
        -0.999_988_866_741_603_7,
        0.000_262_123_814_337_149_3,
        0.0,
        0.000_606_608_006_531_600_6,
        -0.000_263_167_613_763_551_45,
        -0.999_999_781_384_742_7,
        0.0,
        0.306_488_195_135_201_04,
        -0.000_533_261_819_095_686_4,
        0.487_083_226_932_408_46,
        1.0,
    ]);

    robot.with_speed(0.3).move_to(MotionType::Cartesian(pose))?;

    Ok(())
}
