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
        0.999988717111279,
        -0.0017158199764830543,
        0.0006070528432007165,
        0.0,
        -0.0017159793580267732,
        -0.9999888667416037,
        0.0002621238143371493,
        0.0,
        0.0006066080065316006,
        -0.00026316761376355145,
        -0.9999997813847427,
        0.0,
        0.30648819513520104,
        -0.0005332618190956864,
        0.48708322693240846,
        1.0,
    ]);

    robot.with_speed(0.3).move_to(MotionType::Cartesian(pose))?;

    Ok(())
}
