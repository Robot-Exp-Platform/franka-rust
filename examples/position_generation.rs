use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{ArmBehavior, ArmPreplannedMotion, MotionType, RobotResult};

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

    let joint1 = [
        -1.8859580117234012,
        0.10569419878014581,
        0.11769995608333368,
        -1.8258447298768743,
        -0.015440421203763977,
        2.0559226857026385,
        -2.453832040861249,
    ];

    let joint2 = [
        -2.416935646437762,
        -0.012945170855966574,
        -0.2188388517906791,
        -2.1261293795200866,
        0.02128786737717418,
        2.203961069822311,
        -1.1235850853068936,
    ];

    robot.with_speed(0.3).move_to(MotionType::Joint(joint2))?;
    robot.with_speed(0.3).move_to(MotionType::Joint(joint1))?;
    robot.with_speed(0.3).move_to(MotionType::Joint(joint2))?;
    robot.with_speed(0.3).move_to(MotionType::Joint(joint1))?;

    println!("Joint 1 position reached");
    // std::thread::sleep(Duration::from_secs(2));

    Ok(())
}
