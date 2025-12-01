use franka_rust::{FrankaEmika, types::robot_types::SetCollisionBehaviorData};
use robot_behavior::{Arm, ArmPreplannedMotion, MotionType, RobotResult};

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
        -1.885_958_011_723_401_2,
        0.105_694_198_780_145_81,
        0.117_699_956_083_333_68,
        -1.825_844_729_876_874_3,
        -0.015_440_421_203_763_977,
        2.055_922_685_702_638_5,
        -2.453_832_040_861_249,
    ];

    let joint2 = [
        -2.416_935_646_437_762,
        -0.012_945_170_855_966_574,
        -0.218_838_851_790_679_1,
        -2.126_129_379_520_086_6,
        0.021_287_867_377_174_18,
        2.203_961_069_822_311,
        -1.123_585_085_306_893_6,
    ];

    robot.with_scale(0.3).move_to(MotionType::Joint(joint2))?;
    robot.with_scale(0.3).move_to(MotionType::Joint(joint1))?;
    robot.with_scale(0.3).move_joint(&joint2)?;
    robot.with_scale(0.3).move_joint(&joint1)?;

    println!("Joint 1 position reached");
    // std::thread::sleep(Duration::from_secs(2));

    Ok(())
}
