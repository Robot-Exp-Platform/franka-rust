use franka_rust::FrankaEmika;
use robot_behavior::{ArmPreplannedMotionExt, RobotResult};
use std::fs::File;
use std::io::{self, BufRead};
use std::path::Path;

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.3");
    robot.set_default_behavior()?;
    let file_path = "/home/robot-lab-user/WYN/franka_exp_TOPP_ASL/data/test_acc_jerk_compare_A10J300_20250615/interpolation_output5_jerk.csv";

    let path = Path::new(file_path);
    let file = File::open(path)?;
    let reader = io::BufReader::new(file);

    let mut data = Vec::new();

    for line in reader.lines() {
        let line = line?;
        let values: Vec<f64> = line
            .split(',')
            .map(|s| s.trim().parse().unwrap_or_else(|_| 0.0)) // 处理解析错误
            .collect();

        if values.len() == 7 {
            let row: [f64; 7] = [
                values[0], values[1], values[2], values[3], values[4], values[5], values[6],
            ];
            data.push(row);
        } else {
            eprintln!(
                "Warning: Line does not contain exactly 7 values and will be skipped: {}",
                line
            );
        }
    }

    robot.move_joint_path(data).unwrap();

    Ok(())
}
