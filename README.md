# Readme

[English](README.md) | [简体中文](README_cn.md)

unofficial rust implementation of libfranka!

This library is part of the [Universal Robot Driver Project](https://github.com/Robot-Exp-Platform/libhans-rs)! We are committed to providing Rust driver support for more robotic platforms! **Unifying driver interfaces across different robot models, reducing the learning curve for robotics, and delivering more efficient robot control solutions!**

During implementation, we referenced both [libfranka](https://github.com/frankaemika/libfranka) and [libfranka-rs](https://github.com/marcbone/libfranka-rs). Special thanks to open-source contributors, especially [marcbone](https://github.com/marcbone) - their library provided significant assistance! This implementation is not intended to be a direct replica of the official approach, but rather follows a more **idiomatic Rust** methodology.

Additionally, the official driver implementation has limited support for non-`Ubuntu` systems and high dependency on real-time kernels. In this library, we make real-time kernels an optional dependency. We strive to provide support for platforms without real-time kernels or with performance constraints.

| OS         | Architecture | Support |
| ---------- | ------------ | ------- |
| Windows 10 | x86_64       | ✅      |
| Windows 10 | amd64        | ✅      |
| Windows 11 | x86_64       | ✅      |
| Windows 11 | amd64        | ✅      |
| Ubuntu 20.04 | x86_64     | ✅      |
| Ubuntu 22.04 | x86_64     | ✅      |
| macOS 13   | x86_64       | ✅      |
| macOS 14   | aarch64      | ✅      |
| Other      | Other        | ???     |

Other OS/arch combinations may work but remain untested due to hardware limitations. PRs and issues are welcome!

This library also aims to support multiple language bindings through our unified interface. Currently supported:

- [Python](https://pypi.org/project/franka-rust/)
- **C++** (Planned)
- **Java** (Planned)
- **Go** (Planned)
- **C#** (Planned)

## Requirements

- Rust 2024

## Features

- [x] Robot state reading
- [x] Robot behavior parameters, controller parameters, and payload configuration
- [x] Access to official Franka dynamic models
- [ ] Known-target interfaces
  - [x] Joint-space point-to-point motion generator (blocking/async)
  - [x] Cartesian-space point-to-point motion generator (blocking/async)
  - [x] Waypoint spline motion generator (blocking/async)
- [x] Closure-based control interfaces
  - [x] Joint position control
  - [x] Joint velocity control
  - [x] Cartesian position control
  - [x] Cartesian velocity control
  - [x] Torque control
- [ ] Handler tracking interfaces
  - [ ] Joint position tracking
  - [ ] Joint velocity tracking
  - [ ] Cartesian position tracking
  - [ ] Cartesian velocity tracking
  - [ ] Torque tracking
- [x] Official gripper support
- [ ] Official vacuum gripper support

## Quick Start

Add to your `Cargo.toml`:

```toml
[dependencies]
franka-rust = "*"
```

Minimal example (see [examples](/examples) for more):

```rust
fn main() -> RobotResult<()> {
    let mut robot = FrankaRobot::new("172.16.0.3");
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

    robot.move_to(MotionType::Joint(FRANKA_ROBOT_DEFAULT_JOINT), 0.3)?;

    Ok(())
}
```

This code connects to a Franka Emika robot at `172.16.0.3` and performs a blocking move to `FRANKA_ROBOT_DEFAULT_JOINT`.

Simple, right? Give it a try!

## TODO

- [ ] more examples
- [ ] handler interface
