# Readme

[English](README.md) | [简体中文](README_cn.md)

非官方 `libfranka` 的 `Rust` 实现！

本库是[通用机器人驱动计划](https://github.com/Robot-Exp-Platform/libhans-rs)中的一员！我们立志于为更多的机器人平台提供 Rust 语言的驱动支持！**统一不同型号的机器人驱动接口，降低机器人学习成本，提供更高效的机器人控制方案！**

在实现过程中，参考了 [libfranka](https://github.com/frankaemika/libfranka) 实现和 [libfranka-rs](https://github.com/marcbone/libfranka-rs) 实现，感谢开源作者的贡献！尤其是[marcbone](https://github.com/marcbone),他是的库帮助了我很多！本库并不愿意成为对官方的实现方式的完全复现，而是考虑按照更加 **rustly** 的方式来实现。

另外一方面，官方的驱动实现对于非 `ubuntu` 的系统支持并不友好，对实时内核的需求也较高，在本库中我们将实时内核作为可选的依赖，对于有实时内核的用户固然友好，对于没有实时内核或者性能受限的平台我们也尽可能的提供支持。

| 操作系统 | 架构 | 支持 |
| ------- | ---- | ---- |
| Windows 10 | x86_64 | ✅ |
| Windows 10 | amd64  | ✅ |
| Windows 11 | x86_64 | ✅ |
| Windows 11 | amd64  | ✅ |
| Ubuntu 20.04 | x86_64 | ✅ |
| Ubuntu 22.04 | x86_64 | ✅ |
| Macos 13 | x86_64 | ✅ |
| Macos 14 | aarch64 | ✅ |
| other | other | ??? |

一些其他的操作系统和架构或许也可以顺利运行，但是受限于个人设备的限制，我无法进行测试，欢迎大家提交 PR 或者 issue！

本库还尝试支持更多语言的绑定，使用我们的统一接口！目前支持的语言有：

- [Python](https://pypi.org/project/franka-rust/)
- **C++**, 计划中
- **Java**, 计划中
- **Go**, 计划中
- **C#**, 计划中

## 环境要求

- rust 2024

## 功能支持

- [x] 机械臂状态读取
- [x] 机械臂行为参数、控制器参数、负载参数设置
- [x] 获取 franka 官方提供的动力学模型
- [ ] 已知目标类接口
  - [x] 关节空间点到点运动生成器（阻塞、异步）
  - [x] 笛卡尔空间点到点运动生成器（阻塞、异步）
  - [x] 路点样条曲线运动生成器（阻塞、异步）
- [x] 闭包控制接口
  - [x] 关节空间位置运动生成器
  - [x] 关节空间速度运动生成器
  - [x] 笛卡尔空间位置运动生成器
  - [x] 笛卡尔空间速度运动生成器
  - [x] 力矩控制闭包控制器
- [ ] 表象句柄跟踪接口
  - [ ] 关节空间位置句柄跟踪器
  - [ ] 关节空间速度句柄跟踪器
  - [ ] 笛卡尔空间位置句柄跟踪器
  - [ ] 笛卡尔空间速度句柄跟踪器
  - [ ] 力矩控制句柄跟踪器
- [x] 官方夹爪支持
- [ ] 官方真空夹爪支持

## quick start

在你的 `Cargo.toml` 中添加以下依赖：

```toml
[dependencies]
franka-rust = "*"
```

一个极短的示例如下，更多的示例见 [example](/examples) 目录：

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

这段代码连接了 `ip = 172.16.0.3` 的 `Franka Emika`, 会阻塞的抵达 `FRANKA_ROBOT_DEFAULT_JOINT` 位置。

超简单的，对吧？快来试试吧！

## TODO

- [ ] more examples
- [ ] handler interface
