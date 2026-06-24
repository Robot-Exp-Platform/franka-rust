# franka_rust

[English](README.md) | [简体中文](README_zh.md)

`franka_rust` 是 Robot-Exp 驱动栈中的 Franka 机器人 Rust 驱动。它把
Franka FCI 接入 `robot_behavior` 的统一行为特征，让真机、仿真器和 roplat
编排可以使用同一套运动空间、控制空间和状态模型。

这个库不是对 `libfranka` 的逐行翻译。底层协议遵循 Franka FCI 的语义，
但公开 API 更偏向 Rust：显式所有权、类型化空间、作用域内控制闭包，以及
尽量少的冗余封装。

## 能做什么

- 提供 `FrankaEmika`、`FrankaPanda`、`FrankaFR3`、`FrankaFP3` 等型号入口。
- 支持 `robot_behavior` 的关节空间和笛卡尔空间目标运动。
- 支持阻塞式 `control_with`，覆盖关节位置、关节速度、笛卡尔位姿、
  笛卡尔速度和力矩控制。
- 支持 `control_with_async`：每帧控制器闭包可以是异步闭包，但机器人资源
  仍在整个控制会话内被独占借用。
- 保留 Franka 厂家特有配置：碰撞阈值、内部阻抗、guiding mode、负载和坐标帧。
- 支持下载并加载 Franka 官方模型库，用于运动学和动力学计算。
- 保留 Python / C++ FFI 层；这些接口作为 `robot_behavior` 的下游适配存在。

## 0.2 版本设计

`0.2.0` 对齐当前 `robot_behavior` 的控制语义：

- `control_with` 是阻塞任务，直到控制器返回 `done = true` 或机器人报错。
- 控制闭包不再要求 `'static`，可以借用当前函数栈上的轨迹、模型或日志器。
- 同步控制路径直接使用 `std::net::UdpSocket` 运行实时循环。
- 异步控制路径使用 `tokio::net::UdpSocket` 运行实时循环。
- `move_to`、配置类命令、模型加载等普通阻塞指令继续使用简单的同步命令平面。

也就是说，对外仍然只有一个机器人对象；同步和异步只是控制会话的内部实现选择。

## 与其他库的关系

- `robot_behavior` 提供通用行为特征、状态模型和控制器工具。
- `roplat` 集成共享同一套行为词汇；Franka 原生实时节律需要和 roplat
  的异步执行边界一起设计，不应该简单包一层阻塞控制循环。
- `rsbullet`、`libjaka-rs` 是同一 workspace 内的其他后端，它们共享行为词汇，
  但各自使用适合自身的通讯内核。

## 环境要求

- Rust nightly，edition 2024。
- 可访问的 Franka 控制器，并运行兼容的 FCI 服务。
- 真机高频实验建议使用实时 Linux 内核；非实时系统适合开发、编译和低风险验证。

## 安装

```toml
[dependencies]
franka_rust = "0.2"
robot_behavior = "0.6"
```

在当前 workspace 中，根 `Cargo.toml` 会通过 `[patch.crates-io]` 把 `roplat`
和 `robot_behavior` 指向本地源码。

## 快速开始

```rust
use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.2");
    robot.set_default_behavior()?;

    robot.move_to::<JointSpace<7>>(FrankaEmika::JOINT_DEFAULT)?;

    Ok(())
}
```

## 实时控制

```rust
use franka_rust::FrankaEmika;
use robot_behavior::{RobotResult, behavior::*};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.2");
    robot.set_default_behavior()?;

    let mut elapsed = 0.0;
    robot.control_with::<TorqueControl<7>, _>(|joint, dt| {
        elapsed += dt.as_secs_f64();

        let q = joint.meas.q.unwrap_or([0.0; 7]);
        let tau = q.map(|position| -0.5 * position);
        let done = elapsed > 2.0;

        (tau, done)
    })?;

    Ok(())
}
```

如果控制律本身不依赖 Franka 私有能力，优先使用 `robot_behavior::controller`
中的控制器构造函数；`franka_rust` 只负责把这些控制器落到 Franka FCI。

## 示例

`examples/` 目录覆盖：

- 关节和笛卡尔目标运动，
- 关节和笛卡尔实时命令生成，
- 力矩控制，
- `robot_behavior` 提供的阻抗控制器，
- Franka 模型加载，
- 夹爪使用，
- 当前抽象已经足够覆盖的 roplat 示例。

示例会被 `cargo check --all-targets` 编译检查，但只有显式运行时才会连接真机。

## 安全说明

这个库暴露低层实时命令路径，但不会把任意命令自动变成物理安全命令。真机测试前：

- 配置碰撞阈值和负载参数，
- 使用保守的速度、加速度和力矩限制，
- 先在仿真器中验证控制器，
- 保持急停可达，
- 避免在高负载非实时系统上做高频控制实验。

## 当前状态

`franka_rust` 仍是实验性真机驱动。现阶段更重视 API 与 `robot_behavior`
抽象的契合度，而不是保留旧接口名称；在驱动栈稳定前，小版本仍可能调整公开接口。
