# franka_rust

[English](README.md) | [简体中文](README_zh.md)

`franka_rust` 通过 FCI 连接 Rust 应用与 Franka 控制器，提供状态读取、关节与笛卡尔运动、控制会话、夹爪和模型加载，并接入统一的 [`robot_behavior`](https://github.com/Robot-Exp-Platform/robot_behavior) 接口。它适合在不同 Robot-Exp 驱动间复用应用层运动和控制代码，同时保留 Franka 专有设置的场景。

已发布版本为 **`franka_rust 0.2.0`**。这是独立实现，并非官方 C++ `libfranka` 软件包。`FrankaEmika`/`FrankaPanda`、`FrankaFR3`、`FrankaFP3` 等机型别名提供 API 类型，不代表已经匹配控制器固件。

## 设计与流程

机器人对象持有连接。`Robot`、`MoveTo<JointSpace<7>>`、`ControlWith<TorqueControl<7>>` 等行为 trait 描述应用操作，本驱动实现 FCI 的 TCP 指令与 UDP 控制协议，使通用控制器不必包含传输细节。

典型顺序是 **连接 → 读取状态 → 配置设备 → 选择目标运动或控制会话**。状态读取不需要运动或 `set_default_behavior()`。配置、模型加载和轨迹规划与每周期的控制回调分开。

控制会话在结束前借用机器人和控制器，因此回调能借用本地轨迹或日志。阻塞会话使用直接 socket 循环；原生异步会话在调用方的 Tokio runtime 上等待设备协议。旧的异步回调方法有不同契约，见后文。

## 安装与协议选择

使用 Rust nightly：当前驱动与行为库使用不稳定 Rust 特性。先运行 `cargo new franka-read-state` 创建二进制项目，再加入：

```toml
[dependencies]
franka_rust = "0.2.0"
robot_behavior = "0.6.1"
```

直接依赖行为库可导入 trait，并选择已发布的 0.6.1 修复。registry 安装不需要父级 `drives` 工作区，也不要求安装官方 C++ SDK。

设备必须可达并已启用 FCI。默认选择 **FCI 协议 5**（libfranka 0.9.2 布局）；`features = ["fci_v8"]` 选择 **协议 8**（libfranka 0.14.0 布局）。应依据控制器兼容性信息选择，而不是 Rust 机型别名。见[协议常量](src/params.rs)与[响应类型](src/types/robot_types.rs)。

模型加载另行处理：`model()` 使用 Linux/Windows 路径下载控制器提供的模型库，其他系统返回平台不支持错误。主机编译通过不等于已验证 FCI 时序或模型库兼容性。

| Feature | 用途 |
|---|---|
| 默认，无 feature | Rust 驱动、阻塞与原生异步 API。 |
| `fci_v8` | FCI 协议 8。 |
| `to_py` | PyO3 绑定，需要匹配的 Python 开发环境。 |
| `to_cxx` | CXX 绑定，需要 C++ 工具链。 |
| `debug` | 额外诊断。 |

绑定 feature 不是完整的 Python/C++ 部署包。Roplat 集成应在 **`robot_behavior`** 上开启 `roplat` feature。

## 第一个程序：读取一帧状态

将代码放入 `src/main.rs`，决定实际运行前替换地址。程序只连接并接收状态，不启用运动，也不修改碰撞或阻抗设置。

```rust,no_run
use franka_rust::FrankaEmika;
use robot_behavior::{Robot, RobotResult};

fn main() -> RobotResult<()> {
    let mut robot = FrankaEmika::new("172.16.0.2");
    let state = robot.read_state()?;
    println!("joint position: {:?}", state.q);
    println!("joint velocity: {:?}", state.dq);
    println!("robot mode: {:?}", state.robot_mode);
    Ok(())
}
```

只检查编译，不连接设备：

```sh
cargo +nightly check
```

`new()` 返回 `Self`，连接或协议协商失败时可能 panic。`read_state()` 返回 `RobotResult` 并等待控制器数据包。当前构造器不是返回 `Result` 的连接入口。

## 从读取状态到控制

实际设备的工作模式、负载、坐标系和限制明确之后，可从[关节运动](examples/02_00_move_joint_default.rs)或[笛卡尔运动](examples/03_01_move_flange_pose.rs)开始。这些程序会发送运动指令。`set_default_behavior()` 写入碰撞与阻抗默认值，不只是连接初始化。

| 控制器入口 | 执行契约 |
|---|---|
| `control_with` / `control_with_flow` | 阻塞整次会话。 |
| `control_with_async` / `control_with_flow_async` | 整次会话仍阻塞，只有每周期回调为异步；旧包装器创建本地 Tokio runtime。 |
| `AsyncControlWith::control_native_async` | 返回覆盖会话进入、周期和结束的 Future，在启用 I/O 与时间的 Tokio runtime 上等待。 |

不要在已经进入的 Tokio runtime 内调用旧包装器。原生控制覆盖六种关节、笛卡尔与力矩空间，见[实现](src/robot.rs)与[异步运动示例](examples/09_00_async_move_joint.rs)。

Flow 回调返回 `ControlFlow<(), (Command, bool)>`：`Continue((command, false))` 发送并继续；`Continue((command, true))` 发送最后一条指令；`Break(())` 不发送本周期算法指令，结束设备会话。清理包含 `StopMove`、空闲状态接收和终态 Move 响应，共享三秒网络期限，不保证物理停止时间。直接丢弃原生 Future 不会执行异步清理；未确认结束的会话需要重新连接后才能再次运动。

使用 Roplat 图时开启 `robot_behavior/roplat`，原生接口配合 `AsyncControlRhythm`，阻塞接口配合 `ControlRhythm`。见[行为库指南](https://github.com/Robot-Exp-Platform/robot_behavior#readme)。

## 下一步与限制

- [示例目录](examples)：[状态](examples/01_00_read_state.rs)、[控制观测](examples/06_00_control_observation.rs)、[模型查询](examples/07_00_model_live_frames.rs)与[夹爪](examples/08_00_gripper.rs)。
- [机器人实现](src/robot.rs)、[模型模块](src/model.rs)与[已发布 API](https://docs.rs/franka_rust/0.2.0/franka_rust/)。
- [Loopback 测试](src/realtime/tests.rs)不连接机器人，验证网络会话。忽略执行的 `native_loopback_performance` 比较本机会话路径，不是真机时序测量。

本库仍是实验性驱动。API 存在不代表固件覆盖、实时性能或物理停止行为已经验证。具体主机、控制器组合及模型库都需要各自验证。

## 引用

如果您在研究中使用了本项目，请使用以下 BibTeX 条目引用：

```bibtex
@misc{Jizhou2025FrankaRust,
  author = {Yan, Jizhou},
  title = {Franka-{R}ust: An instantiation interface for {Franka} in the general robot behavior library},
  year = {2025},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/Robot-Exp-Platform/franka-rust}}
}
```

## 源码、资源与许可

checkout manifest 将开发依赖固定到 Git revision，需要访问相应源码；registry 安装使用已发布依赖。设置 `ROPLAT_SKIP_ASSET_EXPORT=1` 可跳过构建时向用户数据目录复制示例资源，适用于检查与 CI。

驱动由 Robot-Exp-Platform 维护，使用 [Apache-2.0](LICENSE)。Franka 是设备厂商名称。Panda 模型资源保留[自己的声明](assets/franka_panda/LICENSE.txt)；控制器固件和下载的模型库是厂商组件，适用各自条款。
