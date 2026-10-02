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

也就是说，对外仍然只有一个机器人对象；原生异步能力通过独立特征接入。

## 原生异步控制会话

新增 `AsyncControlWith<S>::control_native_async(&mut callback)`，返回代表完整设备
会话的 `Send` Future。六类通道均可使用：关节位置、关节速度、笛卡尔位姿、
笛卡尔速度、关节状态力矩、完整机械臂状态力矩。

请在启用 I/O 和 time 的 Tokio runtime 中 await。启动、周期 UDP、正常结束和
StopMove 收尾均使用异步网络等待；机器人与回调在整个会话中被借用，不创建嵌套
runtime、后台线程、额外任务或每周期装箱 Future。旧异步回调入口与新入口共享
同一个会话状态机。普通同步内核仍直接使用 std socket。

旧 `control_with_async` / `control_with_flow_async` 继续是**阻塞会话 + async
周期回调**，内部创建 runtime，不能在已进入的 Tokio runtime 中调用。同步
`move_to` 不改变；已有 `move_to_async` 也使用新的非阻塞会话协议。连接、配置和
轨迹规划仍是独立的同步操作。

回调通过 `AsyncControlCallback<Obs, Command>` 静态分派，允许在 await 期间借用
控制器自身状态。一般 `FnMut(Obs, Duration) -> Fut` 闭包在 Fut 为 Send 时可以
直接使用；需要跨 await 可变借用自身状态时，实现一个命名控制器结构体。
`Continue((command, false))` 继续；`Continue((command, true))` 发送最终指令后
正常收尾；`Break(())` 不发送本周期算法指令，而执行设备 StopMove 协议。

在 `#[roplat::system]` 中使用 **AsyncControlRhythm**，让一个周期域中的节点和
子节律可以真正 await，设备收尾完成后再返回。旧 ControlRhythm 仍使用阻塞接口。
两者都需要显式启用 `robot_behavior/roplat`。

异步会话独占借用原有 TCP/UDP 连接，退出或 Drop 时恢复 blocking 模式，允许同步
与原生异步会话交替执行。直接丢弃 Future **不会**异步收尾；启动结果不确定或设备
会话未结束时，后续会话要求重新连接。TCP 帧读到一半被丢弃也会使连接失效。
需要完整收尾和资源归还时应合作返回 Break。取消过程共用三秒网络等待上限，
异步写入保留 TCP write timeout（目前三毫秒）；它们不代表物理停止时间保证。
普通启动与正常终态读取没有增加超时策略。

有限 loopback 测试包含真实原生入口、单线程 Tokio 同层任务进展、协议错误、
丢弃 Future、socket 恢复和连续同步/异步切换。测试不接触真机，真机调度与控制
时序仍需另行验收。

## 与其他库的关系

- `robot_behavior` 提供通用行为特征、状态模型和控制器工具。
- 开启 `robot_behavior/roplat` 后可使用原生 `AsyncControlRhythm` 与旧
  `ControlRhythm`。异步 System 图应使用前者。
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

## 控制会话的提前退出

`control_with_flow` / `control_with_flow_async` 的回调返回
`ControlFlow<(), (Command, bool)>`，两个入口都会阻塞到会话结束；`async`
描述每周期回调。

- `Continue((command, false))`：发送有效命令并继续。
- `Continue((command, true))`：发送最后一条命令，完成正常 FCI 结束握手。
- `Break(())`：本周期不发送算法命令；通过 TCP `StopMove` 结束会话，等设备
  退出运行模式并消费 Move 终结响应。此处不会伪造零命令，也不等同于急停保证。

原 tuple 回调的 `control_with` / `control_with_async` 包装同一条循环。执行与
收尾同时失败时，`RobotException::ControlSession` 保留两个错误。局部 runtime、
UDP 周期和阻塞所有权语义保持原有设计。

`cargo test -p franka_rust --lib realtime::tests` 在临时 loopback TCP/UDP 端口
验证真实生产循环，不连接硬件。测试通过不能替代真机停机行为和时序验收。

取消或失败收尾的 StopMove 应答、idle 状态和 Move 终结响应共用 **3 秒总网络等待
期限**，这不是物理安全停止时间保证。超时返回错误；会话结束尚未确认时，新的
控制会话会明确拒绝并要求重新连接，防止把旧响应错当成新会话结束。收尾后恢复
socket 原有读超时与非阻塞设置，正常周期 I/O 不变。可配置期限留作后续工作。

### 执行上下文限制

Franka 的 async 回调入口内部创建本地 Tokio runtime 并调用 `block_on`。从已进入的 Tokio runtime 直接调用可能因嵌套 runtime 而 panic，因此当前共享 ControlRhythm 尚不能作为任意 Tokio System 上下文中的即插即用真机入口。这是已有边界，后续需单独设计；本轮 mock System 通过不代表验证了该真机执行上下文。

## Offline Loopback Measurement

```sh
ROPLAT_SKIP_ASSET_EXPORT=1 cargo test -p franka_rust --release --lib native_loopback_performance -- --ignored --nocapture --test-threads=1
```

The ignored, finite test compares public std, blocking async-callback and native
async sessions. It rotates their order and reports whole-session time separately
from peer-observed cycle RTT p50/p99 for one and 1000 cycles. Native runs reuse
the caller runtime; the legacy wrapper constructs its runtime per session. The
fixed FCI packet sizes are printed. Loopback RTT includes host scheduling and
codec/filter work; it is not one-way communication latency or a hardware result.
Set `FRANKA_PERF_ROUNDS` and `FRANKA_PERF_CYCLES` to repeat it. Run it alone.
