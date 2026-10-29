# Change log (更新日志)

## Unreleased

- Breaking: 迁移到 roplat_behavior 0.10。运动约束合为一个 `MotionLimits`，由 `Arm::motion_limits` / `set_motion_limits` 读写，只用于一次运动时用 `Motion::with_limits`；默认取声明范围的 10 %。移除 `set_scale`、`get_scale`、`with_scale`、各 `with_*` 约束，以及 `Coord`、`set_coord`、`with_coord`；相对运动改用 `Relative<JointSpace<7>>` 与 `Relative<FlangeSpace>`（偏移在法兰坐标系中）。`CONTROL_PERIOD` 为 `Duration`。
- Refactor: 法兰偏置由 roplat_kinematics 的 `FLANGE` 表达，正运动学、雅可比与 IK 不再经驱动自带的适配；IK 不收敛时仍返回错误。笛卡尔直线运动调用 `plan_cartesian_line_traj_via_time_scaling`，取代上文的 `plan_cartesian_path_traj_via_time_scaling_with_limits`。
- Refactor: 密集关节轨迹由 roplat_trajectory 的 `JointTrajectoryFollower::cycle_timed` 执行，仍按累计的机器人时间取点；最后一个采样点即给出 done，不再多保持一个周期。
- Fix: `model()` 把模型库缓存在系统临时目录（`FrankaModel::cache_path`），不再写入固定的 `/tmp`、`C:\tmp`。
- Chore: Python 绑定改用 roplat_behavior_ffi；移除没有实现的 `franka_rust.hpp`、公开的 `once` 模块与未使用的 crossbeam 依赖。
- Fix: 统一 `ArmState` 的坐标系：`flange` 为真实法兰，`tcp` 为配置的 EE/TCP，`stiffness` 为刚度坐标系，空间向量使用基座轴。`get_endpoint` 改为返回法兰位姿（此前返回 EE）。原生 `RobotState` 与 `MotionType::Cartesian` 保留 FCI 的 EE 语义，统一的笛卡尔控制入口负责法兰到 EE 的转换。
- Fix: DH 链末端为 Joint7，法兰在其后 107 mm；正运动学、雅可比与 IK 共用这一适配。笛卡尔 IK 不收敛时返回错误。
- Fix: 限幅器不再在各轴之和为 0 时跳过；`TOL_NUMBER_PACKETS_LOST` 改为 3.0，与 FCI 协议 5 的参考实现一致。
- Feat: 五种命令在限幅之前先经 100 Hz 低通，顺序与 libfranka 0.9.2 一致。
- Fix: 密集关节轨迹按累计的机器人时间取点，丢包后不再滞后。涉及 `MoveTraj<JointSpace<7>>`，以及阻塞与原生异步的 `MoveTo<FlangeSpace>`。
- Feat: 笛卡尔直线运动使用实例设置的关节、线性与旋转约束，需要 roplat_trajectory 的 `plan_cartesian_path_traj_via_time_scaling_with_limits`。`with_cartesian_jerk`、`with_rotation_*` 不再是空操作，也不再写错字段。
- Fix: 模型输入：雅可比由统一位姿还原 F_T_EE、EE_T_K；默认重力为 [0, 0, -9.81]；科里奥利项使用总惯量。`joint.cmd` 不再填入电机角。
- Docs: 0.5.0 真机验证记录见 docs/franka-rust-0.5.0-validation-2026-10-29.md。

## v0.1.5 (2025-04-06)

- Fix: 修复了 linux 下对于 model 的重力和科里奥利力的问题

- Error: Windows 下 udp 发送的指令无法被接收到，在 20 帧之后会导致 `CommunicationConstraintsViolation` 错误

## v0.1.3 (2025-03-29)

- Feat: model 的下载和加载功能（模型的下载会导致至少 4ms 的延迟）在执行运动中下载可能会导致问题。

## v0.1.0 (2025-03-26)

- Initial release (初始版本)
