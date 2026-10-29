# franka-rust 0.5.0 测试记录与修改清单

测试日期：2026-10-29，Asia/Shanghai。测试人员：Zihan-Yin；记录：Codex。结果截止本日20:54完成的复测。

## 1. 当前结论与测试对象

**当前修复组合的26个例程均已有功能通过记录：25项连接真机验证，1项纯离线验证；当前未解决的例程运行失败为0。** `08_00`按功能分步完成，未连续重放原完整可执行文件；`07_01`本身为离线例程。通过表示下表所列流程和观察完成，不表示全部API、精度、负载或长期稳定性均已验收。

测试对象是当日下午拉取的三个远端默认`main`固定提交，加本次本地修复。以下历史问题、修改和过程均仅属于这个组合。

| 组成 | 版本 / 固定上游提交 |
| --- | --- |
| `franka_rust` | `0.5.0`；`912fb63755ce8f63c2291d0c814cb70eb0486326` |
| `roplat_behavior`及配套control / kinematics / trajectory | `0.9.0`；`faeebc0e132688264f3162ffc7f87b30d29a340c` |
| `roplat`及配套core包 | `0.6.0-rc.1`；`876564b324dac787684a9324971686c4a59e4961` |
| COPP | 实际解析`copp 0.2.2` |
| 工作区 | `/home/peppa/projects/Robot_Exp_Platform/franka-validation-latest`；顶层Cargo patch指向上述配套本地源码 |
| 设备 | Franka Emika/Panda，`172.16.0.2`，原装夹爪，额外负载0；抓取使用用户提供约50mm木块 |
| 协议 / 固件口径 | 实机使用默认FCI v5协议，连接握手成功；未获取控制器完整系统/FCI软件版本。FCI v8仅离线验证 |
| 实际工具链 | `rustc 1.101.0-nightly (c1070d693 2026-09-28)`；配套仓推荐`nightly-2026-10-20`未在本轮另验 |
| 源码状态 | 本地未提交修复；三个业务仓Cargo manifest未改，`roplat`核心源码无修改；外部验证包单列 |

### 本次提交与复现说明

本文记录提交前的验证快照。提交包含功能修复、必要示例/数据、回归测试及本报告；现场IP覆盖留在本地，提交中的联网例程保留上游默认地址172.16.0.3，实际测试使用172.16.0.2。本文`validation/`路径及本机绝对路径是本地证据索引，原始日志、模型和验证工程不随提交发布；旧完整总报告也不纳入本仓docs提交。

本次驱动新增的实例约束入口需要配套`robot_behavior`仓库同名`codex/franka-rust-0.5.0-validation`分支的`roplat_trajectory`改动。复现时需通过Cargo patch将`roplat_trajectory`指向配套源码；该改动发布前，仅解析crates.io的同版本包不包含新增入口。业务manifest和版本号本次保持不变。

## 2. 26个测试点：结果、历史问题、解决情况

“历史问题”仅列本版本组合实际发现或观察到的情况；无记录不代表该API的所有输入均已测试。带记录复测使用同控制主体、同参数的外部观测程序，区别在于预分配状态记录与前后只读，没有改写原例程源码。

<!-- franka-050-results:start -->
| # | 例程 | 当前结果 | 本组合历史问题 / 观察 | 解决情况与验证范围 |
| --- | --- | --- | --- | --- |
| 1 | `00_00_lifecycle` | 真机通过：接口流程 | 未记录运行失败；`init`/`enable`本身返回Ok、无物理启用动作 | 原流程退出0；不把此项记为物理上电或FCI启用验收 |
| 2 | `00_01_recovery` | 真机通过 | `03_01`首次运行产生真实Reflex，需要恢复 | 用户明确授权后运行官方恢复例程，Reflex→Idle、当前错误空；保留历史速度不连续记录 |
| 3 | `01_00_read_state` | 真机通过 | 未记录运行失败 | 原只读流程正常结束；Idle时命令成功率为0不当作通信失败 |
| 4 | `01_01_echo_state` | 真机通过 | 未记录运行失败 | 原流程连续100帧输出并正常退出 |
| 5 | `02_00_move_joint_default` | 真机通过 | 未记录本组合运行失败 | 初测最大实际关节变化约0.216708rad，终点默认位最大误差约0.000482rad；时钟修复后亦用于`03_01`复测前归位并成功 |
| 6 | `02_01_move_joint_offset` | 真机通过 | 未记录本组合运行失败；原输出不便区分失败阶段 | 仅增加阶段标记；初始归位、相对偏移、返回默认位三个阶段均完成，现场正常静止 |
| 7 | `02_02_move_joint_trajectory` | 真机通过 | 未记录运行失败 | 原5秒关节轨迹完成，现场正常静止；其直接控制回调本来就累计dt，不走此次密集轨迹取点补丁 |
| 8 | `02_03_move_joint_trajectory_file` | 真机通过；补丁后复测无卡顿声 | 原文件仅3个路点，不符合直接逐毫秒播放的密集数据语义；修正数据后初测有卡顿声 | 保留3个路点，改515个1ms S曲线采样。时钟补丁后同文件复测：515ms，实测第四/六关节范围约0.03995/0.03999rad，现场正常往返、无卡顿声 |
| 9 | `02_04_move_joint_waypoints` | 真机通过；补丁后复测无卡顿声 | 整进程FIFO40下规划期间遭SIGKILL；普通调度成功后曾有卡顿声 | 离线复现CPU规划触及本机约200ms的RTTIME限制；沿用普通调度，不改主机限额或驱动。时钟补丁后同路点659ms往返，实测范围约0.09958/0.09967rad，现场无卡顿声 |
| 10 | `03_00_move_flange_offset` | 真机通过；补丁后复测无卡顿声 | 真机前发现共用法兰/IK适配问题并初修；初测±3cm往返有卡顿声 | 时钟补丁后同幅度复测；待用户明确看着夹爪后确认正常往返、无卡顿声。实测q离线重建峰值29.878mm、回位约0.099mm；单程约485ms |
| 11 | `03_01_move_flange_pose` | 真机通过：时钟修复后 | 初修后首次真机仍报关节速度/加速度不连续并ReflexAborted；离线几何轨迹未见超限 | 确认密集轨迹按回调数取点的问题并应用累计机器人dt取点补丁。从默认位附近按原目标/scale0.2完整复测退出0；实测位移103.116mm、终点位置误差0.176mm，现场正常无抖动/冲击。采用为本次有效修复；已有一次完整成功 |
| 12 | `04_00_control_joint_position` | 真机通过 | 未记录运行失败 | 原位置控制完整结束，现场动作正常静止 |
| 13 | `04_01_control_joint_velocity` | 真机通过 | 未记录运行失败 | 原正反速度控制完整结束，现场动作正常静止 |
| 14 | `04_02_control_torque` | 真机通过：基本控制 | 未记录运行失败 | 原2秒力矩指令保持完整结束，现场无异常、静止；未做力矩精度标定 |
| 15 | `04_03_control_arm_torque` | 真机通过：基本控制，保留观察 | 约10.5mm下降较快，随后原例程结束；用户确认无冲击、抖动或碰撞 | 运行正常退出0、结束Idle。约0.69秒进程内达到原“漂移超过1cm”结束范围，未跑满10秒；该控制没有位置保持/速度阻尼，不把本次记为10秒耐久或力精度验收，未另改原控制律 |
| 16 | `04_04_control_cartesian_pose` | 真机通过 | 未记录运行失败 | 原归位及完整10秒Cartesian pose往返完成，现场正常静止、无卡顿声；包含原程序自身恢复及碰撞设置调用 |
| 17 | `04_05_control_cartesian_velocity` | 真机通过 | 未记录运行失败 | 原完整8秒速度往返完成，现场正常静止、无卡顿声；前后法兰位置差约0.194mm |
| 18 | `05_00_controller_joint_impedance` | 真机通过 | 未记录运行失败 | 归默认位及原5秒关节阻抗保持完成，现场无抖动/冲击、静止 |
| 19 | `05_01_controller_cartesian_impedance` | 真机通过 | 真机前统一法兰状态与模型Jacobian的原点语义需对齐，已初修 | 原5秒当前位姿阻抗保持完成，现场正常静止；未施加外力做刚度/阻尼标定 |
| 20 | `05_02_controller_joint_traj_impedance` | 真机通过，保留终点残差 | 首次现场未注意小幅动作；终点未完全回默认位，最大约0.020rad误差 | 同参数观测复测1501帧，实测第四/六关节往返范围约4.244°/3.431°，用户确认无异常静止。原PD控制播完即结束，无终点收敛等待；记录残差，不判运行失败、不宣称精准回位 |
| 21 | `06_00_control_observation` | 真机通过，声音来源待辨 | 完整保持动作正常，但1秒诊断和3秒原例程均听到不明声音；用户要求暂不判异常 | 100ms、1秒诊断及原完整3秒均成功；3秒before/after各2966回调，平均命令成功率约96.7%。本例不走密集轨迹播放器，声音备注继续保留 |
| 22 | `07_00_model_live_frames` | 真机通过 | 未记录运行失败 | 原全部模型帧输出完成；本次新下载模型与保存模型SHA一致 |
| 23 | `07_01_model_offline_dynamics` | 离线通过 | 示例单位齐次矩阵漏第16元素1，重力写成-0.98 | 补`pose[15]=1`，重力改为-9.81；原离线例程退出0。使用保存官方模型，不计为真机动作测试 |
| 24 | `08_00_gripper` | 真机通过：分步功能覆盖 | 首次homing返回true但现场未见完整动作，不能直接据此判过；抓取最初缺测试物 | 先50mm定位并确认实动，再从已确认较窄位置homing，用户看到完整闭合/张开；50→70mm开合及stop通过。约50mm木块、60N抓取true，5秒后`is_grasped=true`/48.842mm，stop true；释放后69.905mm/false。原完整含homing流程未在物体放入后连续重播；未改夹爪生产代码 |
| 25 | `09_00_async_move_joint` | 真机通过 | Tokio runtime仅enable_io，使用定时器时panic | 原例程加一行`.enable_time()`后重建复测成功；本次目标在默认位附近，未据此验收大位移异步运动；此路径为关节轨迹PID，不是此次改过的异步Cartesian播放器 |
| 26 | `11_00_safety_envelope` | 真机通过：配置请求 | 原例程写默认行为、scale及碰撞阈值，不自动还原；没有运动/故障恢复步骤 | 原例程退出0并输出scale0.2；随后外部程序恢复已知SDK默认行为成功，最终Idle/当前错误空。验请求成功响应，未独立读回全部阈值或人为碰撞验证 |
<!-- franka-050-results:end -->

### 对“全部能跑通”的范围说明

25项真机与1项离线均有通过记录；部分例程在应用最后的时钟补丁前已通过。时钟补丁后真机重点复验了`03_01`、`02_03`、`02_04`、`03_00`，并重新执行`02_00`归位，**没有将其余例程声称为补丁后全部重跑**。`08_00`是分步覆盖，`09_00`仅默认位附近；上表保留声音、快速下降和终点残差等观察。

## 3. 相对固定上游的实际修改

### 3.1 驱动和配套轨迹库的功能修复

以下是当前树仍保留的改动，未提交/推送。离线反例和最终数值见附录A；本表不把所有改动都称为某个真机故障的已证唯一原因。

| 修改 | 涉及文件（相对各业务仓） | 内容与作用 | 当前验证 / 边界 |
| --- | --- | --- | --- |
| 密集轨迹执行时钟 | 驱动`src/robot.rs` | 新增私有`timed_joint_sample`；同步`MoveTraj<JointSpace<7>>`、原生异步`MoveTo<FlangeSpace>`累计机器人dt，从1ms密集轨迹按时刻取点，必要时插值，保留末点保持/完成语义 | 正常1ms与缺周期回环通过；`03_01`完整复测及三项有声例程复测通过。影响文件/路点/同步Cartesian；异步Cartesian本轮未额外真机验证 |
| Joint7 / 法兰适配 | 驱动`src/franka/kinematics.rs`（新增）、`src/franka.rs`、`src/franka/franka_emika.rs`、`franka_fr3.rs`、`src/robot.rs` | 明确DH末端Joint7到法兰+107mm；FK/Jacobian/IK共用适配；阻塞与原生异步Cartesian以q_d建立一致规划起点 | 零位移无首帧台阶、官方模型对齐；本轮实际机器人为Emika/Panda，未做FR3真机验证 |
| IK收敛判断 | 驱动`src/franka/kinematics.rs` | 按已有`CommonStop`位置/姿态残差判断IK收敛，非收敛返回已有算法错误 | 不可达目标离线拒绝；未引入更严的新残差阈值或重写公共IK算法 |
| Cartesian实例约束 | 驱动`src/robot.rs`；配套`roplat_trajectory/src/time_scaling.rs` | 增加显式`CartesianTimeScalingLimits`与兼容入口；将实例关节v/a/j、线性及旋转v/a/j接入直线Cartesian时间缩放；线性/旋转字段分开、补充有效bound getter | 原API保留，默认边界输出兼容；自定义值离线回归通过。COPP路点/路径及任意control_with的实例覆盖尚未全部接入 |
| 低通接入 | 驱动`src/utils/filters.rs`、`src/types/robot_command.rs` | 复用现有公式，固定100Hz；helper推广至N维；五类命令在原限幅前调用低通：关节位置/速度、Cartesian pose/velocity、力矩 | 五种过滤入口回归通过；没有另加一套滤波器或改变网络时钟 |
| 修正零和限幅旁路 | 驱动`src/utils/rate_limiting.rs` | 删除四处“各轴求和为0即返回”的错误早退，使零和位置、零速度停止等仍进入原限幅器 | 零和/减速离线回归通过；未替换原SDK限幅算法 |
| SDK速度裕量常量 | 驱动`src/utils/rate_limiting.rs` | `TOL_NUMBER_PACKETS_LOST`从0.001对齐协议5参考的3.0，用于已有速度边界的加速度裕量 | 这是边界计算常量，不是实测丢包计数，也不会在丢3帧时退出 |
| 统一状态与工具命令适配 | 驱动`src/types/robot_state.rs`、`src/robot.rs` | `flange`表示真实法兰、`tcp`保留EE、`stiffness`转到基座；速度/加速度/力矩按原点运输；统一Cartesian指令转换回原生FCI EE语义；不再把O_ddP_O当作末端期望加速度 | pose/velocity的同步、阻塞async、原生async六入口报文回归通过；原生状态及低层MotionType的FCI EE语义保留 |
| 电机角与命令状态分离 | 驱动`src/types/robot_state.rs` | `joint.cmd`不再填入电机theta/dtheta；保留原生电机字段，保持目标按既有fallback使用des.q | 合成电机/关节差不再变成保持目标偏移 |
| 模型输入修正 | 驱动`src/model.rs` | 从统一世界位姿还原相对F_T_EE/EE_T_K后求Jacobian；默认重力使用已有[0,0,-9.81]而非质心坐标；Coriolis使用总惯量而非仅负载惯量 | 保存官方模型的离线对照通过；显式重力输入仍有效，未做实机模型精度标定 |

`roplat_behavior`生产代码实际只修改`roplat_trajectory/src/time_scaling.rs`；其余配套包保持固定上游代码。`roplat`核心生产源码无修改。

### 3.2 例程、数据、配置及文档修改

| 类别 | 文件 / 范围 | 实际修改 | 是否控制算法修复 |
| --- | --- | --- | --- |
| 异步示例runtime | `examples/09_00_async_move_joint.rs` | 增加`.enable_time()`，避免缺定时器panic | 例程运行环境修正 |
| 文件轨迹数据 | `examples/safe_joint_trajectory.json` | 3个路点改为515个1ms S曲线样本，原3个路点不变；物理v/a/j最大比例约0.1442/0.2/0.2 | 数据语义修正；JSON大量新增行主要来自密集数组，不是大量保护逻辑 |
| 离线模型输入 | `examples/07_01_model_offline_dynamics.rs` | 补单位齐次矩阵第16元素1；重力-0.98→-9.81 | 示例数值修正 |
| 返回与阶段可见性 | `examples/02_01_move_joint_offset.rs`、`08_00_gripper.rs` | 02_01打印三个运动阶段；08打印homing/move/grasp/stop实际bool | 仅输出；不改变运动幅度、速度或夹持力 |
| 现场IP | 25个联网例程 | 上游默认172.16.0.3改为本机测试设备172.16.0.2 | 本地部署参数，宜与功能修复分开审阅 |
| 使用语义说明 | `README.md`、`README_zh.md` | 补充统一帧/原生EE语义、命令过滤顺序与实例规划边界说明 | 文档 |
| 调度设置 | 外部单项runner | CPU10；02_04采用普通调度解决RTTIME进程终止；按项目记录实际调度策略 | 测试环境，不是上游驱动补丁；未提高系统限额 |

### 3.3 回归测试与外部验证设施

驱动增加`src/tests/initial_fix.rs`、保存模型参考数据`franka_default_model.json`、六入口回环`src/realtime/tests/initial_fix_frames.rs`、执行时钟回环`trajectory_clock.rs`，并在`src/lib.rs`、`src/realtime/tests.rs`登记。夹具使用localhost收发，不连接机器人。

外部`validation/code-review-probe`和单项runner、状态记录、离线模型回放均位于验证工作区；观测采用预分配内存，控制结束后才写日志。外部包增加直接指向已存在`roplat_control`的path依赖，顶层Cargo.lock登记发生变化，未据此升级业务依赖版本。本次整理时锁文件SHA-256为`a2a1800c271d40bf391e2b0c396132491884280341b384b353cc33fc89d1fb69`。

当前修改分布可按四组审阅：驱动/轨迹库功能修复；例程runtime和数据修正；IP/输出/文档；测试设施。初修补丁和后续独立补丁均保存在验证目录，见附录C，不能把早期补丁单独当作当前全部差异。

### 3.4 本轮未加入的修改

驱动`src/network.rs`、`src/robot_impl.rs`、`src/realtime/mod.rs`、`src/realtime/std_udp.rs`、`src/realtime/tokio_udp.rs`与固定上游HEAD逐字节一致；夹爪生产实现也未修改。没有增加“状态周期超过2ms”“成功率低于某值”或额外丢帧/超时的退出条件，没有改写控制器保护、生产网络超时或结束握手。实际3–4ms周期按记录保留，真实FCI错误则单独记录。

原例程自身的碰撞设置保持原样执行，包括04_03/05_01的100阈值；不是为了通过测试临时放宽失败项目的阈值。11_00之后另行恢复已知SDK默认行为，按响应成功记录，没有宣称全部参数已独立读回。

## 4. 例程通过后仍需作者核对的事项

这些事项没有阻止本轮例程流程通过，未以大规模防御性改动扩项处理。

| 事项 | 当前事实 / 待处理范围 |
| --- | --- |
| 两个力矩限制setter | `with_torque`、`with_torque_dot`仍是上游空操作，不能把调用成功当作自定义限值生效 |
| 实例约束覆盖边界 | 本轮接入Cartesian直线规划；COPP的move_path/move_waypoints仍主要消费scale，任意control_with仍使用SDK既有常量限幅 |
| 另一套采样目标时钟 | 依赖库`joint_traj_target_fn`仍按回调推进，影响05_02及09_00使用的轨迹阻抗/PID；本次robot.rs补丁未覆盖它，不把这些接口自动算作同一时钟问题已修复 |
| 05_02终点跟踪 | 约0.020rad残差保留；PD轨迹播完即done，没有终点收敛等待，尚未扩改 |
| 生命周期与模型缓存 | init/enable无物理动作；软件emergency_stop委托stop；model()使用公共`/tmp/model.so`缓存，未增加控制器身份校验 |
| 齐次矩阵与jerk覆盖 | N09的NaN/镜像判断缺口未修；规划默认validate_jerk=false未改，不宣称所有曲线的密集jerk已独立验收 |
| 现场备注 | 06_00不明声音来源待辨；04_03下降较快、05_02终点残差保留。02_03/02_04/03_00的卡顿声在本轮补丁后复测未再出现 |

## 附录A：本组合真机前的初步修复与离线依据

以下均为本版上游反例与修复后对照；数值来自本地算法/保存的官方模型及合成输入，不是向真机发送了这些错误目标。

| 检查点 | 修复前问题 / 反例 | 初修后结果 |
| --- | --- | --- |
| 零和限幅 | 有效零和目标可直接跨约1rad；零速度请求绕过减速 | 删除四处旁路，恢复原限幅计算 |
| 相对零位移 | Joint7/Flange混用时首帧最大关节差0.290928rad、法兰差107.022mm | 首帧差及法兰位置差均归零；实机零位移检查通过 |
| 不可达目标 | 10m目标IK未收敛，仍接受约239984点轨迹 | 按已有收敛标准返回错误 |
| 自定义规划约束 | 仅使用型号常量×scale，实例覆盖未进入Cartesian规划 | 显式约束入口、实例关节/线性/旋转约束回归通过，原入口兼容 |
| 命令低通 | 1ms/100Hz下1e-6阶跃仍原样进入，未调用已有低通 | 实际过滤输出约3.85869545e-7，再进入原限幅 |
| 模型/统一帧 | 工具偏移时Jacobian最大元素差约0.390282；电机/关节合成差20mrad进入hold；默认重力误用质心时力矩差约22.014643Nm | 对应差异分别降至约2.22e-16、0rad、0Nm；原点/状态/模型回归通过 |

初修完成后两协议各53库+2类型测试通过、1性能忽略，26例程双协议release构建通过；保存官方模型完成18个离线场景（包含“未修N09仍存在”的确认，不能理解为18个缺陷全修好）。后续加两项时钟测试后的最终结果见附录C。

## 附录B：0.5.0测试过程摘要

| 北京时间 / 阶段 | 完成情况与问题处理 |
| --- | --- |
| 下午基线与准备 | 拉取固定三仓main并隔离工作区；按本版重新复现问题，实施初修与离线回归；现场IP最终为172.16.0.2 |
| 19:12–19:15，A/B | 连接、只读、模型及100ms/1s/原3s保持完成；记录3s约96.7%命令成功率及不明声音 |
| 19:20–19:28，C | 8个关节/异步例程完成；处理02_04的FIFO/RTTIME问题、09定时器；修正密集文件数据；文件/路点声音留待复测 |
| 19:35–19:46，D与恢复 | 零位移、03_00通过；03_01真实不连续保护；授权后恢复并核对；04_04/04_05完整通过且现场无卡顿声 |
| 19:49–19:58，E | 分步归零、定位、开合、stop、木块抓取/5秒状态及释放完成；首次未见homing动作的反馈保留，不靠true直接判实动 |
| 20:00–20:12，离线分析 | 无设备请求；确认按回调数取点的时钟问题，以真实public MoveTraj回环复现；候选验证后暂存为补丁，等用户决定 |
| 20:23–20:29，F | 5个力矩/阻抗原例程退出0；05_02同参数记录复测证明实际往返；保留快速下降及终点残差 |
| 20:34，G | 11_00配置例程通过，另行恢复SDK默认行为，最终Idle/当前错误空 |
| 20:39–20:43，03_01修复复测 | 正式应用时钟补丁，离线回归后从默认位执行原完整路径通过；用户确认正常；双协议回归与默认26例程重建完成 |
| 20:50–20:54，声音复测 | 02_03、02_04、03_00同参数带记录复测均正常且现场无卡顿声；03_00前两次未看清，明确目视就绪后第三次确认 |

时钟问题的关键回环证据：状态id由1到4，实际间隔3ms时，原代码只取下一个样本；修复后取相应第3个样本。正常1ms序列保持兼容。修复后03_01成功、共享入口三项声音未再出现，支持将补丁作为当前有效解决方案；没有通过旧/新随机A/B证明声音的唯一因果，也没有把一次完整成功等同长期稳定性验证。

本轮最终机械臂状态为Idle、当前错误空；历史首次03_01速度不连续仍保留。最后无测试进程或设备1337/1338已建立TCP连接，测试已暂停。

## 附录C：最终回归与可追溯证据

- 最终执行时钟补丁：默认FCI v5与fci_v8各55库+2类型=57项通过、1项性能测试忽略；仅离线/localhost回环，v8未连接真机。
- 默认FCI v5的26个release例程在最后生产修复后全部重新构建通过；三项声音复测未再改生产源码。
- 现场证据包括命令、源与二进制SHA、前后原生状态、部分逐周期实测q/q_d/dq/dq_d/ddq_d与成功率、用户反馈。原03_01首次失败没有完整发送/首次错误帧trace，这一缺口仍保留。
- 独立复核未完成；本记录按源码差异、离线数值、实际命令返回、保存状态及现场反馈交叉自检。所有修改未暂存、提交或推送。

以工作区`franka-validation-latest/validation/`为证据根目录：

| 内容 | 文件 / 目录 |
| --- | --- |
| 固定基线与解析来源 | [baseline.json](/home/peppa/projects/Robot_Exp_Platform/franka-validation-latest/validation/baseline.json)、顶层Cargo.lock |
| 初修范围与回归 | [initial-fix/source-and-patches.json](/home/peppa/projects/Robot_Exp_Platform/franka-validation-latest/validation/initial-fix/source-and-patches.json)、同目录final-verification.json / probe-results.json |
| A/B、C、D及D后段 | 20261029-zihan-ab/ab-evidence.json；20261029-zihan-c/c-evidence.json；20261029-zihan-d/d-evidence.json；20261029-zihan-d-remaining/d-remaining-evidence.json |
| 夹爪分步 | 20261029-zihan-e/e-evidence.json；20261029-zihan-e-grasp/grasp-evidence.json |
| 力矩/阻抗与配置 | 20261029-zihan-f/f-evidence.json；20261029-zihan-g/g-evidence.json |
| 时钟修复及03_01实测 | [clock-retest-evidence.json](/home/peppa/projects/Robot_Exp_Platform/franka-validation-latest/validation/20261029-zihan-clock-retest/clock-retest-evidence.json)；同目录双协议测试日志、all-current-example-binaries.json |
| 三项声音复测与离线位移重建 | [sound-retest-evidence.json](/home/peppa/projects/Robot_Exp_Platform/franka-validation-latest/validation/20261029-zihan-sound-retest/sound-retest-evidence.json)；同目录trajectory-statistics.json及*.offline-path.json |
| 初修独立补丁 | initial-fix/franka-control-fixes.patch；trajectory-instance-limits.patch；example-local-config.patch |
| 后续独立补丁 | 20261029-zihan-c/async-example-fix.patch、dense-file-fix.patch；[candidate-elapsed-trajectory-clock.patch](/home/peppa/projects/Robot_Exp_Platform/franka-validation-latest/validation/20261029-zihan-0301-analysis/candidate-elapsed-trajectory-clock.patch)（已应用并保留） |

本文件为0.5.0专题整理稿。原完整总报告保留原路径且未覆盖、删除；本稿以原报告新版记录、当前源码差异与上述原始证据重新汇总。

维护：2026-10-29｜provisional:linux-peppa-Lenovo-Legion-Y7000P-1060｜Zihan-Yin｜Codex｜session-codex-zihan-franka-01；整理记录C39。
