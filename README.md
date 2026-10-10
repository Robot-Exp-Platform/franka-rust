# franka_rust

[English](README.md) | [简体中文](README_zh.md)

`franka_rust` connects Rust applications to Franka controllers through FCI. It provides state reading, joint and Cartesian motion, controller sessions, grippers, and model loading through the common [`robot_behavior`](https://github.com/Robot-Exp-Platform/robot_behavior) interfaces. Use it to share application-level motion and control code across Robot-Exp drivers while keeping Franka-specific settings accessible.

The published crate is **`franka_rust 0.2.0`**. This is an independent implementation, not the official C++ `libfranka` package. Model aliases such as `FrankaEmika`/`FrankaPanda`, `FrankaFR3`, and `FrankaFP3` provide API types; an alias does not establish compatibility with a controller firmware.

## Design and workflow

A robot object owns its connection. Behavior traits such as `Robot`, `MoveTo<JointSpace<7>>`, and `ControlWith<TorqueControl<7>>` describe the application operation; this driver implements the FCI TCP command and UDP control protocols. This separation keeps transport details out of reusable controllers.

The usual sequence is **connect → read state → configure the device → choose target motion or a control session**. State reading does not require motion or `set_default_behavior()`. Configuration, model loading, and trajectory planning remain separate from per-cycle controller callbacks.

A control session borrows the robot and controller until it ends, allowing callbacks to borrow local trajectories or loggers. Blocking sessions use a direct socket loop. Native asynchronous sessions await the device protocol on the caller's Tokio runtime; the older async-callback methods have a different contract, explained below.

## Install and select the protocol

Use Rust nightly: the current driver and behavior library use unstable Rust features. Create a binary project with `cargo new franka-read-state`, then add:

```toml
[dependencies]
franka_rust = "0.2.0"
robot_behavior = "0.6.1"
```

The direct behavior dependency supplies the imported traits and selects the published 0.6.1 fixes. Registry installation does not need the parent `drives` workspace or an official C++ SDK installation.

The controller must be reachable with FCI enabled. The default selects **FCI protocol 5** (libfranka 0.9.2 layout); `features = ["fci_v8"]` selects **protocol 8** (libfranka 0.14.0 layout). Choose using the controller's compatibility information, not its Rust model alias. See [protocol constants](src/params.rs) and [response types](src/types/robot_types.rs).

Model loading is separate: `model()` downloads a controller-provided library using Linux/Windows paths; other systems return an unsupported-platform error. Compilation on a host does not establish FCI timing or model-library compatibility.

| Feature | Purpose |
|---|---|
| Default, no features | Rust driver, blocking and native async APIs. |
| `fci_v8` | FCI protocol 8. |
| `to_py` | PyO3 bindings; requires a compatible Python development environment. |
| `to_cxx` | CXX bindings; requires a C++ toolchain. |
| `debug` | Additional diagnostics. |

Binding features are not complete Python/C++ deployment packages. Roplat integration is enabled on **`robot_behavior`** with its `roplat` feature.

## First program: read one state

Put this in `src/main.rs`. Replace the address before deliberately running it. It connects and receives state without enabling motion or changing collision/impedance settings.

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

Check without connecting to hardware:

```sh
cargo +nightly check
```

`new()` returns `Self` and can panic on connection or protocol-negotiation failure. `read_state()` returns `RobotResult` and waits for a controller packet. The constructor is not a fallible connection API.

## From state reading to control

Start with [joint motion](examples/02_00_move_joint_default.rs) or [Cartesian motion](examples/03_01_move_flange_pose.rs) when the controller's operating mode, load, frames, and limits are configured for your setup. Those programs send motion commands. `set_default_behavior()` writes collision and impedance defaults; it is not just connection initialization.

| Controller entry point | Execution contract |
|---|---|
| `control_with` / `control_with_flow` | Blocks for the whole session. |
| `control_with_async` / `control_with_flow_async` | Still blocks; only the per-cycle callback is async. The legacy wrapper creates a local Tokio runtime. |
| `AsyncControlWith::control_native_async` | Returns a future covering session entry, cycles, and termination; await it in a Tokio runtime with I/O and time enabled. |

Do not invoke the legacy wrapper inside an entered Tokio runtime. Native control covers six joint, Cartesian, and torque spaces; see [the implementations](src/robot.rs) and [async motion example](examples/09_00_async_move_joint.rs).

Flow callbacks return `ControlFlow<(), (Command, bool)>`: `Continue((command, false))` sends and continues; `Continue((command, true))` sends a final command; `Break(())` sends no algorithm command that cycle and terminates the device session. Cleanup uses `StopMove`, idle-state reception, and the terminal Move response with a shared three-second network deadline. This does not guarantee a physical stopping time. Dropping a native future does not perform async cleanup; an uncertain or unfinished session requires reconnection before another motion.

For Roplat graphs, use `AsyncControlRhythm` with the native interface or `ControlRhythm` with the blocking interface, after enabling `robot_behavior/roplat`. See the [behavior guide](https://github.com/Robot-Exp-Platform/robot_behavior#readme).

## Next steps and limits

- [Examples](examples): [state](examples/01_00_read_state.rs), [control observations](examples/06_00_control_observation.rs), [model queries](examples/07_00_model_live_frames.rs), and [gripper](examples/08_00_gripper.rs).
- [Robot implementation](src/robot.rs), [model module](src/model.rs), and [published API](https://docs.rs/franka_rust/0.2.0/franka_rust/).
- [Loopback tests](src/realtime/tests.rs) exercise network sessions without a robot. The ignored `native_loopback_performance` test compares local session paths; its results are not hardware timing measurements.

This remains an experimental driver. API availability does not prove firmware coverage, real-time performance, or physical stop behavior. Each host/controller combination and model library requires its own validation.

## Citation

If you use this project in your research, please cite it using the following BibTeX entry:

```bibtex
@misc{Jizhou2025FrankaRust,
  author = {Yan, Jizhou},
  title = {Franka-{R}ust: An instantiation interface for {Franka} in the general robot behavior library},
  year = {2025},
  publisher = {GitHub},
  howpublished = {\url{https://github.com/Robot-Exp-Platform/franka-rust}}
}
```

## Source builds, assets, and license

Checkout manifests pin development dependencies to Git revisions and require access to those sources; registry installation uses published dependencies. `ROPLAT_SKIP_ASSET_EXPORT=1` skips optional build-time copying of example assets to the user data directory, useful for checks and CI.

The driver is maintained by Robot-Exp-Platform under [Apache-2.0](LICENSE). Franka is the manufacturer's name. Panda model assets retain [their notice](assets/franka_panda/LICENSE.txt); controller firmware and downloaded model libraries are separate vendor components with their own terms.
