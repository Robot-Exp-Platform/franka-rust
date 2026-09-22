# franka_rust

[English](README.md) | [简体中文](README_zh.md)

`franka_rust` is a Rust driver for Franka robots in the Robot-Exp driver stack.
It connects Franka FCI to the common `robot_behavior` traits, so the same
motion, control, model and roplat orchestration style can be used across real
robots and simulation backends.

This crate is not a line-by-line port of `libfranka`. The wire protocol follows
Franka FCI semantics, while the public API is shaped around Rust ownership,
typed behavior spaces and scoped controller closures.

## What It Provides

- One public robot object per model: `FrankaEmika`, `FrankaPanda`, `FrankaFR3`
  and `FrankaFP3`.
- `robot_behavior` motion spaces for joint and Cartesian target motion.
- Blocking `control_with` sessions for joint position, joint velocity,
  Cartesian pose, Cartesian velocity and torque control.
- `control_with_async` sessions whose per-cycle controller closure can be
  asynchronous while the robot resource remains borrowed for the whole control
  session.
- Franka-specific configuration such as collision behavior, internal impedance,
  guiding mode, load and frame settings.
- Access to the downloaded Franka model library for kinematics and dynamics.
- Optional Python and C++ FFI layers, kept downstream of `robot_behavior`.

## Design In 0.2

Version `0.2.0` aligns Franka control with the current `robot_behavior`
semantics:

- `control_with` is blocking. It returns when the controller reports `done` or
  the robot returns an error.
- Controller closures no longer need `'static`; they may borrow local
  trajectories, models or loggers for the duration of the call.
- The synchronous control path uses a direct `std::net::UdpSocket` realtime
  loop.
- The async-control path uses a `tokio::net::UdpSocket` realtime loop internally.
- Ordinary blocking commands such as `move_to`, configuration setters and model
  loading remain on the simple blocking command plane.

The result is intentionally small: the robot type is not split into sync and
async variants, and realtime controllers are not routed through a shared
background closure store.

## Ending A Controller Session

`control_with_flow` and `control_with_flow_async` accept
`ControlFlow<(), (Command, bool)>`. Both calls block until the device session
has ended; `async` describes the per-cycle callback.

- `Continue((command, false))` sends the command and continues.
- `Continue((command, true))` sends the final command and completes the normal
  FCI finish handshake.
- `Break(())` sends no command from that cycle. The driver uses TCP `StopMove`,
  waits for the device to leave its running modes, and consumes the terminal
  Move response before returning. This is a device session stop, not a fabricated
  zero command or a claim of an emergency-stop guarantee.

Cancellation / failure cleanup has a shared three-second network waiting deadline
for StopMove, idle-state reception and the terminal Move response. This does not
guarantee a physical stopping time. Timeout returns an error; if session completion
was not confirmed, new control sessions are rejected until the robot is reconnected.
Read-timeout and nonblocking socket settings are restored on exit. Normal per-cycle
I/O is unchanged. Configurable cleanup deadlines remain future work.

The tuple-based `control_with` and `control_with_async` APIs wrap the same loop.
Errors from both execution and cleanup are retained in
`RobotException::ControlSession` when both fail. The existing local runtime,
UDP timing and blocking ownership model are unchanged.

The Franka async-callback entry constructs a local Tokio runtime and calls
`block_on`. Calling it from an already entered Tokio runtime can panic due to
nested runtime entry. Consequently, the shared ControlRhythm is not yet a
drop-in Franka driver for an arbitrary Tokio System context. Resolving this
existing runtime boundary is a separate design task; mock System tests do not
validate that hardware execution context.

Offline loopback regression tests exercise these actual TCP/UDP paths without
connecting to hardware (`cargo test -p franka_rust --lib realtime::tests`). They
verify command suppression, the final-command handshake and StopMove rejection;
real-device stopping behavior and timing still need hardware acceptance.

## Relationship To Other Crates

- `robot_behavior` defines the behavior traits and controller utilities used by
  this driver.
- Enable `robot_behavior/roplat` to use the shared `ControlRhythm`. A drive
  holds the robot for one blocking control session; an async controller does
  not make the surrounding session nonblocking.
- `rsbullet` and `libjaka-rs` are sibling backends in the same workspace; they
  target the same behavior vocabulary but use their own transport kernels.

## Requirements

- Rust nightly, edition 2024.
- A reachable Franka controller running a compatible FCI server.
- A realtime Linux kernel is recommended for hardware experiments, but the
  crate can compile and run on non-realtime systems for development.

## Installation

```toml
[dependencies]
franka_rust = "0.2"
robot_behavior = "0.6"
```

Inside this workspace, both `roplat` and `robot_behavior` are patched to local
paths by the root `Cargo.toml`.

## Quick Start

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

## Realtime Control

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

For production experiments, prefer controller builders from
`robot_behavior::controller` when the control law is generic. Keep only
Franka-specific wiring in this crate.

## Examples

The `examples/` directory covers:

- joint and Cartesian target motion,
- generated joint and Cartesian control commands,
- torque control,
- impedance controllers provided by `robot_behavior`,
- model loading,
- gripper usage,
- roplat-oriented examples where the current abstraction is sufficient.

Examples are compiled by `cargo check --all-targets`; they are not hardware
tests unless explicitly run against a robot.

## Safety Notes

This crate exposes low-level realtime command paths. It does not make arbitrary
commands physically safe. Before hardware tests:

- configure collision thresholds and load data,
- use conservative velocity, acceleration and torque limits,
- test controllers in simulation first,
- keep an operator near the emergency stop,
- avoid running high-frequency control on heavily loaded non-realtime systems.

## Status

`franka_rust` is an experimental hardware driver. The Rust API is currently more
important than preserving old names, so minor releases may still reshape public
interfaces while the Robot-Exp driver stack settles.
