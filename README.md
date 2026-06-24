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

## Relationship To Other Crates

- `robot_behavior` defines the behavior traits and controller utilities used by
  this driver.
- `roplat` integration shares the same behavior vocabulary; a Franka-native
  realtime rhythm should be designed together with the roplat async execution
  boundary instead of wrapping another blocking loop.
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
