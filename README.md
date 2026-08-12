# piper-arm

[![crates.io](https://img.shields.io/crates/v/piper-arm.svg)](https://crates.io/crates/piper-arm)
[![docs.rs](https://docs.rs/piper-arm/badge.svg)](https://docs.rs/piper-arm)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Rust](https://img.shields.io/badge/rust-1.75+-orange.svg)

A **safe, zero-`unsafe`, thread-friendly Rust driver** for the
[AgileX / Songling](https://www.agilex.ai/) **Piper** 6-DOF robotic arm,
implementing the V2 CAN protocol.

Runs on Linux via [SocketCAN](https://www.kernel.org/doc/html/latest/networking/can.html)
(1 Mbps, standard frames). No `tokio` dependency — a background reader thread
plus a mutex-protected snapshot keeps the public API non-blocking.

---

## Features

- **Full V2 protocol coverage** — every feedback channel and control command:
  - Periodic feedback: arm status (`0x2A1`), end-effector pose (`0x2A2–0x2A4`),
    joint angles (`0x2A5–0x2A7`), gripper (`0x2A8`), motor high/low-speed
    driver info (`0x251–0x266`).
  - Motion control: `0x150`, `0x151`, cartesian `0x152–0x154`, joint `0x155–0x157`,
    MoveC `0x158`, gripper `0x159`, MIT per-joint pass-through `0x15A–0x15F`
    (with the 12/4-bit packing and XOR CRC).
  - Configuration: master/slave `0x470`, enable/disable `0x471`, parameter
    search/set `0x472–0x47F`, firmware query `0x4AF`, master-arm home `0x191`.
- **Background reader thread** decodes frames and keeps a `LatestState`
  snapshot; getters never block on I/O.
- **Software limits** (joint angles + gripper range) optional clamping on both
  feedback and control.
- **Forward kinematics** (`DH`) with the V1.6-3+ offset parameter set.
- **FPS / health monitoring** (`is_ok`, per-message Hz).
- **Firmware version** auto-assembly from `0x4AF` frames.
- **Testable without hardware** via the in-memory `MockBus`.
- **Safe Rust**: no `unsafe`, no `Send`/`Sync` bypasses.

---

## Requirements

- Linux with SocketCAN support (any modern kernel).
- A CAN interface configured at **1 Mbps** (built-in USB-to-CAN module, e.g.
  `can0`).
- Rust **1.75+** (edition 2024 toolchain).

> This crate only supports the arm's built-in CAN module over SocketCAN.

---

## Installation

```toml
[dependencies]
piper-arm = "0.1"
```

---

## CAN interface setup

Activate the built-in CAN module at 1 Mbps before use:

```bash
# one module
bash can_activate.sh can0 1000000

# multiple modules (rename by USB port)
bash can_activate.sh can_piper 1000000 "3-1.4:1.0"

# verify
ip link show can0
```

---

## Quick start

Read the joint angles at ~200 Hz:

```rust,no_run
use piper_arm::PiperInterface;
use std::time::Duration;

fn main() {
    let arm = PiperInterface::open_socketcan("can0").expect("open can0");

    loop {
        let joint = arm.get_arm_joint_msgs();
        println!(
            "joints = [{:.3}, {:.3}, {:.3}, {:.3}, {:.3}, {:.3}] deg",
            joint.msg.joint_1 as f64 / 1000.0,
            joint.msg.joint_2 as f64 / 1000.0,
            joint.msg.joint_3 as f64 / 1000.0,
            joint.msg.joint_4 as f64 / 1000.0,
            joint.msg.joint_5 as f64 / 1000.0,
            joint.msg.joint_6 as f64 / 1000.0,
        );
        std::thread::sleep(Duration::from_millis(5));
    }
}
```

### Command the arm

```rust,no_run
use piper_arm::PiperInterface;
use std::time::Duration;

fn main() {
    let arm = PiperInterface::open_socketcan("can0").unwrap();

    arm.piper_init().ok(); // fetch limits + firmware version

    // 1. Enable motors (7 = all, 0x02 = enable)
    arm.enable_arm(7, 0x02).unwrap();

    // 2. Switch to CAN command + joint move mode
    arm.mode_ctrl(0x01, 0x01, 30, 0x00).unwrap();

    // 3. Move joint 1 to -60° (units: 0.001°) and hold others at zero
    loop {
        arm.joint_ctrl(-60_000, 0, 0, 0, 0, 0).unwrap();
        std::thread::sleep(Duration::from_millis(10));
    }
}
```

> **WARNING**: the MIT per-joint protocol (`joint_mit_ctrl`) is an advanced
> feature; incorrect gains can damage the arm. Read the official docs first.

---

## Examples

| Example | Description |
|---|---|
| [`read_joint`](examples/read_joint.rs) | Print joint angles / end pose / fps at 200 Hz |
| [`read_all`](examples/read_all.rs) | Print every feedback channel once per second |
| [`ctrl_joint`](examples/ctrl_joint.rs) | Enable + sweep joint 1 |
| [`ctrl_gripper`](examples/ctrl_gripper.rs) | Enable + cycle the gripper |
| [`set_slave`](examples/set_slave.rs) | Configure the arm as a motion-output (slave) arm |

```bash
cargo run --release --example read_joint -- can0
```

---

## API overview

### Connection

| Method | Description |
|---|---|
| `PiperInterface::open_socketcan(name)` | Open SocketCAN + start reader/monitor threads |
| `PiperInterface::new(bus: Arc<dyn CanBus>)` | Build over any `CanBus` (e.g. `MockBus`) |
| `connect_status()` / `is_ok()` | Connection / CAN-stream health |
| `disconnect()` | Stop threads and close |
| `piper_init()` | Query motor limits + firmware version |
| `get_can_fps()` | Incoming frame rate |

### Feedback getters (non-blocking)

| Method | CAN ID(s) | Payload |
|---|---|---|
| `get_arm_status()` | `0x2A1` | mode, arm status, err code |
| `get_arm_end_pose()` | `0x2A2–0x2A4` | x/y/z (0.001 mm), rpy (0.001°) |
| `get_arm_joint_msgs()` | `0x2A5–0x2A7` | joint angles (0.001°) |
| `get_arm_gripper_msgs()` | `0x2A8` | stroke, effort, FOC status |
| `get_motor_states()` | `0x251–0x256` | speed, current, position, effort |
| `get_driver_states()` | `0x261–0x266` | voltage, temps, FOC flags |
| `get_arm_enable_status()` | — | per-joint enabled flags |
| `get_piper_firmware_version()` | `0x4AF` | version string |
| `get_resp_instruction()` | `0x476` | set-instruction response |

Plus the request/response getters (`get_all_motor_max_acc_limit`,
`get_current_motor_angle_limit_max_vel`, `get_crash_protection_level_feedback`,
`get_gripper_teaching_pendant_param_feedback`, …), the master-arm read-back
getters, and `get_fk_feedback()` / `get_fk_control()` (with FK calc enabled).

### Control methods

| Method | CAN ID(s) | Purpose |
|---|---|---|
| `emergency_stop`, `reset_piper` | `0x150` | e-stop / reset |
| `mode_ctrl`, `motion_ctrl_2` | `0x151` | control + move mode |
| `end_pose_ctrl` | `0x152–0x154` | cartesian target |
| `joint_ctrl` | `0x155–0x157` | joint target (0.001°) |
| `move_c_axis_update_ctrl` | `0x158` | MoveC point select |
| `gripper_ctrl` | `0x159` | gripper target |
| `joint_mit_ctrl` | `0x15A–0x15F` | MIT per-joint (advanced) |
| `enable_arm` / `disable_arm` | `0x471` | motor power |
| `search_motor_*`, `motor_*_set`, `joint_config` | `0x472–0x475` | limits / acc / zero |
| `arm_param_enquiry_and_config` | `0x477` | queries + end load |
| `end_spd_and_acc_param_set` | `0x479` | end-effector speed/acc |
| `crash_protection_config` | `0x47A` | collision levels |
| `master_slave_config` | `0x470` | master/slave linkage |
| `req_master_arm_move_to_home` | `0x191` | master-arm homing |

See the [docs](https://docs.rs/piper-arm) for full signatures and units.

---

## Units

All integer fields match the official protocol — they are **raw protocol
units**, not SI. Convert as follows:

| Field | Unit |
|---|---|
| joint angle / end-effector rotation | `0.001` deg |
| end-effector position | `0.001` mm |
| motor speed / joint speed | `0.001` rad/s |
| motor current / bus current | `0.001` A |
| driver voltage | `0.1` V |
| gripper stroke | `0.001` mm |
| gripper effort | `0.001` N·m |
| joint acceleration | `0.01` rad/s² |
| motor angle limit | `0.1` deg |

---

## Testing

The codec, MIT bit-packing, FK and the full interface flow are covered by
unit/integration tests (known-answer vectors for the protocol and kinematics).

```bash
cargo test          # 38 tests
cargo clippy --all-targets -- -D warnings
```

Offline (no hardware):

```rust,no_run
use piper_arm::{CanBus, MockBus, PiperInterface};

fn main() -> piper_arm::Result<()> {
    let bus = std::sync::Arc::new(MockBus::new());
    let arm = PiperInterface::new(bus)?;
    // push frames, call getters/controls, assert on bus.sent_frames()
    Ok(())
}
```

---

## Project layout

```text
src/
├── can/           CanBus trait, SocketCanBus (socketcan crate), MockBus
├── error.rs       error types
├── interface.rs   PiperInterface: reader/monitor threads + LatestState snapshot
├── kinematics.rs  DH forward kinematics (V1.6-3+ offset set)
├── param.rs       SDK software limits
├── protocol/
│   ├── base.rs    big-endian / sign conversions, FloatToUint
│   └── v2/        CAN IDs, message types, codec, feedback/transmit structs
└── utils/         FPS counter, quaternion<->euler
```

---

## License

MIT — see [LICENSE](LICENSE).

`piper-arm` is an independent open-source implementation and is not affiliated
with AgileX Robotics / Songling.
