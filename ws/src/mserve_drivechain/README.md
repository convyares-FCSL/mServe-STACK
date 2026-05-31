# mserve_drivechain

Owns everything physical about the mServe drivetrain: diff-drive kinematics, DDSM115 motor protocol, and wheel feedback.

## Responsibilities

- Subscribes to `/cmd_vel_safe` (clamped Twist from `mserve_base`)
- Converts body velocity → individual wheel speeds via diff-drive kinematics
- Sends wheel speed commands to the DDSM Hub Motor Driver Board
- Publishes `/wheel_feedback` (actual wheel velocities from the board)
- Publishes `/drivechain_status` (connection state, firmware version)
- Detects comms timeout and fails safe when the board is unreachable

## Why this package owns wheel geometry

`wheel_separation` and `wheel_radius` are physical properties of the DDSM115 assembly, not robot-level policy. Keeping them here means swapping the drivetrain hardware only requires changes inside this package.

## Key classes

| Class | File | Description |
|-------|------|-------------|
| `DrivechainNode` | `src/drivechain_node.cpp` | Lifecycle node — thin ROS adapter |
| `DiffDrive` | `src/diff_drive.cpp` | Pure kinematics, no ROS. Testable standalone. |
| `PacketCodec` | *(planned)* | DDSM115 wire protocol encode/decode |
| `Transport` | *(planned)* | Abstract serial/USB interface |

## Topic boundary

```
/cmd_vel_safe  →  DrivechainNode  →  DDSM Hub Board  →  DDSM115 x2
                       │
                       ├─► /wheel_feedback
                       └─► /drivechain_status
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `cmd_vel_safe_topic` | `/cmd_vel_safe` | Input command topic |
| `wheel_separation` | `0.35` | Centre-to-centre wheel distance (m) — measure on actual chassis |
| `wheel_radius` | `0.08` | DDSM115 tyre radius (m) |
| `wheel_feedback_topic` | `/wheel_feedback` | Wheel velocity feedback |
| `drivechain_status_topic` | `/drivechain_status` | Board connection status |
| `feedback_rate` | `20.0` | Feedback publish rate (Hz) |

## Tests

```bash
colcon test --packages-select mserve_drivechain --event-handlers console_direct+
```

- `test_diff_drive` — kinematics: straight, reverse, pivot, arc
- `test_packet_codec` — DDSM115 protocol (stub until protocol is implemented)
