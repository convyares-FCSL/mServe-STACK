# Architecture Notes

## Purpose

mServe is a ROS 2 C++ robot stack for learning real robot systems in detail. Originally built against ROS 2 Jazzy; as of the July 2026 SD-card migration it runs natively (no Docker) against ROS 2 Lyrical. The project should make the important parts visible: configuration, lifecycle state, command validation, motor communication, simulation, tests, and launch topology.

This matters because the same ROS 2 habits transfer into more serious controls work, including hydrogen compression, dispensing, process control, and safety-adjacent automation. We should avoid hiding the useful learning behind large frameworks too early.

## Control Philosophy

Do not start with `ros2_control`.

`ros2_control` is useful and worth learning later, but the first mServe skeleton should expose:

- Command intake from `/cmd_vel`.
- Safety limits and command clamping.
- Differential-drive kinematics.
- Wheel command generation.
- ESP32 motor-controller communication.
- Wheel feedback parsing.
- Odometry boundary decisions.
- Lifecycle gating so motion cannot start before configure/activate.

The first control stack should be small enough to read in one sitting and close enough to the hardware that each moving part is understandable.

## ESP32 Motor Boundary

> **As implemented:** this boundary exists, but not as a separate ROS package.
> The ESP32 is a physical board (onboard the Waveshare DDSM Driver HAT) running
> its own firmware — `ws/src/mserve_drivechain/drive_firmware/` — and
> `mserve_drivechain` owns the JSON-over-UART client side directly (see
> `drivechain_uart.cpp`) rather than a dedicated `mserve_esp32` ROS package.
> The boundary described below is real; the package split is not. See
> `ws/src/mserve_drivechain/README.md` for the current, detailed writeup.

Create a dedicated C++ package named `mserve_esp32`.

`mserve_base` should own robot-level motion logic:

- Subscribe to velocity commands.
- Validate/clamp commands.
- Convert body velocity into wheel targets.
- Publish drive status.
- Decide whether motion is allowed based on lifecycle state and safety rules.

`mserve_esp32` should own transport and protocol:

- Open and manage the ESP32 connection.
- Send wheel commands to the motor controller.
- Parse wheel feedback, heartbeat, fault, and battery/status packets.
- Publish ESP32 status and wheel feedback.
- Detect comms timeouts.
- Fail safe when the ESP32 disappears.

This keeps control logic testable without hardware and keeps serial/protocol code isolated.

Initial likely topics:

```text
/cmd_vel
/mserve/motor/wheel_command
/mserve/motor/wheel_feedback
/mserve/base/status
/mserve/esp32/status
/mserve/events
```

Initial config should include:

```text
hardware.esp32.transport
hardware.esp32.port
hardware.esp32.baudrate
hardware.esp32.heartbeat_timeout_ms
hardware.esp32.command_timeout_ms
hardware.esp32.protocol_version
```

## Robot Description Philosophy

Avoid overusing Xacro.

Xacro is helpful for repeated geometry, constants, and optional variants, but the first robot model should remain readable. The robot description should not become a macro maze that hides the physical structure.

Preferred first pass:

- Use a plain `mserve.urdf` or a very small `mserve.urdf.xacro`.
- Keep base, wheels, lidar, camera, display, and future arm mount obvious.
- Add comments where physical assumptions matter.
- Avoid embedding control complexity in the description.

Later, if the model becomes repetitive, split carefully:

```text
urdf/
├── mserve.urdf.xacro
├── base.urdf.xacro
├── sensors.urdf.xacro
└── arm_mount.urdf.xacro
```

## Lifecycle Rules

Hardware-facing nodes should be lifecycle nodes:

- `unconfigured`: no hardware resources are active.
- `inactive`: resources may be open, but no motion/data flow is enabled.
- `active`: command processing and publishing are enabled.
- `deactivate`: stop motion/data flow cleanly.
- `cleanup`: release hardware/simulation resources.

This mirrors Lesson 06 and makes startup/shutdown explicit.

## Simulation Rules

Simulation should teach the same boundaries as hardware:

- Gazebo should not bypass command validation.
- Simulated wheel feedback should use the same topic shape as hardware where practical.
- Nav2 should send normal `/cmd_vel`.
- The base node should still be the safety/command boundary.

## Future Systems

`mserve_ai` and `mserve_manipulation` are later packages. They should consume stable robot interfaces rather than reaching into motor or hardware internals.

For the arm, reserve a frame/mount early, but avoid building arm control into the base robot skeleton.
