# Architecture Notes

## Purpose

mServe is a ROS 2 C++ robot stack for learning real robot systems in detail. It runs in Docker (ROS 2 Jazzy) on the Pi 5. The project should make the important parts visible: configuration, lifecycle state, command validation, motor communication, simulation, tests, and launch topology.

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

The boundary is real but is not a separate ROS package. Three layers, each
owning one thing:

`mserve_base` owns robot-level motion logic:

- Subscribe to `/cmd_vel`, validate/clamp commands.
- Convert body velocity into per-motor RPM (diff-drive kinematics, both
  directions — feedback + IMU integrate into `/odom`).
- Publish drive status; decide whether motion is allowed based on lifecycle
  state and safety rules.

`mserve_drivechain` owns transport and protocol (the planned `mserve_esp32`
package, folded in):

- Open and manage the UART connection (`drivechain_uart.cpp`, JSON over
  `/dev/ttyAMA0`).
- Send per-motor commands, parse feedback/status, detect comms timeouts,
  fail safe (zero motors) when the ESP32 disappears.

The ESP32 itself is a physical board (onboard the Waveshare DDSM Driver HAT)
running its own firmware — `ws/src/mserve_drivechain/drive_firmware/` — and
owns the raw DDSM115 binary motor protocol. The Pi side only ever speaks
JSON. See `ws/src/mserve_drivechain/README.md` for the detailed writeup.

This keeps control logic testable without hardware and keeps serial/protocol
code isolated.

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

**As implemented:** this gating is driven by `ws/src/lifecycle_manager/`, a
BehaviorTree.CPP node that configures/activates `mserve_drivechain` then
`mserve_base` on bringup, and runs a shutdown tree (deactivate → shutdown)
on SIGINT/SIGTERM — rather than each node/launch file managing its own
transitions by hand. See `ws/src/lifecycle_manager/README.md`.

## Simulation Rules

Simulation should teach the same boundaries as hardware:

- Gazebo should not bypass command validation.
- Simulated wheel feedback should use the same topic shape as hardware where practical.
- Nav2 should send normal `/cmd_vel`.
- The base node should still be the safety/command boundary.

## C++ Package Conventions

Each C++ package follows:

```text
package_name/
├── CMakeLists.txt
├── package.xml
├── include/package_name/
├── src/           node.cpp, params, BT nodes, main.cpp
└── test/          GTest for ROS-free logic, launch_testing for graph behavior
```

Rules:

- C++17; header/source separation; smart pointers; `main()` wrapped in
  `try/catch`.
- Keep ROS-free logic separable and unit-tested (kinematics, clamping,
  packet codecs, timeout logic).
- Lifecycle nodes for hardware/motion subsystems; plain nodes only for
  operator-facing peripherals (display, joystick, Sense HAT).
- Callback groups for blocking or long-running work.
- `target_link_libraries()` with modern imported targets, not
  `ament_target_dependencies()` — a portability choice; keep new packages
  consistent.
- Hardware nodes wrap the vendor's device/driver class directly, never the
  vendor's whole node — an upstream driver node is a plain node and can't be
  driven by `lifecycle_manager`.
- Register composable nodes only after standalone behavior is clear.

## Future Systems

`mserve_ai` and `mserve_manipulation` are later packages. They should consume stable robot interfaces rather than reaching into motor or hardware internals.

For the arm, reserve a frame/mount early, but avoid building arm control into the base robot skeleton.
