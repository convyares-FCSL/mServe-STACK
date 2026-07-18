# utils

Shared C++ helper utilities for the mServe stack. Package name is `utils`
(dropped the originally-planned `mserve_utils` prefix, like `interfaces` and
`launch` — see root `readme.md`; the `mserve_utils` C++ namespace inside it
kept the prefix, so headers are still `#include <mserve_utils/...>`).

Actively used by `mserve_base`, `mserve_drivechain`, `mserve_camera`,
`mserve_lidar`, `mserve_display`, and `lifecycle_manager` — not a
placeholder.

## What's here (`include/mserve_utils/`)

- **`utils.hpp`** — `get_or_declare_param()` (declare-if-missing + read in
  one call), `make_int_range_descriptor()`/`make_double_range_descriptor()`
  (bounded parameter descriptors — out-of-range values rejected by ROS
  itself, before any callback runs).
- **`qos.hpp`** — named QoS profiles (`mserve_qos::commands`/`feedback`/
  `status`), each reading its settings from parameters so they can be
  overridden in launch config without recompiling. Defaults: `commands`
  (reliable, volatile, depth=1 — only the latest command matters),
  `feedback` (best-effort, volatile, depth=10 — high-rate sensor data, drop
  ok), `status` (reliable, volatile, depth=10).
- **`topics.hpp`** — canonical topic names (`mserve_topics::cmd_vel`,
  `cmd_vel_safe`, `drivechain_status`, etc.), backed by parameters so they
  can be remapped without recompiling. `/mserve/` namespace for
  mServe-internal topics; `/cmd_vel` stays at the ROS standard path so Nav2
  and joystick drivers work without remapping.
- **`lifecycle.hpp`** — `transitionIdFromName()`, maps a transition name
  string to its `lifecycle_msgs` transition ID (used by `lifecycle_manager`'s
  BT nodes).
- **`param_guard.hpp`** — `bounded_double()` descriptor builder + a
  `ParamValidation` result type.
- **`config.hpp`** — shared configuration parameter constants.

`qos.hpp`/`topics.hpp`'s helpers take a `rclcpp_lifecycle::LifecycleNode&` —
they don't compile against a plain `rclcpp::Node`. `mserve_display` (a plain
node, not lifecycle-managed) uses `utils.hpp`'s
`get_or_declare_param(rclcpp::node_interfaces::NodeParametersInterface&, ...)`
overload directly instead, which takes the interface rather than a node
type.

## Build

```bash
cd /home/ecm/mServe-STACK/ws
colcon build --packages-select utils --symlink-install
colcon test --packages-select utils --event-handlers console_direct+
```
