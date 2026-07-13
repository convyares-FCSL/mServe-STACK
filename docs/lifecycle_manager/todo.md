# mserve_lifecycle_manager — TODO

## Stage 1 — COMPLETE
- [x] Extend XML to configure + activate both nodes in sequence
- [x] Add `IsInState` condition node
- [x] Add `RetryUntilSuccessful` decorator in XML
- [x] `Inverter` decorator for idempotent re-run safety
- [x] Named transitions instead of magic numbers
- [x] Snapshot archived to `learning/btcpp_stage_1/`

## Stage 2 / Stage 3 — COMPLETE
- [x] Port to `behaviortree_ros2` `RosServiceNode` base class
- [x] Non-blocking wall timer + `rclcpp::spin()` executor loop
- [x] Split into `lifecycle_manager.hpp` / `.cpp` / `main.cpp`
- [x] Groot2 live visualisation via `BT::Groot2Publisher`
- [x] `node_models.xml` generated for Groot2 palette
- [x] Graceful shutdown tree (`shutdown.xml`)
- [x] Launch file (`mserve_launch`)
- [x] `deps_setup.sh` for fresh machine setup
- [x] Snapshot archived to `learning/btcpp_stage_2/`

## Known limitations / future work

- [x] **Shutdown ordering** — `rclcpp::on_shutdown()` fired too late: the ROS
  context was already invalidated by the time the callback ran, so the
  shutdown tree's service calls silently failed and `mserve_drivechain`/
  `mserve_base` were left stuck `active` on every SIGINT. Fixed (2026-07-12)
  without making `LifecycleManager` a lifecycle node — `main.cpp` disables
  rclcpp's default signal handling (`SignalHandlerOptions::None`) and installs
  a plain `std::signal(SIGINT/SIGTERM, ...)` handler that just flags
  `shutdown_requested_`; the existing 100ms tick timer in `build()` checks
  that flag, runs `shutdown_tree_` while the context is still valid, and
  calls `rclcpp::shutdown()` itself once the tree completes. Also fixed:
  `run_stack.sh` now signals `lifecycle_manager` directly (SIGINT)
  rather than relying on `ros2 launch`'s own signal cascade, which proved
  unreliable when sent programmatically from a script's trap handler.
- [ ] **`IsInState` shows as Action in Groot2** — `RosServiceNode` inherits from
  `ActionNodeBase`, not `ConditionNode`. Cosmetic only, no runtime impact.
- [ ] **Dockerfile** — update to clone and build `behaviortree_ros2` from source
  (native build already vendors it at `ws/src/third_party/BehaviorTree.ROS2/`,
  gitignored, `humble` branch — Dockerfile hasn't been touched to match).

## Stage 4 — COMPLETE (2026-07-13)

- [x] **Multi-instance support** — added `bringup_tree_file`/`shutdown_tree_file`
  string params (default `bringup.xml`/`shutdown.xml`, resolved relative to
  this package's own `share/lifecycle_manager/trees/`) so a second instance
  can manage a different node with its own tree pair, given a distinct node
  name via launch-time `-r __node:=...` remap. First real use: SLAM Toolbox
  (`ws/src/launch/launch/mserve_slam.launch.py` launches a
  `slam_lifecycle_manager` instance pointed at `trees/slam_bringup.xml`/
  `trees/slam_shutdown.xml`), replacing plain `launch_ros` event handlers
  that had no shutdown path. See README's "Running more than one instance".
  Verified on real hardware: default (no override) bringup/shutdown
  unaffected, `slam_toolbox` reaches `active` via the BT and gets a clean
  `active → shutdown_active → finalized` transition on SIGINT.

## Next

- Extend `bringup.xml`/`shutdown.xml` with new sequences as sensors are wired
  in (camera, lidar) — no C++ changes needed, see the README's "How to add a
  managed node" section.
