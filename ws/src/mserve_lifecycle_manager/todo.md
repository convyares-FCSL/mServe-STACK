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

- [ ] **Shutdown ordering** — `on_shutdown` fires too late when `ros2 launch` sends SIGINT
  to all nodes simultaneously. Fix: make `LifecycleManager` a lifecycle node itself so
  launch can sequence its shutdown before managed nodes. Nav2 uses this pattern.
- [ ] **`IsInState` shows as Action in Groot2** — `RosServiceNode` inherits from
  `ActionNodeBase`, not `ConditionNode`. Cosmetic only, no runtime impact.
- [ ] **Dockerfile** — update to clone and build `behaviortree_ros2` from source

## Next — behaviortree_ros2 deep-dive (compression module)

See `lesson_plan_btros2.md` for the full plan.
