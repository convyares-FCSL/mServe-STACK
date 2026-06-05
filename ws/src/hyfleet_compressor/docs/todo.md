# Compressor Module — Build Todo

See `architecture.md` for full design. Key decision: both nodes are lifecycle nodes
owning rclcpp_action servers + internal BT trees. No TreeExecutionServer. Closed.

## Legend
- [x] Done
- [ ] Todo
- [-] Deferred

---

## Stage 1 — BoosterNode with action server + BT

Goal: full wiring end-to-end. Lifecycle node, action server, BT factory, tree from
XML. Simple tree: accept goal → log → succeed. Proves the architecture works before
building the real state machine.

### `hyfleet_booster` package
- [x] Package created, builds clean
- [x] Lifecycle node shell (configure / activate / deactivate / cleanup / shutdown)
- [x] Launched twice: `low_booster` and `high_booster`
- [x] Wired into lifecycle manager bringup tree
- [x] `ControlBooster.action` defined in `mserve_interfaces` (command, target_pressure, feedback)
- [x] `BoosterAction` class — action server in Reentrant cb group, goal callback pattern
- [x] `active_goal_` owned by `BoosterNode`, `BoosterAction` notifies via `set_goal_callback`
- [x] `on_configure()` — action server, goal callback, factory, tree loaded from XML
- [x] `on_activate()` / `on_deactivate()` — toggle `accepting_goals_`
- [x] `on_cleanup()` — abort active goal, cancel timer, reset tree, unconfigure action server
- [x] Tick timer — 100ms wall timer, `tickOnce()`, succeed/abort on completion
- [x] `src/trees/booster.xml` — Stage 1: `AlwaysSuccess` placeholder
- [x] Action name `~/control_booster` — resolves to `/low_booster/control_booster` per instance
- [x] Verified: `ros2 action send_goal /low_booster/control_booster ...` → `SUCCEEDED`

---

## Stage 2 — Booster BT state machine

Unblocked once Stage 1 verified end-to-end.

- [ ] Custom BT node: `StartBooster` (SyncActionNode)
- [ ] Custom BT node: `StopBooster` (SyncActionNode)
- [ ] Phase 2: `PressureInRange` — RosTopicSubNode condition
- [ ] Phase 2: `TemperatureOK` — RosTopicSubNode condition
- [ ] Phase 3: `ForceStopRequested` — blackboard condition node
- [ ] Phase 3: Reactive Parallel wrapping operation + safety monitor
- [ ] Booster state machine in XML: IDLE → STARTING → RUNNING → STOPPING → FAULT
- [ ] Write booster status back to action feedback each tick

---

## Stage 3 — CompressorNode coordinator

Unblocked once Stage 2 verified.

- [ ] Replace current hand-rolled action server with coordinator pattern
- [ ] Coordinator BT tree loaded from XML
- [ ] Phase 1: `RosActionNode` wrapping `/low_booster/control`
- [ ] Phase 1: `RosActionNode` wrapping `/high_booster/control`
- [ ] Goal routing: LOW / HIGH / SYNC based on `target` blackboard key
- [ ] SYNC: Parallel calling both booster RosActionNodes
- [ ] Force stop: cancel in-flight booster goals via action cancel protocol

---

## Stage 4 — Full integration

- [ ] SYNC: `RosTopicSubNode` interstage pressure monitoring
- [ ] SYNC: `RosServiceNode` CPM adjustment on low booster
- [ ] SYNC: `RosServiceNode` interstage solenoid command
- [ ] Safety monitor end-to-end
- [ ] DiagnosticsPublisher
- [ ] Groot2 visualisation on both trees
- [-] TreeExecutionServer — optional, not on critical path

---

## Known gaps / deferred

- `/cmd_sender` and `/telemetry` interfaces not yet defined
- Config loading — currently hardcoded defaults
- Shutdown tree not yet updated for boosters
