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
- [ ] Public header — add members:
  - `rclcpp_action::Server<ControlCompressor>::SharedPtr action_server_`
  - `rclcpp::CallbackGroup::SharedPtr action_cb_group_`
  - `std::shared_ptr<GoalHandleControlCompressor> active_goal_`
  - `BT::BehaviorTreeFactory factory_`
  - `BT::Tree tree_`
  - `rclcpp::TimerBase::SharedPtr tick_timer_`
- [ ] `on_configure()` — create action server (Reentrant cb group), create factory,
      register nodes, load tree from XML
- [ ] `on_activate()` — set `accepting_goals_ = true`
- [ ] `on_deactivate()` — set `accepting_goals_ = false`, abort active goal
- [ ] `on_cleanup()` — destroy action server, tree, reset factory
- [ ] Action server callbacks: `handle_goal`, `handle_cancel`, `handle_accepted`
- [ ] `handle_accepted` — write goal to blackboard, start `tick_timer_`
- [ ] Tick timer callback — `tree_.tickOnce()`, check result, succeed/abort goal
- [ ] Simple XML tree: `config/booster_tree.xml` — Sequence → LogMessage → AlwaysSuccess
- [ ] Verify: `ros2 action send_goal /low_booster/control ...` → succeeds

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
