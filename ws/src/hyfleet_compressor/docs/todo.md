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

Mapping from archive `Booster::update()` state machine. Each hardware command
becomes a RosServiceNode; each telemetry check becomes a RosTopicSubNode.
Timing delays use built-in `Wait` node — no C++ needed.

### Prerequisite — hardware interfaces (CONFIRMED from archive)

All hardware commands go through ONE multiplexed `BoosterCmd` service per booster.
All telemetry comes from ONE topic. Simpler than expected.

**Command service** (one per booster instance):
- Service: `/low_booster/booster_cmd` / `/high_booster/booster_cmd`
- Type: `mserve_interfaces/srv/BoosterCmd`
- Commands: `START_VFD=1`, `STOP_VFD=2`, `SET_PCSV=3`, `CONTROL_SV=4`
- Named typed fields — no payload encoding in BT layer
- ADS bridge owns PLC wire format translation
- Raw PLC contract available via `mserve_interfaces/srv/Cmd.srv` if needed

**Telemetry topic** (single topic, all data):
- Topic: `compressor_telemetry`
- Type: `mserve_interfaces/msg/CompressorTelemetry`
- Inlet pressure:  `hbu_pt_bar[inlet_pt_index]`  — low=0, high=1
- Outlet pressure: `hbu_pt_bar[outlet_pt_index]` — low=7, high=2
- VFD speed: `vfd_speed_rpm[vfd_index]` — low=0, high=1 (array, updated msg)
- Indices come from config params — booster.xml ports pass them in

- [x] SV service confirmed
- [x] VFD service confirmed
- [x] PCSV service confirmed
- [x] Inlet pressure topic confirmed
- [x] Outlet pressure topic confirmed
- [x] VFD speed topic confirmed

### Action nodes (RosServiceNode — all call BoosterCmd service)
- [x] `StartVFD`  — cmd=START_VFD, fields: speed_rpm
- [x] `StopVFD`   — cmd=STOP_VFD, no fields
- [x] `SetPCSV`   — cmd=SET_PCSV, fields: enable, cpm
- [x] `ControlSV` — cmd=CONTROL_SV, fields: device_id, enable

### Condition nodes (RosTopicSubNode)
- [ ] `InletPressureStable` — rolling window stability check (stub only, TODO)
- [x] `VFDAtSpeed` — vfd_speed_rpm[vfd_index] >= target_speed
- [x] `OutletAtPressure` — hbu_pt_bar[outlet_pt_index] >= target_pressure
- [x] `InletPressureSafe` — hbu_pt_bar[inlet_pt_index] >= safe_pressure

### Node registration and params
- [ ] Register all 7 BT nodes with factory in `on_configure`
- [ ] Declare ROS params: `vfd_target_speed`, `vfd_index`, `inlet_pt_index`,
      `outlet_pt_index`, `safe_pressure`, `pcsv_cpm`, `inlet_sv_id`, `hpu_sv_id`,
      `vfd_delay_ms`, `stabilization_ms`, `telemetry_topic`
- [ ] Read params in `on_configure`, write to blackboard
- [ ] Write service name `/{node_name}/booster_cmd` to blackboard at configure time
- [ ] Goal callback: select `active_tree_` from `goal->command`, write `target_pressure`
- [ ] Tick timer updated to use `active_tree_` pointer
- [ ] `BoosterNode` header: add four `BT::Tree` members + `active_tree_` pointer

### XML trees — one per command
- [ ] `booster_start.xml` — Parallel(failure_threshold=1):
      Sequence: ControlSV(inlet,open) → InletPressureStable → StartVFD →
      Wait(vfd_delay_ms) → VFDAtSpeed → Wait(stabilization_ms) →
      ControlSV(HPU,open) → SetPCSV(enable) → OutletAtPressure
      ∥ Safety: InletPressureSafe
- [ ] `booster_start_idle.xml` — same as start but holds at target (no StopVFD at end)
- [ ] `booster_stop.xml` — Sequence: SetPCSV(disable) → ControlSV(HPU,close) →
      StopVFD → VFDAtSpeed(zero) → ControlSV(inlet,close)
- [ ] `booster_safe_stop.xml` — Sequence: SetPCSV(disable) → ControlSV(HPU,close) →
      StopVFD → ControlSV(inlet,close) (no ramp-down wait)

### Remaining C++ nodes
- [ ] `InletPressureStable` — rolling window stability check (replace stub)

### Feedback
- [ ] Publish outlet pressure + percent_complete to action feedback each tick

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
