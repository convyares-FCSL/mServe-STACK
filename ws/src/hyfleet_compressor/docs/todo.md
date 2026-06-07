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
becomes a RosServiceNode. Telemetry is owned by `BoosterNode` via a subscription
to `compressor_telemetry`; a thread-safe `BoosterTelemetryCache` is shared via the
blackboard. Wait nodes extend `BT::StatefulActionNode`; gate nodes extend
`BT::ConditionNode`. Timing delays use built-in `Sleep msec` node — no C++ needed.

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

### Telemetry cache (architecture)
- [x] `BoosterTelemetryCache` — thread-safe struct, `update()` / `latest()` with mutex
- [x] `BoosterNode` owns subscription to `compressor_telemetry`, writes cache on each message
- [x] Cache `shared_ptr` placed on blackboard at configure; shared by all BT nodes and tick loop
- [x] Removed `telemetry_topic` blackboard entry — node owns the subscription directly

### Wait nodes (BT::StatefulActionNode — RUNNING is legal)
- [x] `InletPressureStable` — rolling window, variation <= tolerance → SUCCESS; timeout → FAILURE
- [x] `VFDAtSpeed` — |speed - target| <= ramp_tolerance → SUCCESS; ramp_timeout_ms → FAILURE
- [x] `VFDStopped` — speed <= stop_threshold && state != VFD_RUNNING → SUCCESS; stop_timeout_ms → FAILURE
- [x] `OutletAtPressure` — pressure >= target → SUCCESS; no wall-clock timeout (stall detection: Stage 4)

### Gate nodes (BT::ConditionNode — SUCCESS/FAILURE only)
- [x] `InletPressureSafe` — pressure >= safe_pressure → SUCCESS, else FAILURE (fail-safe: no telemetry → FAILURE)

### Node registration and params
- [x] Register all 9 BT nodes with factory in `on_configure`
- [x] Declare ROS params: all hardware indices, operational limits, timing (ms), VFD speed thresholds
- [x] Added `stability_timeout_ms` (10000), `ramp_timeout_ms` (30000), `stop_timeout_ms` (15000)
- [x] Read params in `on_configure`, write to blackboard
- [x] Service name `/{node_name}/booster_cmd` written to blackboard at configure time
- [x] Goal callback: select `active_tree_` from `goal->command`, write `target_pressure`/`speed_rpm`/`cpm`
- [x] Tick timer uses `active_tree_` pointer

### XML trees — one per command
- [x] `start_tree.xml` — SubTree ID="Start": opens inlet SV, waits InletPressureStable,
      starts VFD, opens HPU SV. Leaves booster in WARM state. SubTree(Stop) is registered
      first so Start can reference it for cleanup on failure.
- [x] `compress_tree.xml` — unified COMPRESS tree driven by `on_target` and `on_inlet_starve`
      blackboard fields. `SubTree(Start)` → `ReactiveSequence(InletGuard ∥ Fallback)`.
      Fallback routes on `OnTargetIs`:
        `ON_TARGET_SUCCEED(0)`: `Parallel(Sequence(SetPCSV(on) → LogCompressionStart → HoldPCSV)
        ∥ OutletAtPressure)` — exits SUCCESS when target reached.
        `ON_TARGET_HOLD(1)`: `KeepRunningUntilFailure(Sequence(same_Parallel →
        PressureBelowThreshold(reenable_pressure_bar, timeout=0)))` — never returns SUCCESS;
        must be preempted by STOP.
      `SetPCSV` is a pure RosServiceNode — fires PCSV-on, returns SUCCESS when PLC confirms.
      `HoldPCSV` is the state owner — stays RUNNING indefinitely; fires PCSV-off async via
      `onHalted()` on every exit path (target reached, inlet pause, goal abort).
- [x] `stop_tree.xml` — SetPCSV(off) → PressureBelowThreshold(hyd_a) →
      PressureBelowThreshold(hyd_b) → ControlSV(HPU,close) → PressureBelowThreshold(hyd_primer)
      → StopVFD → ControlSV(inlet,close) → VFDStopped
- [x] `safe_stop_tree.xml` — Parallel(SetPCSV + HPU SV + StopVFD + inlet SV): slam all off
      simultaneously; no hydraulic checks, no VFD ramp confirmation.

### Feedback
- [x] Publish outlet pressure + percent_complete to action feedback each tick (from telemetry cache)
- [x] `LogCompressionStart` — `SyncActionNode`; logs `Compressing from X bar to target Y bar`
      via `rclcpp::get_logger(ros_node_name)` so Loki groups lines under `[low_booster]`/
      `[high_booster]` rather than `[LogCompressionStart]`; reads `telemetry_cache` off the
      blackboard plus `outlet_pt_index`/`target_pressure` ports (`booster_bt_conditions.cpp`)

### XML tree corrections and SYNC prerequisites

**PARALLEL dependency — done:**
- [x] All four tree IDs renamed to unique values: `Start`, `StartIdle`, `Stop`, `SafeStop`
- [x] `stop_tree.xml` — correct controlled shutdown sequence:
      `SetPCSV(off)` → `PressureBelowThreshold(hyd_a)` → `PressureBelowThreshold(hyd_b)` →
      `ControlSV(HPU,close)` → `PressureBelowThreshold(hyd_primer)` →
      `StopVFD` → `ControlSV(inlet,close)` → `VFDStopped`
- [x] `stop_force_tree.xml` — uncontrolled slam: `Parallel(SetPCSV + HPU SV + StopVFD + inlet SV)`
- [x] `start_tree.xml` — outer Sequence wraps Parallel (safety guard) + `SubTree ID="Stop"`;
      InletPressureSafe halted before shutdown so inlet close does not trip the gate
- [x] `PressureBelowThreshold` — `StatefulActionNode`; polls `pt_bar[pt_index] < threshold_bar`;
      wall-clock timeout → FAILURE. Used for hyd A/B/primer checks in stop sequence.
- [x] `hyd_pressure_threshold_bar` (30.0 bar), `hyd_timeout_ms` (10000 ms) params added
- [x] `stop_tree.xml` registered with factory before `start_tree.xml` is instantiated
      (SubTree reference requires prior registration)

**SYNC dependencies — implemented:**
- [x] `reenable_pressure_bar` (default 230.0 bar) — declared in `booster_params.cpp`,
      written to blackboard in `load_params()`; config-time machine param, not a goal field.
      Absolute re-engage threshold — `PressureBelowThreshold::onRunning()` uses it directly
      when `reenable_pressure_bar` port is provided instead of `threshold_bar`.
- [x] `ON_TARGET_HOLD` maintain loop in `compress_tree.xml` (replaces `start_idle_tree.xml`):
      After target reached, Parallel halts HoldPCSV → PCSV off → WARM.
      `PressureBelowThreshold(outlet, reenable_pressure_bar, timeout_ms=0)` waits for droop.
      Loop repeats via `KeepRunningUntilFailure`. `timeout_ms=0` = no wall-clock limit.
- [x] `InletGuard` node: `INLET_STARVE_PAUSE` mode blocks (RUNNING) while starved, resuming
      when inlet recovers above `inlet_resume_bar`. `inlet_starve_bar` and `inlet_resume_bar`
      are goal fields, not params — coordinator sets them per booster role.

---

## Stage 3 — CompressorNode coordinator

Lesson plan Phase 1 (RosActionNode) and Phase 3 (reactive tree patterns) land here.
PARALLEL mode proves the coordinator→booster action path and multi-goal handling.
SYNC mode layers reactive interstage coordination on top of working PARALLEL pieces.
Coordinator is a router (PARALLEL) and interstage coordinator (SYNC) — boosters do the
hardware work. No BoosterManager inside CompressorNode.

### Package setup (done)
- [x] `hyfleet_compressor` package created from `hyfleet_booster` base — stripped back to
      coordinator core: `ControlCompressor` action, 3 trees (START/STOP/SAFE_STOP),
      no telemetry cache, no hardware BT nodes, `AlwaysSuccess` placeholder trees
- [x] `compressor_bt_nodes.hpp/.cpp` stub added — `BoostLow`/`BoostHigh` `RosActionNode`
      wrappers go here
- [x] Blackboard architecture contracts: `low_booster_action`, `high_booster_action`
- [x] `COLCON_IGNORE` on `hyfleet_compressor_archive` to avoid duplicate package name
- [x] Builds clean; placeholder tree accepts goal and succeeds

### Prerequisites — define before building

- [x] Booster preemption semantics: a new `ControlBooster` goal arriving while the booster
      is already RUNNING must mean "update target_pressure, keep compressing" not
      "halt and restart". Define and confirm in `BoosterNode` before coordinator
      re-goaling is implemented.
- [x] SYNC band semantics confirmed: "halt low at 280 bar" = `START_IDLE` (VFD holds ready,
      no ramp/stabilise cost on re-enable at 220 bar), not a cold `STOP`. START_IDLE
      exists exactly for this — band-control must use the idle/hold path.
      Confirmed by `start_idle_tree.xml` maintain loop implementation and all 5 tests passing.

### PARALLEL mode — multi-goal node structure

PARALLEL and SYNC are different problems. PARALLEL: two independent concurrent
`ControlCompressor` goals (LOW + HIGH) governed by a `targets_overlap` rule.
SYNC: one goal coordinating both boosters internally.

- [x] `CompressorNode` replaces single `active_goal_`/`active_tree_` with `ops_[2]` —
      two `OpSlot` structs, each holding goal handle + per-slot blackboard + tree instance
- [x] Per-operation blackboard — child of `shared_blackboard_`; inherits global keys,
      owns per-slot feedback keys; prevents concurrent trees clobbering each other
- [x] Single tick timer iterates all active operations each 100 ms tick
- [x] `targets_overlap` inline in `on_compressor_goal_accepted`: SYNC aborts all slots;
      non-SYNC preempts the same-target slot only
- [x] Goal acceptance: validate command/target/pressure; assign slot (LOW→0, HIGH→1, SYNC→0);
      preempt existing occupant; call `start_op`; start tick timer
- [-] Re-goaling: new LOW goal while LOW running = relay updated target to in-flight booster
      (smooth update, no halt). Currently aborts and restarts. Deferred — depends on
      extended booster preemption semantics.
- [x] Mode transition — SYNC arriving while LOW/HIGH running: abort all active slots first

### BT nodes — `RosActionNode` wrappers

- [x] `BoostLow`  — `RosActionNode<ControlBooster>` → `/low_booster/control_booster`;
      `setGoal()` fills command/target_pressure/cpm/speed_rpm from blackboard;
      `onFeedback()` writes `low_pressure` / `low_percent_complete` to blackboard;
      `onResultReceived()` → SUCCESS / FAILURE; `onFailure()` handles server unreachable
- [x] `BoostHigh` — same pattern; writes `high_pressure` / `high_percent_complete`
- [x] Register both nodes in `register_bt_nodes()`

### XML trees — PARALLEL paths

Each tree is a single `RosActionNode` call; routing is done at the node level (which
tree is selected) not inside the XML. Command is written to the slot blackboard by
`start_op` before tree instantiation, so one tree file handles start/stop/safe_stop
for each booster target.

- [x] `parallel_low.xml`  — `BoostLow(action_name={low_booster_action}, command={command}, ...)`;
      command = COMPRESS(1) / STOP(2) / FORCE_STOP(3) written to slot blackboard by `start_op`
- [x] `parallel_high.xml` — same pattern for high booster
- [x] `sync.xml`          — two-phase interstage coordination; see SYNC section below

### Feedback — PARALLEL

`onFeedback()` writes to the operation's own blackboard; tick loop reads and publishes
on that operation's goal handle. Blackboard is the bridge — BT nodes never hold goal
handle references.

- [x] Tick loop: for each active op, read its blackboard, publish on its own goal handle
- [x] LOW goal handle publishes `low_pressure` / `low_percent_complete`
- [x] HIGH goal handle publishes `high_pressure` / `high_percent_complete`

### Coordinator params

- [x] Declare `default_speed_rpm` (1500), `eco_cpm`, `performance_cpm`
- [x] Read in `load_params()`, write to shared blackboard; slot blackboards inherit via parent
- [x] `start_op` writes `cpm` and `speed_rpm` to slot blackboard from mode profile

**Locked decision — parameter change contract (applies to `CompressorNode` and
`BoosterNode` alike, see `compressor_params.cpp` / `booster_params.cpp` `on_parameters`):**
- [x] `on_parameters` rejects any change unless `get_current_state().id() ==
      PRIMARY_STATE_UNCONFIGURED` — i.e. reject while `inactive` or `active`, only allow
      from `unconfigured`. Cross-field checks (e.g. `min_pressure_bar < max_pressure_bar`)
      run after the state gate. Reconfiguring (cleanup → set param → configure → activate)
      picks up the new value because `load_params()` re-reads and rewrites the blackboard
      every `on_configure` — no separate "apply" step needed.
- [ ] Test coverage for the above contract (positive + negative) — neither `CompressorNode`
      nor `BoosterNode` currently has one:
      negative: `set_parameters` while `inactive`/`active` → `successful=false`,
      reason `"Parameters can only be changed in UNCONFIGURED state"`, value unchanged;
      positive: `cleanup` → `set_parameters` (new value) → `configure` → `activate` →
      blackboard / behaviour reflects the new value (e.g. `min_pressure_bar`/`max_pressure_bar`
      goal-validation bound, or a booster speed/pressure limit)

### Mode — cleanup item

- [ ] `mode` is currently a per-goal field on `ControlCompressor.action`. This should be
      a stored state owned by `CompressorNode`, changeable via a `~/set_mode` service.
      Per-goal mode forces clients to track state; a service allows mode to be set once
      and persist across fills. Remove `mode` from action once service is in place.

### Force stop and recovery

- [ ] Force stop: cancel in-flight booster goals via ROS action cancel protocol
- [ ] Recovery Fallback: graceful stop → force stop path in XML tree

### PARALLEL mode — verified end-to-end (2026-06-06 night session)

- [x] Fixed `spin_some() called while already spinning` crash (`exit -6`) hit by
      `test_stop`/`test_setpoint_update`: `BoostLow`/`BoostHigh` (`RosActionNode<ControlBooster>`)
      `halt()` → `cancelGoal()` touches the same per-action `SingleThreadedExecutor` that
      `tick()`'s `spin_some()` uses, but without the library's internal mutex, so a goal-accept
      racing a tree-tick on the `MultiThreadedExecutor` could hit it concurrently. Fix: moved
      `action_callback_group_` from `Reentrant` to `MutuallyExclusive` and put the tick timer on
      the same group, serialising goal-accept and tree-tick (`compressor_node.cpp` `on_configure`
      / `set_tick_timer`)
- [x] All 5 BT integration test scripts pass against a single clean stack instance:
      `test_low`, `test_high`, `test_parallel_both`, `test_setpoint_update`, `test_stop`

---

### SYNC mode — reactive interstage coordination (done)

#### SYNC interstage BT nodes

- [x] `InterstageAboveBand` — `StatefulActionNode` in coordinator tree; reads
      `telemetry_cache` from blackboard; polls `pt_bar[interstage_pt_index] >=
      interstage_start_threshold_bar`; returns RUNNING until threshold reached, then SUCCESS.
      SV stays closed until this fires — ensures pressure is ready before coupling.
- [x] `ControlSV` (coordinator) — `RosServiceNode<CompressorCmd>` →
      `/hyfleet_compression/compressor_cmd`; ports: `service_name`, `sv_index`, `enable`;
      cmd=CONTROL_SV. Opens/closes interstage SV as an explicit tree action.

#### SYNC XML tree — `sync.xml` (done)

Two-phase Fallback with abort cleanup:

```
Fallback
  Sequence(sync_run)                             ← success path
    Timeout(sync_overall_timeout_ms)
      Parallel(success_count=1, failure_count=1)
        BoostLow(COMPRESS, ON_TARGET_HOLD, interstage_target_bar)  ← Phase 1: hold band
        Sequence(high_stage)
          InterstageAboveBand                    ← wait for PT >= start_threshold
          ControlSV(interstage_sv_index, true)   ← open SV (Phase 2 begins)
          BoostHigh(COMPRESS, ON_TARGET_SUCCEED, INLET_STARVE_PAUSE, target_pressure)
    ControlSV(interstage_sv_index, false)        ← close SV after success
    BoostLow(STOP)
    BoostHigh(STOP)
  ForceFailure
    Sequence(abort_cleanup)
      ControlSV(interstage_sv_index, false)      ← close SV on abort
      BoostLow(STOP)
      BoostHigh(STOP)
```

SV is closed and both boosters are stopped in both success and abort paths.
Interstage SV-driven physics in the sim (not `sync_mode` param) means the sim
responds correctly to coordinator ControlSV calls.

#### SYNC coordinator telemetry

- [x] `CompressorTelemetryCache` — mirrors `BoosterTelemetryCache`; mutex-guarded
      `shared_ptr<const CompressorTelemetry>` + timestamp; in `hyfleet_compressor` namespace
- [x] `CompressorNode` owns `telemetry_sub_` → writes `telemetry_cache_` on each message
- [x] `telemetry_cache_` placed on `shared_blackboard_` at configure; all BT nodes share it
- [x] `CompressorCmd.srv` — `INVALID=0`, `CONTROL_SV=1`, `CONTROL_HEATER=2`; fields:
      `cmd`, `enable`, `index`, `setpoint`; same style as `BoosterCmd`
- [x] Interstage SYNC params: `interstage_target_bar` (280.0), `interstage_start_threshold_bar`
      (200.0), `interstage_starvation_bar` (175.0), `sync_overall_timeout_ms` (3600000)
- [x] All 6 integration tests passing: `test_low`, `test_high`, `test_parallel_both`,
      `test_setpoint_update`, `test_stop`, `test_sync`

---

## Stage 4 — Full integration

### Integration
- [ ] Safety monitor end-to-end
- [ ] DiagnosticsPublisher
- [ ] Groot2 visualisation on both trees
- [-] TreeExecutionServer — optional, not on critical path

### Booster rounding-out (from `compression_module_decisions.md` §8 — post core-control)

- [ ] `oil_healthy` subscription — `RosTopicSubNode` in each booster; fail-safe on stale:
      no fresh message within N s → treat as unhealthy, refuse/stop goal; fake publisher for
      standalone booster tests
- [ ] Compression ratio monitoring — outlet/inlet; per-instance params `ratio_limit` (1:30 low,
      1:60 high); halt-restart corrective on exceed; escalation guard: repeated exceedances in
      sliding window → abort/fault (circuit-breaker)
- [ ] Discharge temperature trend monitoring — rise rate; primary thermal health signal (covers
      over-ratio and degradation); protective over-temp trip is PLC, ROS monitors the trend
- [ ] Stall / no-progress detection — absence-of-rise in smoothed step-per-stroke over window →
      FAILURE; storage-independent; pairs with inlet-starvation gate (inlet < ~0.2 bar → stop);
      replaces the Stage 4 TODO hook in `OutletAtPressure::onRunning()`
- [ ] Averaged pressure rise per stroke — smooth the compression sawtooth; input to stall
      detector; logged, not a runtime gate
- [ ] End-of-run report — extend `ControlBooster` Result: end pressure, kg compressed
      (⚠️ real-gas Z-factor required at 300–900 bar H₂, not ideal gas), energy (kWh),
      specific energy (kWh/kg), peak ratio, cycle count, duration
- [ ] Cumulative counters — hours run (loaded vs idle), PCSV cycles, start count; fault/event
      log (last fault + history); diagnostics/status output, not high-rate telemetry

---

## Stage 5 — Compression test simulator (`hyfleet_sim`)

Full prompt in `ws/src/hyfleet_compressor/docs/Sim_prompt.md`. Stands in for the ADS
bridge + PLC + physical plant at the ROS boundary — booster and coordinator nodes run
against it **unchanged**. Implement using a local AI agent driven by that prompt.

### Discover before writing code (from prompt §0 / §9)
- [x] Confirmed `CompressorTelemetry` field names / array sizes / per-booster PT, TT, VFD,
      SV index mapping — `sim_node.py` declares `<booster>.inlet_pt_index` etc. matching the
      production layout (`pt_bar[16]`, `tt_celsius[12]`, `vfd_*[2]`, `sv[5]`)
- [x] Confirmed `BoosterCmd.srv` — `sim_node.py` implements `START_VFD` / `STOP_VFD` /
      `SET_PCSV` / `CONTROL_SV` against it directly
- [ ] PLC warn/alarm/lockout flags — not yet surfaced by the sim (see Phase 1 below)

### Phase 1 — single booster, synthetic model — DONE (simplified model)
- [x] `hyfleet_sim` Python node (`sim_node.py`): `BoosterCmd` service per booster
      (`/low_booster/booster_cmd`, `/high_booster/booster_cmd`); publishes
      `CompressorTelemetry` at `physics.telemetry_rate_hz` (default 10 Hz);
      `~/reset` `Trigger` service restores initial conditions; physics tick at
      `physics.physics_rate_hz` (default 100 Hz)
- [x] Compression model: `pcsv_enabled AND vfd_running` → `outlet_p += bar_per_cpm * cpm * dt`;
      otherwise pressure holds (no decay). Sufficient to drive and pass all 5 BT
      integration tests end-to-end against `low_booster`/`high_booster`/`compressor_node`
      running **unchanged**
- [ ] Simplified vs. original spec — deferred, not currently needed by any test:
      VFD start/stop is instantaneous (no first-order ramp `τ ≈ 0.5 s`), no sawtooth
      ripple on the compression curve, no pressure decay (`τ ≈ 10 s`) when PCSV is off,
      and no PLC protective layer (over-pressure/over-temp → alarm/lockout + autonomous stop)
- [x] Acceptance (simplified model): `SetPCSV(cpm=N)` rises at `bar_per_cpm * N` bar/s and
      stops at target via `OutletAtPressure`; `StartVFD`/`StopVFD`/`ControlSV` all wired;
      nodes run unchanged against the sim — verified by all 5 passing test scripts

### Phase 2 — both boosters + SYNC
- [x] `BoosterCmd.srv` field rename: unified `speed_rpm`/`cpm` → `setpoint`;
      `sv_index` → `index`. All booster C++ code + sim updated to match.
- [x] `CompressorCmd` service added to sim at `/hyfleet_compression/compressor_cmd`;
      `CONTROL_SV` updates `_interstage_sv` at `compressor.interstage_sv_index` (default 4);
      `CONTROL_HEATER` logged and ACKed (stub).
- [x] Second booster (`/high_booster/booster_cmd`) — both boosters run independently from
      their own command services and own `BoosterState`
- [x] SYNC interstage coupling: driven by live `_interstage_sv` state (not a static param).
      When interstage SV is open: `high_booster.inlet_p = low_booster.outlet_p` each tick.
- [x] SYNC interstage draw: `physics.interstage_draw_bar_per_s` (default 8.0 bar/s) — when
      interstage SV open and high booster PCSV+VFD active, `low_booster.outlet_p` decays at
      that rate (floored at `low_booster.inlet_p`). Drives low booster hold loop without hardware.

### Launch
- [x] `hyfleet_sim/launch/hyfleet_sim.launch.py` — brings up
      `sim_node → low_booster → high_booster → hyfleet_compression → lifecycle_manager`
      (lifecycle manager delayed 2 s); booster/compressor nodes run **unchanged** against
      the sim. Implemented as its own dedicated launch file rather than a `use_sim` arg
      on the production bringup launch — revisit when migrating to the live workspace (Stage 7)
- [x] All model constants declared as ROS parameters (`physics.*`, per-booster `inlet_pressure_bar`,
      `initial_outlet_bar`, `target_pressure_bar`, index maps); no hardcoded values

---

## Stage 6 — LifecycleManager as lifecycle node

Lesson plan Phase 6. Fix SIGINT shutdown ordering: if `LifecycleManager` is itself a
`LifecycleNode`, `ros2 launch` can sequence shutdown — manager runs its shutdown tree
first, then managed nodes stop. This is the Nav2 pattern.

- [ ] Convert `LifecycleManager` to a lifecycle node
- [ ] Shutdown tree: deactivate → cleanup → shutdown in dependency order
- [ ] Verify clean SIGINT: no nodes hard-killed before shutdown tree completes
- [ ] Update bringup launch to manage the manager itself

---

## Stage 7 — Migration to live hyfleet project

Packages `hyfleet_booster`, `hyfleet_compressor`, and `hyfleet_sim` move to the live repo
on a fork branch. `mserve_*` packages do **not** transfer — their content is absorbed or
replaced. `mserve_launch` is rewritten as `hyfleet_launch`. Merge gate: sim passes full
acceptance criteria. ADS bridge integration is parallel work; this stage ends at a
PR ready to merge once that work lands.

### Branch setup
- [ ] Fork live hyfleet repo; create branch `feature/compression-module`
- [ ] Copy `hyfleet_booster`, `hyfleet_compressor`, `hyfleet_sim` into the live workspace
- [ ] Confirm packages build clean before any further changes

### Interface migration (mserve_interfaces content → live interfaces package)
- [ ] Absorb `CompressorTelemetry.msg`, `BoosterCmd.srv`, `ControlBooster.action`,
      `ControlCompressor.action` into the live project's interfaces package
- [ ] Confirm PLC warn/alarm/lockout flags are present in `CompressorTelemetry.msg`;
      add if not (per decision record §2)
- [ ] Update all `#include` and `find_package` / `rosidl` references in booster/compressor
      packages to point at the live interfaces package
- [ ] Build clean; `ros2 interface show` each message/service/action

### Utils migration (mserve_utils → live equivalents)
- [ ] Identify live project's equivalent of `get_or_declare_param`,
      `make_int_range_descriptor`, `make_double_range_descriptor`; add any that are missing
- [ ] Update all `#include "mserve_utils/..."` in booster/compressor code to use live utils
- [ ] Remove `mserve_utils` from `package.xml` dependencies; build clean

### Launch — rewrite mserve_launch as hyfleet_launch
- [ ] Create `hyfleet_launch` package; rewrite launch to bring up:
      `base → drivechain → low_booster → high_booster → hyfleet_compression`
- [ ] Per-instance param YAML files for `low_booster` and `high_booster` (hardware index
      mapping confirmed from ADS bridge / wiring docs)
- [ ] `use_sim` arg: when true, launch `hyfleet_sim` in place of ADS bridge
- [ ] No "using default" warnings at startup — all declared params covered

### Sim gate (merge criterion)
- [ ] Full START → hold → STOP cycle, both boosters, via `hyfleet_sim` — all Stage 2 and
      Stage 5 acceptance criteria pass in the live workspace
- [ ] SYNC mode: both boosters, interstage coupling
- [ ] Clean shutdown: SIGINT, no nodes hard-killed mid-tree

### PR
- [ ] No regressions in existing live bringup (non-compression nodes unaffected)
- [ ] PR raised; merge dependency noted: ADS bridge work must land first

---

## Known gaps / deferred

- Config loading — currently hardcoded defaults
- Shutdown tree not yet updated for boosters
