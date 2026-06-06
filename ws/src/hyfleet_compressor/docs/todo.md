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
- [x] `start_tree.xml` — Parallel(success_count=1, failure_count=1):
      Sequence: ControlSV(inlet) → InletPressureStable → StartVFD →
      Sleep(vfd_delay_ms) → VFDAtSpeed → Sleep(vfd_stabilization_ms) →
      ControlSV(HPU) → SetPCSV(enable) → OutletAtPressure
      ∥ Safety: InletPressureSafe
- [x] `start_idle_tree.xml` — same as start + AlwaysRunning (holds at target indefinitely)
- [x] `stop_tree.xml` — Sequence: SetPCSV(disable) → ControlSV(HPU,close) →
      StopVFD → VFDStopped → ControlSV(inlet,close)
- [x] `stop_force_tree.xml` — Sequence: SetPCSV(disable) → ControlSV(HPU,close) →
      StopVFD → ControlSV(inlet,close) (no ramp-down wait)

### Feedback
- [x] Publish outlet pressure + percent_complete to action feedback each tick (from telemetry cache)

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

**SYNC dependencies — note now, implement at Stage 3 SYNC:**
- [ ] `reenable_offset_bar` (default 50.0 bar) — operational param added; not a goal field.
      Re-enable threshold = `target_pressure − reenable_offset_bar`. Written to blackboard.
- [ ] `start_idle_tree.xml` — replace `AlwaysRunning` with correct maintain loop:
      Phase 1 (startup, same as START): OFF → COMPRESSING → `OutletAtPressure`
      Transition: `SetPCSV(off)` → WARM
      Phase 2 (maintain loop): `PressureBelowThreshold(outlet, target−offset)` → `SetPCSV(on)` →
      `OutletAtPressure(target)` → `SetPCSV(off)` → repeat
      Holds in WARM between re-engagements. STOP exits from wherever it is → OFF.
      Note: `PressureBelowThreshold` reused here with `outlet_pt_index` and computed threshold.

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
- [ ] SYNC band semantics confirmed: "halt low at 280 bar" = `START_IDLE` (VFD holds ready,
      no ramp/stabilise cost on re-enable at 220 bar), not a cold `STOP`. START_IDLE
      exists exactly for this — band-control must use the idle/hold path.

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

- [x] `parallel_low.xml`  — `BoostLow(action_name={low_booster_action}, command={command}, ...)`
- [x] `parallel_high.xml` — `BoostHigh(action_name={high_booster_action}, command={command}, ...)`
- [ ] `sync.xml`          — stub; see SYNC section below

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

### Mode — cleanup item

- [ ] `mode` is currently a per-goal field on `ControlCompressor.action`. This should be
      a stored state owned by `CompressorNode`, changeable via a `~/set_mode` service.
      Per-goal mode forces clients to track state; a service allows mode to be set once
      and persist across fills. Remove `mode` from action once service is in place.

### Force stop and recovery

- [ ] Force stop: cancel in-flight booster goals via ROS action cancel protocol
- [ ] Recovery Fallback: graceful stop → force stop path in XML tree

---

### SYNC mode — reactive interstage coordination

Unblocked after PARALLEL paths work and booster preemption semantics are confirmed.
SYNC is asymmetric: low is slaved to high, not a symmetric Parallel of equal goals.

#### SYNC interstage BT nodes

- [ ] `InterstageAboveBand` — `ConditionNode`; interstage >= upper threshold (280 bar) →
      SUCCESS (signal to idle low); reads `interstage_pt_index` from blackboard
- [ ] `InterstateBelowBand` — `ConditionNode`; interstage <= lower threshold (220 bar) →
      SUCCESS (signal to re-enable low)
- [ ] Both read interstage from the compressor's telemetry subscription; coordinator needs
      its own subscription to `compressor_telemetry` for interstage PT only

#### SYNC XML tree

`start_sync_tree.xml` — asymmetric, three-phase:
- [ ] Phase 1: `BoostLow(START_IDLE)` — low runs until interstage >= target
- [ ] Phase 2: `BoostHigh(START)` — bring high booster online
- [ ] Phase 3: `ReactiveSequence` — re-evaluates interstage condition every tick while
      high is running; `InterstageAboveBand` → idle low; `InterstateBelowBand` → re-enable
      low via `BoostLow(START_IDLE)`. Uses `ReactiveSequence` not `Sequence` so the
      condition gates are re-checked each tick even while a child RosActionNode is RUNNING.
- [ ] `stop_sync_tree.xml`      — stop high first (dependency order), then low
- [ ] `safe_stop_sync_tree.xml` — halt both immediately

#### SYNC feedback

- [ ] Single goal handle; feedback = `min(low_pressure, high_pressure)` during Phase 1;
      `high_pressure` once high is online; `percent_complete` vs `target_pressure`

#### SYNC coordinator telemetry

- [ ] Coordinator subscribes to `compressor_telemetry` for interstage PT only;
      lightweight — no cache needed, just the latest interstage value on the blackboard
- [ ] Interstage solenoid command — `RosServiceNode` to `BoosterCmd` for SV control
- [ ] CPM adjustment on low booster — `RosServiceNode` if dynamic CPM tuning needed

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
- [ ] Confirm `CompressorTelemetry` field names, array sizes, and PLC warn/alarm/lockout flags;
      add flags to message if missing (per decision record §2)
- [ ] Confirm `BoosterCmd.srv` exists with `device_id: string`; create per decision record §4 if not
- [ ] Confirm per-booster index mapping (vfd, PT, TT) and CSV channel → telemetry field mapping
- [ ] Locate `C_12_1500_60_30_10.csv` in repo; confirm its path

### Phase 1 — single booster, synthetic model
- [ ] `hyfleet_sim` Python node: `BoosterCmd` service (`/low_booster/booster_cmd`); publishes
      `CompressorTelemetry` at ~10 Hz with physics model from prompt §5
- [ ] VFD: first-order ramp `τ ≈ 0.5 s`; compression: `K ≈ 1 bar/s` with sawtooth ripple;
      decay: `τ ≈ 10 s` after PCSV off
- [ ] PLC protective layer: over-pressure / over-temp → set alarm/lockout flag, stop
      compression autonomously (act-first, then report — matches real PLC behaviour)
- [ ] Acceptance: `StartVFD(1500)` ramps in ~0.5 s; `SetPCSV(cpm=10)` rises ~1 bar/s with
      sawtooth; stops at target; `SetPCSV(false)` decays τ ≈ 10 s; nodes run unchanged

### Phase 2 — both boosters + SYNC
- [ ] Second booster (`/high_booster/booster_cmd`); SYNC interstage coupling: high inlet = low outlet
- [ ] Both boosters run independently from their own command services

### CSV replay mode
- [ ] Parse `C_12_1500_60_30_10.csv` (TwinCAT paired-column format, Windows FILETIME timestamps);
      resample to 10 Hz; publish as `CompressorTelemetry`
- [ ] Reproduces HBU 2 outlet trace: 262 → 592 → ~488 bar

### Launch
- [ ] Launch file with `use_sim` arg so existing bringup runs against sim without changes
- [ ] All model constants as ROS parameters; no hardcoded values

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
