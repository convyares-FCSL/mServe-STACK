# Compressor Module — Build Todo

See `architecture.md` for the full design rationale.

## Legend
- [x] Done
- [ ] Todo
- [-] Skipped / deferred

---

## Stage 1 — Booster nodes

Goal: each booster as an independent lifecycle node with its own action server and
simple state machine. No BT yet. Tested standalone before coordinator exists.

### New package needed
- [ ] `hyfleet_booster` package — single `BoosterNode` lifecycle node
- [ ] Launched twice: once as `low_booster`, once as `high_booster`
- [ ] Config file drives all differences (pressure limits, topics, etc.)
- [ ] Add both instances to lifecycle manager bringup/shutdown trees
- [ ] Add both instances to launch file (`low_booster_config.yaml` / `high_booster_config.yaml`)

### Per booster node
- [ ] Lifecycle callbacks: configure / activate / deactivate / cleanup / shutdown
- [ ] Action server: accepts `ControlCompressor` goal for its own booster only
- [ ] Simple state machine: IDLE → STARTING → RUNNING → STOPPING → FAULT
- [ ] Telemetry subscription — interprets its own sensor data
- [ ] Command generation — writes hardware commands to `/cmd_sender`
- [ ] Publishes own status (pressure, active, target_reached, state)

### CmdSender + Telemetry boundaries
- [ ] Define `/cmd_sender` interface (service or topic — TBD)
- [ ] Define `/telemetry` topic structure for each booster

---

## Stage 2 — Coordinator

Goal: CompressorCoordinator with TreeExecutionServer routing goals to booster nodes.

### `hyfleet_compressor` package — rework
- [ ] Replace hand-rolled action server with `TreeExecutionServer`
- [ ] BT tree: route goal based on target (LOW / HIGH / SYNC)
- [ ] `RosActionNode` wrapping `/low_booster/control`
- [ ] `RosActionNode` wrapping `/high_booster/control`
- [ ] SYNC subtree: Parallel node calling both booster RosActionNodes
- [ ] Reactive stop: Parallel + ForceStopRequested blackboard condition
- [ ] SYNC active control: interstage pressure topic condition + CPM adjustment service call
- [ ] Interstage solenoid service call node

### Current action server code — disposition
- [ ] `compressor_action.*` — remove, replaced by TreeExecutionServer
- [ ] `compressor_node.*` — rework to coordinator role only
- [ ] Keep `compressor_config.hpp`, `compressor_types.hpp` — still needed

---

## Stage 3 — BT migration (lesson plan)

Unblocked once Stage 2 is complete. See `lesson_plan_btros2.md`.

- [ ] Phase 1: `RosActionNode` — coordinator BT calling booster action servers (done in Stage 2)
- [ ] Phase 2: `RosTopicSubNode` — interstage pressure, booster safety conditions
- [ ] Phase 3: Reactive tree — force stop pattern, safety monitor Parallel nodes
- [ ] Phase 4: Blackboard — goal routing, force stop flag, interstage pressure keys
- [ ] Migrate LowBooster state machine → BT tree
- [ ] Migrate HighBooster state machine → BT tree
- [ ] Phase 5: `TreeExecutionServer` — coordinator wrapping full BT (done in Stage 2)

---

## Stage 4 — Full integration

- [ ] SYNC CPM active control fully implemented
- [ ] Interstage solenoid management
- [ ] Safety monitor wired (telemetry timeout, oil temp/level, e-stop)
- [ ] End-to-end test: orchestrator → coordinator → both boosters → hardware
- [ ] Groot2 visualisation confirmed working

---

## Known gaps / decisions deferred

- `/cmd_sender` interface not yet defined
- `/telemetry` topic structure not yet defined
- Config loading — currently hardcoded defaults
- Safety monitor scope — telemetry timeout, oil health, e-stop
- `DiagnosticsPublisher` — deferred to Stage 3+
