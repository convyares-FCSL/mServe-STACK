# Compression Module — Decision Summary

*Handoff record for README update + implementation. Tags: **CLOSED** = decided. Core decisions (§1–§7) closed; §8 (monitoring/reporting) is captured scope with a few items to confirm.*

## Reusable principles (apply these to resolve future questions consistently)

- **Signals, not tentacles.** Cross-node interaction is ROS actions/services/topics only — never one node calling another's methods, never a shared blackboard. Necessary physical coupling (SYNC interstage, shared oil) is expressed as observable interfaces; accidental coupling is removed.
- **Fact vs instruction.** A *fact about a node* (index, limits, timing) is a parameter; a *value that's part of an instruction* (a setpoint, a mode) travels with the goal/command.
- **Per-instance vs universal fact.** A fact that *differs per instance* → config parameter. A fact *universal to the hardware type and immutable* → hardcoded `constexpr`.
- **Command, not param, for actuator setpoints.** Anything that moves hardware arrives through a validated, acknowledged, traceable command — never a fire-and-forget `param set`.
- **Fail-safe on stale.** A safety/permission signal that is absent or stale reads as *unsafe*, not "last known good".
- **Share a field only when it's one honest abstraction.** Merge when the meaning is identical (on/off); split when merging would hide a unit difference.
- **Real-time by cycle time, not by "control vs supervision."** Loops faster than ~100 ms (hard real-time — VFD ramp at 5 ms, fast interlocks) execute in the PLC. Loops at ~100 ms and slower live in ROS 2, including genuine closed-loop control — pressure, CPM trim, heating (~1 Hz), dispensing (~2 Hz). ROS 2 closes the slow loops; it just never owns a sub-100 ms one.

## Control authority — three layers (by what each protects)

The master allocation rule. Pick the authority by what is at stake; the principles above sit under it.

| Layer | Protects | Owner | Acts on (examples) | Recovery |
|---|---|---|---|---|
| **Process control** | the operating envelope / reaching + holding spec | **ROS 2** | heater regulation (~1 Hz), operating-range gates (e.g. oil 0–50 °C), pressure & CPM control, **ratio control** (halt-restart + escalation) | auto / recoverable corrective |
| **Machine control** | the equipment | **PLC** | device lockout (e.g. oil < -10 °C), **over-pressure & over-temperature protective trips**, all sub-100 ms loops incl. VFD ramp | latched — requires clearing |
| **Health control** | humans | **Safety PLC** (separate device) | personnel / functional safety | per the safety system |

Rules within this model:
- **Pick the authority by what's at stake:** harmless-but-invalid operating point → ROS; equipment damage → PLC; human harm → safety PLC.
- **Cycle-time override:** process control needing sub-100 ms determinism (VFD ramp, 5 ms) executes in the PLC — ROS commands the setpoint, the PLC runs the loop.
- **Protective layers act first, inform ROS after** (machine PLC and safety PLC). They react autonomously at their own setpoints, then report. Equipment/personnel safety never depends on ROS being up. Information flows protective-layer → ROS (notification); ROS reacts to *declared* state, never re-derives danger from raw values.
- **Recoverability differs:** ROS correctives (ratio halt-restart, operating-range gate) auto-recover; a PLC device lockout is latched; a safety-PLC trip sits above both.

---

## 1. Architecture — CLOSED

- Nav2-style: coordinator + capability nodes communicating over ROS interfaces only.
- Coordinator **and** boosters are **lifecycle nodes**, each owning its own `rclcpp_action` server and ticking an **internal BT tree** (the BT-Navigator pattern).
- **No `TreeExecutionServer`** — it isn't a lifecycle node, and lifecycle management is required. Optional/future only, if external Groot2 monitoring is wanted.
- behaviortree_ros2 used in full: `RosActionNode`, `RosTopicSubNode`, `RosServiceNode` inside the trees. Only the top-level `TreeExecutionServer` wrapper is dropped.
- One booster binary `hyfleet_booster`, launched twice (`low_booster`, `high_booster`) with different config files. No duplication.
- Each node's blackboard is **local**; no cross-node shared blackboard.
- The booster BT commands setpoints, waits on feedback/telemetry conditions, and closes the slow control loops (pressure, CPM trim, ratio). Sub-100 ms timing and protective trips are not ROS's — see Control authority.
- Bringup order: `base → drivechain → low_booster → high_booster → hyfleet_compression` (coordinator last).
- Coordinator: pkg `hyfleet_compressor`, exe `compressor_node`, node name `hyfleet_compression`.

## 2. Telemetry — CLOSED

- One `CompressorTelemetry` message for the whole subsystem, published by the ADS bridge at ~10 Hz (shared sensors such as the interstage PT make a single topic the right call).
- Both boosters subscribe and extract their slice by **config-driven array index**.
- The telemetry/status carries the **PLC's per-device warn / alarm / device-lockout flags and mode**, not just raw sensor values. ROS conditions key off the *declared* state.
- BT condition nodes take the array index as a **port**, so the same C++ node type serves both boosters; index read from config, written to blackboard in `on_configure`.
- ⚠️ Message renamed (e.g. `hbu_pt_bar` → `pt_bar`; vfd field names changed). **README `CompressorTelemetry` section is stale — update field names** and add the PLC status flags. Code fix: `booster_bt_conditions.cpp` lines 52/88/122, `last_msg->hbu_pt_bar` → `last_msg->pt_bar`.

## 3. Oil, temperature & the three layers — CLOSED

- **ROS (process):** heater regulation (~1 Hz) and the **operating-range gate** — won't start/continue if oil is outside band (e.g. <0 or >50 °C). ROS verdict published as `oil_healthy`; an operational gate, auto-recovering when back in range.
- **PLC (machine):** equipment-protection thresholds (e.g. oil < -10 °C) → device alarm/fault → sub-system **lockout**, warning displayed, tells ROS after. Latched. BT reflects the lockout flag → fault/lockout, refuses goals until cleared.
- **Safety PLC (health):** personnel safety, separate device, above both.
- Boosters subscribe to `oil_healthy` and inhibit own commanding. **Fail-safe on stale:** stamp it; "no fresh `oil_healthy` within N" → not good to run. Plant protected by PLC/safety PLC regardless.
- **Two independent stop sources:** PLC autonomous trip/lockout (authoritative) vs ROS force-stop (a *request* via the command path).
- Keep a fake status/`oil_healthy` publisher for standalone booster tests.

## 4. Booster command service — `BoosterCmd.srv` — CLOSED

- PLC contract (`id`/`cmd`/`payload[1..5]`/`ackID`/`result`, TwinCAT) is fixed hardware, **never exposed in ROS**. The **ADS bridge is the only encoder/decoder**.
- **Single multiplexed service**, `cmd` enum + **named typed fields** (mirrors the action's command enum; matches the PLC's serial-per-domain ack model).
- `START_VFD`/`STOP_VFD` kept **discrete** (stop must be unambiguous; 1:1 with PLC E_Cmd).
- `device_id` is a **string** (new solenoids need no interface change). Service **namespaced per booster** → no `id` field.

```
# srv/BoosterCmd.srv
uint8 START_VFD   = 1
uint8 STOP_VFD    = 2
uint8 SET_PCSV    = 3
uint8 CONTROL_SV  = 4

uint8 cmd

float64 speed_rpm     # START_VFD — VFD speed (rpm)
float64 cpm           # SET_PCSV  — compression rate (cpm)
bool    enable        # SET_PCSV / CONTROL_SV — energise the targeted device
string  device_id     # CONTROL_SV — which valve; bridge owns the lookup
---
bool    success
string  message
```

Register in `mserve_interfaces/CMakeLists.txt` under `rosidl_generate_interfaces`.

## 5. Parameters & limits — CLOSED

- Declare in **constructor**; read in `on_configure` (`load_params()`); `on_parameters` = **reject while ACTIVE, accept otherwise**. No live-setpoint path; no `apply_runtime_params()`.
- **No `read_only=true` descriptors.**
- **Limit classification — differs per instance → param; universal + immutable → `constexpr`:**
  - **`constexpr` (`booster_limits.hpp`):** speed 0–1600 rpm, CPM 0–18 (confirmed, identical both boosters). ROS-side command-validation ceilings; PLC enforces its own protective pressure/temperature limits independently.
  - **Param (per-instance):** pressure limits (low vs high), index/mapping params, operating-range bands (oil 0–50 °C), **ratio limits (1:30 low / 1:60 high — ROS operational limits driving the halt-restart corrective, not PLC)**, operational tolerances.
- Descriptor ranges on remaining params; hardcoded ceiling = natural descriptor bound. `min/max_speed_rpm` params removed → constants.
- **Timing split by cycle time.** Sub-100 ms (VFD ramp) → PLC, no ROS ramp params. ROS owns slower loops + supervisory timeouts.
- **Setpoints (cpm/speed) are NOT params** — coordinator-decided, arrive with the goal, validated **on goal-accept**.
- **Validation chain:** setpoint (goal) → goal-accept check → limit (`constexpr` or per-instance member).
- Log the `constexpr` limits at `on_configure` for visibility.

```
constructor   → declare_params()   (plain declare + descriptor ranges; no read_only)
on_configure  → load_params()      (read params; store per-instance limits as members; write rest to blackboard)
on_parameters → if state == ACTIVE: reject; else accept
```

## 6. Operating mode & setpoint decision — CLOSED

- Orchestrator sends `mode` (performance/eco) — a goal field on `ControlCompressor`. **Not** raw CPM.
- Coordinator owns mode → setpoint translation (**profiles**); trims CPM live in SYNC from interstage pressure.
- **Profiles** (per-mode CPM + speed numbers) are **coordinator params** (commissioning-tunable, reject-while-active).
- **CPM channel = (a):** initial CPM in the `ControlBooster` goal; live trims via the adjust-CPM service.
- **Speed mirrors CPM:** `speed_rpm` is a coordinator-decided setpoint in the goal; default **1500 rpm** as a coordinator profile/default param; dynamic later for efficiency.
- **Decision/execution split:** coordinator decides cpm + speed; booster generates the actual `SetPCSV`/`StartVFD` command; PLC ramps the VFD (5 ms).

## 7. Action interfaces — CLOSED

`ControlCompressor` (orchestrator → coordinator):
```
command:         START / STOP / SAFE_STOP
target:          LOW / HIGH / SYNC
target_pressure: bar (validated vs per-target limit on goal-accept)
mode:            performance / eco
```
`ControlBooster` (coordinator → booster):
```
command:         START / START_IDLE / STOP / SAFE_STOP
target_pressure: bar
cpm:             coordinator's decision   # initial in goal; live trims via adjust-CPM service
speed_rpm:       coordinator's decision   # default 1500 (coordinator profile param); dynamic later
Result:          run summary (see §8)     # extended from {accepted, message}
```
- `START_IDLE` = boost to target then hold (poppet engaged, VFD ready) — SYNC, avoids cold-starting the low booster.
- `target_pressure` goal-only, never a param. Booster validates `cpm`/`speed_rpm` against the `constexpr` limits on goal-accept.

## 8. Booster monitoring, diagnostics & reporting — SCOPE (post core-control)

Built after the core control path runs end-to-end. Classified so each lands in the right home.

**Live derived process metrics (ROS computes from telemetry; ride in action `Feedback` / a diagnostics output):**
- **Compression ratio** = outlet/inlet. **ROS process control**, not a PLC limit. Per-instance limit param: **1:30 low, 1:60 high**. On exceed → **halt then restart** (recoverable corrective, not abort); **escalation guard** — repeated exceedances within window T → abort/fault (circuit-breaker: count occurrences in a sliding window). The PLC protects absolute **pressure** (over-pressure), not ratio. Ratio limits also inform the **SYNC interstage window** (coordinator holds interstage so neither stage over-ratios).
- **Output (discharge) temperature + rise rate** — primary thermal monitoring/health signal (covers over-ratio *and* degradation in one reading; VFD current is not instrumented). The protective over-temp **trip** sits with the PLC (equipment, alongside over-pressure); ROS monitors the trend.
- **Averaged pressure rise per stroke** — a smoothing device: averages the sawtooth into a clean rise-rate. For monitoring/logging (**not** a runtime health gate), and the input to the stall detector.
- **Stall / no-progress detection** — while compressing and not yet at target, if the smoothed rise-per-stroke sits at ≈0 over a window → stall (leak / no gas / stuck valve). Tests for the **absence** of rise, not an insufficient *rate*, so it is **storage-independent** (expected rate depends on the storage volume; presence-of-any-rise does not). Near-target flattening is the success condition, not a stall. Pairs with inlet-starvation (inlet < ~0.2 bar → stop).
- **CPM achieved vs commanded; VFD speed achieved vs commanded.**
- **Specific energy (kWh/kg)** — the eco-mode KPI; live and in the report.
- **Post-stop leak-down rate** — pressure decay after PCSV stops; a passive integrity check.

**Predictive / maintenance (machine; PLC retentive counters canonical, ROS reads + reports):**
- **Hours run** — split *loaded* (PCSV active) vs *idle* (VFD spinning).
- **Total PCSV cycles** (better wear proxy than hours for the reciprocating element), **start count**.

**End-of-run report (ROS, at completion; carried in the action `Result`):**
- End pressure, **kg compressed** (⚠️ **real-gas / Z-factor calc — not ideal gas** at 300–900 bar H₂), energy used, **specific energy (kWh/kg)**, peak ratio, cycle count, duration.

**Fault / event log:** last fault + history (what, when, which layer raised it). Feeds the operator console / lockout reason.

**Interface implications:**
- Extend `ControlBooster` (and `ControlCompressor`) action `Result` to carry the run summary.
- Live derived metrics → action `Feedback`. Cumulative counters + health flags → a diagnostics/status output, **not** the high-rate telemetry topic.
- Ratio limits → per-instance **ROS** params (descriptor range).

**To confirm:** (1) cumulative counters (hours/cycles/starts) — PLC retentive or ROS-tracked? (2) source for the H₂ real-gas property model (Z-factor / EOS). (3) over-temperature protective trip assumed PLC (equipment) — confirm.

---

## Decision history (most recent rounds)

1. **CPM channel** → option (a): initial CPM in the goal, live trims via the adjust-CPM service.
2. **`speed_rpm`** → coordinator-decided setpoint (mirrors CPM); default 1500 rpm; dynamic later.
3. **Limits confirmed** → speed 0–1600 rpm, CPM 0–18.
4. **Real-time boundary** → cycle-time rule (~100 ms): sub-100 ms → PLC; ~100 ms and slower → ROS 2 closed-loop.
5. **Protective layers act first, inform ROS after**; ROS reacts to declared state; ROS safety supervisory and timing-relaxed.
6. **Three-layer control authority** → ROS = process; PLC = machine (latched lockout, over-pressure/temperature trips); Safety PLC = health (separate device).
7. **Monitoring & reporting scope (§8), with corrections:** ratio = **ROS** process control (halt-restart + escalation, *not* a PLC limit; PLC protects pressure); step-per-stroke = logged + averaged-as-smoothing (not a health gate); stall = **absence-of-rise** test (storage-independent); predictive thermal signal = **output temperature** (VFD current not instrumented); run report incl. kg via real-gas, kWh/kg. Built after core control.

## Immediate implementation steps

1. `booster_limits.hpp` — `constexpr` speed 0–1600 rpm, CPM 0–18.
2. Create `srv/BoosterCmd.srv` (§4); register in `mserve_interfaces/CMakeLists.txt`; build; `ros2 interface show mserve_interfaces/srv/BoosterCmd`.
3. Add `mode` to `ControlCompressor.action`; add `cpm` + `speed_rpm` to `ControlBooster.action`. Coordinator holds `default_speed_rpm` (1500) and per-mode profile CPM.
4. Confirm `CompressorTelemetry` carries the PLC warn/alarm/lockout flags per device; fix `booster_bt_conditions.cpp` lines 52/88/122 (`hbu_pt_bar` → `pt_bar`); build clean.
5. Update README: telemetry field names + PLC status flags; three-layer control-authority model; `oil_healthy` operational-gate framing + fail-safe-on-stale; real-time-placement rule; record §4–§8.
6. `BoosterNode` param plumbing: `declare_params()` (constructor, descriptor ranges), `load_params()` (`on_configure`), trivial `on_parameters`.
7. First BT node: `StartVFD` as `RosServiceNode<BoosterCmd>` — `setRequest()` fills `cmd=START_VFD` + `speed_rpm` from blackboard; tree waits on at-speed feedback (PLC ramps); `onResponseReceived()` → SUCCESS on `success == true` (accepted, not yet at speed); service name from per-instance config.
8. (Post-core) Build out §8 monitoring/diagnostics/report once the control path runs end-to-end.
