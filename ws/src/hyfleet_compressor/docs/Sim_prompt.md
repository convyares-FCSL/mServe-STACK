# Implementation prompt — Compression Test Simulator (`hyfleet_sim`)

You are a coding agent working in the **mServe-STACK** ROS 2 (Jazzy) workspace. Build a
**test simulator** for the HyFleet compression module. This document gives you the design,
the physics grounded in a real prototype run, and acceptance criteria. Some specifics
(exact message field names, service names, config indices, repo paths) you **must discover
from the repo or ask for** — they are marked **[DISCOVER]** / **[ASK]** below. **Do not
guess interface field names.** If something marked [DISCOVER] cannot be found, stop and ask
rather than inventing it.

---

## 0. Read these first (authoritative; in this order)

1. **`compression_module_decisions.md`** — the module decision record. It defines the
   architecture, the interface contract, the three-layer control authority (ROS = process,
   PLC = machine, safety PLC = human), and that telemetry carries the PLC warn/alarm/lockout
   flags. Locate it in the repo; if it is not present, **[ASK]** for it before proceeding.
2. **`mserve_interfaces/msg/CompressorTelemetry.msg`** — the exact telemetry field names,
   array sizes, and types. **[DISCOVER]** — the message was recently renamed (e.g.
   `hbu_pt_bar` → `pt_bar`). Read the real file; do not assume the layout.
3. **`mserve_interfaces/srv/BoosterCmd.srv`** — the booster command service. If it does not
   yet exist, create it per §4 of the decision record (`cmd` enum: `START_VFD`, `STOP_VFD`,
   `SET_PCSV`, `CONTROL_SV`; fields `speed_rpm`, `cpm`, `enable`, `device_id`; response
   `success`, `message`) and build it. **[DISCOVER/CREATE]**
4. **`mserve_interfaces/action/ControlBooster.action`** and **`ControlCompressor.action`** —
   command semantics (START / START_IDLE / STOP / SAFE_STOP; target_pressure; cpm; speed_rpm).
   **[DISCOVER]**
5. **Per-booster config** (the YAML/params that set `vfd_index`, PT/TT indices, device IDs
   for `low_booster` vs `high_booster`). The simulator must publish telemetry into the **same
   array indices** the booster nodes read. **[DISCOVER]**
6. **The prototype run CSV** `C_12_1500_60_30_10.csv` — real data the model is tuned to
   (format and contents described in §6). Confirm its location in the repo. **[DISCOVER]**

After reading 1–6, briefly confirm back what you found (telemetry fields, service name,
index mapping) and **ask about anything still unresolved before writing code.**

---

## 1. What you are building (the boundary)

A simulator that **stands in for the ADS bridge + PLC + physical plant**. It plugs in at
exactly the ROS boundary the real bridge occupies, so the **booster and coordinator nodes
run against it completely unchanged**. You do **not** modify the nodes under test.

```
  [ booster / coordinator nodes ]   ← unchanged
        │  BoosterCmd service calls            ▲  CompressorTelemetry
        ▼                                      │
  [ hyfleet_sim ]   == ADS bridge + PLC + plant (simulated)
```

Suggested package name `hyfleet_sim` (it simulates the hyfleet compression hardware) — but
follow the repo's existing package conventions; **[ASK]** if unsure. The sim is a plain
node, **not** a lifecycle node (it is the "hardware", not a managed node).

---

## 2. Responsibilities

1. **Advertise the `BoosterCmd` service per booster** at the same names the booster nodes
   call (e.g. `/low_booster/...`, `/high_booster/...`). **[DISCOVER]** the exact service name
   and namespacing from the booster node / decision record. Handle the `cmd` enum:
   - `START_VFD` → set that booster's target VFD speed to `speed_rpm`.
   - `STOP_VFD` → target speed 0.
   - `SET_PCSV` → `enable` true starts compression at `cpm`; false stops it.
   - `CONTROL_SV` → set the named `device_id` valve open/closed.
   - Always return `success` / `message` (the ack the bridge would give).
2. **Publish `CompressorTelemetry`** at **~10 Hz** with the indexed arrays (`pt_bar[...]`,
   `tt_celsius[...]`, vfd speeds — exact names **[DISCOVER]**) populated from the model, into
   the indices each booster expects. **Also populate the PLC warn / alarm / device-lockout
   flags.** If the current message has no such flag fields, **[ASK]** whether to extend the
   message (per the decision record) or stub them.
3. **Run a simple physics model** (§5) that integrates commands into realistic dynamics.

---

## 3. Phasing (build incrementally; validate each before the next)

- **Phase 1 — single booster, closed loop.** One booster, synthetic model. Enough to test
  `StartVFD` and `SetPCSV` end-to-end: command in → speed/pressure respond in telemetry.
- **Phase 2 — both boosters + SYNC.** Add the second booster and the interstage coupling
  (low booster outlet feeds high booster inlet in series/SYNC mode).
- **CSV replay mode** (can be done any time after Phase 1) — see §7.

---

## 4. Tech choices

- **Python (`rclpy`)** — this is a test tool; iteration speed matters and the language need
  not match the C++ stack.
- **Single node.** A ~100 Hz timer steps the physics; a 10 Hz timer publishes telemetry;
  service callbacks just update the commanded state.
- **All model constants are ROS parameters** (gains, time constants, storage volume, ripple
  amplitude, protective limits, per-booster indices) with defaults from §5/§6. Do **not**
  hardcode them — match the project's parameter discipline.
- Provide a **launch file** that brings up the sim in place of the ADS bridge, so the
  existing booster/coordinator bringup runs against it (e.g. a `use_sim` arg or a dedicated
  `sim_bringup` launch). **[DISCOVER]** how the real bringup/launch is structured and mirror it.

---

## 5. Physics model (synthetic mode) — tuned to the real run

Keep it first-order. No real-gas EOS needed for a test rig. Per booster, maintain
`{target_speed, speed, pcsv_enabled, cpm, outlet_p, inlet_p, valves, discharge_temp}` plus
shared `{oil_temp, interstage_p}`.

- **VFD speed:** `speed += (target_speed - speed) * dt / tau_speed`, `tau_speed ≈ 0.5 s`.
  (Do not reproduce the PLC's 5 ms ramp — just the envelope.)
- **Compression:** while `pcsv_enabled` and `speed` is up,
  `dP/dt = K * (speed/1500) * (cpm/10) * (inlet_p/265)` with **`K ≈ 1.0 bar/s`**
  (straight from the run). Add a **sawtooth ripple** of a few bar at the stroke frequency
  (`2 × cpm/60` Hz — double-acting) for realism. Stop rising at `target_pressure`.
- **Decay (PCSV off):** `outlet_p += (settle_p - outlet_p) * dt / tau_decay`,
  **`tau_decay ≈ 10 s`**; `settle_p` slightly below the peak (model the observed settle).
- **Inlet:** constant supply for a standalone booster (default ~265 bar). In **SYNC**, set the
  high booster's `inlet_p` from the low booster's `outlet_p` (the one real coupling).
- **Discharge temp:** rises with compression work, relaxes toward ambient when idle — enough
  to exercise output-temperature monitoring.
- **Optional PLC protective layer** (recommended for testing supervisory paths): if
  `outlet_p` exceeds an over-pressure limit or `discharge_temp`/`oil_temp` exceeds a limit,
  set the alarm/lockout flag in telemetry **and stop compression autonomously** (act-first),
  then keep reporting the flag — this is how the real PLC behaves.

---

## 6. Reference data — `C_12_1500_60_30_10.csv` (validation targets + replay source)

This run is the **high booster (HBU 2)** driven on its own. Use these as both the model
tuning targets and the replay source.

**Behaviour to reproduce (synthetic) / expect (replay):**
- Compression active ~84 → 393 s (~5 min); VFD held **~1500 rpm**.
- Outlet **262 → 592 bar at ~1 bar/s**, riding a sawtooth; inlet ~**265 bar** (200–296);
  peak ratio ~**2.2** (the `60`/`30` in the filename are configured ratio *limits*, not reached).
- PCSV reads **±10** (signed = the two strokes of a double-acting cylinder; 10 cpm) →
  ~6 bar/cycle, ~3 bar/stroke.
- After PCSV stops, outlet **decays 592 → ~488 bar, τ ≈ 10 s**.

**CSV format (TwinCAT export — parse carefully):**
- Metadata rows first (`Name`, `File`, `Starttime`, `Endtime`, blanks); then a channel-name
  row (`Name,<chan>,Name,<chan>,...`), a `SymbolComment` row, a `Data-Type` row, then data.
- Data rows are **paired columns**: `timestamp,value,timestamp,value,...` — each channel has
  its own timestamp. Skip any row whose first cell is not an integer.
- Timestamps are **Windows FILETIME** (100 ns ticks); seconds = `value * 1e-7`. Sample rate
  ~125 Hz in the export (your telemetry republishes at ~10 Hz).
- **24 channels, in order:** `HBU 1 - Inlet`, `HBU 1 - Outlet`, `HBU 2 - Inlet`,
  `HBU 2 - Outlet`, `HPU 2 - Speed Actual`, `HBU 1 - Inlet Temp`, `HBU 1 - Outlet Temp`,
  `HBU 1 - Probe Temp`, `HPU 2 - A`, `HPU 2 - P`, `HPU 2 - B`, `HPU 1 - PCSV`,
  `HBU 1 - RIGHT`, `HBU 1 - LEFT`, `HBU 2 - LEFT`, `HBU 2 - RIGHT`, `HPU 2 - PCSV`,
  `HPU 1 - Current`, `HPU 1 - Power`, `HPU 1 - Speed Actual`, `HPU 2 - Power`,
  `HPU 2 - Current`, `HPU 2 - Speed Actual (1)`, `Oil Temp`.
- You will need to map these channel names onto the `CompressorTelemetry` fields/indices —
  use the per-booster config index mapping from step 0.5. **[DISCOVER]** the mapping; **[ASK]**
  if a channel has no obvious telemetry home.

---

## 7. CSV replay mode

A second mode (e.g. param `mode: synthetic | replay`, `replay_file: <path>`): read the CSV,
resample to 10 Hz, and publish it as `CompressorTelemetry`. Open-loop (it ignores commands),
but it validates the **monitoring/report** code (compression ratio, averaged step-per-stroke,
stall detection, kWh/kg, leak-down) against data you have already plotted. Loop or stop at end
(make it a param).

---

## 8. Acceptance criteria

Phase 1 (single booster, synthetic):
- `StartVFD(speed_rpm=1500)` → telemetry VFD speed ramps to ~1500 over ~0.5 s.
- `SetPCSV(enable=true, cpm=10)` → outlet pressure ramps at ~1 bar/s with a visible sawtooth.
- Outlet rise stops at `target_pressure`.
- `SetPCSV(enable=false)` (or stop) → outlet decays with τ ≈ 10 s.
- Booster/coordinator nodes bring up and drive the sim **without code changes**.

Phase 2:
- Both boosters run independently from their own command services.
- In SYNC, the high booster's inlet tracks the low booster's outlet.

Replay:
- Publishing the CSV reproduces the recorded HBU 2 outlet trace (262 → 592 → ~488).

---

## 9. Discover-from-repo-or-ask checklist (do not guess)

| Item | Source | If not found |
|---|---|---|
| `CompressorTelemetry` field names / array sizes / PLC flags | `mserve_interfaces/msg` | **ASK** (and propose adding flags per decision record) |
| `BoosterCmd.srv` definition | `mserve_interfaces/srv` | **CREATE** per decision record §4, build, confirm |
| Booster command service name + namespacing | booster node source / decision record | **ASK** |
| Per-booster index mapping (vfd, PT, TT) | config YAML / params | **ASK** |
| Channel-name → telemetry-field mapping | config + msg | **ASK** for any unclear channel |
| Workspace build + launch conventions, package naming | existing packages / launch | mirror them; **ASK** if ambiguous |
| Over-pressure / over-temp protective limits | decision record / config | use sensible defaults, flag them, **ASK** |

## 10. Constraints

- Do **not** modify the booster or coordinator nodes — the point is they run unchanged.
- Do **not** reimplement hard real-time (the 5 ms ramp) — model envelopes only.
- Keep all model constants as parameters with sensible defaults; nothing hardcoded.
- Build cleanly in the workspace; provide a launch file; document how to run both modes.
- When you hit any [DISCOVER]/[ASK] item you cannot resolve from the repo, **stop and ask**
  rather than guessing an interface.
