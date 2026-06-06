# Implementation prompt — Compression Test Simulator (`hyfleet_sim`)

You are a coding agent working in the **mServe-STACK** ROS 2 (Jazzy) workspace. Build a
**test simulator** for the HyFleet compression module. This document gives you the design,
the physics grounded in a real prototype run, and acceptance criteria. Some specifics
(exact message field names, service names, config indices, repo paths) you **must discover
from the repo or ask for** — they are marked **[DISCOVER]** / **[ASK]** below. **Do not
guess interface field names.** If something marked [DISCOVER] cannot be found, stop and ask
rather than inventing it.

---

## 0. Confirmed interfaces — read these files before writing any code

The following have been confirmed. Read the actual files to verify exact field names
and types; do not rely solely on the summaries below.

1. **`ws/src/hyfleet_compressor/docs/compression_module_decisions.md`** — architecture and
   interface contract. Read it first.

2. **`CompressorTelemetry.msg`** (`mserve_interfaces/msg/CompressorTelemetry.msg`) — confirmed:
   - `float64[16] pt_bar`, `float64[12] tt_celsius`, `bool[5] sv`
   - `uint8[2] vfd_state`, `float64[2] vfd_speed_rpm`, `float64[2] vfd_power_kw`
   - `uint8 mode` (OFF/STARTUP/AUTO/MANUAL/LOCKOUT)
   - No separate warn/alarm flags — stub mode as AUTO and leave flags for future work.

3. **`BoosterCmd.srv`** (`mserve_interfaces/srv/BoosterCmd.srv`) — confirmed, exists:
   - `cmd`: START_VFD=1, STOP_VFD=2, SET_PCSV=3, CONTROL_SV=4
   - Fields: `enable` (bool), `sv_index` (uint8), `speed_rpm` (float64), `cpm` (float64)
   - Response: `success` (bool), `message` (string)
   - Service name per booster: `/<node_name>/booster_cmd` (e.g. `/low_booster/booster_cmd`)

4. **`ControlBooster.action`** — START=1, START_IDLE=2, STOP=3, SAFE_STOP=4;
   goal: `command`, `target_pressure`, `cpm`, `speed_rpm`

5. **Per-booster index mapping** (0-based, confirmed against current booster node params):

   | | low_booster | high_booster |
   |---|---|---|
   | inlet_pt_index | 0 | 1 |
   | outlet_pt_index | 7 | 2 |
   | inlet_tt_index | 0 | 4 |
   | outlet_tt_index | 2 | 6 |
   | vfd_index | 0 | 1 |

   The sim must publish telemetry into these exact indices.

6. **No CSV replay mode** — the source CSV is not in the repo. Use the physics constants
   in §5 directly (they were derived from that run). Do not implement CSV parsing or replay.

After reading the decision doc and the confirmed files above, briefly confirm what you
will build and **ask about anything still unclear before writing code.**

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

---

## 4. Tech choices

- **Python (`rclpy`)** — this is a test tool; iteration speed matters and the language need
  not match the C++ stack.
- **Single node.** A ~100 Hz timer steps the physics; a 10 Hz timer publishes telemetry;
  service callbacks just update the commanded state.
- **All model constants are ROS parameters** with sensible defaults. Do **not** hardcode
  them — match the project's parameter discipline.
- Provide a **launch file** that brings up the sim alongside the existing bringup.
  Look at `ws/src/mserve_launch/launch/mserve_min.launch.py` and mirror its style.

---

## 5. Physics model — trivial, command-driven

The sim exists only to make pressure respond to commands so the control logic has something
to react to. No fidelity to the real plant is required.

Per booster, maintain: `{pcsv_enabled, cpm, outlet_p, inlet_p, vfd_running}`.

- **VFD:** `START_VFD` → set `vfd_running = True`. `STOP_VFD` → `vfd_running = False`.
  No ramp needed — instant state change is fine.
- **CONTROL_SV:** track valve open/closed state; reflect in `sv[sv_index]` in telemetry.
- **Compression:** each physics tick (100 Hz), if `pcsv_enabled` and `vfd_running`:
  `outlet_p += bar_per_cpm * cpm * dt`
  Stop rising when `outlet_p >= target_pressure` (hold at target).
  `bar_per_cpm` is a ROS parameter (default `1.0` bar/cycle — dial it up for fast test runs).
- **PCSV off:** hold current pressure (no decay needed — the booster nodes don't wait for
  pressure to drop, they wait for the hydraulic line to depressurise via PressureBelowThreshold).
- **Inlet:** constant ~265 bar for standalone. In SYNC: `high_inlet = low_outlet`.
- **No discharge temp, no sawtooth, no real-gas mass, no decay tau.** Add only if a
  specific test case requires it.

---

## 6. Reference data — prototype run (physics tuning targets only)

High booster standalone run — use only to set sensible parameter defaults:
- Inlet supply ~265 bar; outlet range 262 → 592 bar for a full run.
- At 10 cpm, outlet rose ~1 bar/s → use `bar_per_cpm = 1.0 bar/cycle` as default
  (multiply by `cpm * dt` in the physics loop each tick).

That is all that is needed. The index mapping in §0 tells you which telemetry slots to populate.

---

## 7. Acceptance criteria

Phase 1 (single booster, synthetic):
- `StartVFD(speed_rpm=1500)` → `vfd_speed_rpm[0]` ramps to ~1500 over ~0.5 s.
- `SetPCSV(enable=true, cpm=10)` → `pt_bar[7]` (low outlet) ramps at ~1 bar/s with sawtooth.
- Outlet rise stops at `target_pressure`.
- `SetPCSV(enable=false)` → outlet decays with τ ≈ 10 s.
- Booster/coordinator nodes bring up and drive the sim **without code changes**.

Phase 2:
- Both boosters run independently from their own command services.
- In SYNC, `pt_bar[1]` (high booster inlet) tracks `pt_bar[7]` (low booster outlet).

---

## 8. Discover-from-repo-or-ask checklist

| Item | Status |
|---|---|
| `CompressorTelemetry` field names / array sizes | **Confirmed in §0** |
| `BoosterCmd.srv` definition | **Confirmed in §0** |
| Booster command service name + namespacing | **Confirmed in §0** |
| Per-booster index mapping (vfd, PT, TT) | **Confirmed in §0** |
| Workspace build + launch conventions | Mirror existing packages in `ws/src/` |
| Over-pressure / over-temp limits | Use sensible defaults as ROS params; **ASK** if unsure |

## 9. Constraints

- Do **not** modify the booster or coordinator nodes — the point is they run unchanged.
- Do **not** reimplement hard real-time (the 5 ms ramp) — model envelopes only.
- Keep all model constants as parameters with sensible defaults; nothing hardcoded.
- Build cleanly in the workspace; provide a launch file.
- When you hit anything you cannot resolve from the repo, **stop and ask** rather than guessing.
