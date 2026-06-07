# hyfleet_sim

Compression test simulator for the HyFleet compression module. Replaces the ADS bridge, PLC, and physical plant at the ROS boundary so the booster and coordinator nodes can be tested without hardware.

```
[ booster / coordinator nodes ]   ← unchanged
      |  BoosterCmd service calls         ^  CompressorTelemetry (~10 Hz)
      v                                   |
[ CompressorSimNode ]   ==   ADS bridge + PLC + plant (simulated)
```

## What it does

- Advertises `BoosterCmd` services at `/low_booster/booster_cmd` and `/high_booster/booster_cmd`
- Advertises `CompressorCmd` service at `/hyfleet_compression/compressor_cmd` (interstage SV + heater stubs)
- Publishes `CompressorTelemetry` at 10 Hz into the same array indices the booster nodes read
- Runs trivial physics: while PCSV enabled and VFD running, outlet pressure rises at `bar_per_cpm × cpm` per cycle; holds at `target_pressure`; holds when PCSV off
- SYNC interstage coupling: when the interstage SV is open (via `CompressorCmd`), the high booster's inlet tracks the low booster's outlet each physics tick; the low booster outlet decays at `interstage_draw_bar_per_s` when the high booster is active

## Build

```bash
colcon build --packages-select hyfleet_sim
source install/setup.bash
```

## Run

Two terminals are needed: one for the stack, one for the test script.

**Terminal 1 — full stack:**

```bash
cd ~/ai-workspace/projects/mServe-STACK/ws
source install/setup.bash
ros2 launch hyfleet_sim hyfleet_sim.launch.py
```

Wait until you see the lifecycle manager finish activating all three nodes before running any test. Look for:

```
[lifecycle_manager]: All nodes successfully activated
```

**Terminal 2 — run a test:**

```bash
cd ~/ai-workspace/projects/mServe-STACK/ws
source install/setup.bash
python3 src/hyfleet_sim/scripts/test_low.py
```

The stack does not need restarting between tests — each script calls `/compressor_sim/reset` at the start to restore initial pressures and VFD state.

### Test scripts

All scripts are in `ws/src/hyfleet_sim/scripts/`. Each exits 0 on pass, 1 on fail.

| Script | What it tests | Pass condition |
|---|---|---|
| `test_low.py` | LOW booster: COMPRESS → reach 380 bar | Goal `SUCCEEDED`, outlet reaches target |
| `test_high.py` | HIGH booster: COMPRESS → reach 700 bar | Goal `SUCCEEDED`, outlet reaches target |
| `test_parallel_both.py` | Both LOW (380 bar) and HIGH (700 bar) simultaneously | Both goals `SUCCEEDED` independently |
| `test_setpoint_update.py` | LOW running at 350 bar, re-goaled to 480 bar mid-run | First goal `ABORTED` ("replaced by newer goal"), second goal `SUCCEEDED` |
| `test_stop.py` | HIGH started at 700 bar, STOP sent after 3 s | START goal `ABORTED`, STOP goal `SUCCEEDED` |
| `test_sync.py` | SYNC: low builds interstage to 280 bar, SV opens, high compresses to 900 bar | SYNC goal `SUCCEEDED` |

Boosters and coordinator run completely unchanged. The lifecycle manager configures and activates them in order: `low_booster` → `high_booster` → `hyfleet_compression`.

## Physics model

Intentionally minimal — the sim exists to make pressure respond to commands, not to model the plant with fidelity.

| Event | Effect |
|---|---|
| `START_VFD(speed_rpm)` | `vfd_running = True`; speed reflected in telemetry immediately |
| `STOP_VFD` | `vfd_running = False`; speed → 0 |
| `SET_PCSV(enable=true, cpm)` | `pcsv_enabled = True`; outlet rises at `bar_per_cpm × cpm × dt` per physics tick |
| `SET_PCSV(enable=false)` | `pcsv_enabled = False`; outlet holds |
| `CONTROL_SV(sv_index, enable)` | Valve state reflected in `sv[sv_index]` in telemetry; interstage SV (at `compressor.interstage_sv_index`) also enables interstage coupling |
| Interstage SV open | `high.inlet_p = low.outlet_p` each tick; `low.outlet_p` decays at `interstage_draw_bar_per_s` when high PCSV+VFD active |
| Outlet reaches `target_pressure_bar` | Holds — no further rise |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `physics.bar_per_cpm` | `1.0` | Rise rate (bar per cycle). Increase for faster test runs. |
| `physics.physics_rate_hz` | `100.0` | Physics tick rate |
| `physics.telemetry_rate_hz` | `10.0` | Telemetry publish rate |
| `physics.interstage_draw_bar_per_s` | `8.0` | Rate at which active high booster drains the low booster outlet (bar/s). Drives the low booster hold loop without hardware. |
| `compressor.interstage_sv_index` | `4` | `sv[]` index of the interstage solenoid valve |
| `low_booster.inlet_pressure_bar` | `265.0` | Simulated inlet supply |
| `low_booster.initial_outlet_bar` | `265.0` | Outlet pressure at startup |
| `low_booster.target_pressure_bar` | `500.0` | Outlet cap |
| `low_booster.inlet_pt_index` | `0` | `pt_bar[]` index for low inlet |
| `low_booster.outlet_pt_index` | `7` | `pt_bar[]` index for low outlet |
| `low_booster.vfd_index` | `0` | `vfd_speed_rpm[]` index for low booster |
| `high_booster.inlet_pressure_bar` | `265.0` | Simulated inlet supply |
| `high_booster.initial_outlet_bar` | `265.0` | Outlet pressure at startup |
| `high_booster.target_pressure_bar` | `900.0` | Outlet cap |
| `high_booster.inlet_pt_index` | `1` | `pt_bar[]` index for high inlet |
| `high_booster.outlet_pt_index` | `2` | `pt_bar[]` index for high outlet |
| `high_booster.vfd_index` | `1` | `vfd_speed_rpm[]` index for high booster |

Full `tt_celsius` index params also available — see `sim_node.py` parameter declarations.

## SYNC mode

SYNC coupling is driven by the live state of the interstage SV — there is no separate `sync_mode` parameter. When the coordinator sends `CONTROL_SV(sv_index=4, enable=true)`, the sim's physics tick couples the stages: the high booster's inlet tracks the low booster's outlet each tick, and the low booster outlet decays at `interstage_draw_bar_per_s` when the high booster is compressing. Closing the SV removes the coupling. The sim's behaviour therefore exactly follows the coordinator's `sync.xml` tree decisions.
