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
- Publishes `CompressorTelemetry` at 10 Hz into the same array indices the booster nodes read
- Runs trivial physics: while PCSV enabled and VFD running, outlet pressure rises at `bar_per_cpm × cpm` per cycle; holds at `target_pressure`; holds when PCSV off

## Build

```bash
colcon build --packages-select hyfleet_sim
source install/setup.bash
```

## Run

Full stack against the sim (sim + both boosters + coordinator + lifecycle manager):

```bash
ros2 launch hyfleet_sim hyfleet_sim.launch.py
```

Boosters and coordinator run completely unchanged. The lifecycle manager configures and activates them in order: `low_booster` → `high_booster` → `hyfleet_compression`.

## Physics model

Intentionally minimal — the sim exists to make pressure respond to commands, not to model the plant with fidelity.

| Event | Effect |
|---|---|
| `START_VFD(speed_rpm)` | `vfd_running = True`; speed reflected in telemetry immediately |
| `STOP_VFD` | `vfd_running = False`; speed → 0 |
| `SET_PCSV(enable=true, cpm)` | `pcsv_enabled = True`; outlet rises at `bar_per_cpm × cpm × dt` per physics tick |
| `SET_PCSV(enable=false)` | `pcsv_enabled = False`; outlet holds |
| `CONTROL_SV(sv_index, enable)` | Valve state reflected in `sv[sv_index]` in telemetry |
| Outlet reaches `target_pressure_bar` | Holds — no further rise |

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `physics.bar_per_cpm` | `1.0` | Rise rate (bar per cycle). Increase for faster test runs. |
| `physics.physics_rate_hz` | `100.0` | Physics tick rate |
| `physics.telemetry_rate_hz` | `10.0` | Telemetry publish rate |
| `physics.sync_mode` | `false` | When true, high-booster inlet tracks low-booster outlet (SYNC mode) |
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

Set `physics.sync_mode: true` to couple the boosters: the high booster's inlet pressure is set to the low booster's outlet pressure each physics tick. Used for testing the SYNC coordinator path.
