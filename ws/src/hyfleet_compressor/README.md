# hyfleet_compression

Lifecycle ROS 2 node for the HyFleet compression module. Controls two hydraulic boosters
(low 35–300 bar, high 200–900 bar) via a BehaviorTree.ROS2-driven state machine, replacing
the hand-coded switch/case loop in the archived implementation.

## Hardware

- **HPU** (Boosh Hydraulic Power Unit) — two boosters sharing oil temperature / level monitoring
- **Low booster** — inlet SV, HPU SV, VFD, PCSV
- **High booster** — inlet SV, HPU SV, VFD, PCSV
- **Sync solenoid** — owned by this node; open = series mode, closed = parallel mode

## Operation modes

| Mode | Solenoid | Goals accepted | Notes |
|------|----------|---------------|-------|
| Parallel | Closed | 2 independent | Each booster runs to its own target |
| Sync | Open | 1 combined | Node manages interstage pressure via CPM while ramping to final target |

## ROS interfaces

| Interface | Type | Name | Direction |
|-----------|------|------|-----------|
| Telemetry | Topic sub | `compressor_telemetry` | PLC → node |
| Control | Action server | `control_compressor` | Orchestrator → node |
| Diagnostics | Topic pub | `compressor_diagnostics` | Node → operator |
| Oil health | Topic pub | `oil_healthy` | Node → system |
| Low booster cmd | Service client | `control_booster_1` | Node → PLC |
| High booster cmd | Service client | `control_booster_2` | Node → PLC |
| Compressor cmd | Service client | `control_compressor_cmd` | Node → PLC |

## Stop modes

- **Stop** — clear start flag; booster walks the full ordered shutdown sequence
- **Safe stop** — interrupt active phase and enter ordered shutdown
- **Force stop** — immediate: close all solenoids, kill drives, no wait. Triggered by action command OR system-wide `/force_stop` topic

## Safety inhibits

Oil temp < 15 °C or > 60 °C, HBU inlet > 50 °C, HBU outlet > 190 °C, chiller > 70 °C,
telemetry timeout > 2 s. Any violation inhibits both boosters. Heaters controlled to maintain oil temp.

## Lockout

A booster enters lockout on pressure fault, temperature fault, or PLC error. System-wide
lockout also monitored via topic. No commands accepted in lockout except reset — only
clears once all conditions are healthy.

## Build

```bash
colcon build --packages-select hyfleet_compression
source install/setup.bash
```

## Docs

- [System description](docs/description.md) — hardware, interfaces, architecture decisions
- [Lesson plan](docs/lesson_plan_btros2.md) — BehaviorTree.ROS2 learning phases
