# mServe-STACK — Claude Code project instructions

## Teaching style (non-negotiable)

Walk through steps proactively. Show code snippets in chat as part of the explanation.
Do NOT edit or write files unless the user explicitly says "implement", "do it", "write it",
or similar. Do NOT ask questions to check understanding — teach and move forward. Only ask
if you genuinely do not know something (ambiguous constraint, missing context).
The user asks questions when they have them. See `lesson_plan_btros2.md` for the learning
arc context.

## Project layout

```
ws/src/
  hyfleet_booster/     — per-booster lifecycle node + BT state machine (C++20 / ROS 2 Jazzy)
  hyfleet_compressor/  — coordinator lifecycle node, routes goals to boosters via BT
  hyfleet_sim/         — Python simulator standing in for real hardware
```

## Key architecture decisions (locked — do not change without discussion)

- BoosterNode: single `active_goal_` / `active_tree_`; tick timer 100 ms; MTE executor
- CompressorNode: `ops_[2]` (LOW slot=0, HIGH slot=1, SYNC slot=0); per-slot blackboard
  child of `shared_blackboard_`; MutuallyExclusive callback group shared between action
  server and tick timer (fixes `spin_some` race — do not change this)
- BT.CPP `RosActionNode::halt()` → `cancelGoal()` touches a shared `SingleThreadedExecutor`
  without the library mutex — the callback group serialisation is the fix
- Parameters: `on_parameters` rejects any change unless state is UNCONFIGURED
- `reenable_offset_bar` (default 50.0 bar) declared in `booster_params.cpp`, written to
  blackboard at configure; not a goal field
- All 5 integration tests pass end-to-end against the sim:
  `test_low`, `test_high`, `test_parallel_both`, `test_setpoint_update`, `test_stop`

## Build and run

```bash
# Build
colcon build --packages-select hyfleet_booster hyfleet_compressor hyfleet_sim
source install/setup.bash

# Terminal 1 — stack
ros2 launch hyfleet_sim hyfleet_sim.launch.py

# Terminal 2 — test
python3 ws/src/hyfleet_sim/scripts/test_low.py
```

## Stage status

- Stage 1 (BoosterNode shell): done
- Stage 2 (Booster BT state machine): done; all trees implemented and tested
- Stage 3 (CompressorNode PARALLEL mode): done; all 5 tests passing
- Stage 3 (SYNC mode): next — InterstageAboveBand / InterstateBelowBand nodes + sync.xml
- Stage 4–7: see `ws/src/hyfleet_compressor/docs/todo.md`
