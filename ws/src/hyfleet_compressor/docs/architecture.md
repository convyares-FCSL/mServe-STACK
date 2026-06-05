# Compressor Module — Architecture

## Design principle

Nav2-style coordinator pattern. A central BT coordinator orchestrates independent
capability nodes over ROS interfaces (actions, services, topics). No direct class
coupling between nodes. Blackboard is local to the coordinator tree only — cross-node
communication uses ROS interfaces exclusively.

---

## Node layout

```
/compressor/control       CompressorCoordinator
                          TreeExecutionServer + BT
                          Single external entry point for orchestrator

/low_booster/control      BoosterNode (hyfleet_booster package)
                          Launched with low_booster_config.yaml
                          Action server, own state machine / BT
                          Own telemetry interpretation, own command generation

/high_booster/control     BoosterNode (hyfleet_booster package)
                          Same binary, launched with high_booster_config.yaml
                          Identical code — config drives all differences

/cmd_sender               Hardware output boundary

/telemetry                Hardware input / status publishing boundary
```

---

## External interface

Single action server visible to orchestrator:

```
ControlCompressor goal:
    command:          COMPRESS / STOP / SAFE_STOP
    target:           LOW / HIGH / SYNC
    target_pressure:  e.g. 500.0 / 900.0
```

---

## Coordinator BT — goal routing

```
target = LOW
    → RosActionNode → /low_booster/control

target = HIGH
    → RosActionNode → /high_booster/control

target = SYNC
    → Parallel
        ├── RosActionNode → /low_booster/control
        └── RosActionNode → /high_booster/control
```

Concurrent booster goals handled natively by the BT Parallel node. No custom
multi-goal action server logic needed.

---

## SYNC active control

During SYNC the coordinator tree monitors interstage pressure via a topic condition
node. If interstage drifts, the tree adjusts low booster CPM via a service call.
Interstage solenoid is commanded via a service call from the coordinator tree.
Both adjustments are BT nodes within the SYNC subtree.

---

## Blackboard scope

The BT blackboard is **local to the coordinator tree**. It holds:
- Current goal (command, target, target_pressure)
- Interstage pressure (from topic condition node)
- Force stop flag
- Goal routing state

It is **not** a cross-node shared state bus. Booster nodes do not read from the
coordinator blackboard. All cross-node communication uses ROS actions, services,
and topics — these are observable, testable, and restartable independently.

---

## Stop and force stop

Force stop is a blackboard key local to the coordinator tree. A reactive Parallel
node monitors it alongside the active operation:

```
Parallel (failure_threshold=1)
├── ActiveOperation (RosActionNode to booster)
└── Inverter
    └── ForceStopRequested    (reads blackboard key)
```

When force stop is set (from cancel, deactivate, or fault), the Parallel fails on
the next tick and the tree handles cleanup — cancelling in-flight booster goals via
the action protocol.

---

## Why not the archive pattern

The archive's `take_pending_control`, `process_status`, and `abort_active_goals`
coupled the action class and node together via direct method calls. This blurred
responsibilities and grew in complexity as the system was extended.

This architecture removes those links. Each node is independently coherent.
ROS interfaces are the only boundary between capabilities.

---

## Build order

### Stage 1 — Booster nodes
Each booster node built as a lifecycle node with its own action server.
Simple state machine first (no BT). Handles its own telemetry and command generation.
Tested independently before coordinator exists.

### Stage 2 — Coordinator
CompressorCoordinator built with TreeExecutionServer.
BT routes goals to booster action servers via RosActionNode.
SYNC Parallel subtree added.
Reactive stop/force stop pattern added.

### Stage 3 — BT migration
Booster state machines migrated to BT trees (lesson plan phases 1–4).
Blackboard local to each booster tree.
Reactive safety monitoring per booster.

### Stage 4 — Full integration
SYNC active CPM control and interstage solenoid management in coordinator tree.
End-to-end test: orchestrator → coordinator → both boosters → hardware.

---

## Lesson plan mapping

| Phase | Component |
|---|---|
| Phase 1 — RosActionNode | Coordinator BT calling booster action servers |
| Phase 2 — RosTopicSubNode | Interstage pressure condition, booster safety conditions |
| Phase 3 — Reactive tree | Force stop pattern, safety monitor Parallel |
| Phase 4 — Blackboard | Goal routing state, force stop flag, interstage pressure |
| Phase 5 — TreeExecutionServer | Coordinator wrapping the full BT as `/compressor/control` |
