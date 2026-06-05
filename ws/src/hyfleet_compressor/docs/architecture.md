# Compressor Module — Architecture

## Design principle

Nav2-style pattern. Both coordinator and capability nodes are **lifecycle nodes**
that own their own action servers and tick BT trees internally. No direct class
coupling between nodes. Blackboard is local to each node's tree — cross-node
communication uses ROS interfaces exclusively (actions, services, topics).

This mirrors Nav2's BT Navigator exactly: lifecycle node, own action server,
internal BT tree, calls capability servers via RosActionNode.

---

## Telemetry topic — closed decision

A single `compressor_telemetry` topic (type `mserve_interfaces/msg/CompressorTelemetry`)
carries all sensor data for the entire compressor subsystem. Both `BoosterNode` instances
subscribe to this topic and extract their relevant fields using config-driven array indices.

Rationale: some sensors are physically shared between boosters (e.g. interstage pressure
transducer). A single topic is also the natural boundary for PLC→ROS bridging — the PLC
publishes one telemetry packet per cycle. Per-booster topics would require a splitter node
and make shared sensors awkward.

BT condition nodes take `pt_index` (or `vfd_index`) as an input port. The same C++ node
type works for both booster instances — the index in the XML distinguishes them.

This decision is closed. Do not reopen.

---

## TreeExecutionServer — closed decision

TreeExecutionServer is NOT used. It is not a lifecycle node and cannot be lifecycle
managed. Lifecycle management is mandatory for this system — boosters drive VFDs,
hydraulic drives, and solenoids on a hydrogen rig. Configure/activate separation,
controlled deactivate-to-standby, and sequenced bringup/shutdown are requirements.

behaviortree_ros2 is still used fully — for RosActionNode, RosTopicSubNode, and
RosServiceNode inside the trees. These are the lesson plan content. TreeExecutionServer
is one optional convenience wrapper and is the only piece dropped.

This decision is closed. Do not reopen.

---

## Node layout

```
hyfleet_compression           CompressorNode (lifecycle)
                              Owns rclcpp_action server + coordinator BT tree
                              Single external entry point for orchestrator
                              Bringup last — after boosters are active

hyfleet_booster (x2)         BoosterNode (lifecycle)
                              Same binary: launched as low_booster + high_booster
                              Owns rclcpp_action server + booster BT state machine
                              Config file drives all differences
                              Bringup before coordinator
```

---

## Node pattern — identical for coordinator and boosters

```
LifecycleNode
  on_configure  → create BT factory, register custom nodes, load tree from XML
                  create rclcpp_action server (Reentrant callback group)
  on_activate   → set accepting_goals = true
  on_deactivate → set accepting_goals = false, abort active goal via action protocol
  on_cleanup    → destroy action server, destroy tree, reset factory

  goal arrives  → write goal fields to blackboard (command, target, target_pressure)
                  start tick timer
  tick timer    → tree_.tickOnce()
  tree SUCCESS  → succeed the goal, stop timer
  tree FAILURE  → abort the goal, stop timer
```

---

## BoosterNode header members

```cpp
// Action server
rclcpp_action::Server<ControlCompressor>::SharedPtr action_server_;
rclcpp::CallbackGroup::SharedPtr action_cb_group_;
std::shared_ptr<GoalHandleControlCompressor> active_goal_;

// BT
BT::BehaviorTreeFactory factory_;
BT::Tree tree_;

// Tick
rclcpp::TimerBase::SharedPtr tick_timer_;
```

---

## Coordinator BT — goal routing

Goal fields written to blackboard → tree routes on `target` value:

```
target = LOW   → RosActionNode → /low_booster/control
target = HIGH  → RosActionNode → /high_booster/control
target = SYNC  → Parallel
                   ├── RosActionNode → /low_booster/control
                   └── RosActionNode → /high_booster/control
```

---

## Booster BT — internal state machine

```
Parallel (failure_threshold=1)
├── Sequence (operation)
│   ├── StartBooster
│   ├── WaitForPressureTarget    ← RosTopicSubNode
│   └── Succeed
└── Sequence (safety — runs every tick)
    ├── PressureInRange          ← RosTopicSubNode
    ├── TemperatureOK            ← RosTopicSubNode
    └── Inverter
        └── ForceStopRequested   ← blackboard key
```

---

## Blackboard scope

Local to each node's tree. Never shared across nodes.

| Node | Blackboard holds |
|---|---|
| Coordinator | command, target, target_pressure, interstage pressure, force_stop |
| Booster | command, target_pressure, current pressure, booster state, force_stop |

---

## Bringup order

```
base → drivechain → low_booster → high_booster → hyfleet_compression
```

---

## Build order

### Stage 1 — BoosterNode with action server + BT
Lifecycle node, action server, BT factory, tree loaded from XML.
Simple tree: accept goal → log → succeed. Validates full wiring end-to-end.

### Stage 2 — Booster BT state machine
Real state machine in the tree: IDLE → STARTING → RUNNING → STOPPING.
RosTopicSubNode pressure monitoring (lesson plan Phase 2).
Reactive Parallel force stop (lesson plan Phase 3).

### Stage 3 — CompressorNode coordinator
Action server + coordinator BT tree.
RosActionNode calling booster servers (lesson plan Phase 1).
Goal routing LOW / HIGH / SYNC Parallel.

### Stage 4 — Full integration
SYNC CPM control, interstage solenoid, safety monitor, diagnostics.

---

## Lesson plan mapping

| Phase | Where it lands |
|---|---|
| Phase 1 — RosActionNode | Coordinator BT calling booster servers (Stage 3) |
| Phase 2 — RosTopicSubNode | Booster BT pressure/temperature monitoring (Stage 2) |
| Phase 3 — Reactive tree | Booster BT force stop Parallel (Stage 2) |
| Phase 4 — Blackboard | Both trees — goal fields, pressure, force stop flag |
| Phase 5 — TreeExecutionServer | Deferred / optional — not on critical path |
| Phase 6 — LifecycleManager as lifecycle node | Future — fixes SIGINT shutdown ordering |
