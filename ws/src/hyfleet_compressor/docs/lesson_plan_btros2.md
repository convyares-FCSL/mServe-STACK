# behaviortree_ros2 Deep-Dive — Lesson Plan

## Context

Target project: **compression module** — a real industrial system with:
- Controlled stop (deactivate, stay ready)
- Force stop (immediate halt, fault state)
- Final state / lockout (requires manual reset)
- Sensor monitoring (pressure, temperature, flow)
- Long-running operations (compression cycles)

This is fundamentally different from the lifecycle manager — it's a **reactive, continuous tree**
not a one-shot bringup sequence. The tree runs indefinitely, monitoring conditions and
responding to events.

---

## What we already know (from lifecycle manager work)

- `RosServiceNode<SrvT>` — async service calls, `setRequest()` + `onResponseReceived()`
- `RosNodeParams` — passing node handle + timeouts
- `Sequence`, `Fallback`, `Inverter`, `RetryUntilSuccessful`, `Timeout` decorators
- Wall timer ticking alongside `rclcpp::spin()`
- XML-driven trees, Groot2 visualisation

---

## Phase 1 — RosActionNode (long-running operations)

**Why:** Compression cycles are long-running. A service call returns immediately; an action
server streams feedback and completes asynchronously. `RosActionNode` is non-blocking —
the tree returns `RUNNING` each tick while waiting, and `onResultReceived` fires when done.

**Learning exercise:** Build a `RunCompressionCycle` action node that wraps a mock action
server. The action sends goal (target pressure), receives feedback (current pressure %), 
and returns result (success/fault).

Key methods to implement:
- `setGoal()` — fill the action goal
- `onFeedback()` — optional, log progress
- `onResultReceived()` — check result, return SUCCESS/FAILURE
- `onFailure()` — handle SERVER_UNREACHABLE, GOAL_REJECTED, ACTION_ABORTED

**Compression context:** The actual compression cycle action server already exists —
BT replaces the state machine that currently drives it.

---

## Phase 2 — RosTopicSubNode (reactive monitoring)

**Why:** Safety monitoring can't wait for the tree to tick — pressure overload,
temperature fault, e-stop. `RosTopicSubNode` is a condition node that always holds
the latest message and returns SUCCESS/FAILURE instantly based on its value.

**Learning exercise:** Build a `PressureOK` condition node that subscribes to
`/compressor/pressure` and returns FAILURE if pressure exceeds a threshold read
from a blackboard key.

Key concepts:
- `onTick(const TopicT::SharedPtr& last_msg)` — called each tick with latest message
- Returns `SUCCESS` or `FAILURE` based on message content
- No service call, no async — just reads the last received message
- Subscriber shared via static registry (same topic = shared subscription)

**Why this fixes `IsInState`:** A topic-based state monitor inherits from `ConditionNode`
properly — shows as Condition in Groot2, semantically correct.

---

## Phase 3 — Reactive tree patterns

**Why:** The compression module needs to monitor AND act simultaneously. A pure
sequence doesn't work — you need the tree to react to faults mid-operation.

**Patterns to learn:**

### Monitor-and-run (parallel node)
```
Parallel (success_threshold=1, failure_threshold=1)
├── Sequence (run the operation)
│   ├── RunCompressionCycle
│   └── ...
└── Sequence (safety monitor — runs every tick)
    ├── PressureOK
    ├── TemperatureOK
    └── EStopNotPressed
```
If any safety condition fails → Parallel fails → abort the operation.

### Recovery fallback
```
Fallback
├── RunCompressionCycle        ← try normal operation
└── Sequence                  ← recovery path
    ├── LogFault
    ├── EmergencyVent
    └── WaitForReset
```

### Force stop vs graceful stop
- **Stop:** `Sequence(Deactivate, WaitForPressureDrop, Standby)`
- **Force stop:** `Sequence(EmergencyVent, HardStop)` — skip the wait

---

## Phase 4 — Blackboard for shared state

**Why:** Multiple BT nodes need to share data — fault codes, target setpoints,
operator commands. The blackboard is the shared memory of the tree.

**Learning exercise:** Pass target pressure from XML → blackboard → `RunCompressionCycle`
goal. Use `OutputPort` in a `ReadOperatorCommand` node to write to blackboard, read it
in `RunCompressionCycle` via `InputPort`.

Key concepts:
- `{key_name}` in XML means "read from blackboard key" not "literal value"
- `OutputPort` — node writes to blackboard
- Blackboard scoping — subtrees have their own blackboard by default

---

## Phase 5 — TreeExecutionServer (exposing BT as ROS action)

**Why:** The compression module's BT should be triggerable from outside — an operator
console, a higher-level orchestrator, or the HyQube fill sequence. `TreeExecutionServer`
wraps the entire tree as a ROS action server.

**Learning exercise:** Wrap the compression BT in a `TreeExecutionServer`. External
nodes can send a goal ("start compression"), receive feedback (current state), and
get a result (completed/faulted).

**Compression context:** The fill sequence BT (HyQube) becomes a client that triggers
the compression module BT as one step in a larger sequence.

---

## Phase 6 — LifecycleManager as a lifecycle node (fix shutdown ordering)

**Why:** Solves the SIGINT ordering problem from the lifecycle manager work. If
`LifecycleManager` is itself a `LifecycleNode`, `ros2 launch` can sequence shutdown:
stop the manager (runs shutdown tree) → then stop managed nodes.

This is the Nav2 pattern and is directly applicable to the compression module.

---

## Session approach

- Bring the current compression module C++ (state machine) as reference
- Each phase: understand the existing state machine logic → identify which BT pattern
  replaces it → implement and test
- Learning exercises (mock nodes) for any pattern not directly in the compression module
- Groot2 open throughout — watch the reactive tree tick in real time

## Pre-session prep

Bring:
- Current compression module state machine code
- List of action servers, topics, and services it uses
- The stop / force_stop / final_state transition diagram

This will let us map directly from existing logic to BT patterns rather than
building abstract examples.
