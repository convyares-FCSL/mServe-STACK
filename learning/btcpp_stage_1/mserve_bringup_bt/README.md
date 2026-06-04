# btcpp_stage_1 — Learning Reference

This is a **snapshot archive** of `mserve_bringup_bt` at the end of Stage 1 learning.
It is not built or run directly — it exists as a reference for understanding the concepts
before the production port to `behaviortree_ros2`.

---

## What this snapshot demonstrates

A working ROS 2 BT.CPP lifecycle manager built from scratch, covering:

### BT.CPP concepts
| Concept | Where to see it |
|---|---|
| `SyncActionNode` | `ChangeStateNode` — blocking service call |
| `ConditionNode` | `IsInState` — fast state check, no async |
| Input ports / blackboard | `providedPorts()` + `getInput<T>()` in both nodes |
| Static port values at construction | `config.input_ports.find()` in constructors |
| `registerBuilder` with lambda | `build()` in `BaseNode` — passes `rclcpp::Node::SharedPtr` |
| `Sequence` | Stops on first FAILURE — correct for ordered bringup |
| `Fallback` | Tries next child on FAILURE — used for skip-if-done pattern |
| `Inverter` decorator | Flips SUCCESS/FAILURE — "skip if NOT in this state" |
| `RetryUntilSuccessful` decorator | Retries action N times before giving up |
| XML tree loaded from file | `createTreeFromFile` + `ament_index_cpp` for path |

### ROS 2 integration pattern
- `BT::BehaviorTreeFactory` + `registerBuilder` to inject `rclcpp::Node::SharedPtr`
- `shared_from_this()` must be called after construction — use a `build()` method
- `wait_for_service` + `async_send_request` + `spin_until_future_complete` for service calls
- Check both `FutureReturnCode::SUCCESS` AND `response->success` — they are independent

---

## Key design decisions and why

**One client per XML node instance**
The factory constructs one C++ instance per `<ChangeStateNode>` tag. Service clients
are cheap (DDS endpoint registration only) so one-per-instance is fine at this scale.
Keep constructors light — they run N times at tree load.

**`Inverter` over multiple `IsInState` children**
`Fallback(IsInState(inactive), IsInState(active), ChangeState(configure))` breaks as
you add more valid skip states. `Inverter(IsInState(unconfigured))` handles all
post-configure states in one check.

**Named transitions (`"configure"`) over magic numbers (`1`)**
`inline static const std::unordered_map<std::string, uint8_t>` in `ChangeStateNode`
maps names to IDs. XML stays readable, IDs stay in one place.

**`build()` method, not constructor**
`shared_from_this()` cannot be called in a constructor — the `shared_ptr` doesn't
own `this` yet. All BT setup lives in `build()` called after construction.

---

## Known limitations (addressed in Stage 2 / behaviortree_ros2)

| Limitation | Impact | Fix in Stage 2 |
|---|---|---|
| `spin_until_future_complete` blocks the thread | No ROS callbacks while ticking | `RosServiceNode` async pattern |
| One client per XML node (no sharing) | Minor overhead for large trees | `RosNodeParams` client sharing |
| `tickWhileRunning()` with no executor | Node can't process topics/services while BT runs | Proper executor loop |
| Groot2 not available | No live visualisation | Build BT.CPP from source with ZMQ |
| Boilerplate duplicated across node classes | `wait_for_service` / `spin_until_future_complete` repeated | `RosServiceNode` base class |

---

## Package structure

```
src/
  main.cpp              All C++ — BaseNode, ChangeStateNode, IsInState
  trees/
    bringup.xml         Full bringup tree — configure + activate, base + drivechain
```

---

## How to add a managed node (XML only, no C++ needed)

```xml
<Sequence name="my_node_sequence">
    <Fallback name="config_my_node">
        <Inverter>
            <IsInState name="check_unconfigured" node_name="my_node" state="unconfigured"/>
        </Inverter>
        <RetryUntilSuccessful num_attempts="3">
            <ChangeStateNode name="configure_my_node" node_name="my_node" transition="configure"/>
        </RetryUntilSuccessful>
    </Fallback>
    <Fallback name="activate_my_node">
        <IsInState name="check_active" node_name="my_node" state="active"/>
        <RetryUntilSuccessful num_attempts="3">
            <ChangeStateNode name="activate_my_node" node_name="my_node" transition="activate"/>
        </RetryUntilSuccessful>
    </Fallback>
</Sequence>
```

---

## Dependencies

- `behaviortree_cpp` 4.9.0 (`ros-jazzy-behaviortree-cpp`)
- `lifecycle_msgs`
- `ament_index_cpp`
- `rclcpp`
