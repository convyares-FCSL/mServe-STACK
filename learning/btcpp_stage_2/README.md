# btcpp_stage_2 — Learning Reference

This is a **snapshot archive** of `mserve_bringup_bt` at the end of Stage 2 learning.
It is not built or run directly — it exists as a reference showing the port from raw
BT.CPP to the production `behaviortree_ros2` base classes.

Compare with `../btcpp_stage_1/` to see exactly what changed and why.

---

## What changed from Stage 1

| Area | Stage 1 | Stage 2 |
|---|---|---|
| Base class | `BT::SyncActionNode` / `BT::ConditionNode` | `BT::RosServiceNode<SrvT>` for both |
| Node handle | `rclcpp::Node::SharedPtr` passed via lambda | `BT::RosNodeParams` passed via lambda |
| Client creation | Manual in constructor | `setServiceName()` — base class owns client |
| Client sharing | One client per XML instance | Static registry — shared if same service name |
| `wait_for_service` | Manual in `tick()` | Handled by base class |
| `async_send_request` | Manual in `tick()` | Handled by base class |
| `spin_until_future_complete` | Manual, blocks thread | Base class uses dedicated callback group — non-blocking |
| Timeout handling | Manual check in `tick()` | `onFailure(SERVICE_TIMEOUT)` callback |
| What you implement | All of the above | `setRequest()` + `onResponseReceived()` only |
| Thread blocking | Yes — `tick()` blocked until response | No — returns `RUNNING`, ticks again when response arrives |

---

## What this snapshot demonstrates

### `RosServiceNode<SrvT>` pattern
```
Constructor  →  setServiceName()         — tell base class which service to use
setRequest() →  fill Request, return true/false  — called before send
onResponseReceived() →  check Response   — called when response arrives
onFailure()  →  optional error handling  — SERVICE_TIMEOUT, SERVICE_UNREACHABLE, etc.
```

### `RosNodeParams`
```cpp
BT::RosNodeParams params(shared_from_this());
// or with timeouts:
BT::RosNodeParams params;
params.nh = shared_from_this();
params.server_timeout = std::chrono::milliseconds(2000);
params.wait_for_server_timeout = std::chrono::milliseconds(500);
```

### `registerBuilder` with `RosNodeParams`
```cpp
factory.registerBuilder<ChangeStateNode>(
  "ChangeStateNode",
  [this](const std::string& name, const BT::NodeConfig& config) {
    return std::make_unique<ChangeStateNode>(name, config,
      BT::RosNodeParams(this->shared_from_this()));
  });
```

### Empty request pattern (`IsInState` — `GetState` has no inputs)
```cpp
bool setRequest(Request::SharedPtr& request) override {
  (void)request;
  return true;
}
```

### `providedBasicPorts` — include standard `service_name` port
```cpp
static BT::PortsList providedPorts() {
  return providedBasicPorts({
    BT::InputPort<std::string>("node_name"),
    BT::InputPort<std::string>("transition")
  });
}
```

---

## behaviortree_ros2 base classes available (not all used here)

| Class | Header | Use case |
|---|---|---|
| `RosServiceNode<SrvT>` | `bt_service_node.hpp` | Service calls — used here |
| `RosActionNode<ActionT>` | `bt_action_node.hpp` | Long-running actions with feedback |
| `RosTopicSubNode<TopicT>` | `bt_topic_sub_node.hpp` | Condition from latest topic message |
| `RosTopicPubNode<TopicT>` | `bt_topic_pub_node.hpp` | Publish a message as a BT step |
| `TreeExecutionServer` | `tree_execution_server.hpp` | Expose tree as a ROS action server |

---

## Known limitations (remaining — Phase 3+)

| Limitation | Fix |
|---|---|
| `tickWhileRunning()` — no concurrent ROS processing | Replace with executor loop |
| Groot2 not available | Requires source build of BT.CPP with ZMQ enabled |
| No graceful shutdown subtree | Phase 4 |
| Lifecycle transition map in `mserve_utils` — not in `behaviortree_ros2` | Intentional — domain logic stays in mserve |

---

## Package structure

```
src/
  main.cpp              BaseNode + IsInState (RosServiceNode<GetState>)
                                + ChangeStateNode (RosServiceNode<ChangeState>)
  trees/
    bringup.xml         Full bringup tree — configure + activate, base + drivechain
```
