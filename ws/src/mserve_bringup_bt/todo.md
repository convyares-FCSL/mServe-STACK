# mserve_bringup_bt — TODO

## Stage 1 — COMPLETE

- [x] Extend XML to configure + activate both nodes in sequence
- [x] Add `IsInState` condition node
- [x] Add `RetryUntilSuccessful` decorator in XML
- [x] `Inverter` decorator for idempotent re-run safety
- [x] Named transitions instead of magic numbers
- [x] Snapshot archived to `learning/btcpp_stage_1/`
- [ ] Groot2 — deferred to Stage 2 (requires source build with ZMQ)
- [ ] Split `main.cpp` — deferred, `behaviortree_ros2` will define the right boundaries
- [ ] Graceful shutdown subtree — deferred to Stage 2

## Phase 3 — production grade (behaviortree_ros2)

- [x] **Build `behaviortree_ros2` from source**
  - Cloned BehaviorTree/BehaviorTree.ROS2 into `ws/src/`
  - `btcpp_ros2_interfaces` built first, then `behaviortree_ros2`
  - Required `ros-jazzy-generate-parameter-library` apt install
- [ ] **Port `ChangeStateNode` to `BT::RosServiceNode`**
  - Inherits from `BT::RosServiceNode<lifecycle_msgs::srv::ChangeState>`
  - Replaces `spin_until_future_complete` with proper async pattern
  - Handles client sharing and executor integration correctly
- [ ] **Replace `tickWhileRunning()` with a proper ROS executor loop**
  - Node can respond to topics/services while tree is ticking

## Phase 4 — integration

- [ ] Launch file that starts all managed nodes + BT manager together
- [ ] Groot2 live visualisation via `BT::PublisherZMQ`
- [ ] Graceful shutdown subtree on SIGINT
- [ ] Port the web UI lifecycle controls to debug/override only

## behaviortree_ros2 — other available base classes (not used here, for future projects)

These are all in `ws/src/BehaviorTree.ROS2/behaviortree_ros2/include/behaviortree_ros2/`:

| Class | Header | Use case |
|---|---|---|
| `RosActionNode<ActionT>` | `bt_action_node.hpp` | Wraps `rclcpp_action::Client` — non-blocking, handles goal/feedback/result. Use for long-running robot actions (navigation, arm moves) |
| `RosTopicSubNode<TopicT>` | `bt_topic_sub_node.hpp` | Condition node that reads latest message from a topic. Use for sensor checks, state monitoring (e.g. "is battery above 20%?") |
| `RosTopicPubNode<TopicT>` | `bt_topic_pub_node.hpp` | Publishes a message as a BT action. Use for commands that don't need a response (e.g. publish cmd_vel) |
| `TreeExecutionServer` | `tree_execution_server.hpp` | Exposes the BT as a ROS action server — external nodes can trigger and monitor tree execution |

All share `RosNodeParams` for node handle + timeout config. All handle client/subscriber sharing via static registry — same service/topic name = shared instance.

`RosActionNode` error codes: `SERVER_UNREACHABLE`, `SEND_GOAL_TIMEOUT`, `GOAL_REJECTED_BY_SERVER`, `ACTION_ABORTED`, `ACTION_CANCELLED`, `INVALID_GOAL`
`RosServiceNode` error codes: `SERVICE_UNREACHABLE`, `SERVICE_TIMEOUT`, `INVALID_REQUEST`, `SERVICE_ABORTED`

## Closeout actions (before shipping)

- [ ] Update `Dockerfile` to clone and build `behaviortree_ros2` + `btcpp_ros2_interfaces` from source as part of image build
- [ ] Archive Stage 2 snapshot to `learning/btcpp_stage_2/` before any further refactoring
- [ ] Update `scripts/README.md` — add native run instructions alongside Docker ones
