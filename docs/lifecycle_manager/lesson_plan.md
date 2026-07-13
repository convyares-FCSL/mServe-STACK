# BT.CPP Lifecycle Manager — Lesson Plan

## What we covered (session 1)

### Verified BT.CPP install
- `ros-jazzy-behaviortree-cpp` 4.9.0 installed via apt
- Compiled a single-file smoke test with `g++` directly (no CMake) to confirm linking works
- `SyncActionNode`, `tick()`, `NodeStatus::SUCCESS`, `tickWhileRunning()` all confirmed working

### Created ROS 2 package `mserve_lifecycle_manager`
- `ros2 pkg create` scaffolds `CMakeLists.txt` and `package.xml`
- `find_package` + `ament_target_dependencies` replaces manual `-I`/`-L` flags
- `install(TARGETS ...)` to `lib/${PROJECT_NAME}` makes `ros2 run` work

### Connected BT to ROS 2
- `BT::BehaviorTreeFactory` + `registerBuilder` with a lambda to pass `rclcpp::Node::SharedPtr`
- `shared_from_this()` cannot be called in a constructor — moved BT setup to `build()` method called after node construction
- XML loaded from file via `ament_index_cpp::get_package_share_directory` + `createTreeFromFile`

### Key BT concepts learned
- **Factory** — registry that maps XML tag names to C++ classes; `registerNodeType` for standard constructors, `registerBuilder` when you need extra constructor arguments
- **SyncActionNode** — blocks until `tick()` returns; right choice for service calls
- **Ports / blackboard** — how data flows from XML into nodes at runtime; `InputPort` declares it, `getInput<T>()` reads it in `tick()`
- **Static vs runtime ports** — static values (from XML attributes) are available in `config.input_ports` at construction time, before any ticking
- **`tickWhileRunning()`** — ticks the root until it returns SUCCESS or FAILURE

### Built ChangeStateNode
- Calls `lifecycle_msgs/srv/ChangeState` to drive ROS 2 lifecycle transitions
- Client created once in constructor using `config.input_ports` (static port value)
- `wait_for_service` before `async_send_request` to avoid hanging futures
- `spin_until_future_complete` with timeout so it doesn't block forever
- Checks both the RPC result AND `response->success` — service can succeed but transition be rejected

### First working end-to-end run
- `mserve_base` started → BT ran → `/mserve_base/change_state` called → configure transition fired → `Transition succeeded`

---

## What we covered (session 2)

### Extended to full sequence
- Added `Sequence` node to XML — stops at first `FAILURE`, which is the correct bringup behaviour
- Named transitions (`"configure"`, `"activate"`) via `inline static const std::unordered_map` in the node — no magic numbers in XML
- `InputPort<std::string>` + map lookup in `tick()` converts name to lifecycle transition ID

### Instance-per-XML-node model
- The factory constructs **one C++ instance per `<ChangeStateNode>` tag** at `createTreeFromFile` time
- Constructor cost matters: it runs N times at tree load, once per node in the XML
- Keep constructors light — register clients, store names, nothing expensive
- Service clients are cheap (DDS endpoint registration only), so one-per-instance is fine at this scale
- For complex trees with many nodes hitting the same service: `behaviortree_ros2` handles client sharing and async properly — see Phase 3 below

---

## Planned phases

### Phase 2 — condition nodes + resilience (next)
- `GetStateNode` — condition node that checks current lifecycle state before attempting a transition
- Retry decorator in XML — `<RetryUntilSuccessful>` wrapping each action, no C++ changes
- Prevents re-run failures when nodes are already in the target state

### Phase 3 — production grade with `behaviortree_ros2`
- `behaviortree_ros2` is the production library (BehaviorTree/BehaviorTree.ROS2 on GitHub)
- Provides `RosActionNode`, `RosServiceNode`, `RosTopicSubNode` wrappers with proper async, client sharing, and executor integration
- Replaces `spin_until_future_complete` hack with a proper non-blocking async pattern
- Required for trees that need to run alongside a live ROS executor (topics, actions, services simultaneously)
- Build from source — not in apt for Jazzy
- Goal: port `ChangeStateNode` to inherit from `BT::RosServiceNode<lifecycle_msgs::srv::ChangeState>`

### Phase 4 — integration
- Launch file starts all managed nodes + BT manager together
- Groot2 live visualisation via `BT::PublisherZMQ`
- Graceful shutdown subtree on SIGINT
- Web UI demoted to debug/override only

---

## Next session — where to pick up

See `todo.md`.
