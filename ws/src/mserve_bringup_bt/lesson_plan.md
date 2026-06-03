# BT.CPP Lifecycle Manager — Lesson Plan

## What we covered (session 1)

### Verified BT.CPP install
- `ros-jazzy-behaviortree-cpp` 4.9.0 installed via apt
- Compiled a single-file smoke test with `g++` directly (no CMake) to confirm linking works
- `SyncActionNode`, `tick()`, `NodeStatus::SUCCESS`, `tickWhileRunning()` all confirmed working

### Created ROS 2 package `mserve_bringup_bt`
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

## Next session — where to pick up

See `todo.md`.
