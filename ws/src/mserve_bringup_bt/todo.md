# mserve_bringup_bt — TODO

## Next up

- [ ] **Extend the XML tree to configure + activate both nodes in sequence**
  - Add `mserve_drivechain` to `bringup.xml` with transition 1 (configure) then transition 3 (activate)
  - Currently only does configure on `mserve_base`
  - Tree should be: configure_base → activate_base → configure_drivechain → activate_drivechain

- [ ] **Add a `GetStateNode` condition node**
  - Calls `/mserve_base/get_state` before attempting a transition
  - Lets the tree skip a transition if the node is already in the target state
  - Prevents `success=false` when BT is re-run against an already-configured node

- [ ] **Add retry decorator in XML**
  - Wrap each `ChangeStateNode` in a `<RetryUntilSuccessful num_attempts="3">` decorator
  - No C++ changes needed — pure XML

- [ ] **Graceful shutdown subtree**
  - Reverse-order deactivate → cleanup for all managed nodes
  - Hook into SIGINT so Ctrl+C runs shutdown rather than hard-killing

- [ ] **Split main.cpp into headers**
  - `include/mserve_bringup_bt/change_state_node.hpp`
  - `include/mserve_bringup_bt/get_state_node.hpp`
  - `src/main.cpp` becomes just the executor

- [ ] **Groot2 visualisation**
  - Add `BT::PublisherZMQ` to `build()` so Groot2 can connect and show the live tree ticking

## Later / phase 2

- [ ] Replace `tickWhileRunning()` with a proper ROS executor loop so the node can also respond to topics/services while the tree is running
- [ ] Launch file that starts all managed nodes + the BT manager together
- [ ] Port the web UI lifecycle controls to "debug/override" mode only
