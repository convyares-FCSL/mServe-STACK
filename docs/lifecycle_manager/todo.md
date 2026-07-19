# lifecycle_manager — known limitations

Package design, tree layout, multi-instance use, and how to add a managed
node: `ws/src/lifecycle_manager/README.md`.

- [ ] **`IsInState` shows as Action in Groot2** — `RosServiceNode` inherits
  from `ActionNodeBase`, not `ConditionNode`. Cosmetic only, no runtime
  impact.
