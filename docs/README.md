# mServe Docs

All project documentation lives here, except the root `readme.md` entry
point and the per-package READMEs under `ws/src/*/README.md` (which are the
source of truth for each package's design and parameters).

## Files

- `operations.md`: the full run/build/debug reference — every `run_stack.sh`
  flag, SLAM→Nav2 workflow, simulation on the Thor, systemd, logs.
- `architecture.md`: design philosophy, control boundaries, lifecycle rules,
  C++ package conventions.
- `TODO.md`: task tracker — statements of what's next; history lives in
  `git log`.
- `camera/todo.md`, `lidar/todo.md`, `lifecycle_manager/{todo,lesson_plan}.md`:
  per-package known-limitations/notes. Kept here, not alongside package
  source, so all project docs stay in one place; each package README points
  here.

## Source Guide

The C++ lessons in `/home/ecm/ros2-systems-operability/src/2_cpp` are the
main style guide:

- Lesson 05: central YAML configuration.
- Lesson 06: lifecycle nodes.
- Lesson 07: actions.
- Lesson 08: callback groups and executors.
- Lesson 09: composition.
- Lesson 10: launch topology and deployment verification.
