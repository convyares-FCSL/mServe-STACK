# mserve_lidar — known limitations

Package design, SDK vendoring rationale, and parameters:
`ws/src/mserve_lidar/README.md`.

- [ ] **No angle compensation** — see README.md's "Scan message construction"
  section. `ranges[]` is built directly from one revolution's raw ascending
  samples (evenly-spaced-angle approximation, same one every RPLIDAR ROS
  driver uses), not redistributed into fixed 1-degree bins. Revisit if a
  downstream consumer (Nav2 costmap, scan matcher) turns out to need that.
- [ ] **No tests** — most of the package is SDK/serial I/O, but the parameter
  validation in `lidar_params.cpp` is unit-testable.
