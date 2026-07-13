# map

Saved SLAM Toolbox maps land here — this is `async_slam_toolbox_node`'s
working directory (set via `cwd` in `ws/src/launch/launch/mserve_slam.launch.py`),
so both the web UI (`web/lidar.html`'s SLAM Toolbox section) and manual
`ros2 service call` saves resolve a bare name like `my_map` to a file in
this folder, no path typing needed.

Two different things get saved here, for two different purposes — same base
name, different extensions:

- **Save Map** (`/slam_toolbox/save_map`) → `<name>.pgm` + `<name>.yaml` —
  a standard ROS map, for viewing/serving as a static map.
- **Serialize Map** (`/slam_toolbox/serialize_map`) → `<name>.posegraph` +
  `<name>.data` — slam_toolbox's own internal pose-graph format. This is
  what `--slam-local` / `slam_params_local.yaml`'s `map_file_name` actually
  needs to localize against a saved map — `save_map`'s output alone isn't
  enough for that.

To use a saved map for localization, point `map_file_name` in
`ws/src/interfaces/config/slam_params_local.yaml` at
`~/mServe-STACK/map/<name>` (no extension).
