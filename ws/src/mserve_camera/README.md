# mserve_camera

Lifecycle-managed camera node for mServe. Wraps `v4l2_camera`'s
`V4l2CameraDevice` (raw V4L2 device access, not the whole `v4l2_camera_node`)
so bringup/shutdown is gated the same way as `mserve_drivechain`/`mserve_base`,
via `lifecycle_manager`.

## Why not just run `v4l2_camera_node` directly?

`v4l2_camera_node` is a complete, working driver — but it's a plain
`rclcpp::Node`, not a lifecycle node, so it can't be configured/activated in
step with the rest of the stack or included in `lifecycle_manager`'s
bringup/shutdown trees. `V4l2CameraDevice` (the class that actually talks to
`/dev/videoN`) is a clean, non-ROS class with `open()`/`start()`/`stop()`/
`capture()` — `mserve_camera` wraps that directly, the same way
`mserve_drivechain` wraps `DriveUart` instead of depending on someone else's
node.

## Architecture

```
CameraNode (lifecycle)
│
├── on_configure   opens the V4L2 device, negotiates width/height/YUYV
├── on_activate    starts the device streaming + a dedicated capture thread
├── on_deactivate  joins the capture thread, stops the device
├── on_cleanup     releases the device
├── on_shutdown    same as cleanup — safe to call from any state
│
├── Publisher  camera/image_raw     (sensor_msgs/Image, encoding=yuv422_yuy2)
└── Publisher  camera/camera_info   (sensor_msgs/CameraInfo, uncalibrated placeholder)
```

Unlike `mserve_drivechain`/`mserve_base`, there's no BehaviorTree here —
V4L2 capture is a simple blocking loop (`capture()` blocks until a frame is
ready), not a multi-step hardware handshake needing retries/sequencing, so a
dedicated capture thread is enough. This matches how upstream
`v4l2_camera_node` itself runs its own `capture_thread_`.

### Key source files

| File | Purpose |
|---|---|
| `include/mserve_camera/camera_node.hpp` | Node public API |
| `include/mserve_camera/camera_limits.hpp` | Width/height bounds for parameter descriptors |
| `src/camera_node.cpp` | Lifecycle callbacks + capture loop |
| `src/camera_params.cpp` | Parameter declaration, loading, hot-change guard |
| `src/main.cpp` | Entry point |
| `docs/todo.md` | Known limitations, next steps |

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `device` | `/dev/video0` | V4L2 device path |
| `width` | `640` | Capture width (px) |
| `height` | `480` | Capture height (px) |
| `frame_id` | `camera_link_optical` | REP-103 optical frame — matches `mserve_camera.xacro`/`mserve_depth_camera.xacro`'s `camera_link → camera_link_optical` joint, not the physical mount frame |

`device`/`width`/`height` can only be changed in `UNCONFIGURED` state (reopening
the V4L2 device mid-stream isn't supported). Pixel format is fixed to YUYV —
see [Pixel format choice](#pixel-format-choice) below.

## Pixel format choice

Only `YUYV` (uncompressed 4:2:2) is requested — not `MJPG` or `H264`, even
though most USB webcams support those too. Reasoning, from the actual webcam
used during development (`v4l2-ctl --list-formats-ext -d /dev/video0`):

| Format | 640x480 fps | Notes |
|---|---|---|
| `YUYV` | 30 | Uncompressed — buffer copies straight into `sensor_msgs/Image` with `encoding="yuv422_yuy2"`, zero conversion code |
| `MJPG` | 30 (up to 1920x1080) | Needs a JPEG decode step (extra dependency, extra code) for no benefit at this resolution |
| `H264` | 30 | Needs a video decoder — real overkill for a single still-frame-at-a-time robot camera |

If you need higher resolution than 640x480 at full frame rate, this camera's
`YUYV` mode drops to 5-10fps there (see the table in `mserve_camera.README`'s
git history / re-run `v4l2-ctl --list-formats-ext` for your camera) — at that
point switching to `MJPG` and adding a decode step becomes worth it. Not
done yet.

**Known gap**: only pixel format + resolution are requested via
`requestDataFormat()` — the frame *interval* (fps) is never explicitly set
via `VIDIOC_S_PARM`, so the driver keeps whatever interval was last active.
Observed rate on the dev webcam was ~12.6 Hz instead of the format table's
advertised 30 Hz. Fixing this means calling the V4L2 `VIDIOC_S_PARM` ioctl
directly (`V4l2CameraDevice` doesn't expose a wrapper for it) — not done yet,
see `docs/todo.md`.

## Build

```bash
cd ~/mServe-STACK/ws
colcon build --packages-select interfaces utils mserve_camera --cmake-args -DBUILD_TESTING=OFF --symlink-install
source install/setup.bash
```

`v4l2_camera`'s exported CMake target doesn't declare its own transitive
link dependencies (`cv_bridge`, `camera_info_manager`, `image_transport`,
`rclcpp_components`) as find-able config packages — `mserve_camera`'s
`CMakeLists.txt` `find_package()`s them explicitly first. If you see
`v4l2_cameraConfig.cmake` fail with "target not found", that's why.

## Device permissions

`/dev/video0` is owned by group `video`. If the node fails to open the
device with a permissions error:

```bash
sudo usermod -aG video $USER
newgrp video   # or log out/in — group membership doesn't apply to already-open shells
```

## Run

### Via launch (recommended)

```bash
cd ~/mServe-STACK/ws
source install/setup.bash
ros2 launch launch mserve_min.launch.py backend:=sim
```

`camera_node` is launched alongside `mserve_drivechain`/`mserve_base`, and
`lifecycle_manager` configures/activates it as part of the same bringup
tree. `web/run_drivechain_hw.sh` is the normal entry point — see the
top-level readme.

### Manually

```bash
source install/setup.bash
ros2 run mserve_camera camera_node

# separate terminal
ros2 lifecycle set /mserve_camera configure
ros2 lifecycle set /mserve_camera activate

ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --field encoding
```

## Web UI

`web/camera.html` shows lifecycle state, parameters, a live image preview
(via `web_video_server`'s MJPEG stream — raw `sensor_msgs/Image` in YUYV
can't be decoded directly in a browser, so `web_video_server` transcodes it
server-side), `camera_info`, and `/rosout` logs, matching the pattern of
`web/drivechain.html`. See the top-level `web/README.md`.

## Tests

None yet — see `docs/todo.md`.
