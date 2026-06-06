# compressor

Lifecycle ROS 2 node for the compressor package.

## Build

From the workspace root:

```bash
colcon build --packages-select compressor
```

Source the workspace after building:

```bash
source install/setup.bash
```

## Run The Node

Start the lifecycle node:

```bash
ros2 run compressor compressor_main
```

The node currently starts with the ROS node name:

```bash
/compressor_twin
```

## Check Lifecycle State

In another terminal, source the workspace:

```bash
source install/setup.bash
```

List lifecycle nodes:

```bash
ros2 lifecycle nodes
```

Check the compressor node state:

```bash
ros2 lifecycle get /compressor_twin
```

## Change Lifecycle State

Configure the node:

```bash
ros2 lifecycle set /compressor_twin configure
```

Activate the node:

```bash
ros2 lifecycle set /compressor_twin activate
```

Deactivate the node:

```bash
ros2 lifecycle set /compressor_twin deactivate
```

Clean up the node:

```bash
ros2 lifecycle set /compressor_twin cleanup
```

Configure and activate again after cleanup:

```bash
ros2 lifecycle set /compressor_twin configure
ros2 lifecycle set /compressor_twin activate
```

Shutdown the node:

```bash
ros2 lifecycle set /compressor_twin shutdown
```

## Typical Manual Test Flow

Terminal 1:

```bash
source install/setup.bash
ros2 run compressor compressor_main
```

Terminal 2:

```bash
source install/setup.bash
ros2 lifecycle get /compressor_twin
ros2 lifecycle set /compressor_twin configure
ros2 lifecycle get /compressor_twin
ros2 lifecycle set /compressor_twin activate
ros2 lifecycle get /compressor_twin
ros2 lifecycle set /compressor_twin deactivate
ros2 lifecycle set /compressor_twin cleanup
ros2 lifecycle set /compressor_twin shutdown
```

