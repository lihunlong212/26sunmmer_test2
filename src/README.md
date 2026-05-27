# ROS 2 Workspace Overview

This workspace is configured for the 2021 plant-protection UAV task.

## Main Data Flow

- `activity_control_pkg` publishes `/target_position`, manages the waypoint queue, and triggers `/laser/cmd=3` only at spray waypoints that pass color detection.
- `drone_camera_pkg` reads the downward camera center ROI and publishes `/spray_allowed`.
- `laser_control_pkg` subscribes to `/laser/cmd` and controls WiringOP pin 10. Low level turns the laser on, high level turns it off.
- `pid_control_pkg` converts `/target_position` and `/height` into `/target_velocity`.
- `uart_to_stm32` forwards `/target_velocity` during an active route, publishes `/height`, sends `/led_digit` values 1/2/3 as serial frame `0x12`, and sends mission completion as frame `0x66`.

## Route Parameters

Waypoints are edited only in `activity_control_pkg/src/route_target_publisher.cpp`.

`demo1.launch.py` starts `route_target_publisher_node`, and the source-defined route begins immediately after launch. `Target{..., true}` runs the color check and laser pulse; `Target{...}` or `Target{..., false}` is a normal waypoint.

## Color Detection

`drone_camera_node` only checks the center 50x50 ROI of the image. If green pixels are more than 30% of those 2500 pixels, it publishes `/spray_allowed=true`; otherwise it publishes `false`, so the aircraft skips spraying by default.

Important camera parameters:

- `center_roi_width`, `center_roi_height`
- `green_h_min`, `green_h_max`, `green_s_min`, `green_v_min`
- `green_ratio_threshold`

## Common Commands

```bash
colcon build --packages-select activity_control_pkg drone_camera_pkg laser_control_pkg pid_control_pkg uart_to_stm32
ros2 launch my_launch demo1.launch.py
ros2 topic echo /spray_allowed
ros2 topic pub /led_digit std_msgs/msg/UInt8 "{data: 2}" --once
```
