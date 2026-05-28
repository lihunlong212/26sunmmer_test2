from importlib import import_module
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    LaunchDescription = Any
    Node = Any


def generate_launch_description():
    launch_module = import_module("launch")
    launch_ros_actions = import_module("launch_ros.actions")
    LaunchDescription = getattr(launch_module, "LaunchDescription")
    Node = getattr(launch_ros_actions, "Node")

    route_params = {
        # Frames and target output
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        "pillar_left_offset_m": 0.8,
        "barcode_target_z_cm": 130.0,
        # Reach tolerances
        "position_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "height_tolerance_cm": 6.0,
        # Spray gating
        "spray_decision_timeout_sec": 1.5,
        "spray_data_stale_timeout_sec": 0.5,
        "spray_flash_on_sec": 0.3,
        "spray_flash_gap_sec": 0.3,
        "laser_on_command": 1,
        "laser_off_command": 2,
    }

    return LaunchDescription([
        Node(
            package="activity_control_pkg",
            executable="route_target_publisher_node",
            name="route_target_publisher",
            output="screen",
            parameters=[route_params],
        )
    ])
