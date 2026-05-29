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
        # 坐标系和目标位置输出话题。
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        # 条形码观察点 = 杆子坐标向左偏 0.5m，高度保持 105cm；主航线高度在源码里是 140cm。
        "pillar_left_offset_m": 0.5,
        "barcode_target_z_cm": 105.0,
        # 到达判定容差。
        "position_tolerance_cm": 6.0,
        "yaw_tolerance_deg": 5.0,
        "height_tolerance_cm": 6.0,
        # 撒药颜色判断和激光闪烁参数。
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
