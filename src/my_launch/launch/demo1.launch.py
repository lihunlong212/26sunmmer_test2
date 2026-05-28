import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_path(package_name: str, filename: str) -> str:
    package_share = FindPackageShare(package=package_name).find(package_name)
    return os.path.join(package_share, "launch", filename)


def generate_launch_description():
    # 1. 地图、定位、雷达
    cartography_launch_args = {
        # 是否打开 RViz。比赛跑飞时建议 false，调地图时可以改成 true。
        "use_rviz": "false",
    }

    # 2. STM32 串口桥
    uart_params = {
        # 串口桥更新频率，单位 Hz。
        "update_rate": 100.0,
        # TF 源坐标系。
        "source_frame": "map",
        # 飞机机体/雷达坐标系。
        "target_frame": "laser_link",
        # demo1 启动后直接开始任务，所以速度转发立即开启。
        "target_velocity_forwarding_auto_enable": True,
    }

    # 3. 位置 PID 控制
    pid_params = {
        # PID 更新频率，单位 Hz。
        "control_frequency": 50.0,
        # 位置控制使用的坐标系。
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        # XY 平面位置 PID 参数。
        "kp_xy": 0.8,
        "ki_xy": 0.0,
        "kd_xy": 0.2,
        # 偏航角 PID 参数。
        "kp_yaw": 1.0,
        "ki_yaw": 0.0,
        "kd_yaw": 0.2,
        # 高度 PID 参数。
        "kp_z": 1.0,
        "ki_z": 0.0,
        "kd_z": 0.2,
        # 速度限幅，单位分别是 cm/s、deg/s、cm/s。
        "max_linear_velocity": 33.0,
        "max_angular_velocity": 30.0,
        "max_vertical_velocity": 30.0,
    }

    # 4. 航线执行与撒药判断
    route_params = {
        # 坐标系和目标位置输出话题。
        "map_frame": "map",
        "laser_link_frame": "laser_link",
        "output_topic": "/target_position",
        # 启动后先等待雷达发布 /detected_pillar，再起飞。
        # 条形码观察点 = 柱子坐标向左偏 0.8m：例如 (1, -1) -> (1, -0.2)。
        "pillar_left_offset_m": 0.8,
        "barcode_target_z_cm": 130.0,
        # 到达判定容差，单位分别是 cm、deg、cm。
        "position_tolerance_cm": 8.0,
        "yaw_tolerance_deg": 8.0,
        "height_tolerance_cm": 8.0,
        # 到达需要撒药的航点后，最多等待视觉结果多久，单位秒。
        "spray_decision_timeout_sec": 1.5,
        # /spray_allowed 超过这个时间没更新，就认为视觉数据过期，单位秒。
        "spray_data_stale_timeout_sec": 0.5,
        # 绿色时打激光：亮 0.3 秒、灭 0.3 秒、再亮 0.3 秒、最后灭灯。
        "spray_flash_on_sec": 0.3,
        "spray_flash_gap_sec": 0.3,
        # 发给 laser_control_pkg 的命令，1=开灯，2=关灯。
        "laser_on_command": 1,
        "laser_off_command": 2,
        # 航点坐标只在 src/activity_control_pkg/src/route_target_publisher.cpp 中修改。
        # 不要在 launch 文件里放航点数组，避免飞行路线和源码不一致。
    }

    # 5. 激光 GPIO 控制
    laser_params = {
        # WiringOP 引脚号。当前接线：pin10，低电平出光，高电平关光。
        "pin": 10,
        "on_level": 0,
        "off_level": 1,
        # 节点启动时先确保激光关闭。
        "initial_off": True,
        # 手动发送 /laser/cmd=3 时的单次脉冲持续时间，单位秒；航线撒药不用这个参数。
        "pulse_duration": 0.3,
        # 激光命令和状态话题。
        "command_topic": "/laser/cmd",
        "status_topic": "/laser/status",
    }

    # 6. 下视相机颜色识别
    camera_params = {
        # 相机设备和采集参数。
        "camera_device": "/dev/video2",
        "frame_width": 640,
        "frame_height": 480,
        "fps": 15.0,
        "window_name": "drone_camera_preview",
        # 只判断图像正中心 50x50 像素。
        "center_roi_width": 50,
        "center_roi_height": 50,
        # HSV 绿色阈值。
        "green_h_min": 35,
        "green_h_max": 90,
        "green_s_min": 45,
        "green_v_min": 60,
        # 绿色像素占中心 2500 像素的比例，大于 0.10 就认为可以撒药。
        "green_ratio_threshold": 0.10,
        # 颜色识别只输出这个打药判断话题；测试时看 ros2 topic echo /spray_allowed。
        "spray_allowed_topic": "/spray_allowed",
    }

    # 7. 单杆雷达检测
    barcode_params = {
        # 条形码摄像头设备。Code128 条形码使用 video0，颜色识别使用 video2。
        "camera_device": "/dev/video0",
        "frame_width": 640,
        "frame_height": 480,
        "fps": 15.0,
        # 条形码识别结果输出话题，消息类型 std_msgs/String。
        "barcode_topic": "/barcode_text",
        # 是否显示条形码摄像头预览窗口；SSH 无桌面时改成 False。
        "show_preview": True,
        "window_name": "barcode_camera_preview",
        # False 表示同一个条形码只在内容变化时发布一次；True 表示每帧识别到都发布。
        "publish_duplicates": False,
        # 第一次识别到条形码并发布 /barcode_text 后关闭 video0。
        "stop_after_first_publish": True,
    }

    # 8. 单杆雷达检测
    pillar_params = {
        # 雷达输入和杆子坐标输出话题。输出格式：Float32MultiArray [x_m, y_m]。
        "scan_topic": "/scan",
        "output_topic": "/detected_pillar",
        # 搜索杆子的矩形区域，单位 m。
        "map_x_min_m": 0.5,
        "map_x_max_m": 2.0,
        "map_y_min_m": -2.0,
        "map_y_max_m": -0.5,
        # 单帧聚类参数。
        "group_dist_m": 0.25,
        "min_pts_per_group": 4,
        "min_pillar_separation_m": 0.40,
        # 多帧累计投票参数。
        "accumulation_frames": 20,
        "cluster_merge_dist_m": 0.20,
        "min_votes": 8,
    }

    fly_carto_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_launch_path("my_carto_pkg", "fly_carto.launch.py")),
        launch_arguments=cartography_launch_args.items(),
    )

    uart_node = Node(
        package="uart_to_stm32",
        executable="uart_to_stm32_node",
        name="uart_to_stm32",
        output="screen",
        parameters=[uart_params],
    )

    position_pid_controller_node = Node(
        package="pid_control_pkg",
        executable="position_pid_controller",
        name="position_pid_controller",
        output="screen",
        parameters=[pid_params],
    )

    route_node = Node(
        package="activity_control_pkg",
        executable="route_target_publisher_node",
        name="route_target_publisher",
        output="screen",
        parameters=[route_params],
    )

    laser_control_node = Node(
        package="laser_control_pkg",
        executable="laser_control_node",
        name="laser_control_node",
        output="screen",
        parameters=[laser_params],
    )

    drone_camera_node = Node(
        package="drone_camera_pkg",
        executable="drone_camera_node",
        name="drone_camera_node",
        output="screen",
        parameters=[camera_params],
    )

    barcode_camera_node = Node(
        package="barcode_camera_pkg",
        executable="barcode_camera_node",
        name="barcode_camera_node",
        output="screen",
        parameters=[barcode_params],
    )

    pillar_detector_node = Node(
        package="pillar_detector_pkg",
        executable="pillar_detector_node",
        name="pillar_detector",
        output="screen",
        parameters=[pillar_params],
    )

    return LaunchDescription([
        fly_carto_launch,
        uart_node,
        position_pid_controller_node,
        route_node,
        laser_control_node,
        drone_camera_node,
        barcode_camera_node,
        pillar_detector_node,
    ])
