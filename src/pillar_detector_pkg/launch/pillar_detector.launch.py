from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pillar_detector_pkg",
            executable="pillar_detector_node",
            name="pillar_detector",
            output="screen",
            parameters=[{
                # 雷达输入话题。
                "scan_topic": "/scan",
                # 杆子坐标输出话题，格式为 Float32MultiArray [x_m, y_m]。
                "output_topic": "/detected_pillar",
                # 搜索杆子的 map 坐标矩形区域，单位 m。
                "map_x_min_m": 0.5,
                "map_x_max_m": 2.2,
                # y 是负方向区域，所以这里表示只搜索 -2.2m 到 -0.5m 之间。
                "map_y_min_m": -2.2,
                "map_y_max_m": -0.5,
                # 单帧聚类距离，单位 m；相邻雷达点距离小于该值会被归为同一组。
                "group_dist_m": 0.25,
                # 单帧里一组点至少要有这么多个点，才可能被认为是杆子候选。
                "min_pts_per_group": 3,
                # 两个杆子候选之间至少要相隔这么远，单位 m；本题只有一个杆子，用于去重。
                "min_pillar_separation_m": 0.40,
                # 连续累计多少帧雷达数据后再输出最终杆子坐标。
                "accumulation_frames": 20,
                # 多帧候选点合并距离，单位 m；距离小于该值认为是同一个杆子的多次观测。
                "cluster_merge_dist_m": 0.20,
                # 20 帧里至少命中这么多票，才确认杆子有效。
                "min_votes": 8,
            }],
        )
    ])
