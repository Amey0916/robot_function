from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # 探索节点（核心逻辑保留）
    explore_node = Node(
        package="explore_lite_ros2",
        executable="explore_node",
        name="explore_lite_node",
        output="screen",
        parameters=[{
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            "planner_frequency": 0.1,
            "progress_timeout": 45.0,
            "visualize": True,
            "potential_scale": 1e-3,
            "gain_scale": 1.0,
            "min_frontier_size": 0.8,  # 前沿最小尺寸（米）
            "max_frontier_distance": 8.0,  # 前沿最大距离（米）
            "costmap_topic": "global_costmap/costmap",
            "costmap_updates_topic": "global_costmap/costmap_updates",
            "robot_base_frame": "base_link",
            "transform_tolerance": 5.0,
            "action_server_name": "navigate_to_pose",
            "map_topic": "/map",
            "action_timeout": 15.0,  # 动作请求超时，匹配bt_navigator响应节奏
            'use_sim_time': False
        }]
    )

    delayed_explore = TimerAction(
        period=40.0,
        actions=[explore_node]
    )

    return LaunchDescription([
        delayed_explore,
    ])
