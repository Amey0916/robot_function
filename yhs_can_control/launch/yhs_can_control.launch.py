#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import os

def generate_launch_description():
    share_dir = get_package_share_directory('yhs_can_control')
    parameter_file = LaunchConfiguration('params_file_ch')
    use_tf = LaunchConfiguration('use_tf')
    
    use_tf_declare = DeclareLaunchArgument('use_tf', default_value='false', description='Use TF parameter')

    params_declare = DeclareLaunchArgument('params_file_ch',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'cfg.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    yhs_can_control_node = Node(package='yhs_can_control',
                                executable='yhs_can_control_node',
                                name='yhs_can_control_node',
                                output='screen',
                                parameters=[{parameter_file},{'use_tf': use_tf}]
                                )

    return LaunchDescription([
        params_declare,
        use_tf_declare,
        yhs_can_control_node
    ])

