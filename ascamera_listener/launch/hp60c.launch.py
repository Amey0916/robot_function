from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os

def generate_launch_description():

    workspace_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

    config_file_path = os.path.join(workspace_path, 'ascamera_listener', 'configurationfiles')


    ascamera_node_1 = Node(
        namespace= "",
        package='ascamera_listener',
        executable='ascamera_listener_node',
        name='ascamera_hp60c_1',
        respawn=True,
        output='both',
        parameters=[
            {"usb_bus_no": -1},
            {"usb_path": ""},
            {"confiPath": config_file_path},
            {"color_pcl": False},
            {"depth_width": 320},
            {"depth_height": 240},
            {"rgb_width": 640},
            {"rgb_height": 480},
            {"fps": 15},
        ],
        remappings=[]
    )
    
    
    pointcloud_to_laserscan_1 = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/ascamera_hp60c_1/depth/points'),
                    ('scan', '/scan1')],
        parameters=[{
            'target_frame': 'ascamera_hp60c_1',
            'transform_tolerance': 0.01,
            'min_height': -0.1,
            'max_height': 2.0,
            'angle_min': -1.5708, 
            'angle_max': 1.5708, 
            'angle_increment': 0.0087,
            'scan_time': 0.3333,
            'range_min': 0.05,
            'range_max': 1.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan_1'
    )
    
    
    static_transform_publisher_pc_sc_1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_pc_sc_1',
        arguments=[ '0.245', '0', '0.2', '0', '0', '0', '1', 'base_link', 'ascamera_hp60c_1']
    )

    return LaunchDescription([
    ascamera_node_1,
    pointcloud_to_laserscan_1,
    static_transform_publisher_pc_sc_1])




