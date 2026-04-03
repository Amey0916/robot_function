from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config=get_package_share_directory('tmlidar_sdk')+'/rviz/rviz2.rviz'
    return LaunchDescription([
        Node(
            package='tmlidar_sdk',
            namespace='tmlidar_sdk',
            executable='tmlidar_sdk_node',
            name='tmlidar_sdk_node',
            output='screen'
        ),
        
    		Node(
        		package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        		remappings=[('cloud_in', '/tmlidar_points'),
                    ('scan', '/scan')],
        		parameters=[{
            		'target_frame': 'laser_link',
            		'transform_tolerance': 0.01,
            		'min_height': -0.3,
            		'max_height': 2.0,
            		'angle_min': -3.14, 
            		'angle_max': 3.14, 
            		'angle_increment': 0.0087,
            		'scan_time': 0.3333,
            		'range_min': 0.3,
            		'range_max': 50.0,
            		'use_inf': True,
            		'inf_epsilon': 1.0
        		}],
        		name='pointcloud_to_laserscan'
    		),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf',
						arguments=['0.16','0','0.56','0','0','0','1','base_link','laser_link']
        )
    ])