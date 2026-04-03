import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link', description='Frame ID for the IMU sensor'),

        Node(
            package='serial_imu',  
            executable='serial_imu',  
            name='serial_imu_node',
            namespace='',
            output='screen',
            parameters=[], 
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_imu_transform_publisher',
            namespace='',
            output='screen',
            arguments=[str(0), str(0), str(0), str(0), str(0), str(0), str(1),"base_link", LaunchConfiguration('imu_frame_id')],
            parameters=[],
        )
    ])

