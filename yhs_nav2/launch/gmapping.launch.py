import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('yhs_nav2')
    launch_dir = os.path.join(bringup_dir, 'launch')
    
    imu_launch_dir = os.path.join(get_package_share_directory('serial_imu'),'launch')
    lidar_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'),'launch')
    chassis_launch_dir = os.path.join(get_package_share_directory('yhs_can_control'),'launch')
    gmapping_launch_dir = os.path.join(get_package_share_directory('slam_gmapping'),'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_tf = LaunchConfiguration('use_tf')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
                  
    declare_use_tf_cmd = DeclareLaunchArgument('use_tf', default_value='true', description='is use_tf?')


    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

            
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(imu_launch_dir, 'imu_spec_msg.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn}.items()),
                              
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'nvilidar.launch.py')),
        #     launch_arguments={'namespace': namespace,
        #                       'use_sim_time': use_sim_time,
        #                       'autostart': autostart,
        #                       'use_respawn': use_respawn}.items()),

        #激光雷达节点                  
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'rplidar_s2_gmapping_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn}.items()),
                              
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(chassis_launch_dir, 'yhs_can_control.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'use_tf': use_tf}.items()),
                              
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gmapping_launch_dir, 'slam_gmapping.launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn}.items()),

    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_tf_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # -------- 自动启动原点Marker节点 --------
    origin_marker_node = Node(
        package='yhs_nav2',
        executable='origin_marker.py',
        name='origin_marker_node',
        output='screen'
    )
    ld.add_action(origin_marker_node) # 添加到launch中

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld
