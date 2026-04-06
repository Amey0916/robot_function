# 导入必要的launch模块
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1. 包含ascamera_listener的hp60c_multiple.launch.py
    ascamera_pkg_dir = get_package_share_directory('ascamera_listener')
    hp60c_launch_path = os.path.join(
        ascamera_pkg_dir,
        'launch',
        'hp60c_multiple.launch.py'
    )
    ascamera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(hp60c_launch_path)
    )

    # 2. 定义gesture_voice_control功能包的节点
    gesture_body_detection = Node(
        package='gesture_voice_control',
        executable='gesture_body_detection_node',  # 保持你实际可运行的名字
        name='gesture_body_detection_node',
        output='screen',
        emulate_tty=True
    )

    voice_recognition = Node(
        package='gesture_voice_control',
        executable='voice_recognition',
        name='voice_recognition',
        output='screen',
        emulate_tty=True
    )

    device_control = Node(
        package='gesture_voice_control',
        executable='device_control',
        name='device_control',
        output='screen',
        emulate_tty=True
    )

    # 3. 定义my_robot_apps功能包的节点
    turn_to_person = Node(
        package='my_robot_apps',
        executable='turn_to_person_node',
        name='turn_to_person_node',
        output='screen',
        emulate_tty=True
    )

    depth_distance = Node(
        package='my_robot_apps',
        executable='depth_distance_node',
        name='depth_distance_node',
        output='screen',
        emulate_tty=True
    )

    follow_by_distance = Node(
        package='my_robot_apps',
        executable='follow_by_distance_node',
        name='follow_by_distance_node',
        output='screen',
        emulate_tty=True
    )

    # ✅ 4. 新增：come_mode_node
    come_mode = Node(
        package='my_robot_apps',
        executable='come_mode_node',
        name='come_mode_node',
        output='screen',
        emulate_tty=True
    )

    # 5. 组装所有启动项
    ld = LaunchDescription()
    ld.add_action(ascamera_launch)

    ld.add_action(gesture_body_detection)
    ld.add_action(voice_recognition)
    ld.add_action(device_control)

    ld.add_action(turn_to_person)
    ld.add_action(depth_distance)
    ld.add_action(follow_by_distance)
    ld.add_action(come_mode)

    return ld