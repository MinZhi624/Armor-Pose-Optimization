import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('armor_detector')
    rosbag_path = os.path.join(
        os.path.dirname(package_share), '..', '..', '..', 'src',
        'armor_detector', 'Test', 'video', 'video1'
    )

    config = os.path.join(package_share, 'config', 'config.yaml')

    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

        # 播放 rosbag
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play', rosbag_path, '--loop',
                '--disable-keyboard-controls',
                '--remap', '__node:=rosbag2_player'
            ],
            output='screen'
        ),

        # 启动 armor_detector 节点
        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector_node_cpp',
            parameters=[config],
            output='screen'
        ),

        # Foxglove bridge
        ExecuteProcess(
            condition=IfCondition(use_foxglove),
            cmd=['ros2', 'run', 'foxglove_bridge', 'foxglove_bridge',
                 '--ros-args', '-p', ['port:=', foxglove_port]],
            output='screen'
        ),
    ])
