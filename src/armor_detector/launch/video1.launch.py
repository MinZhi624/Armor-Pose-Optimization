import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('armor_detector')
    rosbag_path = os.path.join(
        os.path.dirname(package_share), '..', '..', '..', 'src',
        'armor_detector', 'Test', 'video', 'video1'
    )

    config = os.path.join(package_share, 'config', 'config.yaml')

    return LaunchDescription([
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
    ])
