import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('armor_detector')
    rosbag_path = os.path.join(
        os.path.dirname(package_share), '..', '..', '..', 'src',
        'armor_detector', 'Test', 'video', 'video1'
    )

    config = os.path.join(package_share, 'config', 'config.yaml')

    frame_count = LaunchConfiguration('frame_count')
    timing_interval = LaunchConfiguration('timing_interval')

    rosbag_player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', rosbag_path,
            '--start-paused',
            '--disable-keyboard-controls',
            '--topics', '/image_raw',
            '--remap', '__node:=rosbag2_player'
        ],
        output='screen'
    )

    detector = Node(
        package='armor_detector',
        executable='armor_detector_node',
        name='armor_detector_node_cpp',
        parameters=[
            config,
            {
                'debug.show': False,
                'debug.pose': False,
                'debug.result': False,
                'debug.rosbag_control': True,
                'debug.stats_interval': timing_interval,
                'playback.mode': 'step',
                'playback.max_frames': frame_count,
                'playback.exit_on_complete': True,
            },
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('frame_count', default_value='150'),
        DeclareLaunchArgument('timing_interval', default_value='50'),

        rosbag_player,
        detector,

        # detector 退出后关闭 rosbag player
        RegisterEventHandler(
            OnProcessExit(
                target_action=detector,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='auto test completed'))
                ]
            )
        ),
    ])
