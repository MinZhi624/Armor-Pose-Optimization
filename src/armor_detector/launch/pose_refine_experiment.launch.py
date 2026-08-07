import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('armor_detector')
    base_path = os.path.join(
        os.path.dirname(package_share), '..', '..', '..', 'src',
        'armor_detector', 'Test', 'video'
    )

    # Derive workspace root from package_share
    # package_share = install/armor_detector/share/armor_detector/
    # Need 4 levels up to reach project root: share -> armor_detector -> install -> workspace
    workspace_root = os.path.abspath(os.path.join(package_share, '..', '..', '..', '..'))

    config = os.path.join(package_share, 'config', 'config.yaml')

    video = LaunchConfiguration('video')
    frame_count = LaunchConfiguration('frame_count')
    timing_interval = LaunchConfiguration('timing_interval')
    use_foxglove = LaunchConfiguration('use_foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')

    rosbag_path = PathJoinSubstitution([base_path, video])

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
                'debug.pose': True,
                'debug.rosbag_control': True,
                'debug.stats_interval': timing_interval,
                'debug.pose_refine_csv.enabled': True,
                'debug.pose_refine_csv.root_dir': workspace_root,
                'debug.pose_refine_csv.video': video,
                'debug.pose_refine_topic.enabled': True,
                'playback.mode': 'step',
                'playback.max_frames': frame_count,
                'playback.exit_on_complete': True,
            },
        ],
        output='screen'
    )

    foxglove_bridge = ExecuteProcess(
        condition=IfCondition(use_foxglove),
        cmd=['ros2', 'run', 'foxglove_bridge', 'foxglove_bridge',
             '--ros-args', '-p', ['port:=', foxglove_port]],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('video', default_value='video1'),
        DeclareLaunchArgument('frame_count', default_value='0'),
        DeclareLaunchArgument('timing_interval', default_value='50'),
        DeclareLaunchArgument('use_foxglove', default_value='true'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),

        rosbag_player,
        detector,
        foxglove_bridge,

        # detector 退出后关闭 rosbag player 和 foxglove
        RegisterEventHandler(
            OnProcessExit(
                target_action=detector,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='pose_refine experiment completed'))
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=rosbag_player,
                on_exit=[
                    EmitEvent(event=Shutdown(reason='rosbag playback ended'))
                ]
            )
        ),
    ])
