import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, RegisterEventHandler
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
    analysis_script = os.path.join(
        workspace_root, 'debug', 'pose_refine', 'plot_pose_landscape.py'
    )

    config = os.path.join(package_share, 'config', 'config.yaml')

    video = LaunchConfiguration('video')
    frame_count = LaunchConfiguration('frame_count')
    timing_interval = LaunchConfiguration('timing_interval')

    rosbag_path = PathJoinSubstitution([base_path, video])
    landscape_run_parent = PathJoinSubstitution([
        workspace_root, 'debug', 'pose_refine', 'pose_landscape', video
    ])

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
                'debug.pose_landscape.enabled': True,
                'debug.pose_landscape.root_dir': workspace_root,
                'debug.pose_landscape.video': video,
                'playback.mode': 'step',
                'playback.max_frames': frame_count,
                'playback.exit_on_complete': True,
            },
        ],
        output='screen'
    )

    plotter = ExecuteProcess(
        cmd=['python3', analysis_script, '--latest-run', landscape_run_parent],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('video', default_value='video1'),
        DeclareLaunchArgument('frame_count', default_value='50'),
        DeclareLaunchArgument('timing_interval', default_value='50'),

        rosbag_player,
        detector,

        # 采集结束后为本次最新运行的所有样本直接生成 PNG。
        RegisterEventHandler(
            OnProcessExit(
                target_action=detector,
                on_exit=[
                    plotter,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=plotter,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason='pose_landscape collection and plotting completed')
                    )
                ]
            )
        ),
    ])
