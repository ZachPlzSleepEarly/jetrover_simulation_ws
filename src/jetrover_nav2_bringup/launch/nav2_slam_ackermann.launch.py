import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_jetrover_nav2 = get_package_share_directory('jetrover_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    nav2_params_file = os.path.join(
        pkg_jetrover_nav2,
        'params',
        'nav2_params.yaml'
    )

    slam_params_file = os.path.join(
        pkg_jetrover_nav2,
        'params',
        'slam_toolbox_async.yaml'
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file,
        }.items()
    )

    # Use navigation_launch.py, not bringup_launch.py.
    # In SLAM mode, slam_toolbox provides /map and map->odom.
    # Do not start a static map->odom TF publisher.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params_file,
        }.items()
    )

    cmd_vel_to_ackermann_ref = Node(
        package='jetrover_description',
        executable='cmd_vel_to_ackermann_ref.py',
        name='cmd_vel_to_ackermann_ref',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'input_topic': '/cmd_vel',
            'output_topic': '/ackermann_like_controller/reference',
            'frame_id': 'base_link',
        }]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d',
            os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz')
        ],
        condition=None,
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='True'),

        slam_toolbox_launch,
        nav2_navigation_launch,
        cmd_vel_to_ackermann_ref,
        rviz,
    ])
