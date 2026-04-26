import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_jetrover_bringup = get_package_share_directory('jetrover_nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')

    # bringup_launch.py 一次性拉起 AMCL、map_server、planner、controller 等整个导航栈
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # 传入地图参数，Nav2知道是定位模式，会内部启动 AMCL + Map server
            'map': os.path.join(pkg_jetrover_bringup, 'maps', 'husarion_office.yaml'),  # 预先建好的地图
            'params_file': os.path.join(pkg_jetrover_bringup, 'params', 'nav2_params.yaml'),  # 导航参数
            'package_path': pkg_jetrover_bringup,
        }.items()
    )

    # 启动 RViz2 可视化工具，使用 nav2 默认视图配置文件
    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )

    # 发布静态 TF 变换：map -> odom（初始偏移为零）
    # 在 AMCL 接管定位之前，提供 map 和 odom 坐标系之间的初始关系
    # static_transform_publisher_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='map_to_odom',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    # )
    nodes = [
        nav2_launch_cmd,
        rviz_launch_cmd,
    ]

    return LaunchDescription(nodes)
