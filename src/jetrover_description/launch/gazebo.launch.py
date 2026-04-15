import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import xacro
from os.path import join
from typing import Final

JOINT_STATE_BROADCASTER_CONTROLLER: Final = 'joint_state_broadcaster'
ACKERMANN_LIKE_CONTROLLER: Final = 'ackermann_like_controller'
ARM_CONTROLLER: Final = 'arm_controller'
EEF_CONTROLLER: Final = 'eef_controller'


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_rbot = get_package_share_directory('jetrover_description')

    robot_description_file = os.path.join(pkg_ros_gz_rbot, 'urdf', 'assembly_simulation.xacro')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')

    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    ros2_control_config_file = os.path.join(pkg_ros_gz_rbot, 'config', 'ros2_controllers.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': True},
        ],
    )

    # 拉起 joint_state_broadcaster Controller
    joint_state_broadcaster_spanwer_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            JOINT_STATE_BROADCASTER_CONTROLLER,
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_control_config_file,
        ]
    )
    # 拉起 ackermann_like_controller Controller
    ackermann_like_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            ACKERMANN_LIKE_CONTROLLER,
            '--controller-manager', '/controller_manager',
            '--param-file', ros2_control_config_file,
            '--controller-ros-args', '-r /ackermann_like_controller/tf_odometry:=/tf',
        ],
    )
    arm_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            ARM_CONTROLLER,  # controller_names
            '-c', '/controller_manager',  # [-c CONTROLLER_MANAGER]
            '--param-file', ros2_control_config_file,
        ],
    )
    eef_controller_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            EEF_CONTROLLER,
            '-c', '/controller_manager',  # [-c CONTROLLER_MANAGER]
            '--param-file', ros2_control_config_file,
        ]
    )
    # 1) joint_state_broadcaster 结束后，启动 ackermann controller spawner
    delay_ackermann_after_joint_state_broadcaster_spawner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spanwer_node,
            on_exit=[ackermann_like_controller_spawner_node],
        )
    )
    # 2) ackermann controller spawner 结束后，再启动 arm controller spawner node
    delay_arm_controller_spawner_node_after_ackermann_controller_spawner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=ackermann_like_controller_spawner_node,
            on_exit=[arm_controller_spawner_node],
        )
    )
    # 3) arm controller spawner ndoe 结束后，再启动 eef controller spawner node
    delay_eef_controller_spawner_node_after_arm_controller_spawner_node = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner_node,
            on_exit=[eef_controller_spawner_node],
        )
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items()
    )

    spawn_robot = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                "-topic", "/robot_description",
                "-name", "assembly_chasis",
                "-allow_renaming", "false",  # prevents "_1" duplicate
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.32",
                "-Y", "0.0"
            ],
            output='screen'
        )]
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {'config_file': ros_gz_bridge_config},
            {'use_sim_time': True},
        ],
        output='screen'
    )

    return LaunchDescription([
        # Gz Sim 侧
        gazebo,
        spawn_robot,
        ros_gz_bridge,

        # ROS2 侧
        robot_state_publisher,
        joint_state_broadcaster_spanwer_node,
        delay_ackermann_after_joint_state_broadcaster_spawner_node,
        delay_arm_controller_spawner_node_after_ackermann_controller_spawner_node,
        delay_eef_controller_spawner_node_after_arm_controller_spawner_node,
    ])
