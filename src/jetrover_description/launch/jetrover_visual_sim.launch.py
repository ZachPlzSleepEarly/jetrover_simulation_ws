import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro
from typing import Final

PACKAGE_NAME: Final = 'jetrover_description'
PACKAGE_ROS_GZ_SIM: Final = 'ros_gz_sim'

SPACE: Final = ' '

KEY_USE_SIM_TIME: Final = 'use_sim_time'
KEY_ROBOT_DESCRIPTION: Final = 'robot_description'
KEY_GZ_SIM_RESOURCE_PATH: Final = 'GZ_SIM_RESOURCE_PATH'

CMD_XACRO: Final = 'xacro'


def generate_launch_description():
    use_sim_time = LaunchConfiguration(KEY_USE_SIM_TIME, default='true')

    jetrover_description_pkg_path = get_package_share_directory(PACKAGE_NAME)

    urdf_file = os.path.join(jetrover_description_pkg_path, 'urdf', 'jetrover.xacro')
    urdf_description = xacro.process_file(urdf_file)
    urdf_xml = urdf_description.toxml()

    # 静态TF(static TF)，车（不包括机械臂）
    # pub：/tf, tf2_msgs/msg/TFMessage (relation between joints and links)
    # pub: /robot_description, std_msgs/msg/String (URDF content)
    # sub: /joint_states, sensor_msgs/msg/JointState (update when moving GUI slider)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            KEY_USE_SIM_TIME: use_sim_time,
            KEY_ROBOT_DESCRIPTION: urdf_xml,
        }],
        arguments=[urdf_file],
    )

    # 动态TF转换(dynamic TF Transformation)，机械臂（不包括车）
    # pub: /joint_states, sensor_msgs/msg/JointState
    # sub: /robot_description, std_msgs/msg/String
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
    )

    # Rviz GUI 窗口配置文件
    rviz_config_file = os.path.join(jetrover_description_pkg_path, 'rviz', 'view.rviz')
    # 打开 Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    # 拿到 meshes 资源 给 gazebo spawn 机器人
    set_gazebo_robot_resource = SetEnvironmentVariable(
        KEY_GZ_SIM_RESOURCE_PATH,
        # 拿上层的才对
        os.path.dirname(jetrover_description_pkg_path)
    )
    # gazebo app 打开一个 empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(PACKAGE_ROS_GZ_SIM),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items(),
    )
    # 生成机器人
    spawn = Node(
        package=PACKAGE_ROS_GZ_SIM,
        executable='create',
        arguments=[
            '-name', 'jetrover',
            #  从 robot_description ROS2 TOPIC 读取 URDF
            '-topic', 'robot_description'
        ]
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    ld.add_action(set_gazebo_robot_resource)
    ld.add_action(gazebo)
    ld.add_action(spawn)
    return ld
