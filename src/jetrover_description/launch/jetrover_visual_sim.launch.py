import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import xacro
from typing import Final

PACKAGE_NAME: Final = 'jetrover_description'

SPACE: Final = ' '

KEY_USE_SIM_TIME: Final = 'use_sim_time'
KEY_ROBOT_DESCRIPTION: Final = 'robot_description'

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

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    return ld
