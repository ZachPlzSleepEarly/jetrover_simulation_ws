import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from typing import Final
import xacro

PACKAGE_JETROVER_DESCRIPTION: Final = 'jetrover_description'


def generate_launch_description():
    # Declare Arguments
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('frame_prefix', default_value=''))
    declared_arguments.append(DeclareLaunchArgument('use_sim_time', default_value='true'))
    declared_arguments.append(DeclareLaunchArgument('namespace', default_value=''))
    declared_arguments.append(DeclareLaunchArgument('use_namespace', default_value='false'))
    # Initialize Arguments
    frame_prefix = LaunchConfiguration('frame_prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')

    pkg_jetrover_description = get_package_share_directory(PACKAGE_JETROVER_DESCRIPTION)
    urdf_file = os.path.join(pkg_jetrover_description, 'urdf', 'assembly_simulation.xacro')  # 机器人 URDF 文件地址
    rviz_config_file = os.path.join(pkg_jetrover_description, 'config', 'display.rviz')  # Rviz 窗口配置文件

    robot_description_processed = xacro.process_file(urdf_file)

    robot_description_param = {'robot_description': robot_description_processed.toxml()}
    frame_prefix_param = {'frame_prefix': frame_prefix}
    use_sim_time_param = {'use_sim_time': use_sim_time}

    # 动态TF转换(dynamic TF Transformation)，常驻节点不会退出
    # pub: /joint_states, sensor_msgs/msg/JointState
    # sub: /robot_description, std_msgs/msg/String
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    # 静态TF(static TF)，常驻节点不会退出
    # pub：/tf, tf2_msgs/msg/TFMessage (relation between joints and links)
    # pub: /robot_description, std_msgs/msg/String (URDF content)
    # sub: /joint_states, sensor_msgs/msg/JointState (update when moving GUI slider)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        parameters=[
            robot_description_param,
            frame_prefix_param,
            use_sim_time_param,
        ],
    )

    # sub: /tf, tf2_msgs/msg/TFMessage
    # sub: /robot_description, std_msgs/msg/String
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    nodes = [
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ]
    return LaunchDescription(declared_arguments + nodes)


if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
