import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import xacro
from typing import Final

PACKAGE_NAME: Final = 'jetrover_rebuild_description'
PACKAGE_ROS_GZ_SIM: Final = 'ros_gz_sim'
PACKAGE_ROS_GZ_BRIDGE: Final = 'ros_gz_bridge'

SPACE: Final = ' '

KEY_USE_SIM_TIME: Final = 'use_sim_time'
KEY_ROBOT_DESCRIPTION: Final = 'robot_description'
KEY_GZ_SIM_RESOURCE_PATH: Final = 'GZ_SIM_RESOURCE_PATH'
VALUE_USE_SIM_TIME: Final = 'true'

CMD_XACRO: Final = 'xacro'

ROBOT_XACRO_FILE: Final = 'jetrover_rebuild.xacro'


def generate_launch_description():
    use_sim_time = LaunchConfiguration(KEY_USE_SIM_TIME, default=VALUE_USE_SIM_TIME)

    robot_descpt_pkg_path = get_package_share_directory(PACKAGE_NAME)

    urdf_file = os.path.join(robot_descpt_pkg_path, 'urdf', ROBOT_XACRO_FILE)
    urdf_description = xacro.process_file(urdf_file)
    urdf_xml = urdf_description.toxml()

    robot_description = {
        KEY_USE_SIM_TIME: use_sim_time,
        KEY_ROBOT_DESCRIPTION: urdf_xml,
    }

    # Start Robot state publisher
    # 静态TF(static TF)，车（不包括机械臂）
    # pub：/tf, tf2_msgs/msg/TFMessage (relation between joints and links)
    # pub: /robot_description, std_msgs/msg/String (URDF content)
    # sub: /joint_states, sensor_msgs/msg/JointState (update when moving GUI slider)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # 动态TF转换(dynamic TF Transformation)，机械臂（不包括车）
    # pub: /joint_states, sensor_msgs/msg/JointState
    # sub: /robot_description, std_msgs/msg/String
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='both',
    )

    # Start Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        # Rviz GUI 窗口配置文件
        arguments=['-d', os.path.join(robot_descpt_pkg_path, 'rviz', 'view.rviz')],
        output='both',
    )

    # Start Gazebo Sim with an empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(PACKAGE_ROS_GZ_SIM), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items(),
    )
    # 拿到 meshes 资源 给 gazebo spawn 机器人
    set_gazebo_robot_resource = SetEnvironmentVariable(
        KEY_GZ_SIM_RESOURCE_PATH,
        # 拿上层的才对
        os.path.dirname(robot_descpt_pkg_path)
    )
    # Spawn Robot in Gazebo
    spawn = Node(
        package=PACKAGE_ROS_GZ_SIM,
        executable='create',
        arguments=[
            #  从 robot_description ROS2 TOPIC 读取 URDF
            '-topic', '/robot_description',
            '-name', 'jetrover',
        ],
        output='both',
    )

    ros_gz_bridge_config = os.path.join(robot_descpt_pkg_path, 'config', 'ros_gz_bridge_gazebo.yaml')
    #Bridge ROS topics and Gazebo messages for establishing communication
    start_gazebo_ros_bridge = Node(
        package=PACKAGE_ROS_GZ_BRIDGE,
        executable='parameter_bridge',
        parameters=[{
            'config_file': ros_gz_bridge_config,
        }],
        output='both',
    )

    # Launch the rqt_steering controller standalone
    rqt_robot_steering = ExecuteProcess(
        cmd=['rqt', '--standalone', 'rqt_robot_steering'],
        output='both',
    )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher_gui)
    ld.add_action(rviz)
    ld.add_action(set_gazebo_robot_resource)
    ld.add_action(gazebo)
    ld.add_action(spawn)
    ld.add_action(start_gazebo_ros_bridge)
    ld.add_action(rqt_robot_steering)
    return ld
