from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import launch_ros.actions

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jetrover").to_moveit_configs()
    launch_ros.actions.SetParameter(name='use_sim_time', value=True)
    planning_execution_node = Node(
        package="jetrover_moveit_control",
        executable="planning_with_obstacles",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],

    )

    return LaunchDescription([planning_execution_node])