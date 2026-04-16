from moveit_configs_utils import MoveItConfigsBuilder  # import to load the configuration file
from launch_ros.actions import SetParameter
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("jetrover").to_moveit_configs()
    SetParameter(name="use_sim_time", value=True)
    moveit_planning_node = Node(
        package="jetrover_moveit_control",
        executable="joint_space_planning",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )

    return LaunchDescription([moveit_planning_node])
