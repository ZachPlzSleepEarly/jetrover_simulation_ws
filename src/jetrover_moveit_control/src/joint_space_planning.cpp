#include <chrono>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("JointSpacePlanning");
    
    moveit::planning_interface::MoveGroupInterface move_group(node, "arm");
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;  // this structure is used to store a planning request
    // 规划并在 joint space 中移动到目标点
    std::vector<double> des_joint_pos = {1, 0.750492, -0.8, -0.88063, 0.0};
    if (!move_group.setJointValueTarget(des_joint_pos)) {
        RCLCPP_ERROR(node->get_logger(), "Joint outer bounds");
        return -1;
    }
    move_group.setMaxVelocityScalingFactor(.5);
    move_group.setMaxAccelerationScalingFactor(.5);
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.move();
    }

    rclcpp::sleep_for(std::chrono::seconds(3));

    // 规划并在 joint space 中移动到目标点
    des_joint_pos = {0.0, 0.2, -1.3, -1, 1.0};
    if (!move_group.setJointValueTarget(des_joint_pos)) {
        RCLCPP_ERROR(node->get_logger(), "Joint outer bounds");
        return -1;
    }
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group.move();
    }

    return 0;
}