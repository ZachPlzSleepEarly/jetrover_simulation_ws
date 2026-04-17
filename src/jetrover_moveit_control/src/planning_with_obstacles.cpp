#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include <chrono>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/utils/moveit_error_code.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2/time.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <thread>
#include <vector>
class PlanningWithObstacles : public rclcpp::Node {
public:
    PlanningWithObstacles() : rclcpp::Node(NODE_NAME) {}
    void run();
    void plan();

private:
    bool setup_init_position();
    void setup_world();

    static constexpr const char* NODE_NAME = "planning_with_obstacles";

    rclcpp::Node::SharedPtr _shared_self;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> _move_group;
};

void PlanningWithObstacles::run()
{
    _shared_self = shared_from_this();

    // 子线程执行 Planning
    std::thread planning_thread(&PlanningWithObstacles::plan, this);
    // 转线程持续处理 ROS 回调
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(_shared_self);
    executor.spin();

    // 如果 planning 还没执行结束，等待执行结束
    if (planning_thread.joinable()) {
        planning_thread.join();
    }
}

void PlanningWithObstacles::plan()
{
    // Initialization
    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    _move_group = std::make_unique<moveit::planning_interface::MoveGroupInterface>(_shared_self, "arm");
    if (!setup_init_position()) {
        RCLCPP_ERROR(this->get_logger(), "PlanningWithObstacles::plan init position failed");
        return;
    }

    // 获取机器人基座标系
    const std::string robot_base_frame = _move_group->getPlanningFrame();
    // 获取法兰盘坐标系
    const std::string end_effector_frame = _move_group->getEndEffectorLink();

    setup_world();
    rclcpp::sleep_for(std::chrono::seconds(3));

    // 从法兰盘坐标系变换到机器人基座标系
    geometry_msgs::msg::TransformStamped tf_base_to_eef;
    while (rclcpp::ok()) {
        try {
            tf_base_to_eef = tf_buffer->lookupTransform(robot_base_frame, end_effector_frame, tf2::TimePointZero);
            break;
        } catch (const tf2::TransformException& ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform %s to %s: %s. Retry ...", end_effector_frame.c_str(),
                         robot_base_frame.c_str(), ex.what());
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
    }
    if (!rclcpp::ok()) {
        return;
    }

    // Setup destination point
    geometry_msgs::msg::Pose target_pose;
    target_pose.orientation.w = tf_base_to_eef.transform.rotation.w;
    target_pose.orientation.x = tf_base_to_eef.transform.rotation.x;
    target_pose.orientation.y = tf_base_to_eef.transform.rotation.y;
    target_pose.orientation.z = tf_base_to_eef.transform.rotation.z;
    target_pose.position.x = tf_base_to_eef.transform.translation.x;
    target_pose.position.y = tf_base_to_eef.transform.translation.y;
    target_pose.position.z = tf_base_to_eef.transform.translation.z;

    target_pose.position.z += 0.05;
    target_pose.position.x += 0.15;

    _move_group->setPoseTarget(target_pose);

    // Planning & Execution
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    if (_move_group->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        _move_group->move();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Motion planning failed");
    }

    rclcpp::shutdown();
}

bool PlanningWithObstacles::setup_init_position()
{
    std::vector<double> joint_group_positions = {0.0, 1.0122909662, -1.6231562044, -1.745329252, 0.0};

    if (!_move_group->setJointValueTarget(joint_group_positions)) {
        RCLCPP_ERROR(this->get_logger(), "PlanningWithObstacles::setup_init_position Joint outer bounds");
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan init_joint_plan;
    if (_move_group->plan(init_joint_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        _move_group->move();
        rclcpp::sleep_for(std::chrono::seconds(5));
    } else {
        RCLCPP_ERROR(this->get_logger(), "Initial joint motion failed");
        rclcpp::shutdown();
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Init position success");
    return true;
}

void PlanningWithObstacles::setup_world()
{
    // Setup shape and size
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.1;
    table_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.3;
    table_primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 1.0;
    // Setup pose
    geometry_msgs::msg::Pose table_pose;
    table_pose.orientation.x = 1.0;
    table_pose.position.x = 0.28;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.02;
    // Add a collision object (desk) to the world
    moveit_msgs::msg::CollisionObject table_collision_object;
    table_collision_object.id = "table";
    table_collision_object.header.frame_id = _move_group->getPlanningFrame();
    table_collision_object.primitives.push_back(table_primitive);
    table_collision_object.primitive_poses.push_back(table_pose);
    table_collision_object.operation = table_collision_object.ADD;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(table_collision_object);

    // Setup shape and size
    shape_msgs::msg::SolidPrimitive grasp_primitive;
    grasp_primitive.type = grasp_primitive.CYLINDER;
    grasp_primitive.dimensions.resize(2);
    grasp_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.1;
    grasp_primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.04;
    // Setup pose
    geometry_msgs::msg::Pose grasp_object_pose;
    grasp_object_pose.orientation.w = 1.0;
    grasp_object_pose.position.z = -0.16;
    // Add a grabbed object (desk) to the world
    moveit_msgs::msg::CollisionObject grasp_collision_object;
    grasp_collision_object.id = "grasp";
    grasp_collision_object.header.frame_id = _move_group->getEndEffectorLink();
    grasp_collision_object.primitives.push_back(grasp_primitive);
    grasp_collision_object.primitive_poses.push_back(grasp_object_pose);
    grasp_collision_object.operation = grasp_collision_object.ADD;
    planning_scene_interface.applyCollisionObject(grasp_collision_object);

    // 把抓取物体附着在手上（告诉MoveIt grasp 物体不是普通障碍物，而是被机械手 gripper_link 抓住）
    std::vector<std::string> touch_links;
    touch_links.push_back("l_out_link");
    touch_links.push_back("r_out_link");
    _move_group->attachObject(grasp_collision_object.id, "gripper_link", touch_links);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto planning_node = std::make_shared<PlanningWithObstacles>();
    planning_node->run();
    return 0;
}