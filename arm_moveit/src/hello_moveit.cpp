#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;
using moveit::core::JointModelGroup;
using moveit::planning_interface::PlanningSceneInterface;
using  moveit::core::RobotStatePtr;
static const std::string PLANNING_GROUP = "arm";

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const move_group_node = std::make_shared<rclcpp::Node>(
            "hello_moveit",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("hello_moveit");

    auto move_group = MoveGroupInterface(move_group_node, PLANNING_GROUP);

//    auto target_pose = move_group.getRandomPose();
//    move_group.setPoseTarget(target_pose);

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

    const JointModelGroup* joint_model_group =
            move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    joint_group_positions[0] = -1.0;  // radians
    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }


    auto const [success, plan] = [&move_group] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();

    RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

//    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}