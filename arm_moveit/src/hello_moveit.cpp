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
using moveit::core::JointModelGroup;
using moveit::core::RobotStatePtr;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
static const std::string PLANNING_GROUP = "arm";

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const move_group_node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const LOGGER = rclcpp::get_logger("hello_moveit");

  auto move_group = MoveGroupInterface(move_group_node, PLANNING_GROUP);
  PlanningSceneInterface planning_scene_interface;
  const JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.5;
  move_group.setPoseTarget(target_pose1);

  auto const [success, plan] = [&move_group] {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
  }();

  if (success)
  {
    move_group.execute(plan);
    //        move_group.move();

    // current_pose = move_group.getCurrentPose();

    // RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL x %f", current_pose.pose.position.x);
  }
  else
  {
    RCLCPP_ERROR(LOGGER, "Planning failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}