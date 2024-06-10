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
using geometry_msgs::msg::Pose;
using moveit::core::JointModelGroup;
using moveit::core::RobotStatePtr;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;
using moveit_msgs::msg::CollisionObject;
using moveit_msgs::msg::Constraints;
using moveit_msgs::msg::OrientationConstraint;
using moveit_msgs::msg::RobotTrajectory;
using shape_msgs::msg::SolidPrimitive;
static const std::string PLANNING_GROUP = "arm";

int main(int argc, char *argv[])
{
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const move_group_node = std::make_shared<rclcpp::Node>(
        "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("hello_moveit");

    auto move_group = MoveGroupInterface(move_group_node, PLANNING_GROUP);
    PlanningSceneInterface planning_scene_interface;
    const JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    /********************************Planning to a Pose goal
        geometry_msgs::msg::Pose target_pose1;
        target_pose1.orientation.w = 1.0;
        target_pose1.position.x = 0.28;
        target_pose1.position.y = -0.2;
        target_pose1.position.z = 0.5;
        move_group.setPoseTarget(target_pose1);

        auto const [success, plan] = [&move_group]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(move_group.plan(msg));
            return std::make_pair(ok, msg);
        }();

        if (success)
        {
            move_group.execute(plan);
            //        move_group.move();

            // current_pose = move_group.getCurrentPose();
        }
        else
        {
            RCLCPP_ERROR(LOGGER, "Planning failed!");
        }
    ********************/

    /*************************Planning to a joint-space goal
    RobotStatePtr current_state = move_group.getCurrentState(10);
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    for (auto joint_position : joint_group_positions)
    {
        RCLCPP_INFO(LOGGER, "joint_position %f", joint_position);
    }
    joint_group_positions[0] = -1.0; // radians
    bool within_bounds = move_group.setJointValueTarget(joint_group_positions);
    if (!within_bounds)
    {
        RCLCPP_WARN(LOGGER, "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    }
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);

    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();
    RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
************************************/
    /*************************Planning with Path Constraints
    OrientationConstraint ocm;
    ocm.link_name = "arm_link7_1";
    ocm.header.frame_id = "base_link";
    ocm.orientation.w = 1.0;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    Constraints test_constraints;
    test_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(test_constraints);
 ********/

    /*************************Cartesian Paths
    Pose start_pose2;
    start_pose2.orientation.w = 1.0;
    start_pose2.position.x = 0.55;
    start_pose2.position.y = -0.05;
    start_pose2.position.z = 0.8;
    std::vector<Pose> waypoints;
    waypoints.push_back(start_pose2);

    Pose target_pose3 = start_pose2;

    target_pose3.position.z -= 0.2;
    waypoints.push_back(target_pose3); // down

    target_pose3.position.y -= 0.2;
    waypoints.push_back(target_pose3); // right

    target_pose3.position.z += 0.2;
    target_pose3.position.y += 0.2;
    target_pose3.position.x -= 0.2;
    waypoints.push_back(target_pose3); // up and left

    RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% achieved)", fraction * 100.0);
    move_group.execute(trajectory);
    ********/
    /******************************Adding objects to the environment*************************/
    move_group.setStartState(*move_group.getCurrentState());
    Pose another_pose;
    another_pose.orientation.w = 1.0;
    another_pose.position.x = 0.28;
    another_pose.position.y = -0.2;
    another_pose.position.z = 0.5;
    move_group.setPoseTarget(another_pose);

    // auto const [success, plan] = [&move_group]
    // {
    //     moveit::planning_interface::MoveGroupInterface::Plan msg;
    //     auto const ok = static_cast<bool>(move_group.plan(msg));
    //     return std::make_pair(ok, msg);
    // }();
    // RCLCPP_INFO(LOGGER, "Visualizing plan 5 (with no obstacles) %s", success ? "SUCCESS" : "FAILED");

    SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.1;
    primitive.dimensions[primitive.BOX_Y] = 1.5;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.48;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.25;

    CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box1";
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    RCLCPP_INFO(LOGGER, "Add an object into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    auto const [success, plan] = [&move_group]
    {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
    }();
    RCLCPP_INFO(LOGGER, "Visualizing plan 5 (now with obstaclessss) %s", success ? "SUCCESS" : "FAILED");

    CollisionObject object_to_attach;
    object_to_attach.id = "cylinder1";

    SolidPrimitive cylinder_primitive;
    cylinder_primitive.type = primitive.CYLINDER;
    cylinder_primitive.dimensions.resize(2);
    cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
    cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

    object_to_attach.header.frame_id = move_group.getEndEffectorLink();
    geometry_msgs::msg::Pose grab_pose;
    grab_pose.orientation.w = 1.0;
    grab_pose.position.z = 0.2;

    object_to_attach.primitives.push_back(cylinder_primitive);
    object_to_attach.primitive_poses.push_back(grab_pose);
    object_to_attach.operation = object_to_attach.ADD;
    planning_scene_interface.applyCollisionObject(object_to_attach);

    RCLCPP_INFO(LOGGER, "Attach the object to the robot");
    std::vector<std::string> touch_links;
    touch_links.push_back("finger_1");
    move_group.attachObject(object_to_attach.id, "hand_1", touch_links);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}