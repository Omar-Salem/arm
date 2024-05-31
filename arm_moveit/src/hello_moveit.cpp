#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
    // Initialize ROS and create the Node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>(
            "hello_moveit",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );

    // Create a ROS logger
    auto const LOGGER = rclcpp::get_logger("hello_moveit");
//    ros::AsyncSpinner spinner(1); spinner.start();

    // Create the MoveIt MoveGroup Interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
    RCLCPP_INFO(LOGGER, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
    RCLCPP_INFO(LOGGER, "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
              move_group_interface.getJointModelGroupNames().end(),
              std::ostream_iterator<std::string>(std::cout, ", "));

    auto current_pose = move_group_interface.getCurrentPose();

//    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL getEndEffectorLink %s", move_group_interface.getEndEffectorLink() .c_str());
    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL current_pose.x %f", current_pose.pose.position.x);
    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL current_pose.y %f", current_pose.pose.position.y);
    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL current_pose.z %f", current_pose.pose.position.z);

    auto target_pose = move_group_interface.getRandomPose();
    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL target_pose.x %f", target_pose.pose.position.x);
    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL target_pose.y %f", target_pose.pose.position.y);
    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL target_pose.z %f", target_pose.pose.position.z);

// Set a target Pose
//    auto const target_pose = []{
//        geometry_msgs::msg::Pose msg;
//        msg.orientation.w = 1.0;
//        msg.position.x = 0.005778;
//        msg.position.y = 0.343882;
//        msg.position.z = 0.323648;
//        return msg;
//    }();
//    current_pose.pose.position.x = current_pose.pose.position.x + 0.1;
//    move_group_interface.setRandomTarget();
    move_group_interface.setPoseTarget(target_pose);
//
// Create a plan to that target pose
    auto const [success, plan] = [&move_group_interface] {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok, msg);
    }();

// Execute the plan
    if (success) {
//        move_group_interface.execute(plan);
//        move_group_interface.move();
        bool success = (move_group_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

        current_pose = move_group_interface.getCurrentPose();

//    RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL getEndEffectorLink %s", move_group_interface.getEndEffectorLink() .c_str());
        RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL x %f", current_pose.pose.position.x);
        RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL y %f", current_pose.pose.position.y);
        RCLCPP_INFO(LOGGER, "LLLLLLLLLLLLLLLLLLLLLLLLLLL z %f", current_pose.pose.position.z);
    } else {
        RCLCPP_ERROR(LOGGER, "Planning failed!");
    }

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}