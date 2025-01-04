// Copyright (c) 2024, omar.salem
// Copyright (c) 2024, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <limits>
#include <vector>

#include "arm_hardware_interface/arm_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_hardware_interface {
    ArmHardware::ArmHardware() : node_(std::make_shared<rclcpp::Node>("arm_motors_hw_interface_node")) {
        stateSubscription = node_->create_subscription<arm_interfaces::msg::Motors>(
                "arm/motors_state", 10,
                [this](const arm_interfaces::msg::Motors::SharedPtr motors) {
                    this->readMotorsPositions(motors);
                });
        commandPublisher = node_->create_publisher<arm_interfaces::msg::Motors>("arm/motors_cmd",
                                                                                     10);
    }

    CallbackReturn ArmHardware::on_init(
            const HardwareInfo &info) {
        RCLCPP_INFO(get_logger(), "on_init ...please wait...");

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        joint_1 = make_unique<Motor>("joint_1");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArmHardware::on_configure(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "on_configure ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    vector <StateInterface> ArmHardware::export_state_interfaces() {
        RCLCPP_INFO(get_logger(), "export_state_interfaces ...please wait...");
        vector <StateInterface> state_interfaces;

        state_interfaces.emplace_back(
                joint_1->name, HW_IF_POSITION, &joint_1->position_state);

        return state_interfaces;
    }

    vector <CommandInterface> ArmHardware::export_command_interfaces() {
        RCLCPP_INFO(get_logger(), "export_command_interfaces ...please wait...");
        vector <CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                joint_1->name, HW_IF_POSITION, &joint_1->position_command);

        return command_interfaces;
    }

    CallbackReturn ArmHardware::on_activate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "on_activate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn ArmHardware::on_deactivate(
            const State & /*previous_state*/) {
        RCLCPP_INFO(get_logger(), "on_deactivate ...please wait...");
        return CallbackReturn::SUCCESS;
    }

    return_type ArmHardware::read(
            const Time & /*time*/, const Duration & /*period*/) {
        rclcpp::spin_some(node_);
        return return_type::OK;
    }

    return_type ArmHardware::write(
            const Time & /*time*/, const Duration & /*period*/) {

//        RCLCPP_INFO(get_logger(), "joint_1->position_command %f",joint_1->position_command);

        setMotorsPositions(joint_1->position_command);
        return return_type::OK;
    }

    void ArmHardware::setMotorsPositions(double joint_1) {
        auto cmd_msg = std::make_shared<arm_interfaces::msg::Motors>();
        cmd_msg->joint_1 = joint_1;
        commandPublisher->publish(*cmd_msg);
    }

    void ArmHardware::readMotorsPositions(const arm_interfaces::msg::Motors::SharedPtr motors) {
        joint_1->position_state = motors->joint_1;
    }

}  // namespace arm_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        arm_hardware_interface::ArmHardware, hardware_interface::SystemInterface
)
