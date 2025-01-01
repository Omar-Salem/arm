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

#include "arm_firmware/arm_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arm_firmware {
    ArmHardware::ArmHardware() : node_(std::make_shared<rclcpp::Node>("arm_motors_hw_interface_node")) {
        positionSubscription = node_->create_subscription<arm_interfaces::msg::Motors>(
                "arm/motors_state", 10,
                [this](const arm_interfaces::msg::Motors::SharedPtr motors) {
                    this->readMotorsPositions(motors);
                });
        positionPublisher = node_->create_publisher<arm_interfaces::msg::Motors>("arm/motors_cmd",
                                                                                     10);
    }

    CallbackReturn ArmHardware::on_init(
            const HardwareInfo &info) {
        RCLCPP_INFO(get_logger(), "on_init ...please wait...");

        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
            return CallbackReturn::ERROR;
        }
        baseLink = make_unique<Motor>("baseLink_joint");
        shoulder = make_unique<Motor>("shoulder_joint");
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
                baseLink->name, HW_IF_POSITION, &baseLink->position_state);

        state_interfaces.emplace_back(
                shoulder->name, HW_IF_POSITION, &shoulder->position_state);

        return state_interfaces;
    }

    vector <CommandInterface> ArmHardware::export_command_interfaces() {
        RCLCPP_INFO(get_logger(), "export_command_interfaces ...please wait...");
        vector <CommandInterface> command_interfaces;
        command_interfaces.emplace_back(
                baseLink->name, HW_IF_POSITION, &baseLink->position_command);
        command_interfaces.emplace_back(
                shoulder->name, HW_IF_POSITION, &shoulder->position_command);

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

//        RCLCPP_INFO(get_logger(), "baseLink->position_command %f",baseLink->position_command);

        setMotorsPositions(baseLink->position_command,
                          shoulder->position_command);
        return return_type::OK;
    }

    void ArmHardware::setMotorsPositions(double baseLink,
                                         double shoulder) {
        auto cmd_msg = std::make_shared<arm_interfaces::msg::Motors>();
        cmd_msg->base_link = baseLink;
        cmd_msg->shoulder = shoulder;
        positionPublisher->publish(*cmd_msg);
    }

    void ArmHardware::readMotorsPositions(const arm_interfaces::msg::Motors::SharedPtr motors) {
        baseLink->position_state = motors->base_link;
        shoulder->position_state = motors->shoulder;
    }

}  // namespace arm_firmware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
        arm_firmware::ArmHardware, hardware_interface::SystemInterface
)
