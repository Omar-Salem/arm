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

#ifndef ARM_HARDWARE_INTERFACE__ARM_HARDWARE_HPP_
#define ARM_HARDWARE_INTERFACE__ARM_HARDWARE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "arm_interfaces/msg/motors.hpp"
#include "Motor.h"

using namespace std;
using namespace rclcpp;
using namespace rclcpp_lifecycle;
using namespace hardware_interface;
namespace arm_firmware {
    class ArmHardware : public SystemInterface {
    public:
        ArmHardware();

        CallbackReturn on_init(
                const HardwareInfo &info) override;


        CallbackReturn on_configure(
                const State &previous_state) override;


        std::vector <StateInterface> export_state_interfaces() override;


        std::vector <CommandInterface> export_command_interfaces() override;


        CallbackReturn on_activate(
                const State &previous_state) override;


        CallbackReturn on_deactivate(
                const State &previous_state) override;


        return_type read(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;


        return_type write(
                const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        unique_ptr <Motor> baseLink;
        unique_ptr <Motor> shoulder;
        std::shared_ptr <rclcpp::Node> node_;
        rclcpp::Subscription<arm_interfaces::msg::Motors>::SharedPtr positionSubscription;
        rclcpp::Publisher<arm_interfaces::msg::Motors>::SharedPtr positionPublisher;

        void setMotorsPositions(double baseLink, double shoulder);

        void readMotorsPositions(const arm_interfaces::msg::Motors::SharedPtr motors);
    };

}  // namespace arm_firmware

#endif  // ARM_HARDWARE_INTERFACE__ARM_HARDWARE_HPP_
