/**
* Copyright (c) 2024 Rainbow Robotics
* 
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
* 
*     http://www.apache.org/licenses/LICENSE-2.0
* 
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/

#pragma once

#include <array>
#include <string_view>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rbpodo_hardware/robot.hpp"
#include "rbpodo_hardware/robot_node.hpp"
#include "rbpodo_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rbpodo_hardware {

class RBPodoHardwareInterface : public hardware_interface::SystemInterface {
 public:
  static constexpr size_t kNumberOfJoints{6};

  static constexpr size_t k6DoFDim{6};
  const std::string HW_IF_CARTESIAN_POSE{"cartesian_pose"};
  const std::string HW_IF_CARTESIAN_VELOCITY{"cartesian_velocity"};

  const std::array<std::string, k6DoFDim> kCartesianPosePrefix{"x", "y", "z", "rx", "ry", "rz"}; // unit: m, rad
  const std::array<std::string, k6DoFDim> kCartesianVelocityPrefix{"x", "y", "z", "rx", "ry", "rz"}; // unit: m, rad

  // RCLCPP_SHARED_PTR_DEFINITIONS(RBPodoHardwareInterface)

  RBPodoHardwareInterface();
  /// `RBPodoHardwareInterface` is not copyable
  RBPodoHardwareInterface(const RBPodoHardwareInterface&) = delete;
  /// `RBPodoHardwareInterface` is not copyable
  RBPodoHardwareInterface& operator=(const RBPodoHardwareInterface& other) = delete;
  /// `RBPodoHardwareInterface` is movable
  RBPodoHardwareInterface& operator=(RBPodoHardwareInterface&& other) = delete;
  /// `RBPodoHardwareInterface` is movable
  RBPodoHardwareInterface(RBPodoHardwareInterface&& other) = delete;

  virtual ~RBPodoHardwareInterface();

  rclcpp::Logger getLogger();

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  RBPODO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  RBPODO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                                              const std::vector<std::string>& stop_interfaces) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  RBPODO_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  struct CommandInterfaceState {
    bool claimed{false};
    bool running{false};
  };

  struct CommandInterfaceInfo {
    std::string type;
    size_t size;
    CommandInterfaceState& state;
  };

  std::vector<CommandInterfaceInfo> command_interface_infos_;
  bool mode_changed_{false};

  CommandInterfaceState joint_position_interface_state_;
  CommandInterfaceState joint_velocity_interface_state_;
  CommandInterfaceState joint_effort_interface_state_;
  CommandInterfaceState cartesian_pose_interface_state_;
  CommandInterfaceState cartesian_velocity_interface_state_;

  std::shared_ptr<Robot> robot_;
  std::array<double, kNumberOfJoints> torque_constants_;

  // command interface
  std::array<double, kNumberOfJoints> hw_position_commands_;
  std::array<double, kNumberOfJoints> hw_velocity_commands_;
  std::array<double, kNumberOfJoints> hw_effort_commands_;
  std::array<double, k6DoFDim> hw_cartesian_pose_commands_;
  std::array<double, k6DoFDim> hw_cartesian_velocity_commands_;

  // states
  std::array<double, kNumberOfJoints> hw_position_states_;
  std::array<double, kNumberOfJoints> hw_effort_states_;

  // ROS Node
  std::shared_ptr<RobotNode> robot_node_;
  std::shared_ptr<RobotExecutor> robot_executor_;
};

}  // namespace rbpodo_hardware
