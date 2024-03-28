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

#include "rbpodo_hardware/rbpodo_hardware_interface.hpp"

namespace {
template <typename ArrayLike>
bool isValidCommand(const ArrayLike& arr) {
  return std::all_of(arr.begin(), arr.end(), [](double value) { return std::isfinite(value) && !std::isnan(value); });
}
}  // namespace

namespace rbpodo_hardware {

RBPodoHardwareInterface::RBPodoHardwareInterface() {
  // Joint Position Controller Interface
  CommandInterfaceInfo jpc_info(
      {hardware_interface::HW_IF_POSITION, RBPodoHardwareInterface::kNumberOfJoints, joint_position_interface_state_});
  command_interface_infos_.push_back(jpc_info);

  // Joint Speed Controller Interface
  CommandInterfaceInfo jvc_info(
      {hardware_interface::HW_IF_VELOCITY, RBPodoHardwareInterface::kNumberOfJoints, joint_velocity_interface_state_});
  command_interface_infos_.push_back(jvc_info);

  // Joint Effort Controller Interface
  CommandInterfaceInfo jec_info(
      {hardware_interface::HW_IF_EFFORT, RBPodoHardwareInterface::kNumberOfJoints, joint_effort_interface_state_});
  command_interface_infos_.push_back(jec_info);

  // Cartesian Pose Controller Interface
  CommandInterfaceInfo cpc_info({HW_IF_CARTESIAN_POSE, k6DoFDim, cartesian_pose_interface_state_});
  command_interface_infos_.push_back(cpc_info);

  // Cartesian Velocity Controller Interface
  CommandInterfaceInfo cvc_info({HW_IF_CARTESIAN_VELOCITY, k6DoFDim, cartesian_velocity_interface_state_});
  command_interface_infos_.push_back(cvc_info);
}

RBPodoHardwareInterface::~RBPodoHardwareInterface() = default;

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %ld joints. Expected %ld.", info_.joints.size(), kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  // Need to check feasibility

  for (size_t i = 0; i < info_.joints.size(); i++) {
    torque_constants_[i] = atof(info_.joints[i].parameters.at("torque_constant").c_str());
  }

  if (!robot_) {
    std::string robot_ip;
    bool cb_simulation;
    try {
      robot_ip = info_.hardware_parameters.at("robot_ip");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' is not set");
      return CallbackReturn::ERROR;
    }
    try {
      cb_simulation = (info_.hardware_parameters.at("cb_simulation") == "True");
    } catch (const std::out_of_range& ex) {
      RCLCPP_FATAL(getLogger(), "Parameter 'cb_simulation' is not set");
      return CallbackReturn::ERROR;
    }

    try {
      RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" (mode: %s)...", robot_ip.c_str(),
                  (cb_simulation ? "Simulation" : "Real"));
      robot_ = std::make_shared<Robot>(robot_ip, cb_simulation, getLogger());
    } catch (const std::exception& e) {
      RCLCPP_FATAL(getLogger(), "Could not connect to robot");
      RCLCPP_FATAL(getLogger(), " - what(): %s", e.what());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  }

  robot_node_ = std::make_shared<RobotNode>(rclcpp::NodeOptions(), robot_);
  robot_executor_ = std::make_shared<RobotExecutor>();
  robot_executor_->add_node(robot_node_);
  RCLCPP_INFO(getLogger(), "Robot node start ...");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RBPodoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RBPodoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]));
  }

  for (size_t i = 0; i < k6DoFDim; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(kCartesianPosePrefix[i], HW_IF_CARTESIAN_POSE,
                                                                         &hw_cartesian_pose_commands_[i]));
  }

  for (size_t i = 0; i < k6DoFDim; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        kCartesianVelocityPrefix[i], HW_IF_CARTESIAN_VELOCITY, &hw_cartesian_velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type RBPodoHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(getLogger(), "prepare_command_mode_switch");
  auto revert = [this]() {
    for (auto info : command_interface_infos_) {
      info.state.claimed = info.state.running;
    }
  };

  for (auto info : command_interface_infos_) {
    std::stringstream ss;
    ss << ".*\\/" << info.type;
    std::regex re(ss.str());

    size_t num_start_interface =
        std::count_if(start_interfaces.begin(), start_interfaces.end(),
                      [re](const std::string& interface) { return std::regex_match(interface, re); });
    size_t num_stop_interface =
        std::count_if(stop_interfaces.begin(), stop_interfaces.end(),
                      [re](const std::string& interface) { return std::regex_match(interface, re); });

    if (num_start_interface == info.size) {
      info.state.claimed = true;
    } else if (num_start_interface != 0) {
      RCLCPP_ERROR(getLogger(), "Invalid number of interfaces (%s) to start. Please check the interface.",
                   info.type.c_str());
      revert();
      return hardware_interface::return_type::ERROR;
    }

    if (num_stop_interface == info.size) {
      info.state.claimed = false;
    } else if (num_stop_interface != 0) {
      RCLCPP_ERROR(getLogger(), "Invalid number of interfaces (%s) to stop. Please check the interface.",
                   info.type.c_str());
      revert();
      return hardware_interface::return_type::ERROR;
    }
  }
  size_t num_start_cmd = std::count_if(command_interface_infos_.begin(), command_interface_infos_.end(),
                                       [](const auto& i) { return i.state.claimed; });
  if (num_start_cmd >= 2) {
    RCLCPP_ERROR(getLogger(), "Cannot start more than one command interface.");
    revert();
    return hardware_interface::return_type::ERROR;
  }

  if (start_interfaces.size() == 0) {
    if (stop_interfaces.size() != 0) {
      robot_->release_move_lock();
    }
  } else {
    if (!robot_->try_aquire_move_lock()) {
      RCLCPP_ERROR(getLogger(), "Cannot claim interfaces. (move_lock is locked.)");
      revert();
      return hardware_interface::return_type::ERROR;
    }
  }

  robot_->stop();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBPodoHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces) {
  RCLCPP_INFO(getLogger(), "perform_command_mode_switch");
  (void)start_interfaces;
  (void)stop_interfaces;

  if (joint_velocity_interface_state_.claimed) {
    for (auto& e : hw_velocity_commands_) {
      e = 0.;
    }
  }
  if (joint_effort_interface_state_.claimed) {
    for (auto& e : hw_effort_commands_) {
      e = 0.;
    }
  }

  if (cartesian_velocity_interface_state_.claimed) {
    for (auto& e : hw_cartesian_velocity_commands_) {
      e = 0.;
    }
  }

  for (auto info : command_interface_infos_) {
    info.state.running = info.state.claimed;
  }
  mode_changed_ = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;

  // command and state should be equal when starting
  for (uint i = 0; i < kNumberOfJoints; i++) {
    hw_position_commands_[i] = hw_position_states_[i];
    hw_effort_commands_[i] = 0;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RBPodoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  (void)previous_state;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RBPodoHardwareInterface::read(const rclcpp::Time& time,
                                                              const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  auto data = robot_->read_once();
  for (size_t i = 0; i < kNumberOfJoints; i++) {
    hw_position_states_[i] = data.sdata.jnt_ref[i] * DEG2RAD;
  }
  for (size_t i = 0; i < kNumberOfJoints; i++) {
    hw_effort_states_[i] = data.sdata.jnt_cur[i] * torque_constants_[i];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RBPodoHardwareInterface::write(const rclcpp::Time& time,
                                                               const rclcpp::Duration& period) {
  (void)time;
  (void)period;
  if (!robot_) {
    RCLCPP_ERROR(getLogger(), "Robot is not initialized yet");
    return hardware_interface::return_type::ERROR;
  }

  bool running = false;
  for (auto info : command_interface_infos_) {
    running |= info.state.running;
  }
  auto data = robot_->read_once();
  if (running) {
    if (mode_changed_) {
      if (data.sdata.robot_state != 1) {
        mode_changed_ = false;
      }
    } else {
      if (data.sdata.robot_state == 1) {
        RCLCPP_ERROR(getLogger(), "Unexpected robot state changed");
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  if (!joint_position_interface_state_.running) {
    hw_position_commands_ = hw_position_states_;
  }
  if (!cartesian_pose_interface_state_.running) {
    hw_cartesian_pose_commands_[0] = data.sdata.tcp_pos[0] * MILLIMETER2METER;
    hw_cartesian_pose_commands_[1] = data.sdata.tcp_pos[1] * MILLIMETER2METER;
    hw_cartesian_pose_commands_[2] = data.sdata.tcp_pos[2] * MILLIMETER2METER;
    hw_cartesian_pose_commands_[3] = data.sdata.tcp_pos[3] * DEG2RAD;
    hw_cartesian_pose_commands_[4] = data.sdata.tcp_pos[4] * DEG2RAD;
    hw_cartesian_pose_commands_[5] = data.sdata.tcp_pos[5] * DEG2RAD;
  }

  if (isValidCommand(hw_position_commands_) && joint_position_interface_state_.running) {
    robot_->write_once_joint_positions(hw_position_commands_);
  }
  if (isValidCommand(hw_velocity_commands_) && joint_velocity_interface_state_.running) {
    robot_->write_once_joint_velocities(hw_velocity_commands_);
  }
  if (isValidCommand(hw_effort_commands_) && joint_effort_interface_state_.running) {
    robot_->write_once_joint_efforts(hw_effort_commands_);
  }
  if (isValidCommand(hw_cartesian_pose_commands_) && cartesian_pose_interface_state_.running) {
    robot_->write_once_cartesian_pose(hw_cartesian_pose_commands_);
  }
  if (isValidCommand(hw_cartesian_velocity_commands_) && cartesian_velocity_interface_state_.running) {
    robot_->write_once_cartesian_velocity(hw_cartesian_velocity_commands_);
  }
  return hardware_interface::return_type::OK;
}

rclcpp::Logger RBPodoHardwareInterface::getLogger() {
  return rclcpp::get_logger("RBPodoHardwareInterface");
}

}  // namespace rbpodo_hardware

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(rbpodo_hardware::RBPodoHardwareInterface, hardware_interface::SystemInterface)