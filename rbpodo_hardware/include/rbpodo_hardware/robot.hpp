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

#include <condition_variable>

#include "rbpodo/rbpodo.hpp"
#include "rclcpp/rclcpp.hpp"

#define DEG2RAD (M_PI / 180.)
#define RAD2DEG (180. / M_PI)
#define MILLIMETER2METER 0.001
#define METER2MILLIMETER 1000

namespace rbpodo_hardware {

class Robot {
 public:
  static constexpr size_t kNumberOfJoints{6};
  static constexpr size_t k6DoFDim{6};
  static constexpr double kStateUpdateRate{500};

  struct JointPositionControllerConfig {
    /// Arrival time to the destination (unit: sec. t1 >= 0.002)
    double t1{0.01};

    /// Time to maintain the action after arrival (unit: sec. 0.02 < t2 < 0.2)
    double t2{0.1};

    /// Speed tracking rate (gain > 0)
    double gain{0.5};

    /// Low-pass filter gain (The smaller it is, the smoother the action becomes. 0 < alpha < 1)
    double alpha{0.5};
  };

  struct JointVelocityControllerConfig {
    /// Arrival time to the destination (unit: sec. t1 >= 0.002)
    double t1{0.01};

    /// Time to maintain the action after arrival (unit: sec. 0.02 < t2 < 0.2)
    double t2{0.1};

    /// Speed tracking rate (gain > 0)
    double gain{0.5};

    /// Low-pass filter gain (The smaller it is, the smoother the action becomes. 0 < alpha < 1)
    double alpha{0.5};
  };

  struct JointEffortControllerConfig {
    /// Arrival time to the destination (unit: sec. t1 >= 0.002)
    double t1{0.01};

    /// Time to maintain the action after arrival (unit: sec. 0.02 < t2 < 0.2)
    double t2{0.1};

    /// Compensation mode
    /// mode:
    /// 0: target effort = input effort
    /// 1: target effort = input effort + gravity compensation value
    /// 2: target effort = input effort + friction compensation value 30%
    /// 3: target effort = input effort + gravity compensation value + frcition compensation value 30%
    int compensation_mode{0};
  };

  struct CartesianPoseControllerConfig {
    /// Arrival time to the destination (unit: sec. t1 >= 0.002)
    double t1{0.01};

    /// Time to maintain the action after arrival (unit: sec. 0.02 < t2 < 0.2)
    double t2{0.1};

    /// Speed tracking rate (gain > 0)
    double gain{0.5};

    /// Low-pass filter gain (The smaller it is, the smoother the action becomes. 0 < alpha < 1)
    double alpha{0.5};
  };

  struct CartesianVelocityControllerConfig {
    /// Arrival time to the destination (unit: sec. t1 >= 0.002)
    double t1{0.01};

    /// Time to maintain the action after arrival (unit: sec. 0.02 < t2 < 0.2)
    double t2{0.1};

    /// Speed tracking rate (gain > 0)
    double gain{0.5};

    /// Low-pass filter gain (The smaller it is, the smoother the action becomes. 0 < alpha < 1)
    double alpha{0.5};
  };

  explicit Robot(const std::string& ip, bool simulation, const rclcpp::Logger& logger);

  ~Robot();

  bool try_aquire_move_lock();

  void release_move_lock();

  [[nodiscard]] std::string ip() const { return ip_; }

  rb::podo::SystemState read_once();

  void wait_for_update();

  void rc_error_check(bool print = true, bool clear = true);

  bool set_operation_mode(rb::podo::OperationMode mode);

  bool set_speed_bar(double speed);

  bool stop();

  bool enable_waiting_ack();

  bool disable_waiting_ack();

  bool eval(const std::string& script);

  bool task_load(const std::string& program_name, double timeout = -1.);

  bool task_pause();

  bool task_play(double timeout = -1.);

  bool task_resume(bool collision);

  bool task_stop(double timeout = -1.);

  bool move_stop(double timeout = -1.);

  bool write_once_joint_positions(const std::array<double, kNumberOfJoints>& positions);

  bool write_once_joint_velocities(const std::array<double, kNumberOfJoints>& velocities);

  bool write_once_joint_efforts(const std::array<double, kNumberOfJoints>& efforts);

  bool write_once_cartesian_pose(const std::array<double, k6DoFDim>& pose);

  bool write_once_cartesian_velocity(const std::array<double, k6DoFDim>& velocity);

  void set_joint_position_controller_config(JointPositionControllerConfig jpc_config);

  void set_joint_velocity_controller_config(JointVelocityControllerConfig jvc_config);

  void set_joint_effort_controller_config(JointEffortControllerConfig jec_config);

  void set_cartesian_pose_controller_config(CartesianPoseControllerConfig cpc_config);

  void set_cartesian_velocity_controller_config(CartesianVelocityControllerConfig cvc_config);

  /**
   * Move -- Cobot
   */

  /// move_j
  bool move_j(const std::array<double, kNumberOfJoints>& joint, double speed, double acceleration,
              double timeout_for_waiting_start);

  /// move_l
  bool move_l(const std::array<double, k6DoFDim>& point, double speed, double acceleration,
              double timeout_for_waiting_start);

  /// move_jb2_clear
  bool move_jb2_clear();

  /// move_jb2_add
  bool move_jb2_add(const std::array<double, kNumberOfJoints>& joint, double speed, double acceleration,
                    double blendering_value);

  /// move_jb2_run
  bool move_jb2_run(double timeout_for_waiting_start);

  /// move_pb_clear
  bool move_pb_clear();

  /// move_pb_add
  bool move_pb_add(const std::array<double, k6DoFDim>& point, double speed, rb::podo::BlendingOption option,
                   double blending_value);

  /// move_pb_run
  bool move_pb_run(double acceleration, rb::podo::MovePBOption option, double timeout_for_waiting_start);

  rclcpp::Logger getLogger();

 private:
  std::string ip_;

  rb::podo::Cobot<rb::podo::StandardVector> cobot_;
  rb::podo::CobotData cobot_data_;
  rb::podo::ResponseCollector rc_;
  rb::podo::SystemState state_;
  std::chrono::steady_clock::time_point state_updated_time_;
  std::thread read_thread_;
  std::atomic<bool> read_thread_stop_;
  std::mutex lock_;
  bool updated_{false};
  std::mutex read_lock_;
  std::condition_variable read_cv_;
  std::mutex move_lock_;

  rclcpp::Logger logger_;

  /**
   * controller configs
   */
  // joint position controller
  JointPositionControllerConfig jpc_config_;
  // joint velocity controller
  JointVelocityControllerConfig jvc_config_;
  // joint effort controller (joint torque controller)
  JointEffortControllerConfig jec_config_;
  // cartesian pose controller
  CartesianPoseControllerConfig cpc_config_;
  // cartesian velocity controller
  CartesianVelocityControllerConfig cvc_config_;
};

}  // namespace rbpodo_hardware