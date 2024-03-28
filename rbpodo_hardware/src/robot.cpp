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

#include "rbpodo_hardware/robot.hpp"

#include <chrono>

using namespace std;
using namespace rb::podo;
using namespace std::chrono_literals;

namespace {

template <typename ArrayLike>
ArrayLike convert_to_degree(const ArrayLike& arr) {
  ArrayLike ret;
  assert(ret.size() == arr.size());
  for (size_t i = 0; i < ret.size(); i++) {
    ret[i] = arr[i] * RAD2DEG;
  }
  return ret;
}

}  // namespace

namespace rbpodo_hardware {

Robot::Robot(const string& ip, bool simulation, const rclcpp::Logger& logger)
    : ip_(ip), cobot_(ip), cobot_data_(ip), logger_(logger) {
  if (simulation) {
    set_operation_mode(OperationMode::Simulation);
  } else {
    set_operation_mode(OperationMode::Real);
  }
  set_speed_bar(1.0);

  read_thread_stop_.store(false);
  read_thread_ = thread([this]() {
    auto duration = chrono::nanoseconds((long)(1e9 / kStateUpdateRate));
    auto next_time = chrono::steady_clock::now() + duration;
    while (!read_thread_stop_.load()) {
      this_thread::sleep_until(next_time);
      next_time += duration;

      auto s = cobot_data_.request_data();
      RCLCPP_FATAL_EXPRESSION(getLogger(), !s.has_value(),
                              "request_data() must return valid value with unlimited timeout");
      {
        scoped_lock ul(read_lock_);
        state_ = s.value();
        state_updated_time_ = chrono::steady_clock::now();
        updated_ = true;
      }
      read_cv_.notify_all();
    }
  });
  {
    unique_lock ul(read_lock_);
    updated_ = false;
    read_cv_.wait(ul, [&] { return updated_; });
  }

  disable_waiting_ack();
}

Robot::~Robot() {
  read_thread_stop_.store(true);
  if (read_thread_.joinable()) {
    read_thread_.join();
  }
}

bool Robot::try_aquire_move_lock() {
  return move_lock_.try_lock();
}

void Robot::release_move_lock() {
  return move_lock_.unlock();
}

void Robot::rc_error_check(bool print, bool clear) {
  const auto& error = rc_.error(true);
  for (const auto& e : error) {
    if (print) {
      switch (e.type()) {
        case Response::Type::Warn: {
          RCLCPP_WARN(getLogger(), "Response - [%s][%s]", e.category().c_str(), e.msg().c_str());
          break;
        }
        case Response::Type::Error: {
          RCLCPP_ERROR(getLogger(), "Response - [%s][%s]", e.category().c_str(), e.msg().c_str());
          break;
        }
        default: {
        }
      }
    }
  }

  if (clear) {
    rc_.clear();
  }
}

bool Robot::set_operation_mode(OperationMode mode) {
  scoped_lock sl(lock_);
  {
    rb::podo::RobotState state;
    auto ret = cobot_.get_robot_state(rc_, state, 0.5, true);
    if (!ret.is_success() || state != rb::podo::RobotState::Idle) {
      rc_error_check();
      return false;
    }
  }

  auto ret = cobot_.set_operation_mode(rc_, mode);
  rc_error_check();
  return ret.is_success();
}

bool Robot::set_speed_bar(double speed) {
  scoped_lock sl(lock_);
  auto ret = cobot_.set_speed_bar(rc_, speed);
  rc_error_check();
  return ret.is_success();
}

bool Robot::stop() {
  scoped_lock sl(lock_);
  auto ret = cobot_.task_stop(rc_);
  rc_error_check();
  return ret.is_success();
}

bool Robot::enable_waiting_ack() {
  scoped_lock sl(lock_);
  auto ret = cobot_.enable_waiting_ack(rc_);
  rc_error_check();
  return ret;
}

bool Robot::disable_waiting_ack() {
  scoped_lock sl(lock_);
  auto ret = cobot_.disable_waiting_ack(rc_);
  rc_error_check();
  return ret;
}

bool Robot::eval(const string& script) {
  scoped_lock sl(lock_);
  auto ret = cobot_.eval(rc_, script);
  rc_error_check();
  return ret.is_success();
}

bool Robot::task_load(const string& program_name, double timeout) {
  scoped_lock sl(lock_);
  auto ret = cobot_.task_load(rc_, program_name);
  rc_error_check();
  if (ret.is_success()) {
    ret = cobot_.wait_for_task_loaded(rc_, timeout);
    rc_error_check();
    return ret.is_success();
  }
  return false;
}

bool Robot::task_pause() {
  scoped_lock sl(lock_);
  auto ret = cobot_.task_pause(rc_);
  rc_error_check();
  return ret.is_success();
}

bool Robot::task_play(double timeout) {
  scoped_lock sl(lock_);
  auto ret = cobot_.task_play(rc_);
  rc_error_check();
  if (ret.is_success()) {
    ret = cobot_.wait_for_task_started(rc_, timeout);
    rc_error_check();
    return ret.is_success();
  }
  return false;
}

bool Robot::task_resume(bool collision) {
  scoped_lock sl(lock_);
  auto ret = cobot_.task_resume(rc_, collision);
  rc_error_check();
  return ret.is_success();
}

bool Robot::task_stop(double timeout) {
  scoped_lock sl(lock_);
  auto state = read_once();
  if (state.sdata.task_state == 1)
    return true;

  auto ret = cobot_.task_stop(rc_);
  rc_error_check();
  if (ret.is_success()) {
    ret = cobot_.wait_for_task_finished(rc_, timeout);
    rc_error_check();
    return ret.is_success();
  }
  return false;
}

bool Robot::move_stop(double timeout) {
  scoped_lock sl(lock_);
  auto state = read_once();
  if (state.sdata.robot_state == 1)
    return true;

  auto ret = cobot_.task_stop(rc_);
  rc_error_check();
  if (ret.is_success()) {
    ret = cobot_.wait_for_move_finished(rc_, timeout);
    rc_error_check();
    return ret.is_success();
  }
  return false;
}

rb::podo::SystemState Robot::read_once() {
  scoped_lock sl(read_lock_);
  auto elapsed = chrono::steady_clock::now() - state_updated_time_;
  double elapsed_in_sec = chrono::duration_cast<chrono::nanoseconds>(elapsed).count() / 1.e9;
  RCLCPP_WARN_EXPRESSION(getLogger(), elapsed_in_sec > 0.1, "state was updataed %lf seconds ago (it may cause problem)",
                         elapsed_in_sec);
  return state_;
}

void Robot::wait_for_update() {
  unique_lock ul(read_lock_);
  updated_ = false;
  read_cv_.wait(ul, [&] { return updated_; });
}

bool Robot::write_once_joint_positions(const array<double, kNumberOfJoints>& positions) {
  scoped_lock sl(lock_);
  auto ret = cobot_.move_servo_j(rc_, convert_to_degree(positions), jpc_config_.t1, jpc_config_.t2, jpc_config_.gain,
                                 jpc_config_.alpha);
  rc_error_check();
  return ret.is_success();
}

bool Robot::write_once_joint_velocities(const array<double, kNumberOfJoints>& velocities) {
  scoped_lock sl(lock_);
  auto ret = cobot_.move_speed_j(rc_, convert_to_degree(velocities), jvc_config_.t1, jvc_config_.t2, jvc_config_.gain,
                                 jvc_config_.alpha);
  rc_error_check();
  return ret.is_success();
}

bool Robot::write_once_joint_efforts(const array<double, kNumberOfJoints>& efforts) {
  scoped_lock sl(lock_);
  auto ret = cobot_.move_servo_t(rc_, efforts, jec_config_.t1, jec_config_.t2, jec_config_.compensation_mode);
  rc_error_check();
  return ret.is_success();
}

bool Robot::write_once_cartesian_pose(const array<double, k6DoFDim>& pose) {
  scoped_lock sl(lock_);
  array<double, k6DoFDim> p;
  p[0] = pose[0] * METER2MILLIMETER;
  p[1] = pose[1] * METER2MILLIMETER;
  p[2] = pose[2] * METER2MILLIMETER;
  p[3] = pose[3] * RAD2DEG;
  p[4] = pose[4] * RAD2DEG;
  p[5] = pose[5] * RAD2DEG;
  auto ret = cobot_.move_servo_l(rc_, p, cpc_config_.t1, cpc_config_.t2, cpc_config_.gain, cpc_config_.alpha);
  rc_error_check();
  return ret.is_success();
}

bool Robot::write_once_cartesian_velocity(const array<double, k6DoFDim>& velocity) {
  scoped_lock sl(lock_);
  array<double, k6DoFDim> v;
  v[0] = velocity[0] * METER2MILLIMETER;
  v[1] = velocity[1] * METER2MILLIMETER;
  v[2] = velocity[2] * METER2MILLIMETER;
  v[3] = velocity[3] * RAD2DEG;
  v[4] = velocity[4] * RAD2DEG;
  v[5] = velocity[5] * RAD2DEG;
  auto ret = cobot_.move_servo_l(rc_, v, cvc_config_.t1, cvc_config_.t2, cvc_config_.gain, cvc_config_.alpha);
  rc_error_check();
  return ret.is_success();
}

void Robot::set_joint_position_controller_config(JointPositionControllerConfig jpc_config) {
  jpc_config_ = jpc_config;
}

void Robot::set_joint_velocity_controller_config(JointVelocityControllerConfig jvc_config) {
  jvc_config_ = jvc_config;
}

void Robot::set_joint_effort_controller_config(JointEffortControllerConfig jec_config) {
  jec_config_ = jec_config;
}

void Robot::set_cartesian_pose_controller_config(CartesianPoseControllerConfig cpc_config) {
  cpc_config_ = cpc_config;
}

void Robot::set_cartesian_velocity_controller_config(CartesianVelocityControllerConfig cvc_config) {
  cvc_config_ = cvc_config;
}

bool Robot::move_j(const std::array<double, kNumberOfJoints>& joint, double speed, double acceleration,
                   double timeout_for_waiting_start) {
  scoped_lock sl(lock_);
  cobot_.flush(rc_);
  rc_error_check();
  auto ret = cobot_.move_j(rc_, convert_to_degree(joint), speed * RAD2DEG, acceleration * RAD2DEG);
  if (ret.is_success()) {
    ret = cobot_.wait_for_move_started(rc_, timeout_for_waiting_start);
  }
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_l(const std::array<double, k6DoFDim>& point, double speed, double acceleration,
                   double timeout_for_waiting_start) {
  scoped_lock sl(lock_);
  array<double, k6DoFDim> p;
  p[0] = point[0] * METER2MILLIMETER;
  p[1] = point[1] * METER2MILLIMETER;
  p[2] = point[2] * METER2MILLIMETER;
  p[3] = point[3] * RAD2DEG;
  p[4] = point[4] * RAD2DEG;
  p[5] = point[5] * RAD2DEG;
  cobot_.flush(rc_);
  rc_error_check();
  auto ret = cobot_.move_l(rc_, p, speed * METER2MILLIMETER, acceleration * METER2MILLIMETER);
  if (ret.is_success()) {
    ret = cobot_.wait_for_move_started(rc_, timeout_for_waiting_start);
  }
  std::cout << rc_ << std::endl;
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_jb2_clear() {
  scoped_lock sl(lock_);
  auto ret = cobot_.move_jb2_clear(rc_);
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_jb2_add(const std::array<double, kNumberOfJoints>& joint, double speed, double acceleration,
                         double blendering_value) {
  scoped_lock sl(lock_);
  auto ret =
      cobot_.move_jb2_add(rc_, convert_to_degree(joint), speed * RAD2DEG, acceleration * RAD2DEG, blendering_value);
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_jb2_run(double timeout_for_waiting_start) {
  scoped_lock sl(lock_);
  cobot_.flush(rc_);
  rc_error_check();
  auto ret = cobot_.move_jb2_run(rc_);
  if (ret.is_success()) {
    ret = cobot_.wait_for_move_started(rc_, timeout_for_waiting_start);
  }
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_pb_clear() {
  scoped_lock sl(lock_);
  auto ret = cobot_.move_pb_clear(rc_);
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_pb_add(const std::array<double, k6DoFDim>& point, double speed, rb::podo::BlendingOption option,
                        double blending_value) {
  scoped_lock sl(lock_);
  array<double, k6DoFDim> p;
  p[0] = point[0] * METER2MILLIMETER;
  p[1] = point[1] * METER2MILLIMETER;
  p[2] = point[2] * METER2MILLIMETER;
  p[3] = point[3] * RAD2DEG;
  p[4] = point[4] * RAD2DEG;
  p[5] = point[5] * RAD2DEG;
  auto ret = cobot_.move_pb_add(rc_, p, speed * METER2MILLIMETER, option, blending_value);
  rc_error_check();
  return ret.is_success();
}

bool Robot::move_pb_run(double acceleration, rb::podo::MovePBOption option, double timeout_for_waiting_start) {
  scoped_lock sl(lock_);
  cobot_.flush(rc_);
  rc_error_check();
  auto ret = cobot_.move_pb_run(rc_, acceleration, option);
  if (ret.is_success()) {
    ret = cobot_.wait_for_move_started(rc_, timeout_for_waiting_start);
  }
  rc_error_check();
  return ret.is_success();
}

rclcpp::Logger Robot::getLogger() {
  return logger_;
}

}  // namespace rbpodo_hardware