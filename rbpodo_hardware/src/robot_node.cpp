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

#include "rbpodo_hardware/robot_node.hpp"

using namespace std;
using namespace rb;
using namespace std::chrono_literals;
using namespace std::placeholders;

namespace rbpodo_hardware {

RobotNode::RobotNode(const rclcpp::NodeOptions& options, shared_ptr<Robot> robot)
    : rclcpp::Node("rbpodo_hardware", options), robot_(robot), res_receiver_(robot_->ip()) {
  res_publisher_ = this->create_publisher<rbpodo_msgs::msg::Response>("~/response", 10);
  //
  run_ = true;
  msg_thread_ = thread([this]() {
    while (run_) {
      response_callback();
      this_thread::sleep_for(10us);
    }
  });

  eval_srv_ = create_service<rbpodo_msgs::srv::Eval>("~/eval", bind(&RobotNode::eval, this, _1, _2));
  task_load_srv_ = create_service<rbpodo_msgs::srv::TaskLoad>("~/task_load", bind(&RobotNode::task_load, this, _1, _2));
  task_pause_srv_ =
      create_service<rbpodo_msgs::srv::TaskPause>("~/task_pause", bind(&RobotNode::task_pause, this, _1, _2));
  task_play_srv_ = create_service<rbpodo_msgs::srv::TaskPlay>("~/task_play", bind(&RobotNode::task_play, this, _1, _2));
  task_resume_srv_ =
      create_service<rbpodo_msgs::srv::TaskResume>("~/task_resume", bind(&RobotNode::task_resume, this, _1, _2));
  task_stop_srv_ = create_service<rbpodo_msgs::srv::TaskStop>("~/task_stop", bind(&RobotNode::task_stop, this, _1, _2));
  set_operation_mode_srv_ = create_service<rbpodo_msgs::srv::SetOperationMode>(
      "~/set_operation_mode", bind(&RobotNode::set_operation_mode, this, _1, _2));
  set_speed_bar_srv_ =
      create_service<rbpodo_msgs::srv::SetSpeedBar>("~/set_speed_bar", bind(&RobotNode::set_speed_bar, this, _1, _2));
  set_joint_position_controller_config_srv_ = create_service<rbpodo_msgs::srv::SetJointPositionControllerConfig>(
      "~/set_joint_position_controller_config", bind(&RobotNode::set_joint_position_controller_config, this, _1, _2));
  set_joint_velocity_controller_config_srv_ = create_service<rbpodo_msgs::srv::SetJointVelocityControllerConfig>(
      "~/set_joint_velocity_controller_config", bind(&RobotNode::set_joint_velocity_controller_config, this, _1, _2));
  set_cartesian_pose_controller_config_srv_ = create_service<rbpodo_msgs::srv::SetCartesianPoseControllerConfig>(
      "~/set_cartesian_pose_controller_config", bind(&RobotNode::set_cartesian_pose_controller_config, this, _1, _2));
  set_cartesian_velocity_controller_config_srv_ =
      create_service<rbpodo_msgs::srv::SetCartesianVelocityControllerConfig>(
          "~/set_cartesian_velocity_controller_config",
          bind(&RobotNode::set_cartesian_velocity_controller_config, this, _1, _2));

  move_j_server_ = rclcpp_action::create_server<MoveJ>(
      this, "~/move_j", std::bind(&RobotNode::move_j_handle_goal, this, _1, _2),
      std::bind(&RobotNode::move_j_handle_cancel, this, _1), std::bind(&RobotNode::move_j_handle_accepted, this, _1));
  move_l_server_ = rclcpp_action::create_server<MoveL>(
      this, "~/move_l", std::bind(&RobotNode::move_l_handle_goal, this, _1, _2),
      std::bind(&RobotNode::move_l_handle_cancel, this, _1), std::bind(&RobotNode::move_l_handle_accepted, this, _1));
  move_jb2_server_ = rclcpp_action::create_server<MoveJb2>(this, "~/move_jb2",
                                                           std::bind(&RobotNode::move_jb2_handle_goal, this, _1, _2),
                                                           std::bind(&RobotNode::move_jb2_handle_cancel, this, _1),
                                                           std::bind(&RobotNode::move_jb2_handle_accepted, this, _1));
  move_pb_server_ = rclcpp_action::create_server<MovePb>(
      this, "~/move_pb", std::bind(&RobotNode::move_pb_handle_goal, this, _1, _2),
      std::bind(&RobotNode::move_pb_handle_cancel, this, _1), std::bind(&RobotNode::move_pb_handle_accepted, this, _1));
}

RobotNode::~RobotNode() {
  run_ = false;
  if (msg_thread_.joinable()) {
    msg_thread_.join();
  }
}

void RobotNode::response_callback() {
  res_receiver_.flush(res_storage_);
  while (!res_storage_.empty()) {
    const auto& res = res_storage_.front();

    auto message = rbpodo_msgs::msg::Response();
    switch (res.type()) {
      case podo::Response::Type::ACK: {
        message.type = rbpodo_msgs::msg::Response::ACK;
        break;
      }
      case podo::Response::Type::Info: {
        message.type = rbpodo_msgs::msg::Response::INFO;
        break;
      }
      case podo::Response::Type::Warn: {
        message.type = rbpodo_msgs::msg::Response::WARN;
        break;
      }
      case podo::Response::Type::Error: {
        message.type = rbpodo_msgs::msg::Response::ERROR;

        if (res.category() == "code") {
          int code = atoi(res.msg().c_str());
          if (podo::ErrorCodeMessage.find(code) != podo::ErrorCodeMessage.end()) {
            message.error = podo::ErrorCodeMessage[code].en;
          }
        }

        break;
      }
      case podo::Response::Type::Unknown: {
        message.type = rbpodo_msgs::msg::Response::UNKNOWN;
        break;
      }
    }
    message.category = res.category();
    message.msg = res.msg();
    res_publisher_->publish(message);

    res_storage_.pop_front();
  }
}

rbpodo_msgs::msg::SystemState RobotNode::convert_to_ros_msg(const podo::SystemState& state) {
  rbpodo_msgs::msg::SystemState msg;

  msg.time = state.sdata.time;
  for (size_t i = 0; i < 6; i++) {
    msg.jnt_ref[i] = state.sdata.jnt_ref[i] * DEG2RAD;
    msg.jnt_ang[i] = state.sdata.jnt_ang[i] * DEG2RAD;
    msg.jnt_cur[i] = state.sdata.jnt_cur[i];
    // msg.tcp_ref[i] = state.sdata.tcp_ref[i];
    // msg.tcp_pos[i] = state.sdata.tcp_pos[i];
  }
  msg.tcp_ref[0] = state.sdata.tcp_ref[0] * MILLIMETER2METER;
  msg.tcp_ref[1] = state.sdata.tcp_ref[1] * MILLIMETER2METER;
  msg.tcp_ref[2] = state.sdata.tcp_ref[2] * MILLIMETER2METER;
  msg.tcp_ref[3] = state.sdata.tcp_ref[3] * DEG2RAD;
  msg.tcp_ref[4] = state.sdata.tcp_ref[4] * DEG2RAD;
  msg.tcp_ref[5] = state.sdata.tcp_ref[5] * DEG2RAD;
  msg.tcp_pos[0] = state.sdata.tcp_pos[0] * MILLIMETER2METER;
  msg.tcp_pos[1] = state.sdata.tcp_pos[1] * MILLIMETER2METER;
  msg.tcp_pos[2] = state.sdata.tcp_pos[2] * MILLIMETER2METER;
  msg.tcp_pos[3] = state.sdata.tcp_pos[3] * DEG2RAD;
  msg.tcp_pos[4] = state.sdata.tcp_pos[4] * DEG2RAD;
  msg.tcp_pos[5] = state.sdata.tcp_pos[5] * DEG2RAD;
  for (size_t i = 0; i < 4; i++) {
    msg.analog_in[i] = state.sdata.analog_in[i];
    msg.analog_out[i] = state.sdata.analog_out[i];
  }
  for (size_t i = 0; i < 16; i++) {
    msg.digital_in[i] = (state.sdata.digital_in[i] == 1);
    msg.digital_out[i] = (state.sdata.digital_out[i] == 1);
  }
  for (size_t i = 0; i < 6; i++) {
    msg.jnt_temperature[i] = state.sdata.jnt_temperature[i];
  }
  msg.task_pc = state.sdata.task_pc;
  msg.task_repeat = state.sdata.task_repeat;
  msg.task_run_id = state.sdata.task_run_id;
  msg.task_run_num = state.sdata.task_run_num;
  msg.task_run_time = state.sdata.task_run_time;
  msg.task_state = state.sdata.task_state;
  msg.default_speed = state.sdata.default_speed;
  msg.robot_state = state.sdata.robot_state;
  msg.information_chunk_1 = state.sdata.information_chunk_1;
  for (size_t i = 0; i < 6; i++) {
    msg.jnt_info[i] = state.sdata.jnt_info[i];
  }
  msg.collision_detect_onoff = (state.sdata.collision_detect_onoff == 1);
  msg.is_freedrive_mode = (state.sdata.is_freedrive_mode == 1);
  msg.real_vs_simulation_mode = (state.sdata.real_vs_simulation_mode == 1);
  msg.init_state_info = state.sdata.init_state_info;
  msg.init_error = state.sdata.init_error;
  for (size_t i = 0; i < 2; i++) {
    msg.tfb_analog_in[i] = state.sdata.tfb_analog_in[i];
    msg.tfb_digital_in[i] = (state.sdata.tfb_digital_in[i] == 1);
    msg.tfb_digital_out[i] = (state.sdata.tfb_digital_out[i] == 1);
  }
  msg.op_stat_collision_occur = (state.sdata.op_stat_collision_occur == 1);
  msg.op_stat_sos_flag = state.sdata.op_stat_sos_flag;
  msg.op_stat_self_collision = (state.sdata.op_stat_self_collision == 1);
  msg.op_stat_soft_estop_occur = (state.sdata.op_stat_soft_estop_occur == 1);
  msg.op_stat_ems_flag = state.sdata.op_stat_ems_flag;
  msg.information_chunk_2 = state.sdata.information_chunk_2;
  msg.information_chunk_3 = state.sdata.information_chunk_3;
  for (size_t i = 0; i < 2; i++) {
    msg.inbox_trap_flag[i] = (state.sdata.inbox_trap_flag[i] == 1);
    msg.inbox_check_mode[i] = state.sdata.inbox_check_mode[i];
  }
  msg.eft[0] = state.sdata.eft_fx;
  msg.eft[1] = state.sdata.eft_fy;
  msg.eft[2] = state.sdata.eft_fz;
  msg.eft[3] = state.sdata.eft_mx;
  msg.eft[4] = state.sdata.eft_my;
  msg.eft[5] = state.sdata.eft_mz;
  msg.information_chunk_4 = state.sdata.information_chunk_4;
  for (size_t i = 0; i < 4; i++) {
    msg.extend_io1_analog_in[i] = state.sdata.extend_io1_analog_in[i];
    msg.extend_io1_analog_out[i] = state.sdata.extend_io1_analog_out[i];
  }
  msg.extend_io1_digital_info = state.sdata.extend_io1_digital_info;
  for (size_t i = 0; i < 6; i++) {
    msg.aa_joint_ref[i] = state.sdata.aa_joint_ref[i] * DEG2RAD;
  }
  msg.safety_board_stat_info = state.sdata.safety_board_stat_info;
  return msg;
}

void RobotNode::eval(const rbpodo_msgs::srv::Eval::Request::SharedPtr& request,
                     rbpodo_msgs::srv::Eval::Response::SharedPtr response) {
  response->success = robot_->eval(request->script);
}

void RobotNode::task_load(const rbpodo_msgs::srv::TaskLoad::Request::SharedPtr& request,
                          rbpodo_msgs::srv::TaskLoad::Response::SharedPtr response) {
  response->success = robot_->task_load(request->program_name, request->timeout);
}

void RobotNode::task_pause(const rbpodo_msgs::srv::TaskPause::Request::SharedPtr&,
                           rbpodo_msgs::srv::TaskPause::Response::SharedPtr response) {
  response->success = robot_->task_pause();
}

void RobotNode::task_play(const rbpodo_msgs::srv::TaskPlay::Request::SharedPtr& request,
                          rbpodo_msgs::srv::TaskPlay::Response::SharedPtr response) {
  response->success = robot_->task_play(request->timeout);
}

void RobotNode::task_resume(const rbpodo_msgs::srv::TaskResume::Request::SharedPtr& request,
                            rbpodo_msgs::srv::TaskResume::Response::SharedPtr response) {
  response->success = robot_->task_resume(request->collision);
}

void RobotNode::task_stop(const rbpodo_msgs::srv::TaskStop::Request::SharedPtr& request,
                          rbpodo_msgs::srv::TaskStop::Response::SharedPtr response) {
  response->success = robot_->task_stop(request->timeout);
}

void RobotNode::set_operation_mode(const rbpodo_msgs::srv::SetOperationMode::Request::SharedPtr& request,
                                   rbpodo_msgs::srv::SetOperationMode::Response::SharedPtr response) {
  podo::OperationMode mode;
  switch (request->mode) {
    case rbpodo_msgs::srv::SetOperationMode::Request::REAL: {
      mode = podo::OperationMode::Real;
      break;
    }
    case rbpodo_msgs::srv::SetOperationMode::Request::SIMULATION: {
      mode = podo::OperationMode::Simulation;
      break;
    }
    default: {
      response->success = false;
      return;
    }
  }
  if (!robot_->set_operation_mode(mode)) {
    response->success = false;
    return;
  }
  response->success = true;
}

void RobotNode::set_speed_bar(const rbpodo_msgs::srv::SetSpeedBar::Request::SharedPtr& request,
                              rbpodo_msgs::srv::SetSpeedBar::Response::SharedPtr response) {
  if (request->speed < 0 || request->speed > 1) {
    response->success = false;
    return;
  }

  if (!robot_->set_speed_bar(request->speed)) {
    response->success = false;
    return;
  }
  response->success = true;
}

void RobotNode::set_joint_position_controller_config(
    const rbpodo_msgs::srv::SetJointPositionControllerConfig::Request::SharedPtr& request,
    rbpodo_msgs::srv::SetJointPositionControllerConfig::Response::SharedPtr response) {
  Robot::JointPositionControllerConfig config;
  config.t1 = request->t1;
  config.t2 = request->t2;
  config.gain = request->gain;
  config.alpha = request->alpha;
  robot_->set_joint_position_controller_config(config);
  response->success = true;
}

void RobotNode::set_joint_velocity_controller_config(
    const rbpodo_msgs::srv::SetJointVelocityControllerConfig::Request::SharedPtr& request,
    rbpodo_msgs::srv::SetJointVelocityControllerConfig::Response::SharedPtr response) {
  Robot::JointVelocityControllerConfig config;
  config.t1 = request->t1;
  config.t2 = request->t2;
  config.gain = request->gain;
  config.alpha = request->alpha;
  robot_->set_joint_velocity_controller_config(config);
  response->success = true;
}

void RobotNode::set_joint_effort_controller_config(
    const rbpodo_msgs::srv::SetJointEffortControllerConfig::Request::SharedPtr& request,
    rbpodo_msgs::srv::SetJointEffortControllerConfig::Response::SharedPtr response) {
  if (request->mode > 3) {
    response->success = false;
    return;
  }
  Robot::JointEffortControllerConfig config;
  config.t1 = request->t1;
  config.t2 = request->t2;
  config.compensation_mode = request->mode;
  robot_->set_joint_effort_controller_config(config);
  response->success = true;
}

void RobotNode::set_cartesian_pose_controller_config(
    const rbpodo_msgs::srv::SetCartesianPoseControllerConfig::Request::SharedPtr& request,
    rbpodo_msgs::srv::SetCartesianPoseControllerConfig::Response::SharedPtr response) {
  Robot::CartesianPoseControllerConfig config;
  config.t1 = request->t1;
  config.t2 = request->t2;
  config.gain = request->gain;
  config.alpha = request->alpha;
  robot_->set_cartesian_pose_controller_config(config);
  response->success = true;
}

void RobotNode::set_cartesian_velocity_controller_config(
    const rbpodo_msgs::srv::SetCartesianVelocityControllerConfig::Request::SharedPtr& request,
    rbpodo_msgs::srv::SetCartesianVelocityControllerConfig::Response::SharedPtr response) {
  Robot::CartesianVelocityControllerConfig config;
  config.t1 = request->t1;
  config.t2 = request->t2;
  config.gain = request->gain;
  config.alpha = request->alpha;
  robot_->set_cartesian_velocity_controller_config(config);
  response->success = true;
}

rclcpp_action::GoalResponse RobotNode::move_j_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                          shared_ptr<const MoveJ::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Receive move_j request");
  stringstream ss;
  for (int i = 0; i < 6; i++) {
    if (i != 0)
      ss << ", ";
    ss << goal->joint[i];
  }
  RCLCPP_INFO(get_logger(), "  - joint: [%s]", ss.str().c_str());
  RCLCPP_INFO(get_logger(), "  - speed: %f", goal->speed);
  RCLCPP_INFO(get_logger(), "  - acceleration: %f", goal->acceleration);

  auto state = robot_->read_once();
  if (state.sdata.robot_state == 3) {
    RCLCPP_WARN(get_logger(), "Cannot start move_j (robot_state == 3, robot is moving.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!robot_->try_aquire_move_lock()) {
    RCLCPP_ERROR(get_logger(), "Cannot start move_j (move_lock is locked.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotNode::move_j_handle_cancel(const shared_ptr<GoalHandleMoveJ> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Receive equest to cancel move_j");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotNode::move_j_handle_accepted(const shared_ptr<GoalHandleMoveJ> goal_handle) {
  thread{std::bind(&RobotNode::move_j_execute, this, _1), goal_handle}.detach();
}

void RobotNode::move_j_execute(const shared_ptr<GoalHandleMoveJ> goal_handle) {
  MutexReleaser mr(robot_);

  RCLCPP_INFO(get_logger(), "Executing move_j");

  rclcpp::Rate loop_rate(kMoveFeedbackRate);
  const auto& goal = goal_handle->get_goal().get();
  auto result = std::make_shared<MoveJ::Result>();
  auto feedback = std::make_shared<MoveJ::Feedback>();

  std::array<double, Robot::kNumberOfJoints> joint;
  std::copy(goal->joint.begin(), goal->joint.end(), joint.begin());

  if (!robot_->move_j(joint, goal->speed, goal->acceleration, goal->time_for_waiting_start)) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_INFO(get_logger(),
                "move_j failed to start. This could be due to the target being too close or due to another error");
    return;
  }
  robot_->wait_for_update();
  while (rclcpp::ok()) {
    const auto& state = robot_->read_once();
    if (state.sdata.robot_state == 1) {
      break;
    }
    if (goal_handle->is_canceling()) {
      robot_->move_stop();
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "move_j canceled");
      return;
    }
    feedback->state = convert_to_ros_msg(state);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "move_j finished");
  }
}

rclcpp_action::GoalResponse RobotNode::move_l_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                          shared_ptr<const MoveL::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Receive move_l request");
  stringstream ss;
  for (int i = 0; i < 6; i++) {
    if (i != 0)
      ss << ", ";
    ss << goal->point[i];
  }
  RCLCPP_INFO(get_logger(), "  - point: [%s]", ss.str().c_str());
  RCLCPP_INFO(get_logger(), "  - speed: %f", goal->speed);
  RCLCPP_INFO(get_logger(), "  - acceleration: %f", goal->acceleration);

  auto state = robot_->read_once();
  if (state.sdata.robot_state == 3) {
    RCLCPP_ERROR(get_logger(), "Cannot start move_l (robot_state == 3, robot is moving.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!robot_->try_aquire_move_lock()) {
    RCLCPP_ERROR(get_logger(), "Cannot start move_l (move_lock is locked.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotNode::move_l_handle_cancel(const shared_ptr<GoalHandleMoveL> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Receive equest to cancel move_l");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotNode::move_l_handle_accepted(const shared_ptr<GoalHandleMoveL> goal_handle) {
  thread{std::bind(&RobotNode::move_l_execute, this, _1), goal_handle}.detach();
}

void RobotNode::move_l_execute(const shared_ptr<GoalHandleMoveL> goal_handle) {
  MutexReleaser mr(robot_);

  RCLCPP_INFO(get_logger(), "Executing move_l");

  rclcpp::Rate loop_rate(kMoveFeedbackRate);
  const auto& goal = goal_handle->get_goal().get();
  auto result = std::make_shared<MoveL::Result>();
  auto feedback = std::make_shared<MoveL::Feedback>();

  std::array<double, Robot::k6DoFDim> point;
  std::copy(goal->point.begin(), goal->point.end(), point.begin());

  if (!robot_->move_l(point, goal->speed, goal->acceleration, goal->time_for_waiting_start)) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(),
                "move_l failed to start. This could be due to the target being too close or due to another error");
    return;
  }
  robot_->wait_for_update();
  while (rclcpp::ok()) {
    const auto& state = robot_->read_once();
    if (state.sdata.robot_state == 1) {
      break;
    }
    if (goal_handle->is_canceling()) {
      robot_->move_stop();
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Move_l canceled");
      return;
    }
    feedback->state = convert_to_ros_msg(state);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Move_l finished");
  }
}

rclcpp_action::GoalResponse RobotNode::move_jb2_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                            std::shared_ptr<const MoveJb2::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Receive move_jb2 request");
  RCLCPP_INFO(get_logger(), "  - points:");
  for (const auto& p : goal->points) {
    stringstream ss;
    for (int i = 0; i < 6; i++) {
      if (i != 0)
        ss << ", ";
      ss << p.joint[i];
    }
    RCLCPP_INFO(get_logger(), "    - joint: [%s]", ss.str().c_str());
    RCLCPP_INFO(get_logger(), "      speed: %f", p.speed);
    RCLCPP_INFO(get_logger(), "      acceleration: %f", p.acceleration);
    RCLCPP_INFO(get_logger(), "      blending_value: %f", p.blending_value);
  }

  auto state = robot_->read_once();
  if (state.sdata.robot_state == 3) {
    RCLCPP_WARN(get_logger(), "Cannot start move_jb2 (robot_state == 3, robot is moving.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!robot_->try_aquire_move_lock()) {
    RCLCPP_ERROR(get_logger(), "Cannot start move_jb2 (move_lock is locked.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotNode::move_jb2_handle_cancel(const std::shared_ptr<GoalHandleMoveJb2> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Receive equest to cancel move_jb2");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotNode::move_jb2_handle_accepted(const std::shared_ptr<GoalHandleMoveJb2> goal_handle) {
  thread{std::bind(&RobotNode::move_jb2_execute, this, _1), goal_handle}.detach();
}

void RobotNode::move_jb2_execute(const std::shared_ptr<GoalHandleMoveJb2> goal_handle) {
  MutexReleaser mr(robot_);

  RCLCPP_INFO(get_logger(), "Executing move_jb2");

  rclcpp::Rate loop_rate(kMoveFeedbackRate);
  const auto& goal = goal_handle->get_goal().get();
  auto result = std::make_shared<MoveJb2::Result>();
  auto feedback = std::make_shared<MoveJb2::Feedback>();

  robot_->move_jb2_clear();
  for (const auto& p : goal->points) {
    std::array<double, Robot::k6DoFDim> joint;
    std::copy(p.joint.begin(), p.joint.end(), joint.begin());

    if (!robot_->move_jb2_add(joint, p.speed, p.acceleration, p.blending_value)) {
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "move_jb2_add failed");
      return;
    }
  }
  if (!robot_->move_jb2_run(goal->time_for_waiting_start)) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(),
                "move_jb2 failed to start. This could be due to the target being too close or due to another error");
    return;
  }
  robot_->wait_for_update();
  while (rclcpp::ok()) {
    const auto& state = robot_->read_once();
    if (state.sdata.robot_state == 1) {
      break;
    }
    if (goal_handle->is_canceling()) {
      robot_->move_stop();
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Move_jb2 canceled");
      return;
    }
    feedback->state = convert_to_ros_msg(state);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Move_jb2 finished");
  }
}

rclcpp_action::GoalResponse RobotNode::move_pb_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                           std::shared_ptr<const MovePb::Goal> goal) {
  (void)uuid;
  RCLCPP_INFO(get_logger(), "Receive move_pb request");
  RCLCPP_INFO(get_logger(), "  - points:");
  for (const auto& p : goal->points) {
    stringstream ss;
    for (int i = 0; i < 6; i++) {
      if (i != 0)
        ss << ", ";
      ss << p.point[i];
    }
    RCLCPP_INFO(get_logger(), "    - point: [%s]", ss.str().c_str());
    RCLCPP_INFO(get_logger(), "      speed: %f", p.speed);
    RCLCPP_INFO(get_logger(), "      blending_option: %d", p.blending_option);
    RCLCPP_INFO(get_logger(), "      blending_value: %f", p.blending_value);
  }
  RCLCPP_INFO(get_logger(), "  - acceleration: %f", goal->acceleration);
  RCLCPP_INFO(get_logger(), "  - option: %d", goal->option);

  auto state = robot_->read_once();
  if (state.sdata.robot_state == 3) {
    RCLCPP_WARN(get_logger(), "Cannot start move_pb (robot_state == 3, robot is moving.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!robot_->try_aquire_move_lock()) {
    RCLCPP_ERROR(get_logger(), "Cannot start move_pb (move_lock is locked.)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotNode::move_pb_handle_cancel(const std::shared_ptr<GoalHandleMovePb> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(get_logger(), "Receive equest to cancel move_pb");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotNode::move_pb_handle_accepted(const std::shared_ptr<GoalHandleMovePb> goal_handle) {
  thread{std::bind(&RobotNode::move_pb_execute, this, _1), goal_handle}.detach();
}

void RobotNode::move_pb_execute(const std::shared_ptr<GoalHandleMovePb> goal_handle) {
  MutexReleaser mr(robot_);

  RCLCPP_INFO(get_logger(), "Executing move_pb");

  rclcpp::Rate loop_rate(kMoveFeedbackRate);
  const auto& goal = goal_handle->get_goal().get();
  auto result = std::make_shared<MovePb::Result>();
  auto feedback = std::make_shared<MovePb::Feedback>();

  robot_->move_pb_clear();
  for (const auto& p : goal->points) {
    std::array<double, Robot::k6DoFDim> point;
    std::copy(p.point.begin(), p.point.end(), point.begin());

    if (!robot_->move_pb_add(point, p.speed, (rb::podo::BlendingOption)p.blending_option, p.blending_value)) {
      result->success = false;
      goal_handle->abort(result);
      RCLCPP_WARN(get_logger(), "move_pb failed");
      return;
    }
  }
  if (!robot_->move_pb_run(goal->acceleration, (rb::podo::MovePBOption)goal->option, goal->time_for_waiting_start)) {
    result->success = false;
    goal_handle->abort(result);
    RCLCPP_WARN(get_logger(),
                "move_pb failed to start. This could be due to the target being too close or due to another error");
    return;
  }
  robot_->wait_for_update();
  while (rclcpp::ok()) {
    const auto& state = robot_->read_once();
    if (state.sdata.robot_state == 1) {
      break;
    }
    if (goal_handle->is_canceling()) {
      robot_->move_stop();
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Move_pb canceled");
      return;
    }
    feedback->state = convert_to_ros_msg(state);
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Move_pb finished");
  }
}

RobotExecutor::RobotExecutor() {
  thread_ = thread([this]() { spin(); });
}

RobotExecutor::~RobotExecutor() {
  if (spinning) {
    cancel();
  }
  if (thread_.joinable()) {
    thread_.join();
  }
}

}  // namespace rbpodo_hardware