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

#include "rbpodo_hardware/robot.hpp"
#include "rbpodo_msgs/action/move_j.hpp"
#include "rbpodo_msgs/action/move_jb2.hpp"
#include "rbpodo_msgs/action/move_l.hpp"
#include "rbpodo_msgs/action/move_pb.hpp"
#include "rbpodo_msgs/msg/response.hpp"
#include "rbpodo_msgs/msg/system_state.hpp"
#include "rbpodo_msgs/srv/eval.hpp"
#include "rbpodo_msgs/srv/set_cartesian_pose_controller_config.hpp"
#include "rbpodo_msgs/srv/set_cartesian_velocity_controller_config.hpp"
#include "rbpodo_msgs/srv/set_joint_effort_controller_config.hpp"
#include "rbpodo_msgs/srv/set_joint_position_controller_config.hpp"
#include "rbpodo_msgs/srv/set_joint_velocity_controller_config.hpp"
#include "rbpodo_msgs/srv/set_operation_mode.hpp"
#include "rbpodo_msgs/srv/set_speed_bar.hpp"
#include "rbpodo_msgs/srv/task_load.hpp"
#include "rbpodo_msgs/srv/task_pause.hpp"
#include "rbpodo_msgs/srv/task_play.hpp"
#include "rbpodo_msgs/srv/task_resume.hpp"
#include "rbpodo_msgs/srv/task_stop.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace rbpodo_hardware {

class RobotNode : public rclcpp::Node {
 public:
  static constexpr unsigned int kMoveFeedbackRate = 500;

  using MoveJ = rbpodo_msgs::action::MoveJ;
  using GoalHandleMoveJ = rclcpp_action::ServerGoalHandle<MoveJ>;

  using MoveJb2 = rbpodo_msgs::action::MoveJb2;
  using GoalHandleMoveJb2 = rclcpp_action::ServerGoalHandle<MoveJb2>;

  using MoveL = rbpodo_msgs::action::MoveL;
  using GoalHandleMoveL = rclcpp_action::ServerGoalHandle<MoveL>;

  using MovePb = rbpodo_msgs::action::MovePb;
  using GoalHandleMovePb = rclcpp_action::ServerGoalHandle<MovePb>;

  class MutexReleaser {
   public:
    MutexReleaser(std::shared_ptr<Robot> robot) : robot_(robot) {}

    ~MutexReleaser() { robot_->release_move_lock(); }

   private:
    std::shared_ptr<Robot> robot_;
  };

  RobotNode(const rclcpp::NodeOptions& options, std::shared_ptr<Robot> robot);

  ~RobotNode() noexcept;

 private:
  void response_callback();

  rbpodo_msgs::msg::SystemState convert_to_ros_msg(const rb::podo::SystemState& state);

  void eval(const rbpodo_msgs::srv::Eval::Request::SharedPtr& request,
            rbpodo_msgs::srv::Eval::Response::SharedPtr response);

  void task_load(const rbpodo_msgs::srv::TaskLoad::Request::SharedPtr& request,
                 rbpodo_msgs::srv::TaskLoad::Response::SharedPtr response);

  void task_pause(const rbpodo_msgs::srv::TaskPause::Request::SharedPtr& request,
                  rbpodo_msgs::srv::TaskPause::Response::SharedPtr response);

  void task_play(const rbpodo_msgs::srv::TaskPlay::Request::SharedPtr& request,
                 rbpodo_msgs::srv::TaskPlay::Response::SharedPtr response);

  void task_resume(const rbpodo_msgs::srv::TaskResume::Request::SharedPtr& request,
                   rbpodo_msgs::srv::TaskResume::Response::SharedPtr response);

  void task_stop(const rbpodo_msgs::srv::TaskStop::Request::SharedPtr& request,
                 rbpodo_msgs::srv::TaskStop::Response::SharedPtr response);

  void set_operation_mode(const rbpodo_msgs::srv::SetOperationMode::Request::SharedPtr& request,
                          rbpodo_msgs::srv::SetOperationMode::Response::SharedPtr response);

  void set_speed_bar(const rbpodo_msgs::srv::SetSpeedBar::Request::SharedPtr& request,
                     rbpodo_msgs::srv::SetSpeedBar::Response::SharedPtr response);

  void set_joint_position_controller_config(
      const rbpodo_msgs::srv::SetJointPositionControllerConfig::Request::SharedPtr& request,
      rbpodo_msgs::srv::SetJointPositionControllerConfig::Response::SharedPtr response);

  void set_joint_velocity_controller_config(
      const rbpodo_msgs::srv::SetJointVelocityControllerConfig::Request::SharedPtr& request,
      rbpodo_msgs::srv::SetJointVelocityControllerConfig::Response::SharedPtr response);

  void set_joint_effort_controller_config(
      const rbpodo_msgs::srv::SetJointEffortControllerConfig::Request::SharedPtr& request,
      rbpodo_msgs::srv::SetJointEffortControllerConfig::Response::SharedPtr response);

  void set_cartesian_pose_controller_config(
      const rbpodo_msgs::srv::SetCartesianPoseControllerConfig::Request::SharedPtr& request,
      rbpodo_msgs::srv::SetCartesianPoseControllerConfig::Response::SharedPtr response);

  void set_cartesian_velocity_controller_config(
      const rbpodo_msgs::srv::SetCartesianVelocityControllerConfig::Request::SharedPtr& request,
      rbpodo_msgs::srv::SetCartesianVelocityControllerConfig::Response::SharedPtr response);

  rclcpp_action::GoalResponse move_j_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const MoveJ::Goal> goal);

  rclcpp_action::CancelResponse move_j_handle_cancel(const std::shared_ptr<GoalHandleMoveJ> goal_handle);

  void move_j_handle_accepted(const std::shared_ptr<GoalHandleMoveJ> goal_handle);

  void move_j_execute(const std::shared_ptr<GoalHandleMoveJ> goal_handle);

  rclcpp_action::GoalResponse move_l_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const MoveL::Goal> goal);

  rclcpp_action::CancelResponse move_l_handle_cancel(const std::shared_ptr<GoalHandleMoveL> goal_handle);

  void move_l_handle_accepted(const std::shared_ptr<GoalHandleMoveL> goal_handle);

  void move_l_execute(const std::shared_ptr<GoalHandleMoveL> goal_handle);

  rclcpp_action::GoalResponse move_jb2_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const MoveJb2::Goal> goal);

  rclcpp_action::CancelResponse move_jb2_handle_cancel(const std::shared_ptr<GoalHandleMoveJb2> goal_handle);

  void move_jb2_handle_accepted(const std::shared_ptr<GoalHandleMoveJb2> goal_handle);

  void move_jb2_execute(const std::shared_ptr<GoalHandleMoveJb2> goal_handle);

  rclcpp_action::GoalResponse move_pb_handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                 std::shared_ptr<const MovePb::Goal> goal);

  rclcpp_action::CancelResponse move_pb_handle_cancel(const std::shared_ptr<GoalHandleMovePb> goal_handle);

  void move_pb_handle_accepted(const std::shared_ptr<GoalHandleMovePb> goal_handle);

  void move_pb_execute(const std::shared_ptr<GoalHandleMovePb> goal_handle);

 private:
  std::shared_ptr<Robot> robot_;

  bool run_;
  rb::podo::Cobot<rb::podo::StandardVector> res_receiver_;
  rb::podo::ResponseCollector res_storage_;
  std::thread msg_thread_;

  rclcpp::Publisher<rbpodo_msgs::msg::Response>::SharedPtr res_publisher_;

  rclcpp::Service<rbpodo_msgs::srv::SetOperationMode>::SharedPtr set_operation_mode_srv_;
  rclcpp::Service<rbpodo_msgs::srv::SetSpeedBar>::SharedPtr set_speed_bar_srv_;
  rclcpp::Service<rbpodo_msgs::srv::SetJointPositionControllerConfig>::SharedPtr
      set_joint_position_controller_config_srv_;
  rclcpp::Service<rbpodo_msgs::srv::SetJointVelocityControllerConfig>::SharedPtr
      set_joint_velocity_controller_config_srv_;
  rclcpp::Service<rbpodo_msgs::srv::SetJointEffortControllerConfig>::SharedPtr set_joint_effort_controller_config_srv_;
  rclcpp::Service<rbpodo_msgs::srv::SetCartesianPoseControllerConfig>::SharedPtr
      set_cartesian_pose_controller_config_srv_;
  rclcpp::Service<rbpodo_msgs::srv::SetCartesianVelocityControllerConfig>::SharedPtr
      set_cartesian_velocity_controller_config_srv_;
  rclcpp::Service<rbpodo_msgs::srv::Eval>::SharedPtr eval_srv_;
  rclcpp::Service<rbpodo_msgs::srv::TaskLoad>::SharedPtr task_load_srv_;
  rclcpp::Service<rbpodo_msgs::srv::TaskPause>::SharedPtr task_pause_srv_;
  rclcpp::Service<rbpodo_msgs::srv::TaskPlay>::SharedPtr task_play_srv_;
  rclcpp::Service<rbpodo_msgs::srv::TaskResume>::SharedPtr task_resume_srv_;
  rclcpp::Service<rbpodo_msgs::srv::TaskStop>::SharedPtr task_stop_srv_;

  rclcpp_action::Server<MoveJ>::SharedPtr move_j_server_;
  rclcpp_action::Server<MoveL>::SharedPtr move_l_server_;
  rclcpp_action::Server<MoveJb2>::SharedPtr move_jb2_server_;
  rclcpp_action::Server<MovePb>::SharedPtr move_pb_server_;
};

class RobotExecutor : public rclcpp::executors::MultiThreadedExecutor {
 public:
  RobotExecutor();

  ~RobotExecutor() override;

 private:
  std::thread thread_;
};

}  // namespace rbpodo_hardware