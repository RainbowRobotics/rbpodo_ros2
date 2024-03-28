cmake_minimum_required(VERSION 3.8)
project(rbpodo_msgs)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveJ.action"
  "action/MoveJb2.action"
  "action/MoveL.action"
  "action/MovePb.action"
  "msg/MoveJb2Point.msg"
  "msg/MovePbPoint.msg"
  "msg/Response.msg"
  "msg/SystemState.msg"
  "srv/Eval.srv"
  "srv/SetCartesianPoseControllerConfig.srv"
  "srv/SetCartesianVelocityControllerConfig.srv"
  "srv/SetJointEffortControllerConfig.srv"
  "srv/SetJointPositionControllerConfig.srv"
  "srv/SetJointVelocityControllerConfig.srv"
  "srv/SetOperationMode.srv"
  "srv/SetSpeedBar.srv"
  "srv/TaskLoad.srv"
  "srv/TaskPause.srv"
  "srv/TaskPlay.srv"
  "srv/TaskResume.srv"
  "srv/TaskStop.srv"
  DEPENDENCIES std_msgs builtin_interfaces
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
