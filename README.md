# rbpodo_ros2

> :warning: **IMPORTANT WARNING**: This software is under active development. DO NOT USE in production to avoid potential instability.


## Installation

### Prerequisites

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Install [rbpodo](https://github.com/RainbowRobotics/rbpodo)
  ```bash
  sudo apt install -y build-essential cmake git
  git clone https://github.com/RainbowRobotics/rbpodo.git
  mkdir -p rbpodo/build
  cd rbpodo/build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make
  sudo make install
  ```
- Install ROS 2 package dependencies
  ```bash
  sudo apt install -y \
    ros-humble-ament-cmake \
    ros-humble-joint-state-publisher \
    ros-humble-moveit \
    ros-humble-pluginlib \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-controllers \
    ros-humble-ros2-control \
    ros-humble-rviz2 \
    ros-humble-urdf-launch \
    ros-humble-xacro 
  ```
- Set up environment
  ```bash
  source /opt/ros/humble/setup.bash
  ```

### Build From Source

1. Create a ROS 2 workspace
   ```bash
   mkdir -p ~/rbpodo_ros2_ws/src
   ```
2. Clone repo and build ``rbpodo_ros2`` packages:
   ```bash
   cd ~/rbpodo_ros2_ws
   git clone https://github.com/RainbowRobotics/rbpodo_ros2.git src/rbpodo_ros2
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.sh
   ```

## How to Use

```bash
source ~/rbpodo_ros2_ws/install/setup.bash
ros2 launch rbpodo_bringup rbpodo.launch.py model_id:=rb3_730es_u use_fake_hardware:=false cb_simulation:=false robot_ip:="10.0.2.7" use_rviz:=true
```

```bash
source ~/rbpodo_ros2_ws/install/setup.bash
ros2 launch rbpodo_moveit_config moveit.launch.py model_id:="rb5_850e" use_fake_hardware:=false cb_simulation:=false robot_ip:="10.0.2.7" 
```
