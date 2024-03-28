# rbpodo_ros

> :warning: **IMPORTANT WARNING**: This software is under active development. DO NOT USE in production to avoid potential instability.


## Installation

### Prerequisites

- Install [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Install [rbpodo](https://github.com/RainbowRobotics/rbpodo)
  ```bash
  sudo apt install -y build-essential cmake git
  git clone https://github.com/RainbowRobotics/rbpodo.git
  cd rbpodo
  mkdir build
  cd build
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make
  sudo make install
  ```
- Install ROS 2 package dependencies
  ```bash
  sudo apt install -y \
    PACKAGE_1 \
    PACKAGE_2 \
    PACKAGE_3 \
    ...
  ```

### Build From Source

1. Create a ROS 2 workspace
   ```bash
   mkdir -p ~/rbpodo_ros_ws/src
   ```
2. Clone repo and build ``rbpodo_ros`` packages:
   ```bash
   cd ~/rbpodo_ros_ws/src
   git clone ...
   
   ```

## How to Use

```bash
source ~/rbpodo_ros_ws/install/setup.bash
ros2 launch rbpodo_bringup rbpodo.launch.py model_id:=rb3_730es_u use_rviz:=true
```