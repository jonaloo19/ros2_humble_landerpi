# ros2_humble_landerpi

This repository provides a streamlined, ready-to-use setup for ROS2 Humble, Gazebo (Fortress) simulation environment, and the LanderPi robot model. It is designed to help users quickly install the required ROS2 packages, build the LanderPi workspace (ros2_ws), and launch the LanderPi robot model in Gazebo with minimal configuration effort.

To keep the process simple, the repo assumes that you already have a clean installation of Ubuntu 22.04 (Jammy), which is the officially supported OS for ROS2 Humble. Once Ubuntu 22.04 is in place, the provided scripts will guide you through setting up ROS2 Humble, installing dependencies, and running the LanderPi robot model in Gazebo.

## Install flow

1) Clone this repo (contains the install scripts and the `ros2_ws` workspace):
```bash
git clone https://github.com/jonaloo19/ros2_humble_landerpi.git
cd ros2_humble_landerpi
```

2) Install ROS 2 Humble (skips if already present):
```bash
chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
```

3) Install dependencies, copy the workspace to `~/ros2_ws`, and build:
```bash
chmod +x install_gazebo_landerpi.sh
./install_gazebo_landerpi.sh
```
This script:
- Installs:
  - Build tools: `python3-colcon-common-extensions`
  - Gazebo/ROS bridge: `ros-humble-ros-gz`, `ros-humble-ros-ign-gazebo`
  - Controllers/nav stack pieces: `ros-humble-controller-manager`, `ros-humble-ros2-control`, `ros-humble-ros2-controllers`, `ros-humble-nav2-costmap-2d`, `ros-humble-dwb-critics`, `ros-humble-dwb-core`, `ros-humble-dwb-plugins`
  - Nav bringup: `ros-humble-navigation2`, `ros-humble-nav2-bringup`
  - SLAM: `ros-humble-cartographer`, `ros-humble-cartographer-ros`, `ros-humble-slam-toolbox`
  - Motion planning: `ros-humble-moveit`
  - Utility: `ros-humble-joint-state-publisher`, `ros-humble-joint-state-publisher-gui`, `tmux`
  - Math libs: `libnlopt-dev`, `libnlopt-cxx-dev`, `libsuitesparse-dev`, `liblapack-dev`, `libblas-dev`
- Copies the bundled `ros2_ws` from this repo into `~/ros2_ws` (replaces any existing `~/ros2_ws`).
- Builds the workspace with `colcon build --symlink-install` (runs twice).
- Updates `~/.bashrc` to source `~/ros2_ws/install/setup.bash` and sets `IGN_GAZEBO_RESOURCE_PATH` for meshes.
- Adds commented CPU-render fallbacks for Gazebo in case GPU drivers are an issue.

After running, open a new terminal (or `source ~/.bashrc`) to load the environment.

## Quick test
Without tmux (copy each line into its own terminal):
```bash
ros2 launch robot_gazebo room_worlds.launch.py      # world + robot
```
```bash
ros2 launch robot_gazebo slam.launch.py             # RVIZ with SLAM
```
```bash
ros2 run robot_gazebo teleop_key_control            # optional teleop
```
With tmux (one-shot helper):
```bash
tmux new-session \; \
  send-keys "ros2 launch robot_gazebo room_worlds.launch.py" C-m \; \
  split-window -h \; \
  send-keys "ros2 launch robot_gazebo slam.launch.py" C-m \; \
  split-window -h \; \
  send-keys "ros2 run robot_gazebo teleop_key_control" C-m
```
