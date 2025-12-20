#!/usr/bin/env bash
set -e

echo "============================================================"
echo "  ROS 2 Humble + Gazebo Fortress + LanderPi Install Script"
echo "============================================================"

# Paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$SCRIPT_DIR"
SOURCE_WS="$REPO_ROOT/ros2_ws"
DEST_WS="$HOME/ros2_ws"

# ----------------------------------------------------------------------
# 0. Sanity checks
# ----------------------------------------------------------------------
if [ "$(lsb_release -is)" != "Ubuntu" ]; then
  echo "[ERROR] This script is only tested on Ubuntu (22.04 recommended)."
  exit 1
fi

if [ "$EUID" -eq 0 ]; then
  echo "[ERROR] Do NOT run this script with sudo. The script will use sudo when needed."
  exit 1
fi

# ----------------------------------------------------------------------
# 1. Update system
# ----------------------------------------------------------------------
echo "[1/6] Updating apt..."
sudo apt update

# ----------------------------------------------------------------------
# 2. Install Gazebo Fortress bridge + ROS 2 tools + controllers
# ----------------------------------------------------------------------
echo "[2/6] Installing ROS2 + Gazebo Fortress packages..."

sudo apt install -y python3-colcon-common-extensions
sudo apt install -y libnlopt-dev libnlopt-cxx-dev
sudo apt install -y tmux
sudo apt install -y rsync lsb-release

# ROS–Gazebo bridge (ros_gz / ros_ign)
sudo apt install -y ros-humble-ros-gz
sudo apt install -y ros-humble-ros-ign-gazebo
sudo apt install -y ros-humble-ros-ign-bridge
sudo apt install -y ros-humble-gz-ros2-control
sudo apt install -y ros-humble-libg2o

# Controllers & nav stack components
sudo apt install -y ros-humble-controller-manager \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers

sudo apt install -y ros-humble-nav2-costmap-2d

sudo apt install -y ros-humble-dwb-critics \
                    ros-humble-dwb-core \
                    ros-humble-dwb-plugins

sudo apt install -y ros-humble-navigation2 \
                    ros-humble-nav2-bringup \
                    ros-humble-cartographer \
                    ros-humble-cartographer-ros

# Math libs
sudo apt install -y libsuitesparse-dev liblapack-dev libblas-dev

# MoveIt + joint state tools
sudo apt install -y ros-humble-moveit \
                    ros-humble-joint-state-publisher \
                    ros-humble-joint-state-publisher-gui
sudo apt install -y ros-humble-slam-toolbox

# ----------------------------------------------------------------------
# 3. Copy workspace from cloned repo to ~/ros2_ws
# ----------------------------------------------------------------------
echo "[3/6] Preparing ROS2 workspace at $DEST_WS ..."

if [ ! -d "$SOURCE_WS" ]; then
  echo "[ERROR] Workspace not found at $SOURCE_WS. Run this script from the cloned repo root."
  exit 1
fi

if [ -d "$DEST_WS" ]; then
  TS="$(date +%Y%m%d_%H%M%S)"
  BACKUP_WS="${DEST_WS}_backup_${TS}"
  echo "[info] Existing workspace found at $DEST_WS. Renaming to $BACKUP_WS."
  mv "$DEST_WS" "$BACKUP_WS"
fi

echo "[info] Copying workspace to $DEST_WS"
rsync -a "$SOURCE_WS/" "$DEST_WS/"
echo "[info] Workspace copied."

# ----------------------------------------------------------------------
# 4. Build workspace
# ----------------------------------------------------------------------
echo "[4/6] Building workspace with colcon..."
cd "$DEST_WS"
colcon build --symlink-install
echo "[4/6] Running second build pass..."
colcon build --symlink-install

# ----------------------------------------------------------------------
# 5. Configure bashrc
# ----------------------------------------------------------------------
echo "[5/6] Updating ~/.bashrc ..."

BASHRC="$HOME/.bashrc"

if ! grep -q "source ~/ros2_ws/install/setup.bash" $BASHRC; then
  echo "source ~/ros2_ws/install/setup.bash" >> $BASHRC
  echo "[bashrc] Added: source ~/ros2_ws/install/setup.bash"
fi

if [ -f "$HOME/ros2_ws/.typerc" ] && ! grep -q "source ~/ros2_ws/.typerc" $BASHRC; then
  echo "source ~/ros2_ws/.typerc" >> $BASHRC
  echo "[bashrc] Added: source ~/ros2_ws/.typerc"
fi

if ! grep -q "IGN_GAZEBO_RESOURCE_PATH" $BASHRC; then
  echo "export IGN_GAZEBO_RESOURCE_PATH=\$IGN_GAZEBO_RESOURCE_PATH:~/ros2_ws/install/landerpi_description/share" >> $BASHRC
  echo "[bashrc] Added IGN_GAZEBO_RESOURCE_PATH"
fi

echo "[5/6] Verifying bashrc entries..."
grep -n "source ~/ros2_ws/install/setup.bash" $BASHRC || echo "[warn] setup.bash entry not found in bashrc"
grep -n "source ~/ros2_ws/.typerc" $BASHRC || echo "[warn] .typerc entry not found in bashrc"
grep -n "IGN_GAZEBO_RESOURCE_PATH" $BASHRC || echo "[warn] IGN_GAZEBO_RESOURCE_PATH entry not found in bashrc"

# ----------------------------------------------------------------------
# 6. GPU fallback (optional)
# ----------------------------------------------------------------------
echo "[6/6] Adding optional CPU-render overrides for Gazebo (commented out by default)."

if ! grep -q "LIBGL_ALWAYS_SOFTWARE" $BASHRC; then
  cat <<EOF >> $BASHRC

# Force Gazebo to run on CPU (uncomment if OGRE errors appear)
# export LIBGL_ALWAYS_SOFTWARE=1
# export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe

EOF
  echo "[bashrc] Added (commented) CPU render options."
fi

echo ""
echo "============================================================"
echo " Installation complete!"
echo " Run:"
echo "   source ~/.bashrc"
echo "   ros2 launch robot_gazebo room_worlds.launch.py   # world + robot"
echo " If the launch is unsuccessful, open ~/.bashrc and uncomment the environment parameters to force Gazebo CPU rendering."
echo "============================================================"
