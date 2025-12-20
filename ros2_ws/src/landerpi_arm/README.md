# LanderPi Arm - Stage A: Interactive Grasping

Robotic arm manipulation package for LanderPi platform, implementing MoveIt2-based grasping with configurable parameters.

## Features

- **Action-based interface**: `/arm/pick_pose` action for pose-driven grasping
- **MoveIt2 integration**: Uses `moveit_py` for planning and IK
- **Configurable strategy**: All grasp parameters in `grasp.yaml`
- **Modular design**: Loosely coupled components (TF, Gripper, MoveIt2)
- **Command-line tool**: Easy testing with `send_pick_goal`

## Package Structure

```
landerpi_arm/
├── landerpi_arm/
│   ├── grasp_action_server.py   # Main action server (orchestration)
│   ├── moveit2_client.py        # MoveIt2 planning/execution wrapper
│   ├── gripper_client.py        # Gripper control via JointTrajectory
│   ├── tf_utils.py              # TF2 transformation utilities
│   └── send_pick_goal.py        # Command-line testing tool
├── config/
│   └── grasp.yaml               # Grasp strategy parameters
├── launch/
│   └── arm_bringup.launch.py   # Launch action server
└── README.md                    # This file
```

## Dependencies

- ROS2 Humble
- MoveIt2 (`ros-humble-moveit`)
- `moveit_py` (Python bindings for MoveIt2)
- `tf2_ros`, `tf2_geometry_msgs`
- `landerpi_interfaces` (custom action definitions)
- `hiwonder_moveit_config` (MoveIt configuration)
- `robot_gazebo` (simulation environment)

## Build Instructions

```bash
cd ~/Desktop/code/ros2_ws

# Build interfaces first
colcon build --packages-select landerpi_interfaces

# Source to make interfaces available
source install/setup.bash

# Build arm package
colcon build --packages-select landerpi_arm

# Source again
source install/setup.bash
```

## Usage

### 1. Launch Simulation Environment

```bash
# Terminal 1: Launch Gazebo with robot and world
ros2 launch robot_gazebo room_worlds.launch.py use_sim_time:=true
```

### 2. Launch MoveIt2

```bash
# Terminal 2: Launch MoveIt2 move_group
ros2 launch hiwonder_moveit_config demo.launch.py
```

### 3. Launch Grasp Action Server

```bash
# Terminal 3: Launch grasp action server
ros2 launch landerpi_arm arm_bringup.launch.py
```

### 4. Send Grasp Commands

```bash
# Terminal 4: Test grasping red block at (0.20, 0.10, 0.05)
ros2 run landerpi_arm send_pick_goal \
  --x 0.20 --y 0.10 --z 0.05 \
  --frame base_link \
  --object-id red_block

# Test grasping green block at (0.20, -0.10, 0.05)
ros2 run landerpi_arm send_pick_goal \
  --x 0.20 --y -0.10 --z 0.05 \
  --frame base_link \
  --object-id green_block
```

## Grasp Workflow

The action server executes the following sequence:

1. **Transform pose**: Convert target pose to planning frame (`base_link`)
2. **Open gripper**: Open to configured position (default: 0.0 rad)
3. **Approach**: Move to pregrasp pose (above target by `approach_offset`)
4. **Descend**: Move down to grasp pose (`descend_offset` from pregrasp)
5. **Close gripper**: Close to configured position (default: -1.638 rad)
6. **Lift**: Raise object by `lift_offset`

## Configuration

Edit `config/grasp.yaml` to adjust:

### Planning Parameters
```yaml
planning:
  velocity_scaling: 0.25        # Speed (0-1)
  acceleration_scaling: 0.20    # Acceleration (0-1)
  planning_timeout: 3.0         # Planning timeout (sec)
```

### Grasp Strategy
```yaml
grasp_offsets:
  approach: [0.0, 0.0, 0.12]    # Approach height (m)
  descend: [0.0, 0.0, -0.10]    # Descend distance (m)
  lift: [0.0, 0.0, 0.15]        # Lift height (m)

grasp_orientation:
  roll: 0.0                      # Roll angle (rad)
  pitch: 1.57                    # Pitch angle (rad) - vertical
  yaw: 0.0                       # Yaw angle (rad)
```

### Gripper Control
```yaml
gripper:
  open_position: 0.0             # Open angle (rad)
  close_position: -1.638         # Close angle (rad)
```

## Testing with RoboCup Home World

The `robocup_home.sdf` world includes two demo blocks:

- **Red block**: position `(0.20, 0.10, 0.05)`
- **Green block**: position `(0.20, -0.10, 0.05)`

These are pre-placed near the robot for easy testing.

## Error Codes

| Code | Meaning |
|------|---------|
| 0 | Success |
| 1 | TF transform failed |
| 2 | IK solution not found |
| 3 | Motion planning failed |
| 4 | Trajectory execution failed |
| 5 | Gripper control failed |

## Troubleshooting

### Action server not available
- Ensure MoveIt2 is running: `ros2 topic list | grep move_group`
- Check gripper controller: `ros2 control list_controllers`

### Planning failures
- Increase `planning_timeout` in `grasp.yaml`
- Check if target is within robot workspace
- Verify no collisions in RViz planning scene

### Gripper not responding
- Check controller state: `ros2 control list_controllers`
- Verify joint limits in `hiwonder_moveit_config/config/joint_limits.yaml`

## Next Stages

- **Stage B**: Vision integration (`landerpi_vision` publishes ball poses)
- **Stage C**: Navigation integration (Nav2 approach + grasp)
- **Stage D**: Full mission (exploration + detection + grasp + return)

## License

Apache-2.0

