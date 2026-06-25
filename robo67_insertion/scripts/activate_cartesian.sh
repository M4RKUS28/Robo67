#!/usr/bin/env bash
# After sim_bringup.sh is up: spawn the franka state broadcaster + MMC, then activate
# the Cartesian impedance controller. Run INSIDE multipanda-container. Terminal 2.
set -uo pipefail
source /opt/ros/humble/setup.bash
source ~/multipanda_ws/install/setup.bash

CM=/controller_manager
echo "[activate] spawning franka_robot_state_broadcaster ..."
ros2 run controller_manager spawner franka_robot_state_broadcaster -c "$CM" --controller-manager-timeout 20 || true
echo "[activate] spawning multi_mode_controller ..."
ros2 run controller_manager spawner multi_mode_controller -c "$CM" --controller-manager-timeout 20 || true

echo "[activate] switching to panda_cartesian_impedance_controller ..."
ros2 service call /multi_mode_controller/set_controllers \
  multi_mode_control_msgs/srv/SetControllers \
  "{controllers: [{name: panda_cartesian_impedance_controller, resources: [panda]}]}"

# The MuJoCo sim boots PAUSED -> physics won't step until unpaused.
echo "[activate] unpausing the sim ..."
ros2 service call /set_pause mujoco_ros_msgs/srv/SetPause "{paused: false}" || true

echo "[activate] done. Verify:"
echo "  ros2 topic info /panda/panda_cartesian_impedance_controller/desired_pose"
echo "  ros2 topic echo --once /franka_robot_state_broadcaster/robot_state"
