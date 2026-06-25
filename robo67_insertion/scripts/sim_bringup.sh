#!/usr/bin/env bash
# Launch the multipanda MuJoCo sim (single Panda) with the right library paths.
# Run INSIDE multipanda-container as user 'developer'. Terminal 1.
#   docker exec -it --user developer multipanda-container bash
#   /host/Code/Robo67/robo67_insertion/scripts/sim_bringup.sh
set -euo pipefail
source /opt/ros/humble/setup.bash
source ~/multipanda_ws/install/setup.bash
export DISPLAY="${DISPLAY:-:0}"
export LD_LIBRARY_PATH="/home/developer/Libraries/mujoco/lib:/home/developer/Libraries/libfranka/lib:${LD_LIBRARY_PATH:-}"
exec ros2 launch franka_bringup franka_sim.launch.py "$@"
