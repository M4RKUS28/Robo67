# Phase 0 — Verified ROS interfaces (against the running sim)

Captured 2026-06-25 by launching `franka_sim.launch.py` in `multipanda-container` and
inspecting the live system. These names are now the source of truth in `config/robo67.yaml`.

## Sim bring-up gotchas (IMPORTANT)

1. **`LD_LIBRARY_PATH` must include MuJoCo + libfranka**, or `mujoco_node` dies with
   `libmujoco.so.3.2.0: cannot open shared object file` and the control plugin dies with
   `libfranka.so.0.9: cannot open shared object file`:
   ```
   export LD_LIBRARY_PATH=/home/developer/Libraries/mujoco/lib:/home/developer/Libraries/libfranka/lib:$LD_LIBRARY_PATH
   ```
2. **`franka_sim.launch.py` only spawns `joint_state_broadcaster`.** The MMC and the
   franka state broadcaster must be spawned manually afterwards:
   ```
   ros2 run controller_manager spawner franka_robot_state_broadcaster -c /controller_manager
   ros2 run controller_manager spawner multi_mode_controller          -c /controller_manager
   ```
   (Use `scripts/sim_bringup.sh` then `scripts/activate_cartesian.sh`.)
3. Activate the Cartesian impedance controller via the **namespaced** service:
   ```
   ros2 service call /multi_mode_controller/set_controllers \
     multi_mode_control_msgs/srv/SetControllers \
     "{controllers: [{name: panda_cartesian_impedance_controller, resources: [panda]}]}"
   ```

## Verified names (sim)

| Purpose | Name | Type |
|---|---|---|
| Activate controllers | `/multi_mode_controller/set_controllers` | `multi_mode_control_msgs/srv/SetControllers` |
| Get controllers | `/multi_mode_controller/get_controllers` | `multi_mode_control_msgs/srv/GetControllers` |
| Command pose | `/panda/panda_cartesian_impedance_controller/desired_pose` | `multi_mode_control_msgs/msg/CartesianImpedanceGoal` |
| Set stiffness | `/panda/panda_cartesian_impedance_controller/parameters` | `multi_mode_control_msgs/srv/SetCartesianImpedance` |
| Robot state | `/franka_robot_state_broadcaster/robot_state` | `franka_msgs/msg/FrankaState` |
| Gripper actions | `/panda_gripper_sim_node/{homing,move,grasp,gripper_action}` | `franka_msgs/action/{Homing,Move,Grasp}`, `control_msgs/action/GripperCommand` |
| Error recovery | `/panda_error_recovery_service_server/error_recovery` | `franka_msgs/srv/ErrorRecovery` (**hardware only** — absent in sim) |

Confirmed live: `desired_pose` has the controller subscribed (sub count 1); `robot_state`
publishes `o_t_ee` (EE pose, col-major 4x4), `o_f_ext_hat_k` (ext wrench), `robot_mode: 1` (IDLE).

## Corrections vs. earlier assumptions

- `set_controllers` is **namespaced** under `/multi_mode_controller/...`, not global `/set_controllers`.
- robot_state topic is `/franka_robot_state_broadcaster/robot_state` (no `/panda/` segment in single-arm sim).
- The subscriber example controller `/cartesian_impedance/pose_desired`
  (`franka_controllers/CartesianImpedanceController`) is also present but is **real-only / unconfigured**;
  we do NOT use it.

## Hardware deltas to confirm in Phase 4

- Gripper namespace on real hardware (sim uses `/panda_gripper_sim_node`).
- Whether the real bringup auto-spawns MMC + state broadcaster, or needs the same manual spawn.
- `error_recovery` service is present on hardware (used after `ControlException`).
- Real launch is `franka.launch.py robot_ip:=<fci-ip>` (or `multimode_franka.launch.py`).
