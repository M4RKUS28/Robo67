# Franka Bringup API (franka_ros2 Humble)

Source: https://frankarobotics.github.io/docs/doc/franka_ros2_humble/franka_bringup/doc/index.html

## Basic Robot Startup

```bash
ros2 launch franka_bringup franka.launch.py robot_type:=fr3 robot_ip:=<fci-ip>
```
Starts hardware with only `joint_state_broadcaster` active.

## Controller Management

```bash
# Load and activate a controller
ros2 control load_controller --set-state active gravity_compensation_example_controller

# Deactivate a controller
ros2 control set_controller_state gravity_compensation_example_controller inactive
```

Once a controller using `effort_command_interface` is started, the robot uses the torque interface from libfranka.

## Non-Real-Time Parameter Services

Only available when robot hardware is in **idle mode**.

```bash
ros2 service call /service_server/set_joint_stiffness \
  franka_msgs/srv/SetJointStiffness \
  "{joint_stiffness: [1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0]}"
```

Available services:
- `/service_server/set_cartesian_stiffness`
- `/service_server/set_joint_stiffness`
- `/service_server/set_tcp_frame`
- `/service_server/set_load`
- `/service_server/set_force_torque_collision_behavior`

## Error Recovery

```bash
ros2 action send_goal /action_server/error_recovery franka_msgs/action/ErrorRecovery {}
```

## Frame Notes

- `panda_EE` frame: configurable end-effector frame, adjustable at runtime
- `K` frame: internal Cartesian impedance center
- Neither appears in the URDF (runtime-configurable)

## Multi-Robot / Namespace

```bash
ros2 launch franka_bringup example.launch.py controller_names:=move_to_start_example_controller
```

Config files:
- `franka.config.yaml` — URDF path, namespace, robot details
- `controllers.yaml` — ros2_control framework settings
