# Robo67 Logging & Observability — rostopic reference

Everything the insertion stack does is now observable on rostopics: both camera
feeds, a processed feed with the socket-detection overlay, and the full
insertion telemetry (phase, EE pose, speed, contact force, baseline, contact
state, commanded equilibrium, retries). This is the single source of truth for
the live dashboard and for `ros2 bag` replays.

Topic names are centralised in `robo67_insertion/config_schema.py` (`TopicsCfg`)
and mirrored in `config/robo67.yaml` — override there, not in code.

## Architecture (who owns what)

```
        /dev/video<C920>                         /dev/video<D405>
              │ (single owner)                          │ (single owner)
              ▼                                          ▼
   camera_publisher (overhead)               camera_publisher (gripper)
              │ cam_overhead_raw                         │ cam_gripper_raw
              ▼                                          ▼
   socket_detector  ──► socket_pose             d405_servo ──► servo_correction
              │           socket_detection                │
              └─► cam_overhead_overlay                     └─► cam_gripper_overlay

   hardware_insertion_node ──► /robo67/insertion/*  (telemetry, during a run)
   FrankaState (franka driver) ──► EE pose / wrench / robot_mode  (always-on)

   dashboard live_provider  ── subscribes to ALL of the above (never opens a device)
```

Only **one** process may open a V4L2 `/dev/videoN`, so each camera has exactly
one owner (`camera_publisher`); the detector nodes and the dashboard
**subscribe** to the published feed instead of grabbing the device. This removes
the device contention that previously existed between the detector and the
dashboard.

## Camera feeds — `sensor_msgs/CompressedImage` (`format: "jpeg"`)

| Topic (config key) | Default name | Publisher | Rate | Contents |
|---|---|---|---|---|
| `cam_overhead_raw` | `/robo67/camera/overhead/image_raw/compressed` | `camera_publisher` (camera:=overhead) | `camera.publish_fps` (10 Hz) | Raw overhead C920 JPEG |
| `cam_overhead_overlay` | `/robo67/camera/overhead/overlay/compressed` | `socket_detector` | `rate_hz` (5 Hz) | C920 frame + socket ring/crosshair/base-XY label |
| `cam_gripper_raw` | `/robo67/camera/gripper/image_raw/compressed` | `camera_publisher` (camera:=gripper) | `camera.publish_fps` (10 Hz) | Raw eye-in-hand D405 JPEG |
| `cam_gripper_overlay` | `/robo67/camera/gripper/overlay/compressed` | `d405_servo` | `rate_hz` (5 Hz) | D405 frame + hole ring + servo arrow |

- JPEG quality from `camera.jpeg_quality` (default 80).
- The `/...compressed` suffix follows the `image_transport` convention, so
  `rqt_image_view` and `ros2 bag` recognise the feeds.
- Overlay drawing is the pure, host-tested seam
  `robo67_insertion/lib/image_overlay.py` (`draw_socket_overlay`,
  `draw_servo_overlay`).

## Insertion telemetry — published by `hardware_insertion_node`

Published every tick (throttled to `--telemetry-rate`, default 20 Hz) while a
run is active; enabled by default (`--no-publish-telemetry` to disable). These
are **observational only** — they never command the arm, so they also publish in
`--dry-run` (handy for validating a run before motion).

| Topic (config key) | Default name | Type | Contents |
|---|---|---|---|
| `insertion_phase` | `/robo67/insertion/phase` | `std_msgs/String` | FSM phase (`MOVE_ABOVE`…`DONE`/`ERROR`) |
| `insertion_ee_pose` | `/robo67/insertion/ee_pose` | `geometry_msgs/PoseStamped` | Measured EE pose (base frame, held tool quaternion) |
| `insertion_ee_speed` | `/robo67/insertion/ee_speed` | `std_msgs/Float64` | EE linear speed (m/s), finite-difference |
| `insertion_command_pose` | `/robo67/insertion/command_pose` | `geometry_msgs/PoseStamped` | Commanded equilibrium pose after safety clamps |
| `insertion_wrench` | `/robo67/insertion/wrench` | `geometry_msgs/WrenchStamped` | External wrench `o_f_ext_hat_k` (force + torque) |
| `insertion_fz` | `/robo67/insertion/fz` | `std_msgs/Float64` | External force Z (N) |
| `insertion_fz_baseline` | `/robo67/insertion/fz_baseline` | `std_msgs/Float64` | Free-space Fz baseline (N), from the contact lifecycle |
| `insertion_contact` | `/robo67/insertion/contact` | `std_msgs/Bool` | Contact detected (`|fz - baseline| ≥ contact_fz`) |
| `insertion_retries` | `/robo67/insertion/retries` | `std_msgs/Int32` | Spiral/confirm retry count |
| `insertion_diagnostics` | `/robo67/insertion/diagnostics` | `diagnostic_msgs/DiagnosticArray` | One-stop key/value rollup (all of the above + socket/contact_z/abort/done/error) |

Telemetry aggregation (speed tracker + the diagnostic key/value rollup) is the
pure, host-tested seam `robo67_insertion/lib/telemetry.py`.

## Pre-existing topics (unchanged, still logged)

| Topic | Type | Publisher | Contents |
|---|---|---|---|
| `/robo67/socket_pose` | `geometry_msgs/PoseStamped` | `socket_detector` | Socket top centre in base frame (Z = configured) |
| `/robo67/socket_detection` | `std_msgs/Float64MultiArray` | `socket_detector` | `[u, v, radius_px, score]` (C920 pixels) |
| `/robo67/servo_correction` | `std_msgs/Float64MultiArray` | `d405_servo` | `[dx, dy]` base-frame servo vector (m) |
| `/franka_robot_state_broadcaster/robot_state` | `franka_msgs/FrankaState` | franka driver | EE pose `o_t_ee`, wrench, `robot_mode` (always-on) |

## Bring-up

Always-on logging graph (cameras + detectors):

```bash
ros2 launch robo67_insertion logging.launch.py socket_top_z:=<measured>
ros2 launch robo67_insertion logging.launch.py gripper:=true     # add the D405 feeds
```

Insertion telemetry (operator-launched; prompts before motion):

```bash
# dry-run first — publishes telemetry without commanding the arm
ros2 run robo67_insertion hardware_insertion --socket-from-current --socket-top-dz -0.15 --dry-run
```

Inspect:

```bash
ros2 topic list | grep robo67
ros2 topic echo /robo67/insertion/diagnostics
ros2 run rqt_image_view rqt_image_view /robo67/camera/overhead/overlay/compressed
ros2 bag record -o run1 /robo67/insertion/diagnostics /robo67/camera/overhead/overlay/compressed ...
```

Live dashboard (passive observer; subscribes to everything above):

```bash
# inside multipanda-container, ROS sourced, domain 1
python3 -u dashboard/server/serve.py --mode live --host 0.0.0.0 --port 8088
```
