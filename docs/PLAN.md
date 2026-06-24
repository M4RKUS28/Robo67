# Challenge 1 — Peg-in-Hole: Implementation Plan

**Approach:** classical vision (AprilTag) + force-guided insertion via Cartesian impedance.
**Stack:** `multipanda_ros2` on ROS 2 Humble. Prototype in MuJoCo sim, then real arm.
**CLAUDE.md constraints:** no joint-position controller, Eigen 3.3.9, error_recovery after ControlException.

---

## Architecture Overview

```
Camera (video0 or video2)
    │
    ▼
[socket_detector node]  ─── socket pose in camera frame
    │
    ▼
[extrinsics: cam→base] ─── socket pose in robot base frame
    │
    ▼
[insertion_controller node]
    │  state machine: IDLE → APPROACH → ALIGN → INSERT → SPIRAL → SUCCESS
    │
    ▼
/cartesian_impedance/pose_desired  (Float64MultiArray: [px,py,pz, R00..R22])
```

The insertion controller is a standalone ROS 2 node — not a `ros2_control` plugin — so it runs in user-space Python/C++ and sends pose commands at ~10–50 Hz to the existing Cartesian impedance controller.

---

## Phase 1 — Sim Smoke Test (1–2 hours)

**Goal:** publish a pose command and see the simulated arm move.

1. Launch the sim:
   ```bash
   ros2 launch franka_bringup franka_sim.launch.py
   ```
2. Check which Cartesian impedance controller is loaded and what topic it listens on:
   ```bash
   ros2 control list_controllers
   ros2 topic list | grep -i cart
   ros2 topic info /cartesian_impedance/pose_desired
   ```
   > The topic name may differ from CLAUDE.md — verify it. The message type is `std_msgs/Float64MultiArray` with 12 floats: `[px, py, pz, R00, R01, R02, R10, R11, R12, R20, R21, R22]`.
3. Write `src/smoke_test.py` — publishes a single fixed target pose (10 cm above current EE) at 10 Hz, confirm arm tracks it in RViz.
4. Confirm `/franka_robot_state_broadcaster/...` is publishing `O_T_EE` (4×4 EE pose) and `O_F_ext_hat` (6D external wrench).

**Deliverable:** arm moves to a commanded pose in sim. We know the exact topic names.

---

## Phase 2 — Vision: Socket Detection (2–3 hours)

**Goal:** detect the 3D pose of the hole/socket from overhead camera images.

### Option A — AprilTag (recommended)
Stick an AprilTag on or next to the socket. Use the `apriltag_ros` package:
```bash
# inside the container or on the machine:
sudo apt install ros-humble-apriltag-ros
```
- Node `apriltag_node` subscribes to `/camera/image_raw` and `/camera/camera_info`.
- Outputs `geometry_msgs/PoseStamped` in camera frame.
- Tag size and family must match the printed tag.

### Option B — Known geometry (faster fallback)
If no AprilTag available: detect the round/square socket rim with Hough circles or contour detection in OpenCV. Requires knowing the socket diameter and computing depth from apparent size + camera intrinsics.

### Camera bring-up
The cameras are **not** run as ROS nodes yet. We need a v4l2/GStreamer → ROS bridge:
```bash
# Option 1: v4l2_camera package
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video2 -p image_size:=[1280,720]

# Option 2: usb_cam
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video2
```
Fix C920 exposure first (from `docs/cameras.md`):
```bash
v4l2-ctl -d /dev/video2 --set-ctrl=auto_exposure=1,exposure_time_absolute=150
```

### Camera intrinsics
Run `ros2 run camera_calibration cameracalibrator` with a checkerboard to get `camera_info`. Save the YAML. Without this, the AprilTag 3D pose will be wrong.

**Deliverable:** node that publishes `socket_pose` (PoseStamped in camera frame) at ~30 Hz.

---

## Phase 3 — Camera–Robot Extrinsic Calibration (1–2 hours)

**Goal:** transform socket pose from camera frame to robot base frame.

### Procedure
1. Move the arm to N ≥ 5 known joint configs. For each config:
   - Record EE pose from `/franka_robot_state_broadcaster` (`O_T_EE`).
   - Record a visible calibration target (AprilTag on the peg tip, or end-effector tip in camera image).
2. Solve for `T_cam_base` using `hand_eye` calibration (`cv2.calibrateHandEye` or the `easy_handeye` ROS package).
3. Save as a static TF publisher:
   ```yaml
   # config/cam_to_base.yaml
   parent_frame: panda_link0
   child_frame: camera
   translation: [x, y, z]
   rotation: [qx, qy, qz, qw]
   ```
   ```bash
   ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw panda_link0 camera
   ```

**Alternative (faster, approximate):** measure camera position with a ruler, set manually, refine on hardware.

**Deliverable:** `T_cam_base` transform. Socket detections are now in robot base frame.

---

## Phase 4 — Insertion State Machine (3–4 hours)

**File:** `src/insertion_controller.py`

### State Machine

```
IDLE
  └─ (socket detected) ─→ APPROACH
        └─ (above socket, error < 5 mm) ─→ ALIGN
              └─ (error < 2 mm XY) ─→ INSERT
                    └─ (Fz > 10 N, no depth) ─→ SPIRAL
                          └─ (Fz drops or depth reached) ─→ INSERT
                    └─ (depth reached: dz > peg_length - 2 mm) ─→ SUCCESS
```

### Key Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| `approach_height` | 0.10 m above socket | Safe clearance |
| `align_threshold_xy` | 0.002 m | 2 mm — insertion clearance |
| `insert_speed` | 0.002 m/s | Slow descent |
| `force_threshold` | 10 N | Fz before triggering spiral |
| `spiral_radius` | 0.003 m | 3 mm max spiral excursion |
| `spiral_period` | 3.0 s | One full circle |
| `success_depth` | peg_length − 0.002 m | |
| `stiffness_insert` | 200 N/m | Soften Z for compliance |

### Cartesian Impedance Stiffness
Before insertion, soften the Z-axis stiffness to allow compliance:
```python
# Call the set_cartesian_impedance service
# Values: [tx, ty, tz, rx, ry, rz] stiffness
# tx=ty=500, tz=200 (softer Z), rx=ry=rz=30
```
Verify exact service name on the machine: `ros2 service list | grep impedance`.

### Force Monitoring
Subscribe to `/franka_robot_state_broadcaster/robot_state` and read `O_F_ext_hat_K` (6D wrench: [Fx, Fy, Fz, Tx, Ty, Tz]). Use Fz to detect contact and monitor insertion force.

### Spiral Search
When stuck (Fz > threshold but depth not reached):
```python
t_spiral = (now - spiral_start).to_sec()
dx = spiral_radius * sin(2*pi * t_spiral / spiral_period)
dy = spiral_radius * cos(2*pi * t_spiral / spiral_period)
target.x = socket_x + dx
target.y = socket_y + dy
target.z = current_z  # hold depth, let force guide
```

**Deliverable:** full insertion pipeline runs in MuJoCo sim from detection → success.

---

## Phase 5 — Real Hardware Bring-Up (1–2 hours)

1. **Verify FCI is active** in Franka Desk. Only one commander at a time.
2. **RT kernel check:** `uname -r | grep rt` — must be on a black workstation.
3. **Communication test:**
   ```bash
   ~/Libraries/libfranka/bin/communication_test <fci-ip>
   ```
4. **Launch real arm:**
   ```bash
   ros2 launch franka_bringup franka.launch.py robot_ip:=<fci-ip>
   ```
5. **Tune collision thresholds** before insertion — relax them so reflexes don't fire:
   ```bash
   # Via the set_force_torque_collision_behavior service
   # Lower values = more sensitive, higher = less
   ```
6. **Camera extrinsics** — re-verify once arm is at the real station (lighting, position differ).
7. **First insertion test:** run with a large-tolerance mock socket. Tune spiral radius and force threshold.
8. **After any ControlException:**
   ```bash
   ros2 service call ~/service_server/error_recovery std_srvs/srv/Trigger
   ```

---

## File Layout

```
src/
  smoke_test.py               # Phase 1: verify controller topic
  socket_detector.py          # Phase 2: AprilTag → socket PoseStamped
  calibrate_extrinsics.py     # Phase 3: record poses and solve T_cam_base
  insertion_controller.py     # Phase 4: state machine + pose publisher
config/
  cam_to_base.yaml            # saved extrinsic transform
  apriltag_config.yaml        # tag family, size
  insertion_params.yaml       # all tunable params from Phase 4 table
launch/
  insertion.launch.py         # brings up camera, apriltag, extrinsics TF, insertion controller
```

---

## Key Risks & Mitigations

| Risk | Mitigation |
|------|-----------|
| Camera extrinsics off by >5 mm | Calibrate carefully; add visual feedback before first real insertion |
| ControlException mid-insertion | Loose thresholds before task; `error_recovery` service ready |
| Socket not visible (occlusion) | Position camera to see socket, not peg; 2 cams available |
| Peg hits rim instead of hole | Chamfer the hole print; increase spiral radius; soften Z stiffness |
| Topic names differ on real machine | Always `ros2 topic list` first — do not hardcode names |

---

## What NOT To Do

- Do not use the joint-position controller (bad motor behavior).
- Do not use Eigen 3.4.0 (breaks compilation).
- Do not move Cartesian impedance controller to real arm without testing sim first.
- Do not activate FCI while Desk is still in control (only one commander).
