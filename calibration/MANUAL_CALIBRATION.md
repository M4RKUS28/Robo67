# Manual C920 → robot-base calibration (fallback)

This documents exactly how the overhead-C920 → robot-base homography
(`robo67_insertion/config/c920_homography.npz`) was produced **by hand**, in
case `calibrate_guided.py` / `start_calibration.sh` don't work. It maps a
detected socket **pixel** to a robot-**base XY** so the insertion can drive the
peg to the socket (Z is force-probed by the insertion, so it's not in the map).

## TL;DR of the method (socket-proxy, cube centroid)

The socket is a white cube with a round bore, sitting on dark carpet. We use the
**socket itself as the ground-truth marker**: at several positions we read where
the **robot** is when the peg is seated in the bore (= that socket's base XY) and
pair it with where the **camera** sees the socket. ≥4 spread pairs → fit a
homography.

Two hard-won facts that shaped this:

1. **Detect the white CUBE (its centroid), not the bore.** The bore is
   white-on-white and washes out the moment the C920 is even slightly
   overexposed; the bright cube square is rock-solid at any exposure. The bore
   sits ~centred on the cube, so the centroid is a stable proxy. Use the SAME
   feature for calibration AND insertion (`detect_white_cubes`).
2. **Move the arm with Franka NATIVE guiding (grip handles), not software.** The
   real subscriber Cartesian controller is fixed-stiff (`pos_stiff=500`,
   not live-settable) and its gravity compensation is off (the arm sags ~6 cm),
   so it can neither be soft-floated nor commanded precisely. Native guiding is
   the only free movement — but **it interrupts FCI and can crash the bringup**
   (`tcpThrowIfConnectionClosed` → `franka_control2_node` abort). That's fine:
   `FrankaState` still publishes the EE pose (even in `robot_mode=5` while the
   e-stop is engaged), and if the bringup does die you just relaunch it.

## Prerequisites

- Inside `multipanda-container`, on `ROS_DOMAIN_ID=1`, **`ROS_LOCALHOST_ONLY=1`**
  (must match the bringup, or `FrankaState` never arrives and
  `controller_manager` service calls hang).
- The franka bringup running and healthy (see "Relaunch" below).
- C920 reachable by its **stable by-id symlink**
  `/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_C26B1F5F-video-index0`
  (bare `/dev/videoN` numbers renumber; it was `/dev/video6` this session).
- Only the **socket** cube in view (the blank cube is identical by shape).

## Environment

```bash
source /opt/ros/humble/setup.bash
source /home/developer/multipanda_ws/install/setup.bash
export ROS_DOMAIN_ID=1 ROS_LOCALHOST_ONLY=1
export LD_LIBRARY_PATH=/home/developer/Libraries/libfranka/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/host/Code/Robo67/robo67_insertion:$PYTHONPATH
```

## Per-point procedure (repeat for ≥4 well-spread positions)

For each point you need ONE pixel and ONE base-XY:

### A) Detect the socket pixel (camera only — no bringup needed)

Place the socket where the camera sees it, **arm parked clear**, then grab a
frame and detect the cube centroid:

```bash
# grab one C920 frame (cv2 V4L2 fallback works inside the container; on a host
# with gstreamer you can use gst-launch with exposure_time_absolute~100 instead)
python3 - <<'PY'
import cv2, time
from robo67_insertion.lib.hole_detect import detect_white_cubes, WhiteCubeParams
cap = cv2.VideoCapture("/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_C26B1F5F-video-index0", cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3)            # aperture-priority = well-exposed here
for _ in range(15): cap.read(); time.sleep(0.03)  # warm up
ok, img = cap.read(); cap.release()
d = detect_white_cubes(img, WhiteCubeParams())
print("cube pixel:", (round(d[0].u), round(d[0].v)) if d else "NONE")
cv2.imwrite("/tmp/calib_frame.jpg", img)
PY
```

Record the printed `(u, v)`. (Open `/tmp/calib_frame.jpg` to sanity-check it's on
the socket. If "NONE": ensure the socket — not the blank cube — is in view, not
near the image edge.)

### B) Read the base XY (bringup must be alive)

Guide the peg tip into **that** socket's bore with the grip handles (don't move
the socket); the hole self-centres the peg. Then read the EE pose:

```bash
python3 - <<'PY'
import rclpy, time
from franka_msgs.msg import FrankaState
from robo67_insertion.lib import geometry
rclpy.init(); n = rclpy.create_node("read_ee"); g = {}
def cb(m):
    g["xyz"], _ = geometry.mat4_colmajor_to_xyz_quat(list(m.o_t_ee)); g["mode"] = m.robot_mode
n.create_subscription(FrankaState, "/franka_robot_state_broadcaster/robot_state", cb, 10)
t0 = time.time()
while "xyz" not in g and time.time()-t0 < 6: rclpy.spin_once(n, timeout_sec=0.1)
print("EE %.4f %.4f %.4f mode=%d" % (*g["xyz"], g["mode"]) if "xyz" in g else "NO STATE")
n.destroy_node(); rclpy.shutdown()
PY
```

Record `base_x = EE.x`, `base_y = EE.y` (the `z` values average to the
socket-top height; `mode=5` is normal while the e-stop/guiding is engaged). If
"NO STATE", the bringup crashed — relaunch it (below) and re-read.

Tips for a good fit: spread the 4–6 points across the workspace (vary BOTH image
axes — left/right and near/far), and a center point gives a useful RMS.

## Fit the homography

Collect the rows `u, v, base_x, base_y` and fit:

```bash
python3 - <<'PY'
import numpy as np
from robo67_insertion.nodes.calibration_node import fit_and_save
from robo67_insertion.lib import geometry
corr = np.array([
    # u,   v,    base_x,  base_y     <-- replace with YOUR points
    [561, 598, 0.5397, -0.1649],
    [548, 417, 0.3912, -0.1782],
    [905, 662, 0.5644,  0.0680],
    [1016,210, 0.2390,  0.1670],
    [717, 432, 0.4075, -0.0613],
], float)
px, base = corr[:, :2], corr[:, 2:]
H, rms = fit_and_save(px, base, "/host/Code/Robo67/robo67_insertion/config/c920_homography.npz")
err = np.linalg.norm(geometry.pixel_to_base(H, px) - base, axis=1) * 1000
print(f"RMS = {rms*1000:.2f} mm   per-point = {[round(e,1) for e in err]}")
PY
```

This writes `config/c920_homography.npz`, which `hw_peg_in_hole_vision.py` and
`socket_detector_node` load. `fit_and_save` needs ≥4 points; with exactly 4 the
fit is exact (RMS≈0) so use ≥5 to get a real error estimate.

## The calibration that was actually captured (2026-06-25)

5 socket-proxy points (cube centroid pixel ↔ EE base XY):

| #  | u    | v   | base_x | base_y  | EE z   |
|----|------|-----|--------|---------|--------|
| 1  | 561  | 598 | 0.5397 | -0.1649 | 0.1234 |
| 2  | 548  | 417 | 0.3912 | -0.1782 | 0.1294 |
| 3  | 905  | 662 | 0.5644 |  0.0680 | 0.1339 |
| 4  | 1016 | 210 | 0.2390 |  0.1670 | 0.1235 |
| 5  | 717  | 432 | 0.4075 | -0.0613 | 0.1239 |

Result: **RMS 5.34 mm** (per-point 5.4 / 5.3 / 2.9 / 2.7 / 8.4 mm), socket-top
EE z ≈ **0.127 m**. ~5 mm is well within the insertion's 12 mm spiral-search +
force-probe, i.e. usable as a coarse locator. `c920_corr.csv` and
`c920_homography.npz` in this folder are copies of that result.

## Verify (no motion)

```bash
PYTHONPATH=/host/Code/Robo67/robo67_insertion \
python3 robo67_insertion/scripts/hw_peg_in_hole_vision.py --socket-top-z 0.14 --dry-run
```
It grabs → detects the cube → maps to base XY → reads state → prints planned
setpoints (publishes nothing). Use `--detector cube` (the default) so it matches
this calibration.

## Relaunch the bringup (after a guide/e-stop crash)

```bash
# kill stale bringup (bracket trick so pkill doesn't match its own shell)
for p in "[r]os2 launch" "[f]ranka.launch" "[r]os2_control" "[c]ontroller_man" \
         "[r]obot_state_pub" "[j]oint_state_pub" "[f]ranka_control2" "[s]pawner"; do
  pkill -f "$p" 2>/dev/null; done
sleep 4
# relaunch (detached so it persists), then wait ~25 s and check robot_state hz
setsid bash -c 'ros2 launch franka_bringup franka.launch.py robot_ip:=192.168.1.67 \
  use_fake_hardware:=false arm_id:=panda' >/tmp/franka_bringup.log 2>&1 </dev/null &
sleep 25
ros2 topic hz /franka_robot_state_broadcaster/robot_state   # expect ~29 Hz
```
Preconditions for the relaunch to connect: **e-stop released**, FCI active on
Desk (`https://192.168.1.67/desk/`), control not held elsewhere (SPoC). If
`franka_control2_node` aborts with `tcpThrowIfConnectionClosed`, the robot isn't
accepting FCI — fix that on Desk, then relaunch.

## Gotchas (all hit during the live run)

- **Wrong cube:** the blank cube and the socket look identical from above; the
  detector finds either. Keep only the socket in view.
- **Overexposure:** at the default the bore washes out; the cube detector doesn't
  care, but if you ever go back to bore detection, lock exposure ~40–120.
- **`NO STATE` / hangs:** almost always `ROS_LOCALHOST_ONLY` mismatch with the
  bringup (use `1`) or the bringup crashed (relaunch).
- **Arm won't move / "stiff":** if it's rigid (not just firm), it's wedged in
  `robot_mode=1` Idle after a reflex — relaunch the bringup (re-activating the
  controller alone wasn't enough).
- **Homography is plane-specific:** all points must be taken at the socket-top
  height (they are, since the peg is seated in the bore each time).
