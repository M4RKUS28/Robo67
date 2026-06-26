# calibration/

Two calibrations live here:

1. **Overhead C920 → robot-base homography** (peg-in-hole; `config/c920_homography.npz`).
   Produced 2026-06-25 (socket-proxy method, ~5 mm RMS).
2. **D405 (gripper cam) eye-in-hand / hand-to-eye** (cable insertion;
   `config/d405_handeye.npz`). Recovers the fixed camera↔EE transform via a
   ChArUco board so the wrist camera can map a port pixel + depth → absolute
   base XYZ. See "Hand-eye (D405 gripper cam)" below.

## Files

| File | What |
|------|------|
| `start_calibration.sh` | One command to run the guided C920→base calibration (host → docker-execs into `multipanda-container`, or run inside it). |
| `calibrate_guided.py` | The guided C920 tool. Detects the white-cube centroid (`detect_white_cubes`) + reads the EE pose; you move the arm with Franka native guiding. Fits + saves the homography. |
| `start_handeye_calibration.sh` | One command to run the guided D405 eye-in-hand ChArUco calibration. |
| `calibrate_handeye.py` | The guided hand-eye tool. Grabs D405 color via `pyrealsense2` (factory intrinsics), detects a ChArUco board, reads the EE pose, solves `cv2.calibrateHandEye`, saves `config/d405_handeye.npz`. Pure math in `robo67_insertion/lib/handeye.py`. |
| `MANUAL_CALIBRATION.md` | Step-by-step manual fallback (raw commands) if the guided C920 tool fails — includes the bringup-relaunch recovery and all the gotchas. |
| `c920_homography.npz` | The C920 calibration captured this session (copy of `config/c920_homography.npz`). |
| `c920_corr.csv` | The 5 correspondences (`u,v,base_x,base_y`) it was fit from. |
| `captures/` | Annotated detection frames (`pt*_annot.jpg`, `point_NN.jpg`); `captures/handeye/` holds per-view ChArUco frames from a hand-eye run. |

## Run it

```bash
./calibration/start_calibration.sh
```

Per point (do ≥4, spread out): place the **socket** in view with the **arm
clear** → Enter (it detects the cube); guide the **peg into the bore** with the
grip handles → Enter (it reads the pose). Type `q` to fit + save. It writes
`robo67_insertion/config/c920_homography.npz` and prints the RMS.

Test detection offline (no robot):
```bash
python3 calibration/calibrate_guided.py --test-detect calibration/captures/pt5b_annot.jpg
```

## How it works (and why)

- **Feature = white-cube centroid**, not the bore: the white-on-white bore washes
  out when the C920 is slightly overexposed; the bright square doesn't. The bore
  is ~centred on the cube, so the centroid is a stable proxy. The insertion uses
  the same detector (`hw_peg_in_hole_vision.py --detector cube`, the default).
- **Arm moved by Franka native guiding** (grip handles): the real controller is
  too stiff to soft-float and its gravity comp is off, so software guiding
  doesn't work. Guiding can crash the bringup — that's expected; relaunch it
  (see `MANUAL_CALIBRATION.md`). `FrankaState` still reports the pose meanwhile.
- A homography is **plane-specific**: every point is taken at socket-top height
  (the peg is seated in the bore each time). Z is force-probed by the insertion.

See `MANUAL_CALIBRATION.md` for the full rationale, the exact captured data, and
recovery commands.

## Hand-eye (D405 gripper cam)

Recovers `T_ee_cam` — the FIXED pose of the wrist D405 in the end-effector
frame — so a port detected by the wrist camera maps to an absolute base-frame
XYZ: `X_base = T_base_ee · T_ee_cam · X_cam`. Saved to
`robo67_insertion/config/d405_handeye.npz`.

```bash
./calibration/start_handeye_calibration.sh --square-length 0.025
```

Per view (do ≥8, **vary tilt/yaw a lot** between views — rotation variety is
what makes hand-eye observable): guide the arm so the D405 sees the **whole
ChArUco board** → Enter (it grabs a D405 frame via `pyrealsense2`, detects the
board, estimates its pose, and reads the EE pose). Type `q` to solve + save. It
prints `T_ee_cam`, the camera offset from the EE, and a consistency residual
(spread of the inferred board-in-base pose across views — low = good).

Offline checks (no robot/camera):
```bash
python3 calibration/calibrate_handeye.py --selftest                  # synthetic math
python3 calibration/calibrate_handeye.py --test-detect board.jpg     # detection only
```

How it works (and why):

- **ChArUco board** (chessboard + ArUco markers): robust sub-pixel corners even
  with partial occlusion/glare, and the markers fix the board's identity/pose.
  Default board `5×7`, `DICT_5X5_1000`, `square_length=0.025` m (2.5 cm) —
  override with `--squares-x/-y`, `--square-length`, `--marker-length`,
  `--dictionary`.
- **Square size only affects the TRANSLATION** of `T_ee_cam`, not the rotation,
  and the camera offset is only a few cm, so a few % error is sub-mm — within the
  insertion's spiral search. No ruler? On a tablet, use the tablet PPI
  (`square_m = px/ppi × 0.0254`) or a credit card (exactly **85.60 mm** wide) as
  a reference.
- **D405 frames + intrinsics via `pyrealsense2`** (factory `fx,fy,cx,cy,dist`),
  not V4L2 — no separate intrinsics step. Needs the container deps
  (`robo67_insertion/scripts/container_setup.sh`).
- **Arm moved by Franka native guiding** (grip handles), same as the C920 tool:
  the tool only reads the pose, issues no motion. Pure math lives in
  `robo67_insertion/lib/handeye.py` (host-testable, cv2 imported lazily).
