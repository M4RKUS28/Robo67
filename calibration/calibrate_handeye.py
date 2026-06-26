#!/usr/bin/env python3
"""calibrate_handeye.py -- guided D405 (gripper cam) eye-in-hand calibration.

Recovers the FIXED transform between the wrist-mounted D405 camera and the
end-effector (EE) frame, ``T_ee_cam`` (OpenCV's ``cam2gripper``), by showing the
camera a **ChArUco board** from several robot poses. You move the arm with
**Franka native guiding** (grip handles); this script only DETECTS the board and
READS the EE pose -- it issues no robot motion (mirrors ``calibrate_guided.py``).

Per view (do >= 8, spread across orientations AND positions):
  1. Guide the arm so the D405 sees the WHOLE board (vary tilt/yaw a lot between
     views -- rotation variety is what makes hand-eye observable), then Enter.
     The script grabs a D405 color frame (via ``pyrealsense2`` -- factory
     intrinsics), detects the ChArUco corners, and estimates the board pose
     (``target2cam``). It also reads the EE pose ``T_base_ee`` from FrankaState.
  2. Type ``q`` to finish -> it solves ``cv2.calibrateHandEye`` and saves
     ``config/d405_handeye.npz`` + prints the consistency residual.

THE BOARD (read this):
  * Square size matters for the TRANSLATION of the result (not the rotation).
    Pass the real on-screen size via ``--square-length`` (meters). Default is
    0.025 (2.5 cm). A few % off is fine -- the camera offset is only a few cm.
  * Displaying on a tablet: mind GLARE (the D405 view is heavily backlit) --
    dim the room / matte angle, and keep the whole board flat and in view.

RUN (inside multipanda-container; normally via ./start_handeye_calibration.sh):
    PYTHONPATH=/host/Code/Robo67/robo67_insertion \
    python3 calibration/calibrate_handeye.py --square-length 0.025

Offline detection self-check on a saved still (no robot/ROS/RealSense needed):
    python3 calibration/calibrate_handeye.py --test-detect some_board.jpg

Offline math self-test (synthetic poses; no robot/ROS/RealSense/board needed):
    python3 calibration/calibrate_handeye.py --selftest
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# Resolve the robo67_insertion package (sibling of this calibration/ folder).
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_PKG = os.path.join(_REPO, "robo67_insertion")
for p in (_PKG, _REPO):
    if p not in sys.path:
        sys.path.insert(0, p)

import numpy as np  # noqa: E402

from robo67_insertion.lib import handeye  # noqa: E402

DEFAULT_OUT = os.path.join(_PKG, "config", "d405_handeye.npz")
DEFAULT_CAPTURE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "captures", "handeye")
ROBOT_STATE = "/franka_robot_state_broadcaster/robot_state"


# --------------------------------------------------------------------------- #
# Board spec from CLI.
# --------------------------------------------------------------------------- #
def board_spec_from_args(args) -> handeye.CharucoBoardSpec:
    marker = args.marker_length if args.marker_length is not None else 0.75 * args.square_length
    return handeye.CharucoBoardSpec(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length=float(args.square_length),
        marker_length=float(marker),
        dictionary=args.dictionary,
    )


# --------------------------------------------------------------------------- #
# Annotation.
# --------------------------------------------------------------------------- #
def annotate(img, charuco_corners, charuco_ids, K=None, dist=None, pose=None, path=None):
    import cv2

    out = img.copy()
    try:
        if charuco_corners is not None and hasattr(cv2.aruco, "drawDetectedCornersCharuco"):
            cv2.aruco.drawDetectedCornersCharuco(out, charuco_corners, charuco_ids, (0, 0, 255))
        elif charuco_corners is not None:
            for c in np.asarray(charuco_corners).reshape(-1, 2):
                cv2.circle(out, (int(c[0]), int(c[1])), 4, (0, 0, 255), -1)
    except Exception:
        pass
    if pose is not None and K is not None:
        rvec, tvec = pose
        d = np.zeros((5, 1)) if dist is None else dist
        try:
            cv2.drawFrameAxes(out, np.asarray(K, float), np.asarray(d, float),
                              rvec, tvec, 0.03)
        except Exception:
            pass
    if path:
        cv2.imwrite(path, out)
    return out


# --------------------------------------------------------------------------- #
# RealSense D405 color capture (factory intrinsics).
# --------------------------------------------------------------------------- #
class RealSenseD405:
    """Open the D405 color stream and expose factory intrinsics + frame grab."""

    _RES = [(1280, 720), (848, 480), (640, 480)]

    def __init__(self, serial="", width=None, height=None, fps=30):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipeline = rs.pipeline()
        res = [(width, height)] if width and height else self._RES
        last_err = None
        for (w, h) in res:
            cfg = rs.config()
            if serial:
                cfg.enable_device(str(serial))
            cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
            try:
                profile = self.pipeline.start(cfg)
            except Exception as e:  # resolution unsupported -> try next
                last_err = e
                continue
            vsp = profile.get_stream(rs.stream.color).as_video_stream_profile()
            intr = vsp.get_intrinsics()
            self.width, self.height = w, h
            self.K = np.array([[intr.fx, 0.0, intr.ppx],
                               [0.0, intr.fy, intr.ppy],
                               [0.0, 0.0, 1.0]], dtype=float)
            self.dist = np.asarray(intr.coeffs, dtype=float).reshape(-1, 1)
            return
        raise RuntimeError(f"could not start D405 color stream: {last_err}")

    def grab(self, warmup=5):
        for _ in range(max(1, warmup)):
            frames = self.pipeline.wait_for_frames()
            color = frames.get_color_frame()
        if not color:
            return None
        return np.asanyarray(color.get_data())

    def close(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Offline detection check (no robot / RealSense).
# --------------------------------------------------------------------------- #
def test_detect(args):
    import cv2

    img = cv2.imread(args.test_detect)
    if img is None:
        print(f"could not read {args.test_detect}", file=sys.stderr)
        return 1
    spec = board_spec_from_args(args)
    board, dictionary = handeye.make_charuco_board(spec)
    cc, ci = handeye.detect_charuco(img, board, dictionary)
    n = 0 if cc is None else len(cc)
    print(f"ChArUco corners detected: {n}  (board {spec.squares_x}x{spec.squares_y}, "
          f"square={spec.square_length} m, marker={spec.marker_length:.4f} m, "
          f"dict={spec.dictionary})")
    pose = None
    if n >= 4 and args.intrinsics:
        K, dist = _load_intrinsics(args.intrinsics)
        pose = handeye.estimate_board_pose(cc, ci, board, K, dist)
        if pose is not None:
            rvec, tvec = pose
            print(f"board pose (target2cam): t = {tvec.reshape(3)} m")
    out = os.path.splitext(args.test_detect)[0] + "_charuco.jpg"
    annotate(img, cc, ci, path=out,
             K=(None if not args.intrinsics else _load_intrinsics(args.intrinsics)[0]),
             pose=pose)
    print("annotated:", out)
    return 0 if n >= 4 else 1


def _load_intrinsics(path):
    d = np.load(path)
    K = np.asarray(d["camera_matrix"] if "camera_matrix" in d else d["K"], float)
    dist = np.asarray(d["dist_coeffs"], float) if "dist_coeffs" in d else np.zeros((5, 1))
    return K, dist


# --------------------------------------------------------------------------- #
# Offline math self-test (synthetic poses; no robot / RealSense / board).
# --------------------------------------------------------------------------- #
def selftest(_args):
    print("=== hand-eye math self-test (synthetic) ===")
    rng = np.random.default_rng(0)

    # True camera-in-EE pose: camera looks roughly down (180 deg about x) with a
    # few-cm offset from the EE.
    T_ee_cam_true = handeye.rvectvec_to_matrix([np.pi, 0.05, -0.03], [0.04, -0.01, 0.05])
    # Board fixed somewhere in the base frame.
    T_base_target = handeye.rvectvec_to_matrix([0.0, 0.0, 0.2], [0.45, -0.05, 0.10])

    gripper2base, target2cam = [], []
    for _ in range(10):
        rvec = rng.uniform(-0.6, 0.6, 3) + np.array([np.pi, 0.0, 0.0])  # tool-down-ish + tilt
        tvec = np.array([0.45, -0.05, 0.35]) + rng.uniform(-0.05, 0.05, 3)
        T_base_ee = handeye.rvectvec_to_matrix(rvec, tvec)
        # target2cam = inv(T_ee_cam) @ inv(T_base_ee) @ T_base_target
        t2c = handeye.invert_transform(T_ee_cam_true) @ handeye.invert_transform(T_base_ee) @ T_base_target
        gripper2base.append(T_base_ee)
        target2cam.append(t2c)

    ok_all = True
    for method in ("tsai", "park", "daniilidis"):
        T = handeye.solve_hand_eye(gripper2base, target2cam, method=method)
        t_err_mm = np.linalg.norm(T[:3, 3] - T_ee_cam_true[:3, 3]) * 1000
        R_err = handeye.invert_transform(T) @ T_ee_cam_true
        import cv2
        ang = float(np.degrees(np.linalg.norm(cv2.Rodrigues(R_err[:3, :3])[0])))
        trms, rrms = handeye.hand_eye_residual(gripper2base, target2cam, T)
        ok = t_err_mm < 0.5 and ang < 0.2 and trms * 1000 < 0.5 and rrms < 0.2
        ok_all = ok_all and ok
        print(f"  {method:11s}: t_err={t_err_mm:.4f} mm  R_err={ang:.4f} deg  "
              f"resid(t={trms*1000:.4f} mm, R={rrms:.4f} deg)  {'PASS' if ok else 'FAIL'}")
    print("RESULT:", "PASS" if ok_all else "FAIL")
    return 0 if ok_all else 1


# --------------------------------------------------------------------------- #
# Live guided capture.
# --------------------------------------------------------------------------- #
def run(args):
    import rclpy
    from franka_msgs.msg import FrankaState

    spec = board_spec_from_args(args)
    board, dictionary = handeye.make_charuco_board(spec)
    os.makedirs(args.capture_dir, exist_ok=True)

    # Open the D405 (factory intrinsics) BEFORE touching ROS, so a camera problem
    # fails fast and clearly.
    try:
        cam = RealSenseD405(serial=args.serial, width=args.width, height=args.height)
    except Exception as e:
        print(f"ERROR: could not open the D405 via pyrealsense2: {e}\n"
              "  (Is the D405 plugged in? Is pyrealsense2 installed in this "
              "interpreter? See robo67_insertion/scripts/container_setup.sh.)",
              file=sys.stderr)
        return 1
    print(f"D405 color {cam.width}x{cam.height}  fx={cam.K[0,0]:.1f} fy={cam.K[1,1]:.1f} "
          f"cx={cam.K[0,2]:.1f} cy={cam.K[1,2]:.1f}  dist={cam.dist.reshape(-1)[:5]}")

    rclpy.init()
    node = rclpy.create_node("calibrate_handeye")
    state = {}

    def on_state(m):
        state["o_t_ee"] = list(m.o_t_ee)
        state["mode"] = int(m.robot_mode)
        state["t"] = time.time()
    node.create_subscription(FrankaState, ROBOT_STATE, on_state, 10)

    def fresh_ee(timeout=6.0):
        state.pop("o_t_ee", None)
        t0 = time.time()
        while "o_t_ee" not in state and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        return state.get("o_t_ee"), state.get("mode")

    print("=" * 64)
    print(" Robo67 guided D405 eye-in-hand (hand-to-eye) calibration -- ChArUco")
    print("=" * 64)
    print(" Move the arm ONLY with Franka native guiding (grip handles).")
    print(" Show the WHOLE board to the D405; VARY tilt/yaw a lot between views.")
    print(f" board={spec.squares_x}x{spec.squares_y} square={spec.square_length} m "
          f"marker={spec.marker_length:.4f} m dict={spec.dictionary}")
    print(f" out={args.out}  method={args.method}")

    if fresh_ee(8.0)[0] is None:
        print("\nERROR: no FrankaState. The bringup is down (a guide/e-stop can crash "
              "it). Relaunch it (see MANUAL_CALIBRATION.md) and re-run.", file=sys.stderr)
        cam.close(); node.destroy_node(); rclpy.shutdown(); return 1

    gripper2base, target2cam = [], []
    i = 0
    try:
        while True:
            print(f"\n--- view #{i}  ({len(gripper2base)} captured) ---")
            try:
                tok = input("  Aim the D405 at the board, then Enter to capture "
                            "[q=finish, r=retry]: ").strip().lower()
            except EOFError:
                break
            if tok == "q":
                break
            if tok == "r":
                continue

            # Grab a few frames; keep the one with the most ChArUco corners.
            best = None
            for _ in range(max(1, args.frames)):
                img = cam.grab()
                if img is None:
                    continue
                cc, ci = handeye.detect_charuco(img, board, dictionary)
                n = 0 if cc is None else len(cc)
                if best is None or n > best[0]:
                    best = (n, img, cc, ci)
            if best is None:
                print("     no D405 frame grabbed -- retry."); continue
            n, img, cc, ci = best
            if n < args.min_corners:
                print(f"     only {n} ChArUco corners (need >= {args.min_corners}). "
                      "Get the whole board in view, reduce glare, hold steady; retry.")
                annotate(img, cc, ci, path=os.path.join(args.capture_dir, f"view_{i:02d}_FAIL.jpg"))
                continue

            pose = handeye.estimate_board_pose(cc, ci, board, cam.K, cam.dist)
            if pose is None:
                print("     board pose solve failed -- retry."); continue
            rvec, tvec = pose

            o_t_ee, mode = fresh_ee()
            if o_t_ee is None:
                print("     no robot state (bringup down?). Relaunch it, then Enter to retry.",
                      file=sys.stderr)
                input("     Enter to retry: ")
                o_t_ee, mode = fresh_ee()
                if o_t_ee is None:
                    print("     still no state -- skipping this view."); continue

            T_base_ee = handeye.ee_pose_from_o_t_ee(o_t_ee)
            T_target_cam = handeye.rvectvec_to_matrix(rvec, tvec)
            gripper2base.append(T_base_ee)
            target2cam.append(T_target_cam)
            annotate(img, cc, ci, K=cam.K, dist=cam.dist, pose=pose,
                     path=os.path.join(args.capture_dir, f"view_{i:02d}.jpg"))
            print(f"     OK: {n} corners  board_z={tvec.reshape(3)[2]:.3f} m  "
                  f"EE xyz=({T_base_ee[0,3]:.3f},{T_base_ee[1,3]:.3f},{T_base_ee[2,3]:.3f}) "
                  f"mode={mode}  -> {args.capture_dir}/view_{i:02d}.jpg")
            i += 1
    except KeyboardInterrupt:
        print("\ninterrupted -- proceeding to solve with what was collected.")
    finally:
        cam.close()
        node.destroy_node()
        rclpy.shutdown()

    return solve_report(gripper2base, target2cam, spec, args)


def solve_report(gripper2base, target2cam, spec, args):
    n = len(gripper2base)
    print(f"\n=== solve === ({n} views, method={args.method})")
    if n < args.min_views:
        print(f"ERROR: need >= {args.min_views} views, have {n} -- nothing saved.",
              file=sys.stderr)
        return 1

    T_ee_cam = handeye.solve_hand_eye(gripper2base, target2cam, method=args.method)
    trms, rrms = handeye.hand_eye_residual(gripper2base, target2cam, T_ee_cam)

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    np.savez(
        args.out,
        T_ee_cam=T_ee_cam,                       # cam2gripper: X_ee = T_ee_cam @ X_cam
        T_cam_ee=handeye.invert_transform(T_ee_cam),
        method=args.method,
        n_views=n,
        trans_rms_m=trms,
        rot_rms_deg=rrms,
        square_length=spec.square_length,
        marker_length=spec.marker_length,
        squares_x=spec.squares_x,
        squares_y=spec.squares_y,
        dictionary=spec.dictionary,
        gripper2base=np.asarray(gripper2base, float),
        target2cam=np.asarray(target2cam, float),
        convention="T_ee_cam maps a camera-frame point to the EE frame "
                   "(X_ee = T_ee_cam @ X_cam); equals OpenCV cam2gripper.",
    )
    t = T_ee_cam[:3, 3]
    print(f"T_ee_cam (camera in EE frame):\n{np.array_str(T_ee_cam, precision=5, suppress_small=True)}")
    print(f"camera offset from EE       : ({t[0]*1000:.1f}, {t[1]*1000:.1f}, {t[2]*1000:.1f}) mm")
    print(f"consistency residual        : trans {trms*1000:.2f} mm   rot {rrms:.2f} deg")
    print(f"saved                       : {args.out}")
    bad = trms * 1000 > args.max_trans_rms_mm or rrms > args.max_rot_rms_deg
    if bad:
        print(f"WARNING: residual exceeds limits (trans <= {args.max_trans_rms_mm} mm, "
              f"rot <= {args.max_rot_rms_deg} deg). Capture MORE views with MORE rotation "
              "variety, full board in frame, less glare; then re-run.")
        return 1
    return 0


# --------------------------------------------------------------------------- #
def build_parser():
    ap = argparse.ArgumentParser(description="Guided D405 eye-in-hand (ChArUco) calibration.")
    ap.add_argument("--test-detect", default="", help="offline: detect ChArUco on an image and exit")
    ap.add_argument("--selftest", action="store_true", help="offline math self-test (no ROS/camera)")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--capture-dir", default=DEFAULT_CAPTURE_DIR)
    # ChArUco board geometry (METERS). Default square = 2.5 cm.
    ap.add_argument("--squares-x", type=int, default=5)
    ap.add_argument("--squares-y", type=int, default=7)
    ap.add_argument("--square-length", type=float, default=0.025, help="square side, meters (default 0.025 = 2.5 cm)")
    ap.add_argument("--marker-length", type=float, default=None, help="marker side, meters (default 0.75*square)")
    ap.add_argument("--dictionary", default=handeye.DEFAULT_DICTIONARY)
    # D405 capture (pyrealsense2).
    ap.add_argument("--serial", default="", help="D405 serial (default: first device)")
    ap.add_argument("--width", type=int, default=None, help="color width (default: auto 1280/848/640)")
    ap.add_argument("--height", type=int, default=None)
    ap.add_argument("--frames", type=int, default=4, help="frames grabbed per view (keep the busiest)")
    # Solve.
    ap.add_argument("--method", default="tsai", choices=sorted(handeye.HAND_EYE_METHODS))
    ap.add_argument("--min-views", type=int, default=8)
    ap.add_argument("--min-corners", type=int, default=8, help="min ChArUco corners to accept a view")
    ap.add_argument("--max-trans-rms-mm", type=float, default=5.0)
    ap.add_argument("--max-rot-rms-deg", type=float, default=1.5)
    # Offline pose option for --test-detect.
    ap.add_argument("--intrinsics", default="", help="npz with camera_matrix[/dist_coeffs] for --test-detect pose")
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv if argv is not None else sys.argv[1:])
    if args.selftest:
        return selftest(args)
    if args.test_detect:
        return test_detect(args)
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
